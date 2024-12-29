#!/usr/bin/env python

import pathlib
import itertools
import open3d as o3d
import numpy as np
import math
import logging
from skimage.io import imread
from tqdm import tqdm

DEFAULT_CAMERA = o3d.camera.PinholeCameraIntrinsic(
    width=640, height=480,
    fx=528.0, fy=528.0,
    cx=319.5, cy=239.5
)

logger = logging.getLogger(__name__)

def _pixel_coord_np(width, height):
    """
    Pixel in homogenous coordinate
    Returns:
        Pixel coordinate:       [3, width * height]
    """
    x = np.linspace(0, width - 1, width).astype(np.int32)
    y = np.linspace(0, height - 1, height).astype(np.int32)
    [x, y] = np.meshgrid(x, y)
    return np.vstack((x.flatten(), y.flatten(), np.ones_like(x.flatten())))

def depth_file_to_mesh(image, cameraMatrix=DEFAULT_CAMERA, minAngle=3.0, sun3d=False, depthScale=1000.0):
    """
    Converts a depth image file into a open3d TriangleMesh object

    :param image: path to the depth image file
    :param cameraMatrix: numpy array of the intrinsic camera matrix
    :param minAngle: Minimum angle between viewing rays and triangles in degrees
    :param sun3d: Specify if the depth file is in the special SUN3D format
    :returns: an open3d.geometry.TriangleMesh containing the converted mesh
    """
    depth_raw = imread(image).astype('uint16')
    width = depth_raw.shape[1]
    height = depth_raw.shape[0]

    if sun3d:
        depth_raw = np.bitwise_or(depth_raw>>3, depth_raw<<13)

    depth_raw = depth_raw.astype('float32')
    depth_raw /= depthScale

    logger.debug('Image dimensions:%s x %s', width, height)
    logger.debug('Camera Matrix:%s', cameraMatrix)

    if cameraMatrix is None:
        camera = DEFAULT_CAMERA
    else:
        camera = o3d.camera.PinholeCameraIntrinsic(
            width=width, height=height,
            fx=cameraMatrix[0,0], fy=cameraMatrix[1,1],
            cx=cameraMatrix[0,2], cy=cameraMatrix[1,2]
        )
    return depth_to_mesh(depth_raw.astype('float32'), camera, minAngle)

def depth_to_mesh(depth, camera=DEFAULT_CAMERA, minAngle=3.0):
    """
    Vectorized version of converting a depth image to a mesh, filtering out invalid triangles.

    :param depth: np.array of type float32 containing the depth image
    :param camera: open3d.camera.PinholeCameraIntrinsic
    :param minAngle: Minimum angle between viewing rays and triangles in degrees
    :returns: an open3d.geometry.TriangleMesh containing the converted mesh
    """
    logger.info('Reprojecting points...')

    K = camera.intrinsic_matrix
    K_inv = np.linalg.inv(K)

    # Reproject all points
    pixel_coords = _pixel_coord_np(depth.shape[1], depth.shape[0])
    cam_coords = K_inv @ pixel_coords * depth.flatten()

    # Generate indices for triangles
    h, w = depth.shape
    i, j = np.meshgrid(np.arange(h - 1), np.arange(w - 1), indexing='ij')
    i, j = i.flatten(), j.flatten()

    # Form two triangles for each quad
    idx_t1 = np.stack([i * w + j, (i + 1) * w + j, i * w + (j + 1)], axis=-1)
    idx_t2 = np.stack([i * w + (j + 1), (i + 1) * w + j, (i + 1) * w + (j + 1)], axis=-1)

    # Combine triangle indices
    indices = np.vstack([idx_t1, idx_t2])

    # Extract vertices for each triangle
    verts = cam_coords[:, indices]

    # Calculate normals
    v1 = verts[:, :, 1] - verts[:, :, 0]
    v2 = verts[:, :, 2] - verts[:, :, 0]
    normals = np.cross(v1, v2, axis=0)
    normal_lengths = np.linalg.norm(normals, axis=0)

    # Calculate angles
    centers = verts.mean(axis=2)
    center_lengths = np.linalg.norm(centers, axis=0)
    valid_normals = normal_lengths > 0
    valid_centers = center_lengths > 0

    # Filter invalid triangles (zero-length normals or centers)
    valid = valid_normals & valid_centers

    # Recalculate angles only for valid triangles
    normals = normals[:, valid] / normal_lengths[valid]
    centers = centers[:, valid] / center_lengths[valid]
    angles = np.degrees(np.arcsin(np.abs(np.einsum('ij,ij->j', normals, centers))))

    # Further filter by angle
    angle_valid = angles > minAngle
    indices = indices[valid][angle_valid].astype(np.int32)

    # Create Open3D mesh
    indices = o3d.utility.Vector3iVector(np.ascontiguousarray(indices))
    points = o3d.utility.Vector3dVector(np.ascontiguousarray(cam_coords.transpose()))
    mesh = o3d.geometry.TriangleMesh(points, indices)

    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()

    return mesh