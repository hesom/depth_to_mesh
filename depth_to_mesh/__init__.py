#!/usr/bin/env python

import pathlib
import itertools
import open3d.open3d as o3d
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
    x = np.linspace(0, width - 1, width).astype(np.int)
    y = np.linspace(0, height - 1, height).astype(np.int)
    [x, y] = np.meshgrid(x, y)
    return np.vstack((x.flatten(), y.flatten(), np.ones_like(x.flatten())))

def depth_file_to_mesh(image, cameraMatrix=DEFAULT_CAMERA, minAngle=3.0, sun3d=False):
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
        depth_raw = np.bitwise_or(depth_raw>>3, depth_raw<<13).astype('float32') / 1000.0
    
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
    Converts an open3d.geometry.Image depth image into a open3d.geometry.TriangleMesh object

    :param depth: np.array of type float32 containing the depth image
    :param camera: open3d.camera.PinholeCameraIntrinsic
    :param minAngle: Minimum angle between viewing rays and triangles in degrees
    :returns: an open3d.geometry.TriangleMesh containing the converted mesh
    """
    
    logger.info('Reprojecting points...')
    K = camera.intrinsic_matrix
    K_inv = np.linalg.inv(K)
    pixel_coords = _pixel_coord_np(depth.shape[1], depth.shape[0])
    cam_coords = K_inv @ pixel_coords * depth.flatten()

    indices = o3d.utility.Vector3iVector()
    w = camera.width
    h = camera.height

    with tqdm(total=(h-1)*(w-1)) as pbar:
        for i in range(0, h-1):
            for j in range(0, w-1):
                verts = [
                    cam_coords[:, w*i+j],
                    cam_coords[:, w*(i+1)+j],
                    cam_coords[:, w*i+(j+1)],
                ]
                if [0,0,0] in map(list, verts):
                    continue
                v1 = verts[0] - verts[1]
                v2 = verts[0] - verts[2]
                n = np.cross(v1, v2)
                n /= np.linalg.norm(n)
                center = (verts[0] + verts[1] + verts[2]) / 3.0
                u = center / np.linalg.norm(center)
                angle = math.degrees(math.asin(abs(np.dot(n, u))))
                if angle > minAngle:
                    indices.append([w*i+j, w*(i+1)+j, w*i+(j+1)])

                verts = [
                    cam_coords[:, w*i+(j+1)],
                    cam_coords[:, w*(i+1)+j],
                    cam_coords[:, w*(i+1)+(j+1)],
                ]
                if [0,0,0] in map(list, verts):
                    continue
                v1 = verts[0] - verts[1]
                v2 = verts[0] - verts[2]
                n = np.cross(v1, v2)
                n /= np.linalg.norm(n)
                center = (verts[0] + verts[1] + verts[2]) / 3.0
                u = center / np.linalg.norm(center)
                angle = math.degrees(math.asin(abs(np.dot(n, u))))
                if angle > minAngle:
                    indices.append([w*i+(j+1),w*(i+1)+j, w*(i+1)+(j+1)])
                pbar.update(1)
    
    points = o3d.utility.Vector3dVector(cam_coords.transpose())

    mesh = o3d.geometry.TriangleMesh(points, indices)
    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()
    
    return mesh