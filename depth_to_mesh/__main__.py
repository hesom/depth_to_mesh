#!/usr/bin/env python

import argparse
from depth_to_mesh import *
import open3d as o3d

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Converts a depth image to a triangle mesh')
    parser.add_argument('image', type=str, help='path to the image file')
    parser.add_argument('output', type=str, help='name of output file')
    parser.add_argument('--camera', '-c', type=str, help='path to camera matrix', default=None)
    parser.add_argument('--sun3d', '-s', dest='sun3d', action='store_true', help='set if image is in SUN3D format')
    parser.add_argument('--log', '-l', type=str, help='specify logging level', default='INFO')
    parser.add_argument('--min-angle', '-e', type=float, help='specify the minimum angle in degrees between viewing ray and triangles', default=3)
    parser.add_argument('--depth-scale', '-d', type=float, help='specify the depth scale', default=1000.0)
    args = parser.parse_args()

    numeric_level = getattr(logging, args.log.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % args.log)
    
    logging.basicConfig(level=numeric_level)

    if args.camera is not None:
        cameraMatrix = np.loadtxt(args.camera)
    else:
        cameraMatrix = None

    mesh = depth_file_to_mesh(args.image, cameraMatrix, args.min_angle, args.sun3d, args.depth_scale)

    o3d.io.write_triangle_mesh(args.output, mesh)
