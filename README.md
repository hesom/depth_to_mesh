# depth-to-mesh
depth-to-mesh is a python library / command line tool to convert depth maps into meshes. It uses the uniform structure of depth maps for its triangulation. Degenerate triangles and triangles that are likely wrong, i.e. connecting foreground and background surfaces) are filtered out.

## Installation
Simply running ```pip install depth-to-mesh``` installs all requirements

## Getting Started

To use the command line tool, execute ```python -m depth_to_mesh``` and see the output for instructions on arguments.

You will likely need a different intrinsic camera matrix than the default one. The matrix can be passed with the `--camera <filename>` command line parameter, where `<filename>` is the path to a text file containing the matrix.
The text file should simply have the usual shape of intrinsic matrices, i.e.
```
fx 0 cx
0 fy cy
0 0 1
```
The default matrix in text file form would be
```
528 0 319.5
0 528 239.5
0 0 1
```

The python library converts depth files into Open3D TriangleMesh objects. See the [Open3D documentation](http://www.open3d.org/docs/release/) for instructions on how to use them.

## License

This project is licensed under the MIT License - see the [LICENSE.txt](LICENSE.txt) file for details
