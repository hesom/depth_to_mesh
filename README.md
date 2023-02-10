# depth-to-mesh
depth-to-mesh is a python library / command line tool to convert depth maps into meshes. It uses the uniform structure of depth maps for its triangulation. Degenerate triangles and triangles that are likely wrong, i.e. connecting foreground and background surfaces) are filtered out.

## Installation
Simply running ```pip install depth-to-mesh``` installs all requirements

## Getting Started

To use the command line tool, execute ```python -m depth_to_mesh``` and see the output for instructions on arguments.

The python library converts depth files into Open3D TriangleMesh objects. See the [Open3D documentation](http://www.open3d.org/docs/release/) for instructions on how to use them.

## License

This project is licensed under the MIT License - see the [LICENSE.txt](LICENSE.txt) file for details
