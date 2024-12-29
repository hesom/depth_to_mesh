from setuptools import setup

setup(
    name='depth-to-mesh',
    version='0.1.6',
    description='Converts depth maps into triangle meshes',
    url='https://github.com/hesom/depth_to_mesh',
    author='Hendrik Sommerhoff',
    author_email='sommerhoff.hendrik@gmail.com',
    license='MIT',
    packages=['depth_to_mesh'],
    install_requires=[
        'open3d>=0.18,<0.19',
        'numpy',
        'scikit-image',
        'tqdm'
    ],
    zip_safe=False
)
