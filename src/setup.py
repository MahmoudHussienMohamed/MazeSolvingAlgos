from setuptools import setup, Extension
import os
import pybind11
currDir = os.path.dirname(os.path.abspath(__file__))
cpp_args = ['-std=c++14', '-stdlib=libc++', '-mmacosx-version-min=10.7']
sfc_module = Extension(
    'MazeSolvingAlgos',
    sources=['main.cpp'],
    include_dirs=[
        pybind11.get_include(),
        os.path.join(currDir, "GraphTraversalAlgorithms.h"),
        os.path.join(currDir, "MazeGenerator.h"),
        os.path.join(currDir, "Graph.h"),
        ],
    language='c++',
    extra_compile_args=cpp_args,
    )

setup(
    name='MazeSolvingAlgos',
    version='1.5',
    author="Mahmoud Hussien Mohamed",
    author_email="MahmoudHussienMohamed.mhm@gmail.com",
    url="https://github.com/MahmoudHussienMohamed/MazeSolvingAlgos",
    description='Python module written in C++ for generating and solving rectangular Mazes with Graph Traversal Algorithms.',
    ext_modules=[sfc_module],
    classifiers=[
        "Programming Language :: C++",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: Microsoft :: Windows",
    ],
    requires=["setuptools", "wheel", "pybind11"],
    include_package_data=True,
)
