from setuptools import setup, Extension
import pybind11
cpp_args = ['-std=c++14', '-stdlib=libc++', '-mmacosx-version-min=10.7']
sfc_module = Extension(
    'MazeSolvingAlgos',
    sources=['main.cpp'],
    include_dirs=[pybind11.get_include()],
    language='c++',
    extra_compile_args=cpp_args,
    )

setup(
    name='MazeSolvingAlgos',
    version='1.0',
    description='Python module written in C++ for generating and solving rectangular Mazes with Graph Traversal Algorithms.',
    ext_modules=[sfc_module],
)