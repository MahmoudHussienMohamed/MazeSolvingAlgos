#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "GraphTraversalAlgorithms.h"
#include "MazeGenerator.h"
namespace py = pybind11;
#define AddGTAclass(GTA /*GraphTraversalAlgorithm*/){               \
    py::class_<GTA>(m, #GTA)                                        \
        .def(py::init<Grid&, Index, Index>(),                       \
            py::arg("grid"), py::arg("start"), py::arg("end"))      \
        .def("solve", &GTA::solve)                                  \
        .def("SrcToDestPath", &GTA::SrcToDestPath)                  \
        .def("SrcToDestDistance", &GTA::SrcToDestDistance)          \
        .def("TraversedNodes", &GTA::TraversedNodes)                \
        .def("TraversedNodesNo", &GTA::TraversedNodesNo);           \
    /* All GTAs have common utility functions and constructor */    \
}
PYBIND11_MODULE(MazeSolvingAlgos, m) {
    py::class_<Index>(m, "Index", 
        "   Simple class to encapsulate 2D position of a 1x1 square in the Maze.")
        .def(py::init<size_t, size_t>(), py::arg("row") = 0, py::arg("col") = 0)
        .def_readwrite("row", &Index::row)
        .def_readwrite("col", &Index::col)
        .def("__repr__", &Index::as_string)
        .def("__eq__", &Index::operator==)
        .def("__ne__", &Index::operator!=);
    py::class_<RandomMazeGenerator>(m, "RandomMazeGenerator", 
        "   Simple Random-Generator using BackTracking.")
        .def(py::init<size_t, size_t>(), py::arg("hight") = 0, py::arg("width") = 0)
        .def("generate", &RandomMazeGenerator::generate);
    AddGTAclass(DepthFirstSearch);
    AddGTAclass(BreadthFirstSearch);
    AddGTAclass(DijkstraAlgorithm);
    AddGTAclass(AStar);
    AddGTAclass(BellmanFord);
    AddGTAclass(FloydWarshall);
    AddGTAclass(BidirectionalSearch);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
