# MazeSolvingAlgos
***MazeSolvingAlgos*** is a Python module written in C++ for generating and solving rectangular Mazes with **Graph Traversal Algorithms**. This Module is a part of **TurtleInTheMaze** Project (you can find it [here](https://github.com/MahmoudHussienMohamed/TurtleInTheMaze)) so it's especially created for ***2D grids***. *You can see **full documentation [here](https://github.com/MahmoudHussienMohamed/MazeSolvingAlgos/wiki/Documentation)***.
## Content of module
* Helper Classes
    * Index 
    * RandomMazeGenerator 
* Solving Algorithms Classes
    - DepthFirstSearch
    - BreadthFirstSearch
    - DijkstraAlgorithm
    - AStar
    - BellmanFord
    - FloydWarshall
    - BidirectionalSearch

## How to build and install
Prerequisites:
* C++11 or later compiler
* pybind11 (via `pip install pybind11`)
* setuptools (via `pip install setuptools`)

After Cloning repositry (via `git clone https://github.com/MahmoudHussienMohamed/MazeSolvingAlgos.git`), open **cmd** (*run as administrator*) and do the following:
1. change the working directory to `\MazeSolvingAlgos\src\` 
2. run command: `python setup.py build`
3. After building is finished, run `python setup.py install`
### Update v1.7.0:
If you have a version before **1.7.0**:
- You can simply uninstall the module (via `pip uninstall MazeSolvingAlgos`) then,  re-clone repositry and do the above instructions.
- Or update modified files (***main.cpp***, ***RandomMazeGenerator.h***, ***GraphTraversalAlgorithms.h***, and ***setup.py***) and run [script.py](src/script.py) as administrator after updating `parentDir`.  

## How to use
### RandomMazeGenerator
Use `RandomMazeGenerator` class with arguments `H`, `W`: Height and Width of the maze respectively and call `generate()` to get **W** × **H** random maze (`H`, `W` must be unsigned integers).
``` Python
RMG = RandomMazeGenerator(50, 50) 
maze = RMG.generate()             # Now, maze is 50 * 50 random cells 
```
50 × 50 random maze from ***TurtleInTheMaze*** project:

![sample output](Imgs/rand50by50maze.png)

### Index
Use `Index` class with arguments `row`, `col` to encapsulate a 2D cell's position and to pass `start` and `end` parameters to any ***Graph Traversal Algorithm*** (***Note that `row` and `col` must be unsigned integers***).

``` Python
idx = Index(2, 13) 
print(idx)             # output: "2, 13" 
print(f"cell's column = {idx.col}, cell's row = {idx.row}")             
# output: "cell's column = 13, cell's row = 2"
```
### GTAs
Let GTA be any solving algorithm; GTA accepts `grid`: 2D boolean list representing the maze (*`True` for **orifice** and `False` for **block or wall***), `start`: `Index` object representing the position of starting cell and `end`: same as `start` for the destination cell.
``` Python
WIDTH = HEIGHT = 50
RMG = RandomMazeGenerator(HEIGHT, WIDTH) 
maze = RMG.generate()
start = Index(0, 4)
end = Index(49, 49)

solver = AStar(maze, start, end)    # Convert the maze to Graph. 
solver.solve()                      # Fires the algorithm logic.
solver.SrcToDestDistance()          # Exact cells' number from start to end cell.
solver.SrcToDestPath()              # List of Index objects representing route. 
solver.TraversedNodesNo()           # Number of cells the algorithm visited (may be > total cells of maze).
solver.TraversedNodes()             # List of Index objects representing unique traversed cells. 
```
![Random 50 * 50 Maze](Imgs/5050Maze.png)
![Previous Maze Solved with A*](Imgs/5050MazeWithAstar.png)

(*Traversed cells are gray and solution is the colorful colorful path from green to red*)

Note that:
* Any algorithm have same constructor and methods including but not limited to `Astar`.
* `SrcToDestDistance()` != `len(SrcToDestPath())` and `TraversedNodesNo()` != `len(TraversedNodes())` because I implemented the graph to be weighted for some algorithms and elemenate useless cells to increase memory efficiency and perfomance.

## Source Files
* [MazeGenerator.h](src/MazeGenerator.h): Header file contains `RandomMazeGenerator` class.
* [Graph.h](src/Graph.h): Header file contains most preprocessing (`Unweighted`/`weightedGraph`, `...`).
* [GraphTraversalAlgorithms.h](src/GraphTraversalAlgorithms.h): Header file contains all solving Algorithms.
* [main.cpp](src/main.cpp): Source file which handle Python–C++ binding with pybind11.
* [setup.py](src/setup.py): Helper file for packaging the project for python.
* [script.py](src/script.py): Simple script automates updating the module after any modification.
## Notes
### Update version 1.7.0:
- Added `BidirectionalSearch` algorithm.
- Enhanced `RandomMazeGenerator` and **fixed *odd* dimensions bug**. 