# Theta-Pathfinding
Comparison of A* and basic Theta* on a square grid

Project Information:
  - Elite Framework (DAE provided Framework)
  - A* and Theta* implementations can be found [here](https://github.com/Dixcit-TV/Theta-Pathfinding/tree/main/source/framework/EliteAI/EliteGraphs)
  - [Main application](https://github.com/Dixcit-TV/Theta-Pathfinding/tree/main/source/projects/App_PathFinding/App_Pathfinding.cpp)
  
For this comparison both A* and the A* part of Theta* uses the same logic and data structures. This project is based on a weighted, un-directed grid graph considering diagonal connections.

### Pathfinding

Pathfinding or pathing, in computer applications, represents to attempt at plotting an optimal path between 2 points from a graph. It is closely related to the shortest path problem. Pathfinding algorithms will use differents by starting at one vertex and increasingly expend its neighbors until the destination node is reached. It is important to note that although the focus of my research is based on a grid graph, to find a walkable path, pathfinding algorithm can be used in different problem solving situation such as decision making (calculating the best next moves in chess) or puzzle solving (find the least amount of move to complete a game of sudoku).

### Theta* - Any Angle pathfinding

An alternative to the highly used [A*](https://en.wikipedia.org/wiki/A*_search_algorithm) that I researched is called [Theta*](https://en.wikipedia.org/wiki/Theta*). Theta* is an **any-angle pathing algorithm** which based on A* and can find a near-optimal path in similar execution time. The core of the algorithm is highly similar to A* but when A* is restricted to the grid directions (connections), the Theta* will add an extra processing step in order to trace straight path in any direction between two nodes as long as a line of sight exists between them. For example, if a line of sight exist between the current node neighbor and its parent, this node ignored and the direct path from parent to neighbor will be prefered. By doing so, the path is smoothed while searching.

#### Line of Sight and Bresenham's line algorithm

In order to determine if a line of sight exists between two nodes we need to check if the straight line drawn from one to the other does not intersect any obstacles. A naive implementation of a line of sight check would be to compute the smallest axis-aligned bounding box encapsulating the two nodes and then loop through all the nodes inside the bounding box to verify if 1) it intersects the straight line and 2) represents an obstacle. A problem with that implementation is that the farthest the nodes are from each other the more checks will be perform on uncessary nodes rather than on nodes actually affecting the line of sight. Alternative is the [Bresenham's line algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm).

This algorithm draws a straight line by moving 1 unit (or grid cell) in one axis (x axis if x2 - x1 > y2 - y1, y otherwise) and uses the slope error accumulated through each step to determine if a similar jump in another axis is required. The process is repeated until the destination is reached and allow us to only check the node as close as possible to the line. In the case of Theta*, we can check if each step jump leads to an obstruicted grid node, if it does the line expansion can be stoped and the line of sight is determined invalid.

![Wikipedia - Bresenham's line algorithm](https://github.com/Dixcit-TV/Theta-Pathfinding/blob/main/images/Bresenham's%20line%20algorithm%20-%20Wikipedia.png)

One last consideration I needed to take into account is that Bresenham's line algorithm gives an approximation and some cells may be "missed" when a jumping in both x and y axis (i.e. only (x+1, y+1) will be checked and not (x+1, y) or (x, y+1)), [Bresenham-based supercover line algorithm](http://eugen.dedu.free.fr/projects/bresenham/) takes care of these cases by remembering the error amount of the previous step and uses it to decide if neighbor cells need to be checked.

Theta* path using Bresenham, we can see some invalid corners (crossing blue tiles) are still counted as valid.
![Theta* with Bresenham](https://github.com/Dixcit-TV/Theta-Pathfinding/blob/main/images/Bresenham.png) 

Theta* path using Bresenham - Supercover, the path doesn't intersect with any walls (blue tiles) anymore.
![Theta* with Bresenham - Supercover](https://github.com/Dixcit-TV/Theta-Pathfinding/blob/main/images/Bresenham_Supercover.png)
