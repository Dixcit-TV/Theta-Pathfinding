# Theta-Pathfinding
Comparison of A* and basic Theta* on a square grid.

Project Information:
  - Elite Framework (DAE provided Framework)
  - A* and Theta* implementations can be found [here](https://github.com/Dixcit-TV/Theta-Pathfinding/tree/main/source/framework/EliteAI/EliteGraphs)
  - [Main application](https://github.com/Dixcit-TV/Theta-Pathfinding/tree/main/source/projects/App_PathFinding/App_Pathfinding.cpp)
  
For this comparison both A* and the A* part of Theta* uses the same logic and data structures. This project is based on a weighted, un-directed grid graph considering diagonal connections.

### Pathfinding

Pathfinding or pathing, in computer applications, represents to attempt at plotting an optimal path between 2 points from a graph. It is closely related to the shortest path problem. Pathfinding algorithms will use differents by starting at one vertex and increasingly expend its neighbors until the destination node is reached. It is important to note that although the focus of my research is based on a grid graph, to find a walkable path, pathfinding algorithm can be used in different problem solving situation such as decision making (calculating the best next moves in chess) or puzzle solving (find the least amount of move to complete a game of sudoku).


### Theta* - Any Angle pathfinding

An alternative to the highly used [A*](https://en.wikipedia.org/wiki/A*_search_algorithm) that I researched is called [Theta*](https://en.wikipedia.org/wiki/Theta*). Theta* is an **any-angle pathing algorithm** which based on A* and can find a near-optimal path (generally shorter) in similar execution time. The core of the algorithm is highly similar to A* but when A* is restricted to the grid directions (connections), the Theta* will add an extra processing step in order to trace straight path in any direction between two nodes as long as a line of sight exists between them. For example, if a line of sight exist between the current node neighbor and its parent, this node ignored and the direct path from parent to neighbor will be prefered. By doing so, the path is smoothed while searching.

The following comparison is done using a **25x25 grid**, the distance represent to sum of the **euclidean distance** between each node of the resulting path and the time is the average of **100 iterations** of the visible path. Time and distance can be seen on the bottom right of the images.

A* (blue) vs Theta* (Green)

![](https://github.com/Dixcit-TV/Theta-Pathfinding/blob/main/images/AStarvsThetaStar.png)

![](https://github.com/Dixcit-TV/Theta-Pathfinding/blob/main/images/AStarvsThetaStar2.png)

#### Line of Sight and Bresenham's line algorithm

In order to determine if a line of sight exists between two nodes we need to check if the straight line drawn from one to the other does not intersect any obstacles. A naive implementation of a line of sight check would be to compute the smallest axis-aligned bounding box encapsulating the two nodes and then loop through all the nodes inside the bounding box to verify if 1) it intersects the straight line and 2) represents an obstacle. A problem with that implementation is that the farthest the nodes are from each other the more checks will be perform on uncessary nodes rather than on nodes actually affecting the line of sight. Alternative is the [Bresenham's line algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm).

This algorithm draws a straight line by moving 1 unit (or grid cell) in one axis (x axis if x2 - x1 > y2 - y1, y otherwise) and uses the slope error accumulated through each step to determine if a similar jump in another axis is required. The process is repeated until the destination is reached and allow us to only check the node as close as possible to the line. In the case of Theta*, we can check if each step jump leads to an obstructed grid node, if it does the line expansion can be stoped and the line of sight is determined invalid.

![Wikipedia - Bresenham's line algorithm](https://github.com/Dixcit-TV/Theta-Pathfinding/blob/main/images/Bresenham's%20line%20algorithm%20-%20Wikipedia.png)

One last consideration I needed to take into account is that Bresenham's line algorithm gives an approximation and some cells may be "missed" when a jumping in both x and y axis (i.e. only (x+1, y+1) will be checked and not (x+1, y) or (x, y+1)), [Bresenham-based supercover line algorithm](http://eugen.dedu.free.fr/projects/bresenham/) takes care of these cases by remembering the error amount of the previous step and uses it to decide if neighbor cells need to be checked.

Theta* path using Bresenham, we can see some invalid corners (crossing blue tiles) are still counted as valid.
![Theta* with Bresenham](https://github.com/Dixcit-TV/Theta-Pathfinding/blob/main/images/Bresenham.png) 

Theta* path using Bresenham - Supercover, the path doesn't intersect with any walls (blue tiles) anymore.
![Theta* with Bresenham - Supercover](https://github.com/Dixcit-TV/Theta-Pathfinding/blob/main/images/Bresenham_Supercover.png)


### What about A* with post processing path smoothing?

While A* restricts the node expansions and path tracing to the grid axis, which would cause a lot of unnatural 90º or 45º direction changes if we send an agent down the resulting path. The path can be subjected to a post processing smoothing algorithm which will (in a same fashion as the Theta* preprocessing) look at all the nodes of the path and evaluate if a straight line is valid with any nodes further down the line and discard all the existing in-between steps. A* + path smoothing (PS) will result in a final path shorter than A* but, theoretically, Theta* will still have a shorter path as A* will still selects more nodes during pathing and only those nodes are used when smoothing.

The following images shows the comparison between the final path of A* + PS and Theta* using the same setup as the previous A*, Theta* comparison. The smoothing algorithm is counted in the timing and is done using **Bresenham-based supercover line algorithm**.

A*

![](https://github.com/Dixcit-TV/Theta-Pathfinding/blob/main/images/Astar.png)

A* + SP

![](https://github.com/Dixcit-TV/Theta-Pathfinding/blob/main/images/AStar_Smoothed.png)

A* + SP (blue) vs Theta* (Green)

![](https://github.com/Dixcit-TV/Theta-Pathfinding/blob/main/images/AStar_SmoothedvsThetaStar.png)
![](https://github.com/Dixcit-TV/Theta-Pathfinding/blob/main/images/AStar_SmoothedvsThetaStar2.png)


## Conclusion

More times than not, Theta* result in a smoother and shorter path than A* with a time difference close to neglectable. It should be noted that the difference in timing can vary greatly depending on the used Heuristic for those algorithms (comparisons were always made using the same Heuristic). Theta* indeed seemed to often offer more interesting results with a **Euclidean Distance Heuristic** with which it resulted in a shorter path with a faster execution time.
Reading other papers about similar comparison between A*, A* + PS and Theta* led me to the assumption that A* + SP would suffer from the run time of the post smoothing and bring Theta* closer to a win. In my case it wasn't so black and white although Theta* does provide significant benefits over a basic A* when a shorter or more natural path is required. On the other hand, Theta* might lose a great part of it when compared to A* with path smoothing. Two main cases araised: when many random obstacles are set, causing slaloming and detours, Theta* will try to find a shorter path around the majority of obstacles, while A* will run right through it. Smoothing such A* path will still result in many un-natural hard corners and be slightly longer. The second case is for more distinct path restriction, with less small obstacles scattered but more walls and corridors. Then more often than I expected Theta* and A* + PS resulted in a (close to) identical path, run time then would be the tie cutter.

### Future work and reflection

The tests and comparison performed here were quite restricted, although Theta* already showed some nice advantage over a basic A* when smooth path are wanted, A* + PS came to close the gap. For furture expansion of this research, I shall extend my test cases to include variation in grid size (very small, very large) and work with more wall aparition patterns and density. This would give more concrete results. 
Also it is worth noting that a slightly altered version of Theta* exists call Lazy Theta* which tries to delay a much as possible the line of sight checks, reducing them in number (avoid line of sight checking for every considered nodes). It could be a new contender to add to the list here.


## Resources

[A* Search Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)

[Any-Angle path planning](https://en.wikipedia.org/wiki/Any-angle_path_planning)

[Theta* Search Algorithm](https://en.wikipedia.org/wiki/Theta*)

[Bresenham's line algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)

[Supercover line algorithm](http://eugen.dedu.free.fr/projects/bresenham/)

[Theta*: Any-Angle Path Planning on Grids - Nash, Koening, Daniel, Felner](http://idm-lab.org/bib/abstracts/papers/aaai07a.pdf)

[Lazy Theta* - Nash, Koening, Tovey](http://idm-lab.org/bib/abstracts/papers/aaai10b.pdf)
