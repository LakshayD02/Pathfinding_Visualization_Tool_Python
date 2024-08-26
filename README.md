# Pathfinding_Visualization_Tool_Python
It allows users to visualize the behavior of two popular pathfinding algorithms: A* (A-star) and BFS (Breadth-First Search). The tool features an intuitive graphical interface where users can interactively set up obstacles and start points, then watch in real-time as the algorithms calculate and display the shortest path.

# Features of the tool:

- Implement Algorithms: Implement pathfinding algorithms like A*, Dijkstra's, and Breadth-First Search (BFS).
- Visualization: Visualize the algorithms in real-time using a graphical user interface.
- Interactive: Allow users to set start and end points, and obstacles on a grid.
- User Control: Provide options to step through the algorithm or run it in real-time.

# Interacting with the Tool

- Start Point: Click on a cell in the grid where you want to set the start point. The cell will turn green.
- End Point: Click on a cell in the grid where you want to set the end point. The cell will turn blue.
- Obstacles: Click on any cell to place an obstacle (wall). The cell will turn gray.
- Removing Points and Obstacles:
- Right Mouse Button: Click on a cell with the right mouse button to remove obstacles or clear the start and end points. The cell will return to the default color (black).

# Running the Algorithm:

- A Algorithm*: Press the SPACE key to run the A* pathfinding algorithm. The algorithm will search for a path from the start point to the end point and visualize the search process.
- BFS Algorithm: Press the B key to run the Breadth-First Search (BFS) algorithm. Similar to A*, it will search for a path and visualize the process.
Visual Feedback:

- Open Cells: Cells being explored during the search will turn white.
- Closed Cells: Cells that have been fully explored and cannot be part of the path will turn gray.
- Path: Once the path is found, it will be highlighted in red.
