# Pathfinding_Visualization_Tool_Python

## Description

This Python tool allows users to visualize the behavior of two popular pathfinding algorithms: A* (A-star) and Breadth-First Search (BFS).  It features an intuitive graphical interface where users can interactively set up obstacles, start, and end points on a grid.  Users can then watch in real-time as the algorithms calculate and display the shortest path.  This tool is excellent for understanding how these algorithms work and comparing their performance.

## Features

* **Algorithm Implementations:** Implements A* (A-star) and Breadth-First Search (BFS) pathfinding algorithms. 🤖

* **Real-time Visualization:** Visualizes the search process of the algorithms in real-time using a graphical user interface. 👀

* **Interactive Grid:** Users can interactively set start and end points, and place obstacles (walls) on the grid. 🧱

* **User Control:** Options to step through the algorithm or run it in real-time. ⏯️

* **Start/End Point Placement:** Click to set the start (green) and end (blue) points. 🟢🔵

* **Obstacle Placement:** Click to place obstacles (walls) on the grid (gray). 🚧

* **Removal:** Right-click to remove obstacles or clear start/end points (back to black). ⬛

* **A* Algorithm Execution:** Press SPACE to run the A* algorithm. ✨

* **BFS Algorithm Execution:** Press B to run the BFS algorithm. 🔍

* **Visual Feedback:**
    * **Open Cells (Explored):** Cells being explored turn white. ◻️
    * **Closed Cells (Fully Explored):** Cells that have been fully explored turn gray. ⬛
    * **Path:** The final shortest path is highlighted in red. 🟥

## Technologies Used

* **Python:** The primary programming language. 🐍

* **Pygame (or similar GUI library):** For creating the graphical user interface. 🎮

## Ideal For

* **Students:** Learning about pathfinding algorithms and data structures. 🧑‍🎓

* **Game Developers:**  Visualizing pathfinding for game AI. 🎮

* **Algorithm Enthusiasts:** Exploring and comparing different search algorithms. 🤓

## How to Run

1. **Clone the repository:** `git clone <repo url>`

2. **Install Pygame:** `pip install pygame` (or `pip install -r requirements.txt` if you have one)

3. **Run the program:** `python pathfinding_visualizer.py` (or `python3 pathfinding_visualizer.py`)

## How to Use

1. Click on the grid to set the start (green) and end (blue) points.

2. Click on cells to create obstacles (walls - gray).

3. Right-click to remove obstacles or clear start/end points.

4. Press SPACE to run the A* algorithm.

5. Press B to run the BFS algorithm.

6. Observe the visualization of the search process and the final path (red).

# A* Algorithm

![A* Algorithm](https://github.com/LakshayD02/Pathfinding_Visualization_Tool_Python/blob/main/A-Star%20Algorithm.png) <br/>

# Breadth First Search (BFS)

![BFS](https://github.com/LakshayD02/Pathfinding_Visualization_Tool_Python/blob/main/BFS.png)
