import pygame
import heapq
import math
from collections import deque

# Constants
WIDTH = 800
HEIGHT = 600
GRID_SIZE = 20
BACKGROUND_COLOR = (0, 0, 0)
WALL_COLOR = (50, 50, 50)
PATH_COLOR = (255, 0, 0)
START_COLOR = (0, 255, 0)
END_COLOR = (0, 0, 255)
OPEN_COLOR = (255, 255, 255)
CLOSED_COLOR = (128, 128, 128)
TEXT_COLOR = (255, 255, 255)

class Node:
    def __init__(self, row, col, width):
        self.row = row
        self.col = col
        self.x = col * width
        self.y = row * width
        self.width = width
        self.color = BACKGROUND_COLOR
        self.neighbors = []
        self.g_score = float('inf')  # To keep track of g_score in A*
        self.f_score = float('inf')  # To keep track of f_score in A*

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == CLOSED_COLOR

    def is_open(self):
        return self.color == OPEN_COLOR

    def is_wall(self):
        return self.color == WALL_COLOR

    def is_start(self):
        return self.color == START_COLOR

    def is_end(self):
        return self.color == END_COLOR

    def reset(self):
        self.color = BACKGROUND_COLOR

    def make_start(self):
        self.color = START_COLOR

    def make_end(self):
        self.color = END_COLOR

    def make_wall(self):
        self.color = WALL_COLOR

    def make_open(self):
        self.color = OPEN_COLOR

    def make_closed(self):
        self.color = CLOSED_COLOR

    def make_path(self):
        self.color = PATH_COLOR

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors = []
        if self.row < len(grid) - 1 and not grid[self.row + 1][self.col].is_wall():  # Down
            self.neighbors.append(grid[self.row + 1][self.col])
        if self.row > 0 and not grid[self.row - 1][self.col].is_wall():  # Up
            self.neighbors.append(grid[self.row - 1][self.col])
        if self.col < len(grid[0]) - 1 and not grid[self.row][self.col + 1].is_wall():  # Right
            self.neighbors.append(grid[self.row][self.col + 1])
        if self.col > 0 and not grid[self.row][self.col - 1].is_wall():  # Left
            self.neighbors.append(grid[self.row][self.col - 1])

    def __lt__(self, other):
        return self.f_score < other.f_score


# A* algorithm
def astar_algorithm(draw, grid, start, end):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start] = 0
    f_score = {node: float("inf") for row in grid for node in row}
    f_score[start] = heuristic(start.get_pos(), end.get_pos())

    while open_set:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        current = heapq.heappop(open_set)[1]

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            tentative_g_score = g_score[current] + 1
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor.get_pos(), end.get_pos())
                neighbor.g_score = g_score[neighbor]
                neighbor.f_score = f_score[neighbor]
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    neighbor.make_open()

        draw()
        if current != start:
            current.make_closed()

    return False

# BFS algorithm
def bfs_algorithm(draw, grid, start, end):
    queue = deque([start])
    came_from = {}
    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start] = 0

    while queue:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        current = queue.popleft()
        
        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            if g_score[neighbor] == float("inf"):
                came_from[neighbor] = current
                g_score[neighbor] = g_score[current] + 1
                queue.append(neighbor)
                neighbor.make_open()

        draw()
        if current != start:
            current.make_closed()

    return False

# Heuristic function for A* algorithm
def heuristic(pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    return abs(x1 - x2) + abs(y1 - y2)

# Reconstruct path from end node to start node
def reconstruct_path(came_from, current, draw):
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()

# Draw grid and nodes
def draw_grid(win, grid):
    for row in grid:
        for node in row:
            node.draw(win)

def draw(win, grid, algorithm):
    win.fill(BACKGROUND_COLOR)
    draw_grid(win, grid)
    pygame.display.update()

def main():
    pygame.init()
    win = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Pathfinding Visualization")

    grid = [[Node(row, col, GRID_SIZE) for col in range(WIDTH // GRID_SIZE)] for row in range(HEIGHT // GRID_SIZE)]
    start = None
    end = None

    run = True
    while run:
        draw(win, grid, None)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if pygame.mouse.get_pressed()[0]:  # Left mouse button
                pos = pygame.mouse.get_pos()
                row, col = pos[1] // GRID_SIZE, pos[0] // GRID_SIZE
                node = grid[row][col]
                if not start and node != end:
                    start = node
                    start.make_start()
                elif not end and node != start:
                    end = node
                    end.make_end()
                elif node != end and node != start:
                    node.make_wall()
            elif pygame.mouse.get_pressed()[2]:  # Right mouse button
                pos = pygame.mouse.get_pos()
                row, col = pos[1] // GRID_SIZE, pos[0] // GRID_SIZE
                node = grid[row][col]
                node.reset()
                if node == start:
                    start = None
                elif node == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)
                    astar_algorithm(lambda: draw(win, grid, "A*"), grid, start, end)
                if event.key == pygame.K_b and start and end:
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)
                    bfs_algorithm(lambda: draw(win, grid, "BFS"), grid, start, end)

    pygame.quit()

if __name__ == "__main__":
    main()
