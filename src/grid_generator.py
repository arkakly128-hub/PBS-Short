# grid_generator.py
# Generate random grids for pathfinding benchmarks
import random
from typing import List, Tuple

def generate_random_grid(rows: int, cols: int, obstacle_ratio: float = 0.3) -> List[List[int]]:
    grid = [[0 for _ in range(cols)] for _ in range(rows)]
    for i in range(rows):
        for j in range(cols):
            if random.random() < obstacle_ratio:
                grid[i][j] = 1  # 1 = obstacle
    return grid

def find_empty_spot(grid: List[List[int]]) -> Tuple[int, int]:
    while True:
        x = random.randint(0, len(grid)-1)
        y = random.randint(0, len(grid[0])-1)
        if grid[x][y] == 0:
            return x, y

def generate_solvable_instance(rows=50, cols=50, obs=0.3):
    grid = generate_random_grid(rows, cols, obs)
    start = find_empty_spot(grid)
    goal = start
    while goal == start:
        goal = find_empty_spot(grid)
    grid[start[0]][start[1]] = 0
    grid[goal[0]][goal[1]] = 0
    return grid, start, goal
