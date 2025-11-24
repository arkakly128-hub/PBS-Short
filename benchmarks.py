# benchmarks.py
# Benchmark PBS+Short vs BFS, DFS, GBFS, A* on random grids
import time
import heapq
from src.pbs_short import PBSShort
from src.grid_generator import generate_solvable_instance
import matplotlib.pyplot as plt
import os
import numpy as np

# -----------------------
# Heuristics & Helpers
# -----------------------
def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def get_neighbors(grid, pos):
    rows, cols = len(grid), len(grid[0])
    x, y = pos
    candidates = [(x-1,y), (x+1,y), (x,y-1), (x,y+1)]
    return [(nx,ny) for nx,ny in candidates if 0<=nx<rows and 0<=ny<cols and grid[nx][ny]==0]

# -----------------------
# Classic Algorithms
# -----------------------
def bfs(grid, start, goal):
    from collections import deque
    visited = set([start])
    q = deque([(start, [start])])
    while q:
        node, path = q.popleft()
        if node == goal:
            return path
        for n in get_neighbors(grid, node):
            if n not in visited:
                visited.add(n)
                q.append((n, path+[n]))
    return None

def dfs(grid, start, goal):
    visited = set([start])
    stack = [(start, [start])]
    while stack:
        node, path = stack.pop()
        if node == goal:
            return path
        for n in get_neighbors(grid, node):
            if n not in visited:
                visited.add(n)
                stack.append((n, path+[n]))
    return None

def gbfs(grid, start, goal):
    heap = [(manhattan(start, goal), start, [start])]
    visited = set([start])
    while heap:
        _, node, path = heapq.heappop(heap)
        if node == goal:
            return path
        for n in get_neighbors(grid, node):
            if n not in visited:
                visited.add(n)
                heapq.heappush(heap, (manhattan(n, goal), n, path+[n]))
    return None

def a_star(grid, start, goal):
    open_set = [(manhattan(start, goal), 0, start, [start])]
    g_score = {start: 0}
    while open_set:
        _, g, current, path = heapq.heappop(open_set)
        if current == goal:
            return path
        for neighbor in get_neighbors(grid, current):
            if neighbor in path:
                continue
            tentative_g = g + 1
            if tentative_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g
                f = tentative_g + manhattan(neighbor, goal)
                heapq.heappush(open_set, (f, tentative_g, neighbor, path+[neighbor]))
    return None

# -----------------------
# Plotting Helpers
# -----------------------
def plot_runtime(results, grid_size):
    names = [r[0] for r in results]
    times = [r[2] for r in results]
    plt.figure(figsize=(8,4))
    plt.bar(names, times, color='skyblue')
    plt.ylabel('Time (s)')
    plt.title(f'Algorithm Runtime on {grid_size}x{grid_size} Grid')
    plt.savefig(f'plots/runtime_{grid_size}x{grid_size}.png')
    plt.show()

def plot_path_lengths(results, grid_size):
    names = [r[0] for r in results]
    lengths = [len(r[1]) if r[1] else 0 for r in results]
    plt.figure(figsize=(8,4))
    plt.bar(names, lengths, color='lightgreen')
    plt.ylabel('Path Length (steps)')
    plt.title(f'Algorithm Path Length on {grid_size}x{grid_size} Grid')
    plt.savefig(f'plots/pathlength_{grid_size}x{grid_size}.png')
    plt.show()

def plot_pbs_path(grid, path, start, goal, grid_size):
    data = np.array(grid)
    plt.figure(figsize=(8,8))
    plt.imshow(data, cmap='Greys', origin='upper')
    if path:
        px, py = zip(*path)
        plt.plot(py, px, color='red', linewidth=2)
    plt.scatter([start[1]], [start[0]], color='green', marker='o', label='Start')
    plt.scatter([goal[1]], [goal[0]], color='blue', marker='x', label='Goal')
    plt.legend()
    plt.title(f'PBS+Short Path on {grid_size}x{grid_size} Grid')
    plt.savefig(f'plots/pbs_path_{grid_size}x{grid_size}.png')
    plt.show()

# -----------------------
# Benchmark Runner
# -----------------------
def run_benchmark(grid_size=50, obstacle_ratio=0.3, total_beam=200, num_beams=5):
    os.makedirs("plots", exist_ok=True)
    grid, start, goal = generate_solvable_instance(grid_size, grid_size, obstacle_ratio)
    print(f"{grid_size}×{grid_size} grid with {int(obstacle_ratio*100)}% obstacles")
    results = []

    # PBS+Short
    pbs = PBSShort(grid)
    t0 = time.time()
    path = pbs.search(start, goal, total_beam=total_beam, num_beams=num_beams)
    t = time.time()-t0
    results.append(("PBS+Short", path, t))

    # BFS
    t0 = time.time()
    path = bfs(grid, start, goal)
    t = time.time()-t0
    results.append(("BFS", path, t))

    # DFS
    t0 = time.time()
    path = dfs(grid, start, goal)
    t = time.time()-t0
    results.append(("DFS", path, t))

    # GBFS
    t0 = time.time()
    path = gbfs(grid, start, goal)
    t = time.time()-t0
    results.append(("GBFS", path, t))

    # A*
    t0 = time.time()
    path = a_star(grid, start, goal)
    t = time.time()-t0
    results.append(("A*", path, t))

    # Print table
    print(f"{'Algorithm':<12} {'PathFound':<10} {'Steps':<6} {'Time(s)':<8}")
    for name, path, t in results:
        print(f"{name:<12} {str(path is not None):<10} {len(path) if path else '-':<6} {t:.4f}")

    # Generate plots
    plot_runtime(results, grid_size)
    plot_path_lengths(results, grid_size)
    plot_pbs_path(grid, results[0][1], start, goal, grid_size)

if __name__ == "__main__":
    run_benchmark(grid_size=50)
    # To showcase PBS+Short advantage on large grids, uncomment:
    # run_benchmark(grid_size=500)
