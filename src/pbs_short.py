# pbs_short.py
# Polynomial Beam Search + Short (PBS+Short)
# Author: Arka Brahma - Class 10 student (age 15-16)
# GitHub: arkakly128-hub/PBS-Short

from __future__ import annotations
from typing import List, Tuple, Optional, Callable
import heapq

Coordinate = Tuple[int, int]

def manhattan(a: Coordinate, b: Coordinate) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

class PBSShort:
    def __init__(self, grid: List[List[int]], heuristic: Callable = manhattan):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.heuristic = heuristic

    def is_valid(self, pos: Coordinate) -> bool:
        x, y = pos
        return 0 <= x < self.rows and 0 <= y < self.cols and self.grid[x][y] == 0

    def get_neighbors(self, pos: Coordinate) -> List[Coordinate]:
        x, y = pos
        candidates = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        return [p for p in candidates if self.is_valid(p)]

    def polynomial_split(self, total_beam: int, num_beams: int) -> List[int]:
        """Split total_beam into num_beams parts: 1², 2², 3², ... scaled to sum to total_beam"""
        if num_beams <= 0:
            return []
        squares = [i*i for i in range(1, num_beams + 1)]
        total = sum(squares)
        factor = total_beam / total
        sizes = [int(s * factor) for s in squares]
        # Fix rounding errors
        current = sum(sizes)
        for i in range(total_beam - current):
            sizes[i % num_beams] += 1
        return [max(1, s) for s in sizes]  # at least 1 per beam

    def search(self,
               start: Coordinate,
               goal: Coordinate,
               total_beam: int = 200,
               num_beams: int = 5) -> Optional[List[Coordinate]]:

        beam_sizes = self.polynomial_split(total_beam, num_beams)

        # Each beam item: (score, path_length, path)
        beam_list = [
            [(-self.heuristic(start, goal), 0, [start])]   # negative for min-heap → max-heap behavior
            for _ in range(num_beams)
        ]

        while True:
            next_beams: List[List[Tuple[int, int, List[Coordinate]]]] = [[] for _ in beam_list]
            goal_paths = []

            for i in range(num_beams):
                candidates = []
                current_beam = beam_list[i]

                for _, plen, path in current_beam:
                    node = path[-1]
                    if node == goal:
                        goal_paths.append(path)
                        continue

                    for neighbor in self.get_neighbors(node):
                        if neighbor in path:
                            continue
                        new_path = path + [neighbor]
                        new_plen = len(new_path)
                        score = new_plen + self.heuristic(neighbor, goal)
                        heapq.heappush(candidates, (score, new_plen, new_path))

                # Keep top-k for this sub-beam
                k = beam_sizes[i]
                while len(candidates) > k and candidates:
                    next_beams[i].append(heapq.heappop(candidates))
                while candidates:
                    next_beams[i].append(heapq.heappop(candidates))

            if goal_paths:
                # Return shortest among all found
                return min(goal_paths, key=len)

            # Check if all beams died
            if all(len(b) == 0 for b in next_beams):
                return None

            beam_list = next_beams

if __name__ == "__main__":
    # Demo usage
    grid = [[0]*10 for _ in range(10)]
    pbs = PBSShort(grid)
    path = pbs.search((0,0), (9,9))
    print("Demo path:", path)
