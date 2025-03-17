import heapq
import time
import tracemalloc

DIRECTIONS = {"u": (-1, 0), "d": (1, 0), "l": (0, -1), "r": (0, 1)}


class Sokoban:
    def __init__(self, grid):
        self.grid = [list(row) for row in grid]
        self.height = len(grid)
        print(self.height)
        self.width = len(grid[0])
        print(self.width)
        self.agent, self.boxes, self.targets = self.find_positions()

    def find_positions(self):
        agent = None
        boxes = set()
        targets = set()

        for r in range(self.height):
            for c in range(self.width):
                if self.grid[r][c] == "@":
                    agent = (r, c)
                elif self.grid[r][c] == "+":
                    agent = (r, c)
                    targets.add((r, c))
                elif self.grid[r][c] == "$":
                    boxes.add((r, c))
                elif self.grid[r][c] == "*":
                    boxes.add((r, c))
                    targets.add((r, c))
                elif self.grid[r][c] == ".":
                    targets.add((r, c))
        print(boxes)
        print(agent)
        print(targets)
        return agent, frozenset(boxes), frozenset(targets)

    def is_goal(self, boxes):
        return boxes == self.targets

    def heuristic(self, boxes):
        return sum(
            min(abs(bx - tx) + abs(by - ty) for tx, ty in self.targets)
            for bx, by in boxes
        )

    def get_neighbors(self, state):
        agent, boxes = state
        neighbors = []

        for action, (dr, dc) in DIRECTIONS.items():
            new_agent = (agent[0] + dr, agent[1] + dc)

            if self.grid[new_agent[0]][new_agent[1]] == "#":
                continue

            new_boxes = set(boxes)
            move_type = action

            if new_agent in boxes:
                new_box = (new_agent[0] + dr, new_agent[1] + dc)
                if new_box in boxes or self.grid[new_box[0]][new_box[1]] == "#":
                    continue
                new_boxes.remove(new_agent)
                new_boxes.add(new_box)
                move_type = action.upper()

            neighbors.append((move_type, (new_agent, frozenset(new_boxes))))

        return neighbors

    def solve(self):
        start_time = time.time()
        tracemalloc.start()

        start_state = (self.agent, self.boxes)
        frontier = [(self.heuristic(self.boxes), 0, start_state, [])]
        visited = set()
        nodes_explored = 0

        while frontier:
            _, cost, (agent, boxes), path = heapq.heappop(frontier)
            nodes_explored += 1

            if self.is_goal(boxes):
                end_time = time.time()
                current, peak = tracemalloc.get_traced_memory()
                tracemalloc.stop()

                steps = len(path)
                weight = cost
                time_taken = (end_time - start_time) * 1000
                memory_used = peak / (1024 * 1024)

                print(f"A* search")
                print(
                    f"Steps: {steps}, Weight: {weight}, Node: {nodes_explored}, Time (ms): {time_taken:.2f}, Memory (MB): {memory_used:.2f}"
                )
                print("Solution:", "".join(path))

                return path

            if (agent, boxes) in visited:
                continue
            visited.add((agent, boxes))

            for action, next_state in self.get_neighbors((agent, boxes)):
                new_cost = cost + 1
                heapq.heappush(
                    frontier,
                    (
                        new_cost + self.heuristic(next_state[1]),
                        new_cost,
                        next_state,
                        path + [action],
                    ),
                )

        tracemalloc.stop()
        print("⛔ Không tìm thấy giải pháp!")
        return None


# 🛠 Dữ liệu bài toán
grid = [
    "#######",
    "#  ####",
    "#    ##",
    "# $  ##",
    "### ###",
    "# $ $ #",
    "#..@..#",
    "#  $  #",
    "###  ##",
    "#######",
]

# 🚀 Giải bài toán
game = Sokoban(grid)
solution = game.solve()


import heapq
import time
import tracemalloc

DIRECTIONS = {"u": (-1, 0), "d": (1, 0), "l": (0, -1), "r": (0, 1)}


class SokobanUCS:
    def __init__(self, grid):
        self.grid = [list(row) for row in grid]
        self.height = len(grid)
        self.width = len(grid[0])
        self.agent, self.boxes, self.targets = self.find_positions()

    def find_positions(self):
        agent = None
        boxes = set()
        targets = set()

        for r in range(self.height):
            for c in range(self.width):
                if self.grid[r][c] == "@":
                    agent = (r, c)
                elif self.grid[r][c] == "+":
                    agent = (r, c)
                    targets.add((r, c))
                elif self.grid[r][c] == "$":
                    boxes.add((r, c))
                elif self.grid[r][c] == "*":
                    boxes.add((r, c))
                    targets.add((r, c))
                elif self.grid[r][c] == ".":
                    targets.add((r, c))

        return agent, frozenset(boxes), frozenset(targets)

    def is_goal(self, boxes):
        return boxes == self.targets

    def get_neighbors(self, state):
        agent, boxes = state
        neighbors = []

        for action, (dr, dc) in DIRECTIONS.items():
            new_agent = (agent[0] + dr, agent[1] + dc)

            if self.grid[new_agent[0]][new_agent[1]] == "#":
                continue  # Không thể đi vào tường

            new_boxes = set(boxes)
            move_type = action

            if new_agent in boxes:  # Nếu đẩy hộp
                new_box = (new_agent[0] + dr, new_agent[1] + dc)
                if new_box in boxes or self.grid[new_box[0]][new_box[1]] == "#":
                    continue  # Không thể đẩy vào hộp khác hoặc tường
                new_boxes.remove(new_agent)
                new_boxes.add(new_box)
                move_type = action.upper()  # Đánh dấu là đẩy hộp

            neighbors.append((move_type, (new_agent, frozenset(new_boxes))))

        return neighbors

    def solve(self):
        start_time = time.time()
        tracemalloc.start()

        start_state = (self.agent, self.boxes)
        frontier = [(0, start_state, [])]  # Priority queue according to cost
        visited = set()
        nodes_explored = 0

        while frontier:
            cost, (agent, boxes), path = heapq.heappop(frontier)
            nodes_explored += 1

            if self.is_goal(boxes):
                end_time = time.time()
                current, peak = tracemalloc.get_traced_memory()
                tracemalloc.stop()

                steps = len(path)
                time_taken = (end_time - start_time) * 1000
                memory_used = peak / (1024 * 1024)

                print(f"Uniform Cost Search (UCS)")
                print(
                    f"Steps: {steps}, Cost: {cost}, Nodes: {nodes_explored}, Time (ms): {time_taken:.2f}, Memory (MB): {memory_used:.2f}"
                )
                print("Solution:", "".join(path))

                return path

            if (agent, boxes) in visited:
                continue
            visited.add((agent, boxes))

            for action, next_state in self.get_neighbors((agent, boxes)):
                new_cost = cost + 1
                heapq.heappush(frontier, (new_cost, next_state, path + [action]))

        tracemalloc.stop()
        print("⛔ Không tìm thấy giải pháp!")
        return None


# 🛠 Dữ liệu bài toán
grid = [
    "#######",
    "#  ####",
    "#    ##",
    "# $  ##",
    "### ###",
    "# $ $ #",
    "#..@..#",
    "#  $  #",
    "###  ##",
    "#######",
]

# 🚀 Giải bài toán với UCS
game = SokobanUCS(grid)
solution = game.solve()

from collections import deque
import time
import tracemalloc

DIRECTIONS = {"u": (-1, 0), "d": (1, 0), "l": (0, -1), "r": (0, 1)}


class Sokoban:
    def __init__(self, grid):
        self.grid = [list(row) for row in grid]
        self.height = len(grid)
        self.width = len(grid[0])
        self.agent, self.boxes, self.targets = self.find_positions()

    def find_positions(self):
        agent = None
        boxes = set()
        targets = set()

        for r in range(self.height):
            for c in range(self.width):
                if self.grid[r][c] == "@":
                    agent = (r, c)
                elif self.grid[r][c] == "+":
                    agent = (r, c)
                    targets.add((r, c))
                elif self.grid[r][c] == "$":
                    boxes.add((r, c))
                elif self.grid[r][c] == "*":
                    boxes.add((r, c))
                    targets.add((r, c))
                elif self.grid[r][c] == ".":
                    targets.add((r, c))

        return agent, frozenset(boxes), frozenset(targets)

    def is_goal(self, boxes):
        return boxes == self.targets

    def get_neighbors(self, state):
        agent, boxes = state
        neighbors = []

        for action, (dr, dc) in DIRECTIONS.items():
            new_agent = (agent[0] + dr, agent[1] + dc)

            if self.grid[new_agent[0]][new_agent[1]] == "#":
                continue

            new_boxes = set(boxes)
            move_type = action

            if new_agent in boxes:
                new_box = (new_agent[0] + dr, new_agent[1] + dc)
                if new_box in boxes or self.grid[new_box[0]][new_box[1]] == "#":
                    continue
                new_boxes.remove(new_agent)
                new_boxes.add(new_box)
                move_type = action.upper()

            neighbors.append((move_type, (new_agent, frozenset(new_boxes))))

        return neighbors

    def solve_bfs(self):
        start_time = time.time()
        tracemalloc.start()

        start_state = (self.agent, self.boxes)
        frontier = deque([(start_state, [])])
        visited = set()
        nodes_explored = 0

        while frontier:
            (agent, boxes), path = frontier.popleft()
            nodes_explored += 1

            if self.is_goal(boxes):
                end_time = time.time()
                current, peak = tracemalloc.get_traced_memory()
                tracemalloc.stop()

                steps = len(path)
                time_taken = (end_time - start_time) * 1000
                memory_used = peak / (1024 * 1024)

                print(f"BFS search")
                print(
                    f"Steps: {steps}, Node: {nodes_explored}, Time (ms): {time_taken:.2f}, Memory (MB): {memory_used:.2f}"
                )
                print("Solution:", "".join(path))

                return path

            if (agent, boxes) in visited:
                continue
            visited.add((agent, boxes))

            for action, next_state in self.get_neighbors((agent, boxes)):
                frontier.append((next_state, path + [action]))

        tracemalloc.stop()
        print("⛔ Không tìm thấy giải pháp!")
        return None


# 🛠 Dữ liệu bài toán
grid = [
    "#######",
    "#  ####",
    "#    ##",
    "# $  ##",
    "### ###",
    "# $ $ #",
    "#..@..#",
    "#  $  #",
    "###  ##",
    "#######",
]

# 🚀 Giải bài toán bằng BFS
game = Sokoban(grid)
solution = game.solve_bfs()

import time
import tracemalloc

DIRECTIONS = {"u": (-1, 0), "d": (1, 0), "l": (0, -1), "r": (0, 1)}


class Sokoban:
    def __init__(self, grid):
        self.grid = [list(row) for row in grid]
        self.height = len(grid)
        self.width = len(grid[0])
        self.agent, self.boxes, self.targets = self.find_positions()

    def find_positions(self):
        agent = None
        boxes = set()
        targets = set()

        for r in range(self.height):
            for c in range(self.width):
                if self.grid[r][c] == "@":
                    agent = (r, c)
                elif self.grid[r][c] == "+":
                    agent = (r, c)
                    targets.add((r, c))
                elif self.grid[r][c] == "$":
                    boxes.add((r, c))
                elif self.grid[r][c] == "*":
                    boxes.add((r, c))
                    targets.add((r, c))
                elif self.grid[r][c] == ".":
                    targets.add((r, c))

        return agent, frozenset(boxes), frozenset(targets)

    def is_goal(self, boxes):
        return boxes == self.targets

    def get_neighbors(self, state):
        agent, boxes = state
        neighbors = []

        for action, (dr, dc) in DIRECTIONS.items():
            new_agent = (agent[0] + dr, agent[1] + dc)

            if self.grid[new_agent[0]][new_agent[1]] == "#":
                continue

            new_boxes = set(boxes)
            move_type = action

            if new_agent in boxes:
                new_box = (new_agent[0] + dr, new_agent[1] + dc)
                if new_box in boxes or self.grid[new_box[0]][new_box[1]] == "#":
                    continue
                new_boxes.remove(new_agent)
                new_boxes.add(new_box)
                move_type = action.upper()

            neighbors.append((move_type, (new_agent, frozenset(new_boxes))))

        return neighbors

    def solve_dfs(self):
        start_time = time.time()
        tracemalloc.start()

        start_state = (self.agent, self.boxes)
        stack = [(start_state, [])]  # Stack for DFS
        visited = set()
        nodes_explored = 0

        while stack:
            (agent, boxes), path = stack.pop()
            nodes_explored += 1

            if self.is_goal(boxes):
                end_time = time.time()
                current, peak = tracemalloc.get_traced_memory()
                tracemalloc.stop()

                steps = len(path)
                time_taken = (end_time - start_time) * 1000
                memory_used = peak / (1024 * 1024)

                print(f"DFS search")
                print(
                    f"Steps: {steps}, Node: {nodes_explored}, Time (ms): {time_taken:.2f}, Memory (MB): {memory_used:.2f}"
                )
                print("Solution:", "".join(path))

                return path

            if (agent, boxes) in visited:
                continue
            visited.add((agent, boxes))

            for action, next_state in self.get_neighbors((agent, boxes)):
                stack.append((next_state, path + [action]))  # Push in stack

        tracemalloc.stop()
        print("⛔ Không tìm thấy giải pháp!")
        return None


# 🛠 Dữ liệu bài toán
grid = [
    "#######",
    "#  ####",
    "#    ##",
    "# $  ##",
    "### ###",
    "# $ $ #",
    "#..@..#",
    "#  $  #",
    "###  ##",
    "#######",
]

# 🚀 Giải bài toán bằng DFS
game = Sokoban(grid)
solution = game.solve_dfs()

import heapq
import time
import tracemalloc

DIRECTIONS = {"u": (-1, 0), "d": (1, 0), "l": (0, -1), "r": (0, 1)}


class Sokoban:
    def __init__(self, grid):
        self.grid = [list(row) for row in grid]
        self.height = len(grid)
        self.width = len(grid[0])
        self.agent, self.boxes, self.targets = self.find_positions()

    def find_positions(self):
        agent = None
        boxes = set()
        targets = set()

        for r in range(self.height):
            for c in range(self.width):
                if self.grid[r][c] == "@":
                    agent = (r, c)
                elif self.grid[r][c] == "+":
                    agent = (r, c)
                    targets.add((r, c))
                elif self.grid[r][c] == "$":
                    boxes.add((r, c))
                elif self.grid[r][c] == "*":
                    boxes.add((r, c))
                    targets.add((r, c))
                elif self.grid[r][c] == ".":
                    targets.add((r, c))

        return agent, frozenset(boxes), frozenset(targets)

    def is_goal(self, boxes):
        return boxes == self.targets

    def heuristic(self, boxes):
        """Function calculate sum of distance's Manhattan from boxes to target"""
        return sum(
            min(abs(bx - tx) + abs(by - ty) for tx, ty in self.targets)
            for bx, by in boxes
        )

    def get_neighbors(self, state):
        agent, boxes = state
        neighbors = []

        for action, (dr, dc) in DIRECTIONS.items():
            new_agent = (agent[0] + dr, agent[1] + dc)

            if self.grid[new_agent[0]][new_agent[1]] == "#":
                continue

            new_boxes = set(boxes)
            move_type = action

            if new_agent in boxes:
                new_box = (new_agent[0] + dr, new_agent[1] + dc)
                if new_box in boxes or self.grid[new_box[0]][new_box[1]] == "#":
                    continue
                new_boxes.remove(new_agent)
                new_boxes.add(new_box)
                move_type = action.upper()

            neighbors.append((move_type, (new_agent, frozenset(new_boxes))))

        return neighbors

    def solve_gbfs(self):
        start_time = time.time()
        tracemalloc.start()

        start_state = (self.agent, self.boxes)
        frontier = [(self.heuristic(self.boxes), start_state, [])]  # Only use h(n)
        visited = set()
        nodes_explored = 0

        while frontier:
            _, (agent, boxes), path = heapq.heappop(frontier)
            nodes_explored += 1

            if self.is_goal(boxes):
                end_time = time.time()
                current, peak = tracemalloc.get_traced_memory()
                tracemalloc.stop()

                steps = len(path)
                time_taken = (end_time - start_time) * 1000
                memory_used = peak / (1024 * 1024)

                print(f"Greedy Best-First Search (GBFS)")
                print(
                    f"Steps: {steps}, Node: {nodes_explored}, Time (ms): {time_taken:.2f}, Memory (MB): {memory_used:.2f}"
                )
                print("Solution:", "".join(path))

                return path

            if (agent, boxes) in visited:
                continue
            visited.add((agent, boxes))

            for action, next_state in self.get_neighbors((agent, boxes)):
                heapq.heappush(
                    frontier,
                    (
                        self.heuristic(next_state[1]),
                        next_state,
                        path + [action],
                    ),  
                )

        tracemalloc.stop()
        print("⛔ Không tìm thấy giải pháp!")
        return None


# 🛠 Dữ liệu bài toán
grid = [
    "#######",
    "#  ####",
    "#    ##",
    "# $  ##",
    "### ###",
    "# $ $ #",
    "#..@..#",
    "#  $  #",
    "###  ##",
    "#######",
]

# 🚀 Giải bài toán bằng GBFS
game = Sokoban(grid)
solution = game.solve_gbfs()