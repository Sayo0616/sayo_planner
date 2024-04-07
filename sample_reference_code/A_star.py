import heapq

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # cost from start node to current node
        self.h = 0  # heuristic estimate from current node to goal node
        self.f = 0  # total cost

    def __lt__(self, other):
        return self.f < other.f

def heuristic(current, goal):
    # 曼哈顿距离作为启发式函数
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

def astar(grid, start, goal):
    open_list = []
    closed_set = set()

    start_node = Node(start)
    goal_node = Node(goal)

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position == goal_node.position:
            path = []
            while current_node is not None:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # 返回路径，从起点到终点

        closed_set.add(current_node.position)

        for next_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # 上下左右四个方向
            node_position = (current_node.position[0] + next_position[0], current_node.position[1] + next_position[1])

            if node_position[0] < 0 or node_position[0] >= len(grid) or node_position[1] < 0 or node_position[1] >= len(grid[0]):
                continue

            if grid[node_position[0]][node_position[1]] == 1:  # 如果是障碍物
                continue

            if node_position in closed_set:
                continue

            new_node = Node(node_position, current_node)
            new_node.g = current_node.g + 1
            new_node.h = heuristic(new_node.position, goal)
            new_node.f = new_node.g + new_node.h

            # 如果新节点已经在 open_list 中，且其 f 值更高，则跳过
            for open_node in open_list:
                if new_node.position == open_node.position and new_node.f >= open_node.f:
                    break
            else:
                heapq.heappush(open_list, new_node)

    return None  # 如果找不到路径，则返回 None

# 示例用法
grid = [[0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 1, 1, 0],
        [0, 0, 0, 0, 0]]

start = (0, 0)
goal = (4, 4)

path = astar(grid, start, goal)
if path:
    print("Found path:", path)
else:
    print("No path found.")
