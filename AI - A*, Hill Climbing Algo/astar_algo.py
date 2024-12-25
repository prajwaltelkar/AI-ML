from pyamaze import maze, agent, textLabel, COLOR
from queue import PriorityQueue
import math
import time


def heuristic(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def getNeighbor(cell, direction):
    x, y = cell
    if direction == 'E':
        return (x, y + 1)
    elif direction == 'W':
        return (x, y - 1)
    elif direction == 'N':
        return (x - 1, y)
    elif direction == 'S':
        return (x + 1, y)
    else:
        raise ValueError("Invalid direction")


def aStar(m):
    start = (4, 1)
    goal = (4, 7)
    open = PriorityQueue()
    open.put((0, start))
    came_from = {}
    cost_so_far = {start: 0}
    nodes_expanded = 0

    start_time = time.time()

    while not open.empty():
        nodes_expanded += 1
        current_cost, current = open.get()
        print("Current cell:", current)
        if current == goal:
            break

        for d in 'ESNW':
            if m.maze_map[current][d]:
                if d == 'E':
                    new_cost = cost_so_far[current] + 3 + 5  # Cost for moving east
                else:
                    new_cost = cost_so_far[current] + 3

                neighbor = getNeighbor(current, d)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    open.put((priority, neighbor))
                    came_from[neighbor] = current
                    print("    ->", d, "Neighbor:", neighbor, "New cost:", new_cost, "Priority:", priority)

    path_cost = cost_so_far[goal]  # Total cost of the path

    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    end_time = time.time()
    time_taken = end_time - start_time
    return path, path_cost, nodes_expanded, time_taken


if __name__ == '__main__':
    m = maze(6, 7)
    m.CreateMaze(loadMaze='Assignment_5.csv', x=4, y=7, theme=COLOR.light)
    apath, apath_cost, a_star_nodes_expanded, a_star_time_taken = aStar(m)

    a1 = agent(m, 4, 1, footprints=True, color=COLOR.blue, filled=True, shape='arrow')
    m.tracePath({a1: apath})

    print("\nA* Algorithm:")
    print("Path:", apath)
    print("Path Cost (excluding Heuristics):", apath_cost)
    print("Number of Nodes Expanded:", a_star_nodes_expanded)
    print("Time Taken:", a_star_time_taken)

    label = textLabel(m, 'A Star Algorithm: Path Cost', apath_cost)

    m.run()
