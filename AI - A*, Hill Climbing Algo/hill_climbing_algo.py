from pyamaze import maze, agent, textLabel, COLOR
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


def hillClimbing(m):
    start = (4, 1)
    goal = (4, 7)
    current = start
    path = [current]
    total_path_cost = 0
    visited = set()
    i = 0
    nodes_expanded = 0

    start_time = time.time()

    print("Start State:", current)
    while current != goal:
        nodes_expanded += 1
        print(f"Iteration: ", i)
        visited.add(current)
        neighbors = []
        for d in 'ESNW':
            if m.maze_map[current][d]:
                neighbor = getNeighbor(current, d)
                cost = 3  # Base transition cost
                if d == 'E':
                    cost += 5  # Additional cost for moving east (sunlight penalty)
                total_cost = cost + heuristic(neighbor, goal)  # Total cost of reaching the neighbor
                neighbors.append((neighbor, total_cost, cost, d))

        print("Current:", current, "Neighbors:", neighbors)
        neighbors.sort(key=lambda x: x[1])  # Sort neighbors based on total cost
        print("Sorted Neighbors:", neighbors)
        for neighbor in neighbors:
            if neighbor[0] in visited:
                neighbors.remove(neighbor)
                break  # Exit the loop after removing the neighbor
        print("Sorted Neighbors after removing duplicates since backtracking not allowed:", neighbors)

        if len(neighbors) == 0:
            print("No valid neighbors. Stuck at:", current)
            break

        next_neighbor, total_cost, path_cost, direction = neighbors[0]

        current = next_neighbor
        path.append(current)
        total_path_cost += path_cost  # Update path cost with the correct cost value, including the penalty for moving east
        i = i+1
        print("Next State:", current, "Path Cost:", total_path_cost, "Total Cost:", total_cost, "Checked Neighbor:",
              direction)
        print("\n")

    end_time = time.time()
    time_taken = end_time - start_time

    return path, total_path_cost, nodes_expanded, time_taken


if __name__ == '__main__':
    m = maze(6, 7)
    m.CreateMaze(loadMaze='Assignment_5.csv', x=4, y=7, theme=COLOR.light)
    hpath, htotal_path_cost, hill_climbing_nodes_expanded, hill_climbing_time_taken = hillClimbing(m)

    a1 = agent(m, 4, 1, footprints=True, color=COLOR.blue, filled=True, shape='arrow')
    m.tracePath({a1: hpath})

    print("\nHill Climbing Algorithm:")
    print("Path:", hpath)
    print("Path Cost (excluding Heuristics):", htotal_path_cost)
    print("Number of Nodes Expanded:", hill_climbing_nodes_expanded)
    print("Time Taken:", hill_climbing_time_taken)

    l = textLabel(m, 'Hill Climbing Total Path Cost', htotal_path_cost)

    m.run()
