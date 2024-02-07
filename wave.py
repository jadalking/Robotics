from map import *

POSITION = [0, 0, DIRECTION.South]
I, J, K = 0, 1, 2

# Given a map and cell, finds neighboring cells that are not blocked and have not been visited
# Also sets their costs to 1 + curr cell's cost.
# Returns the list of valid neighbors
def find_neighbors(a_map, cell):
    neighbors = []
    i, j = cell[I], cell[J]
    cost = a_map.getCost(i, j)

    if a_map.getNeighborObstacle(i, j, DIRECTION.North) == 0 and a_map.getNeighborCost(i, j, DIRECTION.North) == 0:
        north_neighbor = [i - 1, j, DIRECTION.North]
        a_map.setNeighborCost(i, j, DIRECTION.North, cost + 1)
        neighbors.append(north_neighbor)

    if a_map.getNeighborObstacle(i, j, DIRECTION.South) == 0 and a_map.getNeighborCost(i, j, DIRECTION.South) == 0:
        south_neighbor = [i + 1, j, DIRECTION.South]
        a_map.setNeighborCost(i, j, DIRECTION.South, cost + 1)
        neighbors.append(south_neighbor)

    if a_map.getNeighborObstacle(i, j, DIRECTION.East) == 0 and a_map.getNeighborCost(i, j, DIRECTION.East) == 0:
        east_neighbor = [i, j + 1, DIRECTION.East]
        a_map.setNeighborCost(i, j, DIRECTION.East, cost + 1)
        neighbors.append(east_neighbor)

    if a_map.getNeighborObstacle(i, j, DIRECTION.West) == 0 and a_map.getNeighborCost(i, j, DIRECTION.West) == 0:
        west_neighbor = [i, j - 1, DIRECTION.West]
        a_map.setNeighborCost(i, j, DIRECTION.West, cost + 1)
        neighbors.append(west_neighbor)

    return neighbors

def generate_path(a_map, start, goal):
    path = [goal]
    i, j = goal[I], goal[J]
    cost = a_map.getCost(i, j)

    while True:
        if a_map.getNeighborObstacle(i, j, DIRECTION.North) == 0 and a_map.getNeighborCost(i, j, DIRECTION.North) == cost - 1:
            next_step = [i - 1, j, DIRECTION.North]
        elif a_map.getNeighborObstacle(i, j, DIRECTION.South) == 0 and a_map.getNeighborCost(i, j, DIRECTION.South) == cost - 1:
            next_step = [i + 1, j, DIRECTION.South]
        elif a_map.getNeighborObstacle(i, j, DIRECTION.East) == 0 and a_map.getNeighborCost(i, j, DIRECTION.East) == cost - 1:
            next_step = [i, j + 1, DIRECTION.East]
        elif a_map.getNeighborObstacle(i, j, DIRECTION.West) == 0 and a_map.getNeighborCost(i, j, DIRECTION.West) == cost - 1:
            next_step = [i, j - 1, DIRECTION.West]
        else:
            print("generate_path(): could not find next step")
            return

        i, j = next_step[I], next_step[J]
        cost = a_map.getCost(i, j)

        # avoid adding start to path; we know we're done backtracking if the next step is start
        if i == start[I] and j == start[J]:
            break

        # add next step to front of list to maintain order
        path.insert(0, next_step)

    return path



# Main algorithm for filling out cost map, returns a path
def wavefront(a_map, start, goal):
    queue = [start]  # list of cells that need to be visited
    a_map.setCost(start[I], start[J], 1)  # have to set origin cost to 1 bc 0 is used for unvisited cells

    while queue:
        cell = queue.pop(0)

        # if at goal, stop and backtrack
        if cell[I] == goal[I] and cell[J] == goal[J]:
            path = generate_path(a_map, start, goal)

        # otherwise get neighbors, set their costs, and add them to list of cells to visit
        else:
            neighbors = find_neighbors(a_map, cell)
            queue += neighbors

    return path


# Main function
if __name__ == "__main__":
    starter_map = EECSMap()
    starter_map.printObstacleMap()

    # Basic testing with some (start, goal) pairs
    # upper left corner to bottom right corner
    start, goal = POSITION, [7, 7, DIRECTION.South]

    # one step east
    # start, goal = POSITION, [0, 1, DIRECTION.South]

    # three steps south
    # start, goal = POSITION, [3, 0, DIRECTION.South]

    path = wavefront(starter_map, start, goal)

    print
    starter_map.printCostMap()

    print "\nPath to get from " + str(start) + " to " + str(goal) + ": "
    for step in path:
        print step