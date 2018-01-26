import heapq


class PathNode:
    """
    state: A tuple of XY Coordinates
    parent: PathNode (if null start state)
    path_cost: Number
    action_to: Currently the same as state
    """

    def __init__(self, state, parent, step_cost, action_to):
        self.state = state
        self.parent = parent
        if parent is None:
            self.path_cost = 0
        else:
            self.path_cost = parent.path_cost + step_cost
        self.action_to = action_to

    def getPathCost(self):
        return self.path_cost

    def returnPath(self):
        return_path = []
        pnode = self.parent
        return_path.append(self.action_to)
        while pnode is not None and pnode.action_to is not None:
            return_path.insert(0, pnode.action_to)
            pnode = pnode.parent
            print

        return return_path

    def getNeighbors(self, width, height):
        succ = []
        size = 1
        if (self.state[0] - size >= 0):
            succ.append((self.state[0] - size, self.state[1]))
        if (self.state[0] + size < height):
            succ.append((self.state[0] + size, self.state[1]))
        if (self.state[1] - size >= 0):
            succ.append((self.state[0], self.state[1] - size))
        if (self.state[1] + size < width):
            succ.append((self.state[0], self.state[1] + size))
        return succ

    def __eq__(self, other):
        return self.state == other.state


class PriorityQueue:
    """
      Implements a priority queue data structure. Each inserted item
      has a priority associated with it and the client is usually interested
      in quick retrieval of the lowest-priority item in the queue. This
      data structure allows O(1) access to the lowest-priority item.
    """

    def __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def length(self):
        return len(self.heap)

    def contains(self, item):
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                return True
        return False

    def update(self, item, priority):
        # If item already in priority queue with higher priority, update its priority and rebuild the heap.
        # If item already in priority queue with equal or lower priority, do nothing.
        # If item not in priority queue, do the same thing as self.push.
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
            else:
                self.push(item, priority)


def heuristic(current, goal):
    """
    Heuristic to be Used by the AStar Algorithm.
    """
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])  # Manhattan Distance

def aStar_xy(start_xy, end_xy, grid):
    x,y = start_xy
    start = grid.cell_index(x,y)
    x,y = end_xy
    end = grid.cell_index(x,y)

    cell_results = aStar(start, end, grid)
    return cell_results
    point_results = []
    for cell in cell_results:
        x,y = cell
        point_results.append(grid.cell_position(x,y))
    return point_results

def aStar(start, end, OccupancyGrid):  # Assumes that 0,0 is the bottom left corner
    grid = OccupancyGrid.grid
    if start == end:
        return []
    # Initialize the Goal Node, open Queue, and closedSet
    goal = PathNode(end, None, 0, None)
    openSet = PriorityQueue()
    openSet.push(PathNode(start, None, 0, None), 0)
    closedSet = set()

    # Perform the actual Astar Algorithm
    while not openSet.isEmpty():
        # Get the next node off of the queue and add it to the closed set
        currentNode = openSet.pop()
        closedSet.add(currentNode.state)
        # Check to see if it is the Goal, and if it is return the path to it as points
        if goal == currentNode:
            return currentNode.returnPath()
        # Check each neighbor of the current Node to see if we found it already
        for neighbor in currentNode.getNeighbors(OccupancyGrid.width, OccupancyGrid.height):
            # Get the stepCost and Make a Neighbor Node
            stepCost = grid[neighbor[0]][neighbor[1]]
            neighborNode = PathNode(neighbor, currentNode, stepCost, neighbor)
            # Check to see if the neighbors have already been checked
            # which would mean they have the optimal cost, I think
            if neighbor not in closedSet and not openSet.contains(neighborNode):
                # print neighborNode.getPathCost()
                # print heuristic(neighbor, end)
                openSet.push(neighborNode, neighborNode.getPathCost() + heuristic(neighbor, end))
                # Added this after I got home, so I could not test it with an empty OccGrid
            elif neighbor not in closedSet and openSet.contains(neighborNode):
                openSet.update(neighborNode, neighborNode.getPathCost() + heuristic(neighbor, end))

    return []