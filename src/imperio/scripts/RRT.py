"""
Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)
"""

import matplotlib.pyplot as plt
import random
import math
import copy

halt_for_visualization = True

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList,
                 randAreaX, randAreaY, expandDis=1.0, goalSampleRate=5, maxIter=500):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size]]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand_x, self.maxrand_x = randAreaX
        self.minrand_y, self.maxrand_y = randAreaY
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def planning(self):
        """
        Pathplanning
        """

        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand_x, self.maxrand_x), random.uniform(
                    self.minrand_y, self.maxrand_y)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.get_nearest_list_index(self.nodeList, rnd)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind

            if not self.collision_check(newNode, self.obstacleList):
                continue

            self.nodeList.append(newNode)

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def get_nearest_list_index(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def collision_check(self, node, map):
        threshold = .7
        #convert the node to a space in the cell
        row, col = map.cell_index(node.x, node.y)
        #There was an error, this point shouldn't be possible
        row_max = map.height - 1
        col_max = map.width - 1
        if row > row_max or col > col_max:
            print(row, col)
            print("out of bounds")
            return False

        #check that there isn't anything in the grid
        #TODO : the current testing settup are inverted occupancy grids
        if map.grid[row][col] > threshold:
            #print("OBSTACLE")
            #print(node.x, node.y)
            return True
        else:
            print("Not Clear")
            return False

        return True

    #TODO : make this work with obstacles
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def get_path_length(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d

    return le


def get_target_point(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    partRatio = (le - targetL) / lastPairLen
    #  print(partRatio)
    #  print((ti,len(path),path[ti],path[ti+1]))

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio
    #  print((x,y))

    return [x, y, ti]


def line_collision_check(first, second, obstacleList):
    # Line Equation
    #TODO : Make this work with obstacles
    return True

    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    try:
        a = y2 - y1
        b = -(x2 - x1)
        c = y2 * (x2 - x1) - x2 * (y2 - y1)
    except ZeroDivisionError:
        return False

    for (ox, oy, size) in obstacleList:
        d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
        if d <= (size):
            return False

    #  print("OK")

    return True  # OK

def path_smoothing(path, maxIter, obstacleList):
    #  print("PathSmoothing")

    le = get_path_length(path)

    for i in range(maxIter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        #  print(pickPoints)
        first = get_target_point(path, pickPoints[0])
        #  print(first)
        second = get_target_point(path, pickPoints[1])
        #  print(second)

        if first[2] <= 0 or second[2] <= 0:
            continue

        if (second[2] + 1) > len(path):
            continue

        if second[2] == first[2]:
            continue

        # collision check
        if not line_collision_check(first, second, obstacleList):
            continue

        # Create New path
        newPath = []
        newPath.extend(path[:first[2] + 1])
        newPath.append([first[0], first[1]])
        newPath.append([second[0], second[1]])
        newPath.extend(path[second[2] + 1:])
        path = newPath
        le = get_path_length(path)

    return path



def path_planning(start, goal, map):
    print("Start RRT path planning")
    #TODO : Add the obstacles in

    min_x, min_y = map.cell_position(0,0)
    max_x, max_y = map.cell_position(map.width - 1, map.height - 1)

    obstacleList = map
    # Set Initial parameters
    area_x, area_y = map.cell_position(map.width, map.height)
    rrt = RRT(start=start, goal=goal,
              randAreaX=[min_x, max_x], randAreaY=[min_y, max_y], obstacleList=obstacleList)
    path = rrt.planning()
    draw_tree(path, map)
    smooth_path = path_smoothing(path, 1000, obstacleList)
    #draw_tree(smooth_path, map)

    #TODO : Add path smoothing in with obstacles
    #smooth_path.reverse()
    #return smooth_path

    path.reverse()
    return path

   #TODO : CAN BE REMOVED, ONLY FOR TESTING/DEBUGGING
def draw_tree(waypoints, map):
    if halt_for_visualization == False:
        return
    for x in range(1, len(waypoints)):
        x1, y1 = waypoints[x - 1]
        x2, y2 = waypoints[x]
        plt.plot([x1, x2], [y1, y2])

    # configure plot axises
    min_x, min_y = map.cell_position(0, 0)
    max_x, max_y = map.cell_position(map.width - 1, map.height - 1)

    '''obstacle_grid = map
    for col in range(0, obstacle_grid.width - 1):
        for row in range(0, obstacle_grid.height - 1):
            if obstacle_grid.grid[row][col] < .7:
                x, y = obstacle_grid.cell_position(row, col)
                plt.plot(x, y)
                '''

    #to keep things in scale
    plt.xlim(min_y, max_y)
    plt.ylim(min_y, max_y)

    plt.show()
