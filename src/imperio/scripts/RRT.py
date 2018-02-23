""" Path planning using RRT with options for different types of path smoothing

Author: James Madison University
Date: 2/11/2018
Version: 1
"""

import matplotlib.pyplot as plt
import scipy.interpolate as si
import numpy as np
import random
import math
import copy

halt_for_visualization = False

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, map, randAreaX, randAreaY):
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
        self.expandDis = 1.0
        self.goalSampleRate = 5
        self.maxIter = 500
        self.map = map

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

            if not collision_check(newNode, self.map):
                continue

            self.nodeList.append(newNode)

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
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

class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def get_path_length(path):
    length = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        length += d

    return length


def get_target_point(path, targetL):
    length = 0
    ti = 0
    last_pair_len = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        length += d
        if length >= targetL:
            ti = i - 1
            last_pair_len = d
            break

    part_ratio = (length - targetL) / last_pair_len

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * part_ratio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * part_ratio

    return [x, y, ti]

def collision_check(node, map):
    threshold = .7
    #convert the node to a space in the cell
    row, col = map.cell_index(node.x, node.y)

    #check that space is in bounds of map
    row_max = map.height - 1
    col_max = map.width - 1
    if row > row_max or col > col_max:
        #Out of bounds
        return False

    #check that there isn't anything in that grid space
    #TODO : the current testing settup are inverted occupancy grids, check on ones coming from robot
    if map.grid[row][col] > threshold:
        return True
    else:
        return False


def line_collision_check(first, second, map):
    # Line Equation

    check = .05

    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    total_distance = distance(first, second)
    check_distance = check

    while check_distance < total_distance:
        ratio = check_distance/total_distance

        x_check = ((1 - ratio)*x1 + ratio * x2)
        y_check = ((1 - ratio)*y1 + ratio * y2)

        if collision_check(Node(x_check, y_check), map) == False:
            return False
        check_distance += check

    return True



def path_smoothing(path, maxIter, obstacleList):
    #  print("PathSmoothing")

    le = get_path_length(path)

    for i in range(maxIter):
        #TODO : Never gets from A to B straight, check random for any issues
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

def remove_redundant(path):
    minimal_path = [path[0]]
    for i in range(1, len(path)):
        if path[i] != path[i-1]:
            minimal_path.append(path[i])
    return minimal_path



def path_planning(start, goal, map):
    min_x, min_y = map.cell_position(0,0)
    max_x, max_y = map.cell_position(map.width - 1, map.height - 1)

    rrt = RRT(start, goal, map, [min_x, max_x], [min_y, max_y])
    path = rrt.planning()
    #draw_tree(path, map)

    smooth_path = remove_redundant(path_smoothing(path, 1000, map))
    #draw_tree(smooth_path, map)

    """ Save for further testing
    #Testing the BSpline
    xlist = []
    ylist = []
    for point in path:
        x,y = point
        xlist.append(x)
        ylist.append(y)

    sampling_number = 100
    x = np.array(xlist)
    y = np.array(ylist)
    rx, ry = bspline_path(x, y, sampling_number)

    plt.xlim(min_y, max_y)
    plt.ylim(min_y, max_y)
    plt.plot(rx, ry, 'r', label="B-spline path")
    plt.show()

    comb = []
    for i in range(0, len(rx)):
        comb.append((rx[i], ry[i]))
    draw_tree(comb, map)
    """

    smooth_path.reverse()
    return smooth_path


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

    #to keep things in scale
    plt.xlim(min_y, max_y)
    plt.ylim(min_y, max_y)

    plt.show()

def bspline_path(x,y, sn):
    N = 3
    t = range(len(x))
    x_tup = si.splrep(t, x, k=N)
    y_tup = si.splrep(t, y, k=N)

    x_list = list(x_tup)
    xl = x.tolist()
    x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    yl = y.tolist()
    y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, sn)
    rx = si.splev(ipl_t, x_list)
    ry = si.splev(ipl_t, y_list)

    return rx, ry

def find_best_rrt_path(start, goal, map, num_paths):
    print("Finding best RRT path out of %s paths" % num_paths)
    lowest_path_score = 9999999999
    lowest_path = None
    for i in range(0, num_paths):
        path = path_planning(start, goal, map)
        path_score = calculate_path_score(path)
        if path_score < lowest_path_score:
            lowest_path_score = path_score
            lowest_path = path
        print("{}/{} completed".format(i+1, num_paths))

    return lowest_path

def calculate_path_score(path):
    angular_score = 0
    for i in range(1, len(path) - 1):
        #Using the points a, b, c to find angle from ab to bc
        a = path[i-1]
        b = path[i]
        c = path[i+1]

        ab_dist = distance(a, b)
        ac_dist = distance(a, c)
        bc_dist = distance(b, c)

        #If the line is not a straight line
        if ab_dist + bc_dist - ac_dist > 0.01:
           try:
               angle = math.acos((bc_dist**2 - ac_dist**2 + ab_dist**2)/(2*bc_dist*ab_dist))
           except ValueError:
                print("ab {} bc {} ac {}".format(ab_dist, bc_dist, ac_dist))
        else:
            angle = math.pi

        angular_score += abs(math.pi - angle) ** 2
    angular_score = angular_score/len(path)
    point_num_score = len(path)
    return angular_score + point_num_score

def distance(point_a, point_b):
    x1, y1 = point_a[0], point_a[1]
    x2, y2 = point_b[0], point_b[1]
    return abs(math.sqrt((x2-x1)**2 + (y2-y1)**2))







