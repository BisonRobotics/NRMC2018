""" Path planning using RRT with options for different types of path smoothing

Author: James Madison University
Date: 2/11/2018
Version: 1
"""

import numpy as np
import random
import math
import copy
import operator
import multiprocessing as mp
import rospy

halt_for_visualization = False
use_threading = True

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
        self.expandDis = 0.15
        self.goalSampleRate = 5
        self.maxIter = 500
        self.map = map

        self.start_time = rospy.get_time()

    def planning(self):
        """
        Pathplanning
        """

        self.nodeList = [self.start]
        while True:
            if rospy.get_time() - self.start_time > 20:
                rospy.logwarn("[IMPERIO] : RRT Time Out")
                raise Exception

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
    threshold = .1
    #convert the node to a space in the cell
    row, col = map.cell_index(node.x, node.y)

    #check that space is in bounds of map
    row_max = map.height - 1
    col_max = map.width - 1
    if row > row_max or col > col_max:
        #Out of bounds
        return False

    #check that there isn't anything in that grid space
    if map.grid[row][col] < threshold:
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

    le = get_path_length(path)

    for i in range(maxIter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        first = get_target_point(path, pickPoints[0])
        second = get_target_point(path, pickPoints[1])

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

    goal_x, goal_y = goal
    if goal_x > max_x or goal_x < min_x or goal_y > max_y or goal_y < min_y:
        rospy.loginfo("[IMPERIO] : Goal outside of the costmap determined by occupancyGrid width(x) and height(y)")
        return []

    start_x, start_y = start[0], start[1]
    if start_x > max_x or start_x < min_x or start_y > max_y or goal_y < min_y:
        rospy.loginfo("[IMPERIO] : Robot localized to a point not found on costmap determined by occupancyGrid width(x) and height(y)")
        return []

    rrt = RRT(start, goal, map, [min_x, max_x], [min_y, max_y])
    try:
        path = rrt.planning()
    except Exception:
        raise Exception


    smooth_path = remove_redundant(path_smoothing(path, 1000, map))

    smooth_path.reverse()
    return smooth_path

def find_best_rrt_path(start, goal, map, num_paths):
    rospy.loginfo("[IMPERIO] : Finding best RRT path out of %s paths" % num_paths)

    if use_threading:
        return parallel_paths(start, goal, map, num_paths)

    lowest_path_score = 9999999999
    lowest_path = []

    for i in range(0, num_paths):
        try:
            path = path_planning(start, goal)
        except Exception:
            return []

        path_score = calculate_path_score(path)
        if path_score < lowest_path_score:
            lowest_path_score = path_score
            lowest_path = path

    return lowest_path

def parallel_paths(start, goal, map, num_paths):
    remain = num_paths%4
    #args = []
    #for i in range(0, 4):
    #    paths = num_paths/4 if (remain < 1) else (num_paths/4 + 1)
    #    remain -= 1
    #    arg = [start, goal, map, paths]
    #    args.append(arg)

    arg_0 = [start, goal, map, num_paths/4]
    arg_1 = [start, goal, map, num_paths/4]
    arg_2 = [start, goal, map, num_paths/4]
    arg_3 = [start, goal, map, num_paths/4]


    #Signaling number to the decoder of the shared states
    sig_num = -42

    #Trust me, this is better than a message queue (it's essentially a pipe)
    #ret_val = []
    #for i in range(0,4):
    #    ret_val.append(mp.Array('d', [sig_num] * 100))

    rv_0 = mp.Array('d', [sig_num] * 100)
    rv_1 = mp.Array('d', [sig_num] * 100)
    rv_2 = mp.Array('d', [sig_num] * 100)
    rv_3 = mp.Array('d', [sig_num] * 100)

    # Create threads, one for each core
    nicolenotunix = mp.Process(target=rrt_process, args=(arg_0, rv_0))
    dashneptune = mp.Process(target=rrt_process, args=(arg_1, rv_1))
    jacobhuesman = mp.Process(target=rrt_process, args=(arg_2, rv_2))
    fworg64 = mp.Process(target=rrt_process, args=(arg_3, rv_3))
    thread_pool = [nicolenotunix, dashneptune, jacobhuesman, fworg64]

    # Start the thread pool
    for thread in thread_pool:
        thread.start()
        rospy.sleep(0.01)

    # Wait for the threads to finish
    for thread in thread_pool:
        thread.join()
        rospy.sleep(0.01)

    ret_val = [rv_0, rv_1, rv_2, rv_3]
    args = [arg_0, arg_1, arg_2, arg_3]

    results = []
    for val in ret_val:
        results.append(list_to_path(val[:], sig_num))

    results = sorted(results, key=operator.itemgetter(0))
    best_score, best_path = results[0]
    return best_path


def rrt_process(data, ret):
    start, goal, map, paths = data
    lowest_score = 9999999
    lowest_path = None

    if paths == 0:
        ret[0] = lowest_score
        return

    try:

        for i in range(0, paths):
            path = path_planning(start, goal, map)
            path_score = calculate_path_score(path)
            if path_score < lowest_score:
                lowest_path = path
                lowest_score = path_score


        path_array = path_to_array(lowest_path)
        for i in range(0, len(path_array)):
            ret[i + 1] = path_array[i]
        ret[0] = lowest_score

    except Exception:
        ret[0] = lowest_score


def path_to_array(path):
    path_array = []
    for point in path:
        x,y = point
        path_array.append(x)
        path_array.append(y)
    return path_array

def list_to_path(array, sig_num):
    score = array[0]
    array_path = []
    for i in range(0, len(array)/2):
        index = i*2+1
        x = array[index]
        y = array[index + 1]
        if x == sig_num and y == sig_num:
            break
        array_path.append((x,y))
    return (score, array_path)

def calculate_path_score(path):
    if len(path) == 0:
        return 9999999999999999
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
                rospy.loginfo("[IMPERIO] : ab {} bc {} ac {}".format(ab_dist, bc_dist, ac_dist))
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







