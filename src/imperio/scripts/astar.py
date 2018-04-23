"""
A* grid based planning
James Nordike
"""

import matplotlib.pyplot as plt
import math
from math import pi,sin,cos,ceil

class Node:
    """
    Initializes a node. 
    x: The horizontal position of the node.
    y: The vertical position of the node.
    t: The theta orientation of the node.
    cost: The cost of moving to this node.
    pind: The index of the parent node.
    """
    def __init__(self, x, y, t, cost, pind):
        self.x = int(x)
        self.y = int(y)
        if (t > pi):
          t = t - pi
        elif (t < -pi):
          t = t + pi
        self.t = t        # Node theta [-pi,pi]
        self.cost = cost  # Node Cost
        self.pind = pind  # Parent Index
        self.ct = ceil(t)

    def __str__(self):
        return str(self.x) + "," + str(self.y) + ","  + str(self.t) + "," + str(self.cost) + "," + str(self.pind)
        
"""
Calculates the final path to the goal.
"""
def calc_final_path(ngoal, closedset, reso):
    # generate final course
    r = [(ngoal.x * reso, ngoal.y * reso, ngoal.t)]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        r.append((n.x * reso, n.y * reso, n.t))
        pind = n.pind
    r.reverse()
    return r
    
"""
Heuristic of AStar.
ngoal: Node of the Goal
x: current x
y: current y
"""
def calc_h(ngoal, x, y):
    w = 10.0  # weight of heuristic
    d = w * math.sqrt((ngoal.x - x)**2 + (ngoal.y - y)**2)
    return d

def calc_index(node, xwidth, xmin, ymin):
    return ((node.y - ymin) * xwidth + (node.x - xmin)) * 8 + (node.ct + 4)
    
#All motions in all thetas are accounted.
def get_motion_model(theta):
    # dx, dy, dt, cost
    motion = [[ round(cos(theta)), round(sin(theta)), 0, 1],#1
              [ round(cos(theta - pi/4)), round(sin(theta - pi/4)), -pi/4, math.sqrt(2)],#2
              [-round(cos(theta + pi/4)), -round(sin(theta + pi/4)),  pi/4, math.sqrt(2)],#3
              [-round(cos(theta)), round(sin(theta)), 0, 1],#4
              [-round(cos(theta - pi/4)), -round(sin(theta - pi/4)), -pi/4, math.sqrt(2)],#5
              [ round(cos(theta + pi/4)), round(sin(theta + pi/4)),   pi/4, math.sqrt(2)]]#6
    
    return motion

# Fixx this
def calc_obstacle_map(Map):

    minx = round(Map.origin_x - Map.width/2)
    miny = round(Map.origin_y - Map.height/2)
    maxx = round(Map.origin_x + Map.width/2)
    maxy = round(Map.origin_y + Map.height/2)
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = Map.width #Both of these can be divided by the resolution
    ywidth = Map.height
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)

    # obstacle map generation
    obmap = Map.grid#This needs to be the grid but with a lower resolution

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def verify_node(node, obmap, minx, miny, maxx, maxy, threshold):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[node.x][node.y] < threshold:
        return False

    return True

#TODO: Make Wrapper function
def AStar(start, goal, Map):
    print start
    #print goal
    print len(Map.grid)*Map.resolution
    results = a_star_planning(start[0], start[1], start[2], goal[0], goal[1], goal[2], Map)
    #print results
    #return [[0,0,0],[1,0,0], [6,6,0]]
    return results
    
def a_star_planning(sx, sy, st, gx, gy, gt, Map):
    """
    sx: start x position [m]
    sy: start y position [m]
    st: start theta position[m]
    gx: goal x position [m]
    gy: goal y position [m]
    gt: goal theta position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]//Assuming the robot is the size of one square
    """
    reso = Map.resolution
    print Map.resolution
    nstart = Node(sx / reso, sy / reso, st, 0.0, -1)
    ngoal = Node(gx / reso, gy / reso, gt, 0.0, -1)
    threshold = .3
    ###ox = [iox / reso for iox in ox] 
    ###oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(Map)
    motion = get_motion_model(st)

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart
    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_h(ngoal, openset[o].x, openset[o].y))
        current = openset[c_id]
        
        if current.x == ngoal.x and current.y == ngoal.y and current.t == ngoal.t:#Insert Distance Threshold
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        motion = get_motion_model(current.t)
        for i in range(len(motion)):
            node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.t + motion[i][2], current.cost + motion[i][3], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if not verify_node(node, obmap, minx, miny, maxx, maxy, threshold):
                continue

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    r = calc_final_path(ngoal, closedset, reso)

    return r


