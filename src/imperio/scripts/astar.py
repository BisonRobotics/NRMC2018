#!/usr/bin/env python
""" Performs A* calculation on an occupancy grid

Author: James Madison University
Date: 12/19/2017
Version: 1

"""

class Node:
    def __init__(self,value, x, y):
        self.value = value
        self.x = x
        self.y = y
        self.parent = None
        self.H = 0
        self.G = 0
    def move_cost(self,other):
        return other.value

#determines if the index is in the occupancy grid
def index_valid(x, y, grid):
    if x < 0 or y < 0:
        return False
    return True


#Returns the Children as Nodes (Up, Down, Left, Right)
def children(point,grid):
    x = point.x
    y = point.y
    print("X then Y")
    print(x)
    print(y)

    print(grid)
    print(grid[0])


    links = []

    if index_valid(x, y-1, grid):
        node = Node(grid[x][y-1], x, y-1)
        links.append(node)

    if index_valid(x, y+1, grid):
        node = Node(grid[x][y+1], x, y+1)
        links.append(node)

    if index_valid(x - 1, y, grid):
        node = Node(grid[x - 1][y], x - 1, y)
        links.append(node)

    if index_valid(x, y-1, grid):
        node = Node(grid[x + 1][y], x + 1, y)
        links.append(node)

    print("done")
    print(links)

    links = [Node(grid[d[0]][d[1]], d[0], d[1]) for d in [(x-1, y),(x,y - 1),(x,y + 1),(x+1,y)]]
    return [link for link in links if link.value <= .5]#Change this if needed to change the threshold
    
def heuristic(point,point2):
    return abs(point.x - point2.x) + abs(point.y - point2.y)

#Returns the path as points
def aStar(start, goal, grid):
    #The open and closed sets (Store Nodes)
    openset = set()
    closedset = set()
    #Current point is the starting point (Change it to a Node from a point)
    current = Node(0,start[0], start[1])
    end = Node(0,goal[0], goal[1])
    #Add the starting point to the open set
    openset.add(current)
    #While the open set is not empty
    while openset:
        #Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o:o.G + o.H)
        #If it is the item we want, retrace the path and return it
        if current == goal:
            path = []
            while current.parent:
                path.append(current.point)
                current = current.parent
            path.append(current.point)
            return path[::-1]
        #Remove the item from the open set
        openset.remove(current)
        #Add it to the closed set
        closedset.add(current)
        #Loop through the node's children/siblings
        for node in children(current,grid):
            #If it is already in the closed set, skip it
            if node in closedset:
                continue
            #Otherwise if it is already in the open set
            if node in openset:
                #Check if we beat the G score 
                new_g = current.G + current.move_cost(node)
                if node.G > new_g:
                    #If so, update the node to have a new parent
                    node.G = new_g
                    node.parent = current
            else:
                #If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + current.move_cost(node)
                node.H = heuristic(node, end)
                #Set the parent to our current item
                node.parent = current
                #Add it to the set
                openset.add(node)
    #Throw an exception if there is no path
    raise ValueError('No Path Found')

