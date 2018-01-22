import math

class Node:
    def __init__(self, value, point):
        self.value = value
        self.point = point
        self.parent = None
        self.H = 0
        self.G = 0
    def move_cost(self,other):
        return other.value

#Returns the Children as Nodes (Up, Down, Left, Right)
def children(point,grid, width, height):
    sample_size = 1
    x,y = point.point
    links = []

    if x - 1 >= 0:
        value = grid[x-1][y]
        node = Node(value, (x-1,y))
        links.append(node)

    if y - 1 >= 0:
        value = grid[x][y-1]
        node = Node(value, (x,y-1))
        links.append(node)

    if x + 1 < width:
        value = grid[x+1][y]
        node = Node(value, (x+1,y))
        links.append(node)

    if y + 1 < height:
        value = grid[x][y+1]
        node = Node(value, (x,y+1))
        links.append(node)

    return [link for link in links if link.value <= .5]#Change this if needed to change the threshold
    
def heuristic(point,point2):
    return abs(point.point[0] - point2.point[0]) + abs(point.point[1]-point2.point[0])

#Returns the path as points
"""
Uses the Astar algorithm to find the shortest path from the start to the goal using the
occupancy grid given. The Start and Goal are tuples and OccGrid is the Occupancy Grid
as defined within the map_utils file.
"""
def aStar(global_start, global_goal, OccGrid):
    x,y = global_goal
    goal = OccGrid.cell_index(x, y)
    x,y = global_start
    start = OccGrid.cell_index(x,y)
    print("goal : {} start : {}", goal, start)
    #Gets the grid from the occupancy grid and uses that as the map.
    grid = OccGrid.grid
    #The open and closed sets (Store Nodes)
    openset = set()
    closedset = set()
    #Current point is the starting point (Change it to a Node from a point)
    current = Node(0,start)
    end = Node(0,goal)
    #Add the starting point to the open set
    openset.add(current)
    #While the open set is not empty
    
    while openset:
        #Where G is actual cost and H is hueristic
        #Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o:o.G + o.H)
        #If it is the item we want, retrace the path and return it
        #TODO : calculate threshold based on resulotion of occupancy grid
        #print("current point :", current.point)
        current_x, current_y = current.point
        goal_x, goal_y = goal
        abs_distance = math.sqrt((current_x - goal_x) ** 2 + (current_y - goal_y) ** 2)
        if abs_distance <= 0:
            path = []
            while current.parent:
                global_point = OccGrid.cell_position(current.point[0], current.point[1])
                path.append(global_point)
                current = current.parent
            global_point = OccGrid.cell_position(current.point[0], current.point[1])
            path.append(global_point)
            return path[::-1]
        #Remove the item from the open set
        openset.remove(current)
        print(len(openset))
        #Add it to the closed set
        closedset.add(current)
        #Loop through the node's children/siblings
        for node in children(current,grid, OccGrid.height, OccGrid.width):
            #If it is already in the closed set, skip it
            if node in closedset:
                print("FUCK")
                continue
            #Otherwise if it is already in the open set
            if node in openset:
                print(".")
                #Check if we beat the G score 
                new_g = current.G + current.move_cost(node)
                if new_g != 0:
                    print("NEW_G does not equal 0")
                if node.G != 0:
                    print("NODE.G does not equal 0")
                if node.G > new_g:
                    #If so, update the node to have a new
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
    #TODO handle this in a treatable way . . .
    #Throw an exception if there is no path
    raise ValueError('No Path Found')
