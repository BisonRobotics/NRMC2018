class Node:
    def __init__(self,value,point):
        self.value = value
        self.point = point
        self.parent = None
        self.H = 0
        self.G = 0
    def move_cost(self,other):
        return other.value

#Returns the Children as Nodes (Up, Down, Left, Right)
def children(point,grid):
    x,y = point.point
    links = [Node(grid[d[0]][d[1]], (d[0],d[1])) for d in [(x-1, y),(x,y - 1),(x,y + 1),(x+1,y)]]
    return [link for link in links if link.value <= .5]#Change this if needed to change the threshold
    
def heuristic(point,point2):
    return abs(point.point[0] - point2.point[0]) + abs(point.point[1]-point2.point[0])

#Returns the path as points
"""
Uses the Astar algorithm to find the shortest path from the start to the goal using the
occupancy grid given. The Start and Goal are tuples and OccGrid is the Occupancy Grid
as defined within the map_utils file.
"""
def aStar(start, goal, OccGrid):
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
        #Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o:o.G + o.H)
        #If it is the item we want, retrace the path and return it
        if current.point == goal:
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
