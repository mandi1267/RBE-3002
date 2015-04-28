#!/usr/bin/env python

# Amanda Adkins
# Alex Caracappa


import rospy
import time
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from final_project_adkins_caracappa.srv import *
from math import sqrt
from std_msgs.msg import Float64
from std_msgs.msg import Bool


def handle_a_star(req):

    print "a star"

    # get the starting cell, and goal cell
    startCell = (req.startX, req.startY)
    goalCell = (req.goalX, req.goalY)
    
    # clear the path and way points from the map
    updateGridCells([], 5)
    updateGridCells([], 4)

    # initialize frontier and explored nodes
    evaluatedNodes = [] # contains list of cell reference numbers for cells that have been fully explored
    openSet = [startCell] # contains a list of cell reference numbers for cells that have been reached
    predecessors = {}
    g_score = {} #[float("inf")]*len(occGrid)   # minimal cost from start to cell 
                                            # cost for cell is given by g_score[cell reference number]
    f_score = {} #[float("inf")]*len(occGrid)   # estimated total path cost for given cell, also indexed by reference number

    g_score[startCell] = 0   # distance from start to start is 0

    f_score[startCell] = g_score[startCell] + getHeuristic(startCell, goalCell) # estimated cost is g_score + heuristic

    # while there are reached, but unexplored nodes, keep visitng their neighbors
    while (len(openSet) != 0):
        if (rospy.is_shutdown()):
            break
        # # update the map with the new frontier and visited cells
        updateGridCells(openSet, 3)
        updateGridCells(evaluatedNodes, 2)

        # get the cell with the lowest f_score (total estimated cost)
        current = openSet[0] # initialize to the first node, and update if one with lower estimated cost is found   
        for i in openSet:
            if (f_score.has_key(i)):
                if (f_score[i] < f_score[current]):
                    current = i
        
	#if (abs(current[0]- goalCell[0]) < 0.05) and (abs(current[1] - goalCell[1]) < 0.05):
        #    print "close cell"

        
        # if the goal has been reached
#        if (current[0] == goalCell[0]) and (current[1] == goalCell[1]):
        if (abs(current[0]- goalCell[0]) < 0.075) and (abs(current[1] - goalCell[1]) < 0.075):
            # start building the path, starting from the end
            fullPath = [current]
            # while we're not at the start, add predecessors to the path
            while (current[0] != startCell[0]) or (current[1] != startCell[1]):
                current = predecessors[current]
                fullPath.insert(0, current)
                if getCostBetweenNeighbors(fullPath[0], fullPath[1]) > 2:
                    print "path through obstacle"
            # update the grid cells a final time
            #updateGridCells(openSet, 3)
            #updateGridCells(evaluatedNodes, 2)
            # pause so the update is visible
            time.sleep(0.5)
            # clear the frontier and explored nodes 
            updateGridCells([], 2)
            updateGridCells([], 3)

            # return the path
            pathX, pathY = splitPathIntoXY(fullPath)
            return AstarResponse(pathX, pathY)

        # remove the node that we're about to explore from the frontiers list and add it to the explored list
        openSet.remove(current)
        evaluatedNodes.append(current)

        # look at all of its neighbors
        for neighbor in getNeighbors(current):
            # if we already explored a neighbor, skip remaining calculations on it
            if neighbor in evaluatedNodes:
                continue
            # calculate the tentative cost from the start to this neighbor through the current cell
            tentative_g_score = g_score[current] + getCostBetweenNeighbors(current, neighbor)
            if not g_score.has_key(neighbor):
            	g_score[neighbor] = float("inf")

            # if we haven't seen this cell yet, or if the path through the current cell is 
            # better than a previously discovered path
            if (neighbor not in openSet) or (tentative_g_score < g_score[neighbor]):
                # update the predecessor of the neighbor, the cost to the neighbor, and the estimated cost from 
                # start to goal through this neighbor cell
                predecessors[neighbor] = current
                g_score[neighbor] = tentative_g_score 
                f_score[neighbor] = g_score[neighbor] + getHeuristic(neighbor, goalCell)
      
                # if this neighbor is not in the set of frontier cells, add it 
                if (neighbor not in openSet):
                    openSet.append(neighbor)

# split a list of (x,y) tuples into two lists containing x and y respectively
def splitPathIntoXY(pointsPath):
	xCoords = []
	yCoords = []
	for i in range(len(pointsPath)):
		cellTup = pointsPath[i]
		xCoords.append(cellTup[0])
		yCoords.append(cellTup[1])
	return xCoords, yCoords


# call back function for the map
# saves the map resolution, origin, map dimensions, and occupancy data as globals 
# for access within other functions
def read_occupancy_grid(data):
    print "reading occupancy grid"
    global mapRes
    mapRes = data.info.resolution
    global mapResDefined
    mapResDefined = True

    global xOriginMap
    xOriginMap = data.info.origin.position.x

    global yOriginMap
    yOriginMap = data.info.origin.position.y

    global mapWidth
    mapWidth = data.info.width
    print "width"
    print mapWidth

    global mapHeight
    mapHeight = data.info.height
    print "map Height"
    print mapHeight

    global occGrid 
    occGrid = data.data

    global minX
    global minY
    global maxX
    global maxY

    minX, minY = getXY(0)
    maxX, maxY = getXY(len(data.data) - 1)

    displayObstacles()


# display the obstacles on the graph
def displayObstacles():
    obstacles = []
    for i in range(len(occGrid)):
        if occGrid[i] >= 50:
            obstacles.append(getXY(i))
    if pubsInitialized:
        updateGridCells(obstacles, 5)

# get a cells coordinates from its reference number
# returns tuple with (x,y)
def getXY(number):
    xOffset = xOriginMap + (mapRes/2)
    yOffset = yOriginMap + (mapRes/2)

    yPos = int(number/mapWidth)
    xPos = number%mapWidth

    xPos = xPos*mapRes + xOffset
    yPos = yPos*mapRes + yOffset

    xPos = xPos/mapRes
    xPos = round(xPos, 0)
    xPos = xPos*mapRes
    yPos = yPos/mapRes
    yPos = round(yPos, 0)
    yPos = yPos*mapRes

    return (xPos, yPos)

# get the estimated distance between 2 cells (by number)
# distance based on octile 
def getHeuristic(cell1, cell2):
    x1, y1 = splitCellTuple(cell1)
    x2, y2 = splitCellTuple(cell2)

    xDiff = abs(x1-x2)
    yDiff = abs(y1-y2)

    diagDist = min(xDiff, yDiff)

    straightDist = max(xDiff, yDiff) - diagDist
    total = straightDist + (diagDist*sqrt(2))

    return total

def splitCellTuple(cellTup):
	return cellTup[0], cellTup[1]

# takes in a list of numbers corresponding to cells and 
# a number representing which topic to publish them to
# 1 - gridCellPub
# 2 - exploredCellsPub
# 3 - frontierCellsPub
# 4 - wayPointsPub
# 5 - pathPub
def updateGridCells(tupleList, topicNum):
    # convert the list of cell numbers to points
    listOfPoints = []
    for i in tupleList:
        xPos, yPos = splitCellTuple(i)
        listOfPoints.append(makePoint(xPos, yPos))
    # publish the points as grid cells
    publishGridCells(listOfPoints, topicNum)

# get the numbers corresponding to the neighbors of a cell with the given number
# cell number is the index of a cell in the occupancy grid
def getNeighbors(cellTup):
    # initialize neighbors list
    neighbors = []
    # get the coordinates of the cell on the map
    xPos, yPos = splitCellTuple(cellTup)

    # if the cell is not at the smallest X value
    # it can have a neighbor with a smaller X value
    if xPos != minX:
        newNum = ((xPos-mapRes), yPos)
        neighbors.append(newNum)

        if yPos != minY:
            newNum = ((xPos-mapRes), (yPos-mapRes))
            neighbors.append(newNum)
        if yPos != maxY:
            newNum = ((xPos-mapRes), (yPos+mapRes))
            neighbors.append(newNum)

    # not max X -> can have neighbor at 1 grid cell more in x direction
    if xPos != maxX:
        newNum = ((xPos + mapRes), yPos)
        neighbors.append(newNum)

        if yPos != minY:
            newNum = ((xPos+mapRes), (yPos-mapRes))
            neighbors.append(newNum)
        if yPos != maxY:
            newNum = ((xPos+mapRes), (yPos+mapRes))
            neighbors.append(newNum)

    # not min Y -> can have neighbor at 1 grid cell less in y direction 
    if yPos != minY:
        newNum = (xPos, (yPos - mapRes))
        neighbors.append(newNum)
    # not max Y -> can have neighbor at 1 grid cell more in y direction
    if yPos != maxY:
        newNum = (xPos, (yPos + mapRes))
        neighbors.append(newNum)
    # return the list of neighbors
    return neighbors

    # get the cell's reference number from its coordinates
def makeNumFromXY(x, y):
    xOffset = xOriginMap + (mapRes/2)
    yOffset = yOriginMap + (mapRes/2)

    # print "offsets"
    # print xOffset
    # print yOffset

    row = round(((y - yOffset)/mapRes), 0)
    col = round(((x - xOffset)/mapRes), 0)

    number = mapWidth*row + col

    return int(round(number, 0))

# make a point object from an x and y coordinate
def makePoint(xCoord, yCoord):
    newPoint = Point()
    newPoint.x = xCoord
    newPoint.y = yCoord
    return newPoint

# this function takes a list of points and publishes a GridCells object with those points
# to a topic corresponding the the given number
# the numbers relate to the topics as follows:
# 1 - gridCellPub
# 2 - exploredCellsPub
# 3 - frontierCellsPub
# 4 - wayPointsPub
# 5 - pathPub
def publishGridCells(listOfPoints, topicNum):
        
    # create new grid cells object
    gridCells = GridCells()

    # set the list of points, frame, and cell size
    gridCells.cells =  listOfPoints
    gridCells.header.frame_id = "/map"

    gridCells.cell_width = mapRes
    gridCells.cell_height = mapRes

    # publish the grid to the correct topic
    if topicNum == 1:
        gridCellPub.publish(gridCells)
    elif topicNum == 2:
        exploredCellsPub.publish(gridCells)
    elif topicNum == 3:
        frontierCellsPub.publish(gridCells)
    elif topicNum == 4:
        wayPointsPub.publish(gridCells)
    elif topicNum == 5:
        pathPub.publish(gridCells)

# get the cost between 2 neighboring cells
# if they aren't neighboring, this isn't accurate
# cells referenced by cell number
def getCostBetweenNeighbors(cell1, cell2):
    timesExpanded = expansionWidth/mapRes
    
    cell1X, cell1Y = splitCellTuple(cell1)
    cell2X, cell2Y = splitCellTuple(cell2)
    num1 = makeNumFromXY(cell1X, cell1Y)
    num2 = makeNumFromXY(cell2X, cell2Y)

    if ((occGrid[num1] >= 50) or (occGrid[num2] >= 50)):
        return float("inf")
    elif (((cell1X - cell2X) == 0) or ((cell1Y - cell2Y) == 0)):
        return mapRes
    else:
        return mapRes*sqrt(2)

def read_expansionWidth(data):
    global expansionWidth
    expansionWidth = data.data

def a_star_server():
    s = rospy.Service('a_star', Astar, handle_a_star)
    rospy.spin()

def driving_ready(data): 
    if (data.data):
        global drivingReady 
        drivingReady = True

def a_star_resp(data):
    if (not data.data):
        global lab4Replied
        lab4Replied = True
    

# This is the program's main function
if __name__== "__main__":

	
    rospy.init_node('a_star_server')

    global mapResDefined
    global pathPub
    global wayPointsPub
    global gridCellPub
    global exploredCellsPub
    global frontierCellsPub

    global pubsInitialized 
    pubsInitialized = False

    mapResDefined = False

    occupancySub = rospy.Subscriber("res_changed_map", OccupancyGrid, read_occupancy_grid, queue_size = 1)
    exploredCellsPub = rospy.Publisher('explored_cells', GridCells, queue_size=3)
    frontierCellsPub = rospy.Publisher('frontier_cells', GridCells, queue_size=3)
    wayPointsPub = rospy.Publisher('path_waypoints', GridCells, queue_size = 3)
    pathPub = rospy.Publisher('full_path', GridCells, queue_size=3)
    gridCellPub = rospy.Publisher('grid_for_map', GridCells, queue_size=3)
    expansionWidthSub = rospy.Subscriber('expansion_size', Float64, read_expansionWidth, queue_size=1)
    pubsInitialized = True


    global aStarNodePub 
    aStarNodePub = rospy.Publisher('astar_node_ready', Bool, queue_size=3)
    global astarNodeSub
    aStarNodeSub = rospy.Subscriber('astar_node_ready', Bool, a_star_resp, queue_size=3)

    global drivingReady
    drivingReady = False
    global lab4Replied
    lab4Replied = False

    drivingNodePub = rospy.Publisher('driving_node_ready', Bool, queue_size=3) 
    drivingSub = rospy.Subscriber('driving_node_ready', Bool, driving_ready, queue_size=3)


    while (not drivingReady) and (not rospy.is_shutdown()):
        1+1
    
    ready = Bool()
    ready.data = False

    drivingNodePub.publish(ready)

    ready.data = True

    while (not lab4Replied) and (not rospy.is_shutdown()):
        aStarNodePub.publish(ready)
        time.sleep(0.1)

    a_star_server()
