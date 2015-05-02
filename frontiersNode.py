#!/usr/bin/env python

# Amanda Adkins
# Alex Caracappa


from final_project_adkins_caracappa.srv import DriveStraight
from final_project_adkins_caracappa.srv import Rotate
from final_project_adkins_caracappa.srv import GoToGoal
import rospy
import time
from math import sqrt
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from math import pi
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point

from subprocess import call

# returns list of fronteirs
def identifyFrontiers():
	allFrontiers = []
        frontierTooShortLimit = 2

	frontierCells = getAllFrontierCells()
	while (len(frontierCells) != 0):
		currentFrontier = []
		checkForNeighbors = frontierCells[0]
		currentFrontier.append(checkForNeighbors)
		frontierCells.remove(checkForNeighbors)
		cellNeighbors = getNeighbors(checkForNeighbors)
		finishedFindingNeighbors = 0
		while (len(currentFrontier) > finishedFindingNeighbors):
			newToRemove = []
			for i in frontierCells:
				if i in cellNeighbors:
					currentFrontier.append(i)
					newToRemove.append(i)
			for i in newToRemove:
				frontierCells.remove(i)
			finishedFindingNeighbors = finishedFindingNeighbors + 1
			if (finishedFindingNeighbors != len(currentFrontier)):
				checkForNeighbors = currentFrontier[finishedFindingNeighbors]
				cellNeighbors = getNeighbors(checkForNeighbors)
                if (len(currentFrontier) > frontierTooShortLimit):
                        allFrontiers.append(currentFrontier)
        print allFrontiers
	print len(allFrontiers)
	return allFrontiers

def getAllFrontierCells():
	allCells = []
	for i in range(len(occGrid)):
		if occGrid[i] == 0:
			coords = getXY(i)
			neighbors = getNeighbors(coords)
			unknownNeighbor = False
			while (len(neighbors) != 0):
                                currentNeighbor = neighbors[0]
				if occGrid[makeNumFromXY(currentNeighbor[0], currentNeighbor[1])] == -1:
					unknownNeighbor = True
					break
				neighbors.remove(neighbors[0])
			if unknownNeighbor == True:
				allCells.append(coords)
	return allCells

def makeNumFromXY(x, y):
    xOffset = xOriginMap + (mapRes/2)
    yOffset = yOriginMap + (mapRes/2)

    row = round(((y - yOffset)/mapRes), 0)
    col = round(((x - xOffset)/mapRes), 0)

    number = mapWidth*row + col

    return int(round(number, 0))


# get a cells coordinates from its reference number
# returns tuple with (x,y)
def getXY(number):
    xOffset = xOriginMap + (mapRes/2)
    yOffset = yOriginMap + (mapRes/2)

    yPos = int(number/mapWidth)
    xPos = number%mapWidth

    xPos = xPos*mapRes + xOffset
    yPos = yPos*mapRes + yOffset

    return (xPos, yPos)

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


# currently picking by checking length of frontier
def pickFrontier(listOfFrontiers):
	# might change a lot
	longestFrontier = listOfFrontiers[0]
	for frontier in listOfFrontiers:
                print len(longestFrontier)
                print len(frontier)
		if len(frontier) > len(longestFrontier):
			longestFrontier = frontier
                        print "selecting new frontier"

	return longestFrontier


def getFrontierCentroid(frontier):
	xTotal = 0
	yTotal = 0
	cellCount = 0

	for cell in frontier:
		xTotal = xTotal + cell[0]
		yTotal = yTotal + cell[1]
		cellCount = cellCount + 1

	xTotal = xTotal/cellCount
	yTotal = yTotal/cellCount

        closestCell = getClosestCell(xTotal, yTotal)
        closestCellNum = makeNumFromXY(closestCell[0], closestCell[1])

        if (occGrid[closestCellNum] == 0):
                return closestCell

	xDiff = abs(cell[0] - closestCell[0])
	yDiff = abs(cell[1] - closestCell[1])

	smallestTotalDiff = sqrt(xDiff**2 + yDiff**2)
	for cell in frontier:
		xDiff = abs(cell[0] - closestCell[0])
		yDiff = abs(cell[1] - closestCell[1])

		totalDiff = sqrt(xDiff**2 + yDiff**2)
		if (totalDiff < smallestTotalDiff):
			smallestTotalDiff = totalDiff
			closestCell = cell

	return closestCell

# return the index of the cell most closely matching the given x and y coordinates
def getClosestCell(xCoord, yCoord):

    # calculate the most likely cell by rounding the x and y coordinates to the numbers divisible by mapRes
    tempXCoord = (xCoord/mapRes)
    tempXCoord = round(tempXCoord, 0)
    tempXCoord = tempXCoord*mapRes - mapRes/2

    tempYCoord = (yCoord/mapRes)
    tempYCoord = round(tempYCoord, 0)
    tempYCoord = tempYCoord*mapRes - mapRes/2

    # calculate the temporary cell number
    tempCell = (tempXCoord, tempYCoord) 

    # get the neighbors of the cell
    neighborsList = getDirectNeighbors(tempCell) 

    # set the closest recorded x and y coordinates
    closestXCoord = tempXCoord
    closestYCoord = tempYCoord

    # check to see if any of the neighboring cells are closer to the given coordinates than the discovered cell
    # could result  if truncation happens
    for cell in neighborsList:
        tempXCoord, tempYCoord = splitCellTuple(cell)
        # if any of the neighboring cells is closer, set that as the closest cell
        if ((abs(closestXCoord - xCoord)) > (abs(tempXCoord - xCoord))) and ((abs(closestYCoord - yCoord)) > (abs(tempYCoord - yCoord))):
            closestXCoord = tempXCoord
            closestYCoord = tempYCoord

    # return the closest cell number
    return closestXCoord, closestYCoord

# get the numbers corresponding to the neighbors of a cell with the coordinates
def getDirectNeighbors(cellData):
    # initialize neighbors list
    neighbors = []
    xPos, yPos = splitCellTuple(cellData)
    
    # if the cell is not at the smallest X value
    # it can have a neighbor with a smaller X value
    if xPos != minX:
        newNum = ((xPos-mapRes), yPos)
        neighbors.append(newNum)
    # not max X -> can have neighbor at 1 grid cell more in x direction
    if xPos != maxX:
        newNum = ((xPos + mapRes), yPos)
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

# split a 2 part tuple up
def splitCellTuple(cellTup):
	return cellTup[0], cellTup[1]

# call back function for the map
# saves the map resolution, origin, map dimensions, and occupancy data as globals 
# for access within other functions
def read_occupancy_grid(data):
    global mapReceived

    global mapRes
    mapRes = data.info.resolution

    global xOriginMap
    xOriginMap = data.info.origin.position.x

    global yOriginMap
    yOriginMap = data.info.origin.position.y

    global mapWidth
    mapWidth = data.info.width

    global mapHeight
    mapHeight = data.info.height

    global occGrid 
    occGrid = list(data.data)

    global minX
    global minY
    global maxX
    global maxY

    minX, minY = getXY(0)
    maxX, maxY = getXY(len(data.data) - 1)

    mapReceived = True


def preexplorationMove():
    # rotate 360
    speed = 0.1
    print "waiting for rotate service"
    rospy.wait_for_service('rotate')
    rotate = rospy.ServiceProxy('rotate', Rotate)
    print "running rotate"    
    response = rotate(1.9*pi)
    # drive straight 0.06
    print "waiting for drive service"
    rospy.wait_for_service('drive_straight')
    print "getting drive function"
    driveFunction = rospy.ServiceProxy('drive_straight', DriveStraight)
    print "running drive"
    driveFunction(0.06, speed)

def goal_nav_ready(data):
    if (data.data):
        global goalReady
        goalReady = True
    
def mapRespRecvd(data):
    if (not data.data):
        global mapResp
        mapResp = True

# make a point object from an x and y coordinate
def makePoint(xCoord, yCoord):
    newPoint = Point()
    newPoint.x = xCoord
    newPoint.y = yCoord
    return newPoint

# takes in a list of numbers corresponding to cells and 
# a number representing which topic to publish them to
# 7 - frontierCells
def updateGridCells(tupleList, topicNum):
    # convert the list of cell numbers to points
    listOfPoints = []
    for i in tupleList:
        xPos, yPos = splitCellTuple(i)
        listOfPoints.append(makePoint(xPos, yPos))
    # publish the points as grid cells
    publishGridCells(listOfPoints, topicNum)

# this function takes a list of points and publishes a GridCells object with those points
# to a topic corresponding the the given number
# the numbers relate to the topics as follows:
# 7 - frontierCells
def publishGridCells(listOfPoints, topicNum):
        
    # create new grid cells object
    gridCells = GridCells()

    # set the list of points, frame, and cell size
    gridCells.cells =  listOfPoints
    gridCells.header.frame_id = "/map"

    gridCells.cell_width = mapRes
    gridCells.cell_height = mapRes

    # publish the grid to the correct topic
    if topicNum == 7:
        frontierPub.publish(gridCells)
    else:
        time.sleep(0.5)
        if topicNum == 8:
            smallFrontPub0.publish(gridCells)
        elif topicNum == 9:
            smallFrontPub1.publish(gridCells)
        elif topicNum == 10:
            smallFrontPub2.publish(gridCells)
        elif topicNum == 11:
            smallFrontPub3.publish(gridCells)
        elif topicNum == 12:
            smallFrontPub4.publish(gridCells)
    

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('main_final_project')

    

    global frontierPub
    global smallFrontPub0
    global smallFrontPub1
    global smallFrontPub2
    global smallFrontPub3
    global smallFrontPub4

    occupancySub = rospy.Subscriber("res_changed_map", OccupancyGrid, read_occupancy_grid, queue_size=1)

    frontierPub = rospy.Publisher('frontier_grid_cells', GridCells, queue_size=3)
    smallFrontPub0 = rospy.Publisher('smallFrontier0_grid_cells', GridCells, queue_size=3)
    smallFrontPub1 = rospy.Publisher('smallFrontier1_grid_cells', GridCells, queue_size=3)
    smallFrontPub2 = rospy.Publisher('smallFrontier2_grid_cells', GridCells, queue_size=3)
    smallFrontPub3 = rospy.Publisher('smallFrontier3_grid_cells', GridCells, queue_size=3)
    smallFrontPub4 = rospy.Publisher('smallFrontier4_grid_cells', GridCells, queue_size=3)

    global mapReceived 
    mapReceived = False

    global frontierNodePub 
    frontierNodePub = rospy.Publisher('frontier_node_ready', Bool, queue_size=3)
    frontierNodeSub = rospy.Subscriber('frontier_node_ready', Bool, mapRespRecvd, queue_size=3)

    global mapResp
    mapResp = False
    global goalReady
    goalReady = False
    ready = Bool()
    ready.data = False



    goalNavPub = rospy.Publisher('goalnav_node_ready', Bool, queue_size=3)
    goalNavSub = rospy.Subscriber('goalnav_node_ready', Bool, goal_nav_ready, queue_size=3)

    while (not goalReady) and (not rospy.is_shutdown()):
        1+1
    goalNavPub.publish(ready)
    ready.data = True
    while (not mapResp) and (not rospy.is_shutdown()):
        frontierNodePub.publish(ready)
        time.sleep(0.1)
    
    
    while (mapReceived == False and (not rospy.is_shutdown())):
    	1+1

    # ready to go
    print "got map"
    preexplorationMove()
    print "done with initial move"
    frontiersList = identifyFrontiers()
    while (len(frontiersList) != 0):
        allFrontierCells = []
        for cellsList in frontiersList:
            allFrontierCells = allFrontierCells + cellsList
        updateGridCells(allFrontierCells, 7)
        for index in range(len(frontiersList)):
            updateGridCells(frontiersList[index], 8+(index%5))
            print frontiersList[index]
        
        print "picking frontier"
    	frontier = pickFrontier(frontiersList)
    	goal = getFrontierCentroid(frontier)
        print "goal"
        print goal
    	rospy.wait_for_service('goal_nav_service')
    	driveToGoal = rospy.ServiceProxy('goal_nav_service', GoToGoal)
    	response = driveToGoal(goal[0], goal[1])
        if (response.status == 0):
            break
        print "done with current frontier"
    	preexplorationMove()
        print "identifying new frontiers"
    	frontiersList = identifyFrontiers()

    #call("spd-say 'I'm done.'");
    while (not rospy.is_shutdown()):
    	print "DONE EXPLORING"
    	time.sleep(0.5)
