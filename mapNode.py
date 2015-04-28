#!/usr/bin/env python

# Amanda Adkins
# Alex Caracappa

import rospy
from math import ceil
from final_project_adkins_caracappa.srv import *
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64
import time
from std_msgs.msg import Bool

# take in the published graph and chenge the resolution
def increaseMapResolution(occupancyGridData):
    # get the dimensions and grid from the published graph
    currentMapRes = occupancyGridData.info.resolution
    occGrid = occupancyGridData.data
    oldMapWidth = occupancyGridData.info.width
    oldMapHeight = occupancyGridData.info.height

    # determine how many cells of the old occupancy grid go into the new occupancy grid
    # and determine the height and width of the new map
    multiplier = int(round((desiredMapRes/currentMapRes), 0))
    newMapWidth = int(ceil(oldMapWidth/multiplier))
    newMapHeight = int(ceil(oldMapHeight/multiplier))

    # fill the new map with zeroes
    newMap = [0]*(newMapHeight*newMapWidth)

    if (currentMapRes == desiredMapRes):
        newMap = occGrid
        return newMap, newMapHeight, newMapWidth
    
    # go through every group of cells multiplier by multiplier and check to see if htere is any free space or obstacles
    for i in range(newMapHeight):
        # if not at the last row of the new map
        # the size of the block will be the multiplier
        if (i != (newMapHeight - 1)):
            yDim = multiplier
        # if the old map is divisible by the multiplier
        # the last row of the new map will represent the data from multiplier rows of the old map
        elif (oldMapHeight%multiplier != 0):
            yDim = oldMapHeight%multiplier
        # if there aren't a number of rows evenly divisible by multiplier
        # the last row will be composed of the remaining rows
        else:
            yDim = multiplier

        # go through the groups in the x direction
        for j in range(newMapWidth):
            # if not at the last column of the new map
            # the size of the block will be the multiplier
            if (j != (newMapWidth -1)):
                xDim = multiplier
            # if columns of the the old map is divisible by the multiplier
            # the last column of the new map will represent the data from multiplier columns of the old map
            elif (oldMapWidth%multiplier != 0):
                xDim = oldMapWidth%multiplier
            # if there aren't a number of rows evenly divisible by multiplier
            # the last row will be composed of the remaining rows
            else:
                xDim = multiplier
            # set variables to indicate if a free space or obstacle are found in the block of cells representing
            # one cell in the old graph
            zeroFound = 0
            obstacleFound = 0
            # go through the cells of the old map that make up 1 cell in the new map
            for k in range(yDim):
                for l in range(xDim):
                    # compute the index into the old map
                    cellIndex = oldMapWidth*(i*multiplier + k) + j*multiplier + l
                    # check for an obstacle or free space
                    if (occGrid[cellIndex]) == 100:
						obstacleFound = 100
                    if (occGrid[cellIndex] == 0):
                        zeroFound = 1
            # if no subcells had an obstacle or free space, it is unknown
            if (obstacleFound == 0) and (zeroFound == 0):
                newMap[i*newMapWidth + j] = -1
            # if some cells had known state, set to if an obstacle was found or not (0 or 100)
            else: 
                newMap[i*newMapWidth + j] = obstacleFound
    return newMap, newMapHeight, newMapWidth


# expand the obstacles in the occupancy grid
# occGrid - occupancy grid
# mapRes - resolution of the map
# mapWidth - width in cells of the map
# mapHeight - height in cells of the map
def expandObstacles(occGrid, mapRes, mapWidth, mapHeight):
    # times expanded indicates how many times the obstacles should be expanded
    timesExpanded = (expansionWidth/mapRes) - 1
    # number of times the obstacles ahve been expanded 
    iterations = 0
    print "starting to expand"
    while (iterations!=timesExpanded):
        # go through every cell
        for i in range(len(occGrid)):
            # if the cell has an obstacle or an expanded obstacle, 
            # expand it again and makr the cell 1 less than the original found cell
            if occGrid[i] == (100 - iterations):
            	column = i%mapWidth
            	row = int(i/mapWidth)

                # set the neighhhbors appropriately
            	neighbors = getGridNeighbors(mapWidth, mapHeight, column, row)
                
                for neighbor in neighbors:
                    cellIndex = neighbor[0] + mapWidth*neighbor[1]
                    if (occGrid[cellIndex] == 0) or occGrid[cellIndex] == -1:
                        occGrid[cellIndex] = (99 - iterations)
        # increment the number of times the obstacles have been expanded
        iterations = iterations + 1
        print iterations
    return occGrid

# get the neighbors of the given grid cell
# gridWidth - width of the map
# gridHeight - height of the map
# cellCol - column that the cell is in
# cellRow - row that the cell is in 
# returns a list of tuples
# tuples are the colun and row of the cells
def getGridNeighbors(gridWidth, gridHeight, cellCol, cellRow):
    # say that there are initially no neighbors
    neighbors = []

    # if the cell isn't in the first column
    # add neighbors that are in the prior column
    if (cellCol != 0):
        newNum = ((cellCol-1), cellRow)
        neighbors.append(newNum)

        # if the cell isn't in the first row, add neighbor that has a smaller row and column
        if cellRow != 0:
            newNum = ((cellCol-1), (cellRow-1))
            neighbors.append(newNum)
        # if the cell isn't in the last row, add neighbor that has a larger row and smaller column
        if cellRow != (gridHeight -1):
            newNum = ((cellCol-1), (cellRow+1))
            neighbors.append(newNum)

    # not max X -> can have neighbor at 1 grid cell more in x direction
    if cellCol != (gridWidth -1):
        newNum = ((cellCol + 1), cellRow)
        neighbors.append(newNum)

        if cellRow != 0:
            newNum = ((cellCol+1), (cellRow-1))
            neighbors.append(newNum)
        if cellRow != (gridHeight - 1):
            newNum = ((cellCol+1), (cellRow+1))
            neighbors.append(newNum)

    # not min Y -> can have neighbor at 1 grid cell less in y direction 
    if cellRow != 0:
        newNum = (cellCol, (cellRow - 1))
        neighbors.append(newNum)
    # not max Y -> can have neighbor at 1 grid cell more in y direction
    if cellRow != (gridHeight -1):
        newNum = (cellCol, (cellRow + 1))
        neighbors.append(newNum)
    # return the list of neighbors
    return neighbors

# print when the modified map (with the modififed resolution was published)
def checkMapReceived(data):
    print "modified map received"

# takes in the map published by the gmapping program
# changes the resolution of the map
# and expands the obstacles
# the map is then republished with the topic res_changed_map
def read_map(data):
    global workingOnMap
    workingOnMap = True
    print "manually reducing map resolution"
    # change the map's resolution
    newOccGrid, newMapHeight, newMapWidth = increaseMapResolution(data)
    # expand the obstacles 
    newOccGrid = expandObstacles(newOccGrid, desiredMapRes, newMapWidth, newMapHeight)

    # publish the new grid
    newPublishedGrid = OccupancyGrid()
    newPublishedGrid.info.resolution = desiredMapRes 
    newPublishedGrid.info.origin.position.x= data.info.origin.position.x
    newPublishedGrid.info.origin.position.y= data.info.origin.position.y
    newPublishedGrid.info.width = newMapWidth
    newPublishedGrid.info.height = newMapHeight
    newPublishedGrid.data = tuple(newOccGrid)
    print "publishing map"


    scaledMapPub.publish(newPublishedGrid)

    # publish the amount that the obstacles were expanded by
    expansionVal = Float64()
    expansionVal.data = expansionWidth

    obstacleExpansionDimensionPub.publish(expansionVal)

    workingOnMap = False


def frontier_ready(data): 
    if (data.data):
        global frontierReady 
        frontierReady = True


if __name__== "__main__":

    rospy.init_node('map_expander')
   

    global workingOnMap
    workingOnMap = False

    
    global desiredMapRes
    global currentMapRes

    global mainMapSub
    global scaledMapPub
    global obstacleExpansionDimensionPub

    obstacleExpansionDimensionPub = rospy.Publisher('expansion_size', Float64, queue_size = 3)

    global expansionWidth
    expansionWidth = 0.4 # set to even multiple of desiredMapRes
    
    desiredMapRes = 0.2
    # set deisredMap res to some multiple of actual map res


    global frontierReady
    frontierReady = False

    mapNodePub = rospy.Publisher('map_node_ready', Bool, queue_size=3)

    global frontierCalcSub
    
    ready = Bool()
    ready.data = False
    frontiersPub = rospy.Publisher('frontier_node_ready', Bool, queue_size=3)
    frontierCalcSub = rospy.Subscriber('frontier_node_ready', Bool, frontier_ready, queue_size=3)

    while (not frontierReady) and (not rospy.is_shutdown()):
        1+1
    frontiersPub.publish(ready)

    scaledMapPub = rospy.Publisher('res_changed_map', OccupancyGrid, queue_size=2)
    modifiedMapSub = rospy.Subscriber('res_changed_map', OccupancyGrid, checkMapReceived, queue_size = 3)
    mainMapSub = rospy.Subscriber('map', OccupancyGrid, read_map, queue_size = 3)

    expansionVal = Float64()
    expansionVal.data = expansionWidth

    obstacleExpansionDimensionPub.publish(expansionVal)
    print "start waiting for map"

    rospy.spin()
    print "exiting"
