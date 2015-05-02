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
from math import pi
from std_msgs.msg import Float64

from geometry_msgs.msg import PointStamped
import tf
from std_msgs.msg import Bool
from subprocess import call


def read_odometry(data):
    global robotX
    global robotY
    # robotX = data.pose.pose.position.x
    # robotY = data.pose.pose.position.y


    odomFramePoint = PointStamped()
    tol = 0.05
    odomFramePoint.header.frame_id = "/odom"
    odomFramePoint.point.x = data.pose.pose.position.x
    odomFramePoint.point.y = data.pose.pose.position.y
    odomFramePoint.point.z = 0

    mapFramePoint = PointStamped()

    mapListener.waitForTransform("/map", "/odom", rospy.Time(), rospy.Duration(20))
    mapFramePoint = mapListener.transformPoint("map", odomFramePoint)

    robotX = mapFramePoint.point.x
    robotY = mapFramePoint.point.y 


# call back function for the map
# saves the map resolution, origin, map dimensions, and occupancy data as globals 
# for access within other functions
def read_occupancy_grid(data):
    global mapReceived

    global mapRes
    mapRes = data.info.resolution
    print "map res set to %f" % (mapRes)

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
    print "recvd"
    # display the obstacles on the map
    displayObstacles()

# split a 2 part tuple up
def splitCellTuple(cellTup):
	return cellTup[0], cellTup[1]

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

# call back function for inital pose 
# sets that the start position has been select4ed and sets the cell number of the initial position in startPos
def read_initialpose(data):
    global startAdded
    global startPos

    xCoord = data.pose.pose.position.x
    yCoord = data.pose.pose.position.y

 	# start pos is tuple
    startPos = getClosestCell(xCoord, yCoord)
    startAdded = True

# this function takes a list of points and publishes a GridCells object with those points
# to a topic corresponding the the given number
# the numbers relate to the topics as follows:
# 1 - gridCellPub
# 2 - exploredCellsPub
# 3 - frontierCellsPub
# 4 - wayPointsPub
# 5 - pathPub
# 6 -s goalCellPub
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
    elif topicNum == 6:
        goalCellPub.publish(gridCells)

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

# given a path between 2 nodes, reduce it to the waypoints of the path
def reducePath(path):
    # initialize the new path
    newPath = []
    newPath.append(path[0])
    newPath.append(path[1])
    # how often the robot should stop to replan 
    # max distance without planning will be approx 0.4 m
    stopDist = 1

    # go through the full path and eliminate all nodes that are in the same direction
    # except the last in a given direction
    for i in range(2, len(path)):
        if rospy.is_shutdown():
            break
        # get the cell numbers and coordinates of the 3 latest cells
        lastPoint = newPath[len(newPath) - 1]
        pointBefore = newPath[len(newPath) - 2]
        currentPoint = path[i]

        # get the x and y coordinates of the current point, point before that, and 2 points before the current path
        x0, y0 = splitCellTuple(pointBefore)
        x1, y1 = splitCellTuple(lastPoint)
        x2, y2 = splitCellTuple(currentPoint)

        # if the difference in x coordinates or the difference in y coordinates are the same
        # remove the intermedidate cell because it isn't a corner
        # and add the most recent cell (treating it as a potential waypoint)
        oldXDiff = x1 - x0
        oldYDiff = y1 - y0

        newXDiff = x2 - x1
        newYDiff = y2 - y1

        print "oldXDiff %f, oldYDiff %f" %(oldXDiff, oldYDiff)

        # if the robot was moving in the y direction between both sets of points
        if oldXDiff == 0 and newXDiff == 0:
            # if the past waypoint was more than 0.4 m from the prior, add a new way point
            if (abs(oldYDiff) > stopDist):
                newPath.append(currentPoint)
                print "longer than 1 m, new way point"
            # if not, replace the last waypoint with the current point
            else: 
                newPath[len(newPath)-1] = currentPoint
                print" replacing way point"
        # if the robot will move in the x direction between both sets of points
        elif oldYDiff == 0 and newYDiff == 0:
            # if the past waypoint was more than 0.4 m from the prior, add a new way point
            if (abs(oldXDiff) > stopDist):
                newPath.append(currentPoint)
                print "longer than 1 m, new way point"
            # if not, replace the last waypoint with the current point
            else:
                newPath[len(newPath)-1] = currentPoint
                print" replacing way point"
        # if gets to this point, the robot is traveling diagonally
        # if the robot wasn't traveling purely in the X direction for either set of path points
        elif (oldYDiff != 0) and (newYDiff != 0):
            # if the robot is traveling in the same diagonal
            if (oldXDiff/oldYDiff) == (newXDiff/newYDiff):
                # still traveling in diagonal
                # if the distance between the prior waypoint and waypoint before that is greater than 
                # the stop distance, add a new waypoint
                if (sqrt(oldXDiff**2 + oldYDiff**2) > stopDist):
                    newPath.append(currentPoint)
                    print "longer than 1 m, new way point"
                # if not, replace the old way point
                else:
                    newPath[len(newPath)-1] = currentPoint
                    print" replacing way point"
            # different diagonal, new way point
            else:
                newPath.append(currentPoint)
        # different direction, new way point
        else:
            newPath.append(currentPoint)
    print newPath
    # return the reduced path
    return newPath

# takes in a list of numbers corresponding to cells and 
# a number representing which topic to publish them to
# 1 - gridCellPub
# 2 - exploredCellsPub
# 3 - frontierCellsPub
# 4 - wayPointsPub
# 5 - pathPub
# 6 - goalCellPub
def updateGridCells(tupleList, topicNum):
    # convert the list of cell numbers to points
    listOfPoints = []
    for i in tupleList:
        xPos, yPos = splitCellTuple(i)
        listOfPoints.append(makePoint(xPos, yPos))
    # publish the points as grid cells
    publishGridCells(listOfPoints, topicNum)

# make a point object from an x and y coordinate
def makePoint(xCoord, yCoord):
    newPoint = Point()
    newPoint.x = xCoord
    newPoint.y = yCoord
    return newPoint

# this moves the robot to the waypoint with the given number, 
# this function assusmes that the robot is facing the correct direction
def moveBetweenWaypoints(currentCell, nextCell):
    currentX = robotX
    currentY = robotY
    #currentX, currentY = splitCellTuple(currentCell)
    waypointX, waypointY = splitCellTuple(nextCell)
    # should have robot X and robot Y at this point
    # xDiff = (waypointX - robotX)**2
    # yDiff = (waypointY - robotY)**2

    xDiff = (waypointX - currentX)**2
    yDiff = (waypointY - currentY)**2

    speed = 0.2
    distToTravel = sqrt(xDiff + yDiff)
    print "distance is %f " % distToTravel
    print "start waiting for service"
    rospy.wait_for_service('drive_straight')
    driveFunction = rospy.ServiceProxy('drive_straight', DriveStraight)
    print "start using service"
    resp = driveFunction(distToTravel, speed)
    if (resp.status == 1):
        print "backing up"
        backupDist = -0.5
        rospy.wait_for_service('drive_straight')
        driveFunction = rospy.ServiceProxy('drive_straight', DriveStraight)
        print "start using service"
        resp = driveFunction(backupDist, speed)
        
# change the angle between waypoints
# comingFrom is the waypoint that the robot just came from
# atThisPoint is the waypoint that the robot is at
# goingTo is the waypoint that the robot is going to
# this uses the angle between 2 vectors (the paths between the waypoints)
# to figure out how much to turn
def changeAngleBetweenWaypoints(comingFrom, atThisPoint, goingTo):
    # get the x and y coordinates
    x0, y0 = splitCellTuple(comingFrom)
    x1, y1 = splitCellTuple(atThisPoint)
    x2, y2 = splitCellTuple(goingTo)

    # figure out vectors that would represent the paths between the vectors
    oldxdif = x1 - x0
    oldydif = y1 - y0

    newxdif = x2 - x1
    newydif = y2 - y1


    if (oldxdif > 0) and (oldydif > 0): # was NE
        oldDir = 7
    elif (oldxdif > 0) and (oldydif == 0): # was E
        oldDir = 6
    elif (oldxdif > 0) and (oldydif < 0): # was SE
        oldDir = 5
    elif (oldxdif == 0) and (oldydif < 0): # was S
        oldDir = 4
    elif (oldxdif < 0) and (oldydif < 0): # was SW
        oldDir = 3
    elif (oldxdif < 0) and (oldydif == 0): # was W
        oldDir = 2
    elif (oldxdif < 0) and (oldydif > 0): # was NW
        oldDir = 1
    else: # was N
        oldDir = 0

    if (newxdif > 0) and (newydif > 0): # going NE
        newDir = 7
    elif (newxdif > 0) and (newydif == 0): # going E
        newDir = 6
    elif (newxdif > 0) and (newydif < 0): # going SE
        newDir = 5
    elif (newxdif == 0) and (newydif < 0): # going S
        newDir = 4 
    elif (newxdif < 0) and (newydif < 0): # going SW
        newDir = 3
    elif (newxdif < 0) and (newydif == 0): # going W
        newDir = 2 
    elif (newxdif < 0) and (newydif > 0): # going NW
        newDir = 1
    else: # going N
        newDir = 0

    totalDir = newDir - oldDir

    if (totalDir > 4):
        totalDir = totalDir - 8
    elif (totalDir < -4):
        totalDir = totalDir + 8

    changeAngle = totalDir*pi/4
    
    if (changeAngle != 0):
        rospy.wait_for_service('rotate')
        rotate = rospy.ServiceProxy('rotate', Rotate)
        response = rotate(changeAngle)

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

# get the cell's reference number from its coordinates
def makeNumFromXY(x, y):
    xOffset = xOriginMap + (mapRes/2)
    yOffset = yOriginMap + (mapRes/2)

    number = mapWidth*round(((y-yOffset)/mapRes), 0) + round(((x - xOffset)/mapRes), 0)

    return int(number)

def driveWaypointsInitial(robotPath):
    print "starting to go to goal"
    currentNode = robotPath[0]
    goalNode = robotPath[len(robotPath)-1]
    nextWayPoint = robotPath[1]

    # correctly orient robot for first move
    rospy.wait_for_service('face_point')
    face_point = rospy.ServiceProxy('face_point', FacePoint)
    returnVal = face_point(nextWayPoint[0], nextWayPoint[1]) 

    # set a tolerance for the robot    
    withinXTol = ((goalNode[0] - mapRes) < currentNode[0]) and ((goalNode[0] + mapRes) > currentNode[0])
    withinYTol = ((goalNode[1] - mapRes) < currentNode[1]) and ((goalNode[1] + mapRes) > currentNode[1]) 
    print "starting to drive"
    # if not within the tolerance
    while not (withinXTol and withinYTol):
        # display where the robot is and where the goal is
        updateGridCells([currentNode, goalNode], 6)
        print "moving between nodes"
        # drive to the next waypoint
        moveBetweenWaypoints(currentNode, nextWayPoint)

        if (occGrid[makeNumFromXY(goalNode[0], goalNode[1])] != 0):
                break
       
        # update the notes
        prevNode = currentNode
        lastRecordedNode = nextWayPoint
        # get the closest cell to where the robot is
        currentNode = getClosestCell(robotX, robotY)

        # check if within tolerance
        withinXTol = ((goalNode[0] - 2*mapRes) < currentNode[0]) and ((goalNode[0] + 2*mapRes) > currentNode[0])
        withinYTol = ((goalNode[1] - 2*mapRes) < currentNode[1]) and ((goalNode[1] + 2*mapRes) > currentNode[1])
        # if withint he tolreance, don't execute the rest of the loop
        if withinXTol and withinYTol:
            print "all done"
            break
        # print the current node adn goal node
        print currentNode
        print goalNode
        # run Astar again
        rospy.wait_for_service('a_star')        
        a_star = rospy.ServiceProxy('a_star', Astar)
        returnVal = a_star(currentNode[0], currentNode[1], goalNode[0], goalNode[1])
        path = fixPath(returnVal.pathX, returnVal.pathY)
        # display the path on the map
        updateGridCells(path, 5)
        # reduce the path to waypoints
        robotPath = reducePath(path)

        updateGridCells(robotPath, 4)

        # get the next waypoint to go to
        nextWayPoint = robotPath[1]
        print "turning"
        # turn until faceing that waypoint
        rospy.wait_for_service('face_point')
        face_point = rospy.ServiceProxy('face_point', FacePoint)
        returnVal = face_point(nextWayPoint[0], nextWayPoint[1])  

        #changeAngleBetweenWaypoints(prevNode, lastRecordedNode, nextWayPoint) 

# merge a two lists of x and y values into a list of (x,y) tuples
def fixPath(xVals, yVals):
    tuplesList = []
    numTuples = len(xVals)
    for i in range(numTuples):
        newTuple = (xVals[i], yVals[i])
        tuplesList.append(newTuple)
    return tuplesList


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


def read_map(data):
    print "map received"

# display the obstacles on the map
def displayObstacles():
    obstacles = []
    for i in range(len(occGrid)):
        if occGrid[i] >= 50:
            obstacles.append(getXY(i))
    if pubsInitialized:
        updateGridCells(obstacles, 4)

def moveToGoal(req):
    print "going to goal"
    goalPos = req.xGoal, req.yGoal
    startPos = getClosestCell(robotX, robotY)
    print startPos
    print goalPos
    updateGridCells([startPos, goalPos],6)
    print "waiting for a star"
    rospy.wait_for_service('a_star')
    a_star = rospy.ServiceProxy('a_star', Astar)
    updateGridCells([startPos, goalPos],6 )
    print "running a star"
    returnVal = a_star(startPos[0], startPos[1], goalPos[0], goalPos[1])
    if (occGrid[makeNumFromXY(startPos[0], startPos[1])] < 50): 
        if (occGrid[makeNumFromXY(goalPos[0], goalPos[1])] < 50):
            if (returnVal.throughObs == 1):
                return GoToGoalResponse(0)
    path = fixPath(returnVal.pathX, returnVal.pathY)
    updateGridCells(path, 5)
    # calculate the way points of the path
    wayPoints = reducePath(path)
    print "trying to drive"
    # start driving along the waypoints
    driveWaypointsInitial(wayPoints)
    return GoToGoalResponse(1)

    
def goal_nav_server():
    goalNavService = rospy.Service('goal_nav_service', GoToGoal, moveToGoal)
    rospy.spin()

def astar_ready(data):
    if (data.data):
        global aStarReady
        aStarReady = True

def goalNavResp(data):
    if (not data.data):
        global frontierRespRecvd
        frontierRespRecvd = True
    


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('main_code_for_lab4')

    global mapReceived
    mapReceived = False

    global mapListener
    mapListener = tf.TransformListener()

    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    global gridCellPub
    global exploredCellsPub
    global frontierCellsPub
    global wayPointsPub
    global pathPub
    global expandedMapPub
    global obstacleExpansionDimensionPub
    global goalCellPub

    global pubsInitialized
    pubsInitialized = False
    
    occupancySub = rospy.Subscriber("res_changed_map", OccupancyGrid, read_occupancy_grid, queue_size=1)

    originalMapSub = rospy.Subscriber('map', OccupancyGrid, read_map, queue_size =1)
    
    # set up the publishers and subscribers for the twist messages, odometry messages, and bumper event messages
    sub = rospy.Subscriber("odom", Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
    initialPoseSub = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, read_initialpose, queue_size=1)
#    moveBaseGoal = rospy.Subscriber("move_base_simple/goal_lab", PoseStamped, read_moveBaseGoal, queue_size = 1)
    gridCellPub = rospy.Publisher('grid_for_map', GridCells, queue_size=3) # Publisher for commanding robot motion
    exploredCellsPub = rospy.Publisher('explored_cells', GridCells, queue_size=3)
    frontierCellsPub = rospy.Publisher('frontier_cells', GridCells, queue_size=3)
    wayPointsPub = rospy.Publisher('path_waypoints', GridCells, queue_size = 3)
    pathPub = rospy.Publisher('full_path', GridCells, queue_size=3)
    goalCellPub = rospy.Publisher('goal_cell', GridCells, queue_size=3)

    
    pubsInitialized = True

    global aStarReady
    aStarReady = False
    global frontierRespRecvd
    frontierRespRecvd = False

    global goalNavNodePub 
    goalNavNodePub = rospy.Publisher('goalnav_node_ready', Bool, queue_size=3)
    goalNavNodeSub = rospy.Subscriber('goalnav_node_ready', Bool, goalNavResp, queue_size=3)
    global aStarPub
    aStarPub = rospy.Publisher('astar_node_ready', Bool, queue_size=3)

    ready = Bool()
    ready.data = False

    aStarSub = rospy.Subscriber('astar_node_ready', Bool, astar_ready, queue_size=3)

    while (not aStarReady) and (not rospy.is_shutdown()):
        1+1
    aStarPub.publish(ready)
    ready.data = True

    while (not frontierRespRecvd) and (not rospy.is_shutdown()):
        goalNavNodePub.publish(ready)
        time.sleep(0.1)

    print "waiting for map"
    while (not mapReceived) and (not rospy.is_shutdown()):
        time.sleep(0.01)

    goal_nav_server()
