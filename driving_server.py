#!/usr/bin/env python

# Amanda Adkins
# Alex Caracappa


import rospy
import time

from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from math import pi
from math import sqrt
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion
from final_project_adkins_caracappa.srv import *
import tf
from std_msgs.msg import Bool


#Bumper Event Callback function
# set a global indicating if the center of the bumper is pressed
def readBumper(msg):
    if (msg.state == 1):
        # if want robot to stop when bumper is hit, uncomment next line
        global bumperState
        bumperState = 1
    print "bumper hit"

#Odometry Callback function.
# saves the needed odometry data to globals
def read_odometry(data):
    px = data.pose.pose.position.x
    py = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]

    roll, pitch, yaw = euler_from_quaternion(q)

    global odomX
    global odomY
    global odomTheta

#    odomX = px
#    odomY = py
    odomTheta = yaw

    odomFramePoint = PointStamped()
    tol = 0.05
    odomFramePoint.header.frame_id = "/odom"
    odomFramePoint.point.x = px
    odomFramePoint.point.y = py
    odomFramePoint.point.z = 0

    mapFramePoint = PointStamped()

    mapListener.waitForTransform("/map", "/odom", rospy.Time(), rospy.Duration(20))
    mapFramePoint = mapListener.transformPoint("map", odomFramePoint)

    odomX = mapFramePoint.point.x
    odomY = mapFramePoint.point.y 


# takes in a DriveStraight srv with the speed and the distance to drive
# speed should always be positive
# distance to drive can be positive (driving forward) or negative (backing up)
def driveStraight(req):
    global bumperState
    speed = req.driveSpeed
    distance = req.driveDist

    if (distance < 0):
        speed = abs(speed) * -1
        distance = abs(distance)
        print "backing up"
    else:
        speed = abs(speed)

    print "distance %f" % (distance)
    print "speed %f" % (speed)
    poleRate = 0.1      # rate at which robot checks to see if it has reached goal
    tol = 0.05         # give robot 5 cm tolerance so that it doesn't miss appropriate 
                        # window and keep driving because poleRate is to slow to be exact

    global odomX # measured x position in global frame
    global odomY # measured y position in global frame
    print "figuring out start"
    xStart = odomX
    yStart = odomY

    # compute the distance that has been traveled so far
    distTraveled = sqrt((xStart - odomX)**2 + (yStart - odomY)**2)

    driveSuccessful = True

    # while the robot hasn't driven far enough, keep driving
    print "start running drive straight loop"
    while (not ((abs(distTraveled) > abs((distance - tol))) and (abs(distTraveled) < abs((distance + tol))))):
        # if ctrl c is pressed, stop the robot and break from the loop
        if rospy.is_shutdown():
            publishTwist(0,0)
            break
        # keep driving with zero angular velocity
        distTraveled = sqrt((xStart - odomX)**2 + (yStart - odomY)**2)
        # if the bumper hasn't been pressed, 
        if bumperState == 0:  
            publishTwist(speed, 0)
        else:
            driveSuccessful = False
            break
        time.sleep(poleRate)
    bumperState = 0

    # stop the robot when it finishes travelling the indicated distance
    for i in range(3):
        publishTwist(0,0)
        time.sleep(1)

    if driveSuccessful:
        return DriveStraightResponse(0)
    else:
        return DriveStraightResponse(1)

# Accepts an angle and makes the robot rotate around itself 
# angle is counter clockwise (left) in radians
# robot rotates around its center changing its orientation by 'angle' radians
def rotate(req):
    global bumperState
    angle = req.rotateAngle
    tol = 0.15 # tolerance of ~5 degrees
    poleRate = 0.1 # rate at which to send robot Twist messages
    angularVel = 1 # angular velocity of robot
    linearVel = 0 # robot is simply rotating around its center and should have no linear motion
    # if the robot should turn right, make the robot rotate the other direction by switching the sign of the angular velocity
    if (angle < 0):
        angularVel = angularVel * -1

    startAngle = odomTheta
    # determine the goal
    goalAngle = startAngle + angle

    # if the goal is outside the range that the direction measurements are returned in
    # add or subtract 2pi from it to put it back in range
    if goalAngle > pi:
        goalAngle = goalAngle - 2*pi
    elif goalAngle < (pi * -1):
        goalAngle = goalAngle + 2*pi

    driveSuccessful = True

    # drive the robot while it is still not oriented correctly (has not rotated enough)
    while (odomTheta < (goalAngle - tol)) or (odomTheta > (goalAngle + tol)):
        if rospy.is_shutdown():
            publishTwist(0,0)
            break
        if bumperState == 0:   
            publishTwist(linearVel, angularVel)
            time.sleep(poleRate)
        else:
            driveSuccessful = False 
            break

    for i in range(3):
        publishTwist(0,0)
        time.sleep(poleRate)

    bumperState = 0
    if driveSuccessful:
        return RotateResponse(0)
    else:
        return RotateResponse(1)


# publishes twist messages with u assigned to linear speed in the x direction and w assigned to angular z speed
def publishTwist(u, w):
    global pub

    twist = Twist()
    twist.linear.x = u
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = w
    twistPub.publish(twist)

# function to start driving server functions
def driving_server():

    driveService = rospy.Service('drive_straight', DriveStraight, driveStraight)
    rotateService = rospy.Service('rotate', Rotate, rotate)
    facePointService = rospy.Service('face_point', FacePoint, driveUntilFacingPoint)

    rospy.spin()

# drive unti lthe robot is facing the given point
def driveUntilFacingPoint(req):

    slowDownVal = 0.2
    poleRate = 0.075
    speed = 1
    angularVel = speed
    tol = 0.025
    maxProportionalComp = 0.5

    # set a point with the point to face it the map frame
    mapFrameGoal = PointStamped()
    mapFrameGoal.header.frame_id = "/map"
    mapFrameGoal.point.x = req.x
    mapFrameGoal.point.y = req.y
    mapFrameGoal.point.z = 0

    # get the point in the robot's frame
    global mapListener
    mapListener = tf.TransformListener()

    goalInRobotFrame = PointStamped()

    mapListener.waitForTransform("base_footprint", "map", rospy.Time(), rospy.Duration(20))
    goalInRobotFrame = mapListener.transformPoint("base_footprint", mapFrameGoal)

    driveSuccessful = True

    # while the goal in the robot's frame doesn't have a near zero y component
    # (while the robot isn't looking right at the point)
    while (abs(goalInRobotFrame.point.y) > tol):
        # turn the robot the counter clockwise if the y coordinate is positive (in front of the robot)
        if (goalInRobotFrame.point.y > 0):
            angularVel = speed
        # turn the robot clockwise if the y coordinate is negative (behind the robot)
        else:
            angularVel = -1*speed

        if (abs(goalInRobotFrame.point.y) < slowDownVal):
            # proportional control when within slowDownVal
            angularVel = angularVel*(1-maxProportionalComp) + maxProportionalComp*angularVel*(abs((goalInRobotFrame.point.y)/slowDownVal))
       
        # publish a twist message
        if bumperState != 0:   
            driveSuccessful = False 
            break

        publishTwist(0, angularVel)
        time.sleep(poleRate)

        goalInRobotFrame = PointStamped()
        # get the point in the robot's frame
        mapListener.waitForTransform("/base_footprint", "/map", rospy.Time(), rospy.Duration(3.0))
        goalInRobotFrame = mapListener.transformPoint("base_footprint", mapFrameGoal)
        
    for i in range(3):
        publishTwist(0,0)

    if driveSuccessful:
        return FacePointResponse(0)
    else:
        return FacePointResponse(1)

def goal_nav_ready(data):
    global goalReady
    goalReady = True

def frontier_ready(data): 
    global frontierReady 
    frontierReady = True

def astar_ready(data):
    global aStarReady 
    aStarReady = True
    
def map_ready(data):
    global mapReady 
    mapReady = True
    

if __name__== "__main__":
    rospy.init_node('driving_server')

    global goalReady
    global frontierReady 
    global aStarReady 
    global mapReady 

    goalReady = False
    frontierReady = False
    aStarReady = False
    mapReady = False

    global drivingNodePub 
    drivingNodePub = rospy.Publisher('driving_node_ready', Bool, queue_size=3)

    global goalNavSub
    global frontierCalcSub
    global aStarSub
    global mapSub

    goalNavSub = rospy.Subscriber('goalnav_node_ready', Bool, goal_nav_ready, queue_size=3)
    frontierCalcSub = rospy.Subscriber('frontier_node_ready', Bool, frontier_ready, queue_size=3)
    aStarSub = rospy.Subscriber('astar_node_ready', Bool, astar_ready, queue_size=3)
    mapSub = rospy.Subscriber('map_node_ready', Bool, map_ready, queue_size=3)

    time.sleep(5)

    ready = Bool()
    ready.data = True
    drivingNodePub.publish(ready)
    print "waiting for other nodes"
    while (not (goalReady and frontierReady and aStarReady and mapReady)) and (not rospy.is_shutdown()):
        1+1
    print "done waiting"

    global twistPub

    global bumperState
    bumperState = 0

    bumper_sub = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

    global mapListener
    mapListener = tf.TransformListener()


    # set up the publishers and subscribers for the twist messages, odometry messages, and bumper event messages
    sub = rospy.Subscriber("odom", Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
    twistPub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size =1 ) # Publisher for commanding robot motion
    
    driving_server()
