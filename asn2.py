#!/usr/bin/env python
import roslib
import rospy
from fw_wrapper.srv import *
from map import *

# -----------SERVICE DEFINITION-----------
# allcmd REQUEST DATA
# ---------
# string command_type
# int8 device_id
# int16 target_val
# int8 n_dev
# int8[] dev_ids
# int16[] target_vals

# allcmd RESPONSE DATA
# ---------
# int16 val
# --------END SERVICE DEFINITION----------

# ----------COMMAND TYPE LIST-------------
# GetMotorTargetPosition
# GetMotorCurrentPosition
# GetIsMotorMoving
# GetSensorValue
# GetMotorWheelSpeed
# SetMotorTargetPosition
# SetMotorTargetSpeed
# SetMotorTargetPositionsSync
# SetMotorMode
# SetMotorWheelSpeed


LEFT_SHOULDER, RIGHT_SHOULDER = 3, 2
BACK_LEFT_LEG, BACK_RIGHT_LEG = 7, 6
HEAD = 1
PORT = 1

# FOR REPRESENTING ROBOT POSITION
POSITION = [0, 0, DIRECTION.South]
I, J, K = 0, 1, 2

# HARD-CODED MAP FOR PARTS I, II, & III
starter_map = EECSMap()


# wrapper function to call service to set a motor mode
# 0 = set target positions, 1 = set wheel moving
def setMotorMode(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorMode', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get motor wheel speed
def getMotorWheelSpeed(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor wheel speed
def setMotorWheelSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorWheelSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor target speed
def setMotorTargetSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get sensor value
def getSensorValue(port):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetSensorValue', port, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set a motor target position
def setMotorTargetPositionCommand(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get a motor's current position
def getMotorPositionCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to check if a motor is currently moving
def getIsMotorMovingCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetIsMotorMoving', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    
# TODO implement this correctly: wrapper function to call service to sync motor wheel speed
def syncMotorWheelSpeeds(motor_ids, target_vals):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        #resp1 = send_command('SetWheelSpeedSync', motor_ids, target_vals, 0, [0], [0])

        # resp1 = send_command('SetWheelSpeedSync', motor_ids[0], target_vals[0], motor_ids[1], target_vals[1], 0)
        resp1 = send_command('SetWheelSpeedSync', motor_ids[0], target_vals[0], 1, motor_ids[1:], target_vals[1:])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


# *****************************************************************
# ************************* OUR FUNCTIONS *************************

# TODO: Turn by the specified degrees (-90, 90, and 180 for now)
def turn_degrees(degrees):
    # MOTOR WHEEL VALUES
    # COUNTER-CLOCKWISE: 0 to 1023
    # CLOCKWISE: 1024 to 2047
    if degrees == 90:
        setMotorMode(LEFT_SHOULDER, 1)
        setMotorMode(BACK_LEFT_LEG, 1)
        for i in range(8):
            setMotorWheelSpeed(LEFT_SHOULDER, 900)
            setMotorWheelSpeed(BACK_LEFT_LEG, 900)
            # setMotorWheelSpeed(RIGHT_SHOULDER, 0)
            # setMotorWheelSpeed(BACK_RIGHT_LEG, 0)

        while 1 == getIsMotorMovingCommand(LEFT_SHOULDER) == getIsMotorMovingCommand(BACK_LEFT_LEG):
            print("left motors still moving")
            pass

        # TODO currently having problem where it jolts back; to fix, maybe set the speed to 0 instead of resetting the entire mode
        setMotorMode(LEFT_SHOULDER, 0)
        setMotorMode(BACK_LEFT_LEG, 0)
    elif degrees == -90:
        setMotorMode(RIGHT_SHOULDER, 1)
        setMotorMode(BACK_RIGHT_LEG, 1)
        for i in range(8):
            setMotorWheelSpeed(RIGHT_SHOULDER, 1024 + 900)
            setMotorWheelSpeed(BACK_RIGHT_LEG, 1024 + 900)
            # setMotorWheelSpeed(LEFT_SHOULDER, 0)
            # setMotorWheelSpeed(BACK_LEFT_LEG, 0)

        while 1 == getIsMotorMovingCommand(RIGHT_SHOULDER) == getIsMotorMovingCommand(BACK_RIGHT_LEG):
            print("right motors still moving")
            pass

        setMotorMode(RIGHT_SHOULDER, 0)
        setMotorMode(BACK_RIGHT_LEG, 0)
        # setMotorMode(BACK_RIGHT_LEG, 0)
        # setMotorMode(RIGHT_SHOULDER, 0)



# Turn to the given direction and update heading accordingly
def turn(dir):
    # main idea: if we're south and need to go north, turn 180; vice versa; same idea applies for east/west
    # otherwise turn +/- 90 to desired heading?
    NORTH_FACING = {DIRECTION.North: 0, DIRECTION.South: 180, DIRECTION.East: 90, DIRECTION.West: -90}
    SOUTH_FACING = {DIRECTION.North: 180, DIRECTION.South: 0, DIRECTION.East: -90, DIRECTION.West: 90}
    EAST_FACING = {DIRECTION.North: -90, DIRECTION.South: 90, DIRECTION.East: 0, DIRECTION.West: 180}
    WEST_FACING = {DIRECTION.North: 90, DIRECTION.South: -90, DIRECTION.East: 180, DIRECTION.West: 0}

    if POSITION[K] == DIRECTION.North:
        turn_degrees(NORTH_FACING[dir])
    elif POSITION[K] == DIRECTION.South:
        turn_degrees(SOUTH_FACING[dir])
    elif POSITION[K] == DIRECTION.East:
        turn_degrees(EAST_FACING[dir])
    elif POSITION[K] == DIRECTION.West:
        turn_degrees(WEST_FACING[dir])
    else:
        print("turn: invalid heading for K: " + str(POSITION[K]))
        return
    # update heading
    POSITION[K] = dir



# TODO: Walk one cell in the given direction
# from map.py: DIRECTION = enum(North=1, East=2, South=3, West=4)
def walk_one_cell(dir):
    # if robot is not already facing given direction, we need to turn in that direction
    if POSITION[K] != dir:
        turn(dir)

    # walk forward (if there is no obstacle there, we can go)
    if starter_map.getNeighborObstacle(*POSITION) == 0:
        # TODO find out how many iterations it takes to walk 1 cell
        setMotorWheelSpeed(LEFT_SHOULDER, 1023)
        setMotorWheelSpeed(RIGHT_SHOULDER, 1023 + 1000)  # maybe set this to 2047
        setMotorWheelSpeed(BACK_LEFT_LEG, 1023)
        setMotorWheelSpeed(BACK_RIGHT_LEG, 1023 + 1000)  # maybe set this to 2047

        # Update position: new (i, j) will depend on curr (i, j, k)
        # basically either add or subtract 1 to either i or j
        if dir == DIRECTION.North:
            POSITION[I] -= 1
        elif dir == DIRECTION.South:
            POSITION[I] += 1
        elif dir == DIRECTION.West:
            POSITION[J] -= 1
        elif dir == DIRECTION.East:
            POSITION[J] += 1
    else:
        print("walk_one_cell: cannot move, obstacle found at " + str(POSITION))


def test_localization():
    # (0, 0, 0) to (3, 0, 0)
    walk_one_cell(DIRECTION.South)
    walk_one_cell(DIRECTION.South)
    walk_one_cell(DIRECTION.South)
    walk_one_cell(DIRECTION.South)


    # (0, 0, 0) to (0, 3, 0)
    walk_one_cell(DIRECTION.East)
    walk_one_cell(DIRECTION.East)
    walk_one_cell(DIRECTION.East)
    walk_one_cell(DIRECTION.East)


    # (0, 0, 0) to (2, 2, 0)
    walk_one_cell(DIRECTION.South)
    walk_one_cell(DIRECTION.South)
    walk_one_cell(DIRECTION.East)
    walk_one_cell(DIRECTION.East)


def planning():
    # 1. be able to accept starting and ending positions from command line
    start = input("Enter start position: ")
    goal = input("Enter goal position: ")

    # parse inputs (assumes values separated by spaces, e.g. '0 0 0')
    try:
        start = list(map(int, start.split()))
        goal = list(map(int, goal.split()))
    except:
        print("planning: could not parse start and/or goal")
        return

    # 2. set the cost of each grid cell

    # 3. generate path from start to goal

    # 4. generate command sequence

    # 5. walk path



# Main function
if __name__ == "__main__":
    """
    *** TODO BEFORE FEB 6 ***
    LOCALIZATION:
    - maybe look more into the SetWheelSpeedSync command from the srv_wrapper
    
    PLANNING:
    - look into algorithms we could use to set costs and generate paths
    
    *** TODO ON FEB 6 ***
    LOCALIZATION:
    - find how many wheel iterations it takes to travel from center of one cell to the center of the next
        - test with test_localization(), measure error, and optimize
    - fix turning:
        - find how many wheel iterations it takes to turn left, right, and around
        - find out why it jolts back at the end of a turn
        - in order to stay centered in cell by the end of a turn, we may have to back up a little bit before turning, so we should find out by how much
    """
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")

    # Print obstacle map
    starter_map.printObstacleMap()
    #test_localization()

    count = 0

    # control loop running at 10hz
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        # walk_one_cell(DIRECTION.South)

        # print("trying to move wheels...")
        # setMotorMode(BACK_LEFT_LEG, 1)
        # setMotorMode(BACK_RIGHT_LEG, 1)
        # setMotorMode(LEFT_SHOULDER, 1)
        # setMotorMode(RIGHT_SHOULDER, 1)


        # Counting how long it takes to turn right
        # print("the count is " + str(count))
        # setMotorWheelSpeed(LEFT_SHOULDER, 1023)
        # setMotorWheelSpeed(RIGHT_SHOULDER, 0)
        # setMotorWheelSpeed(BACK_LEFT_LEG, 1023)
        # setMotorWheelSpeed(BACK_RIGHT_LEG, 0)
        # count += 1

        #syncMotorWheelSpeeds([BACK_LEFT_LEG, BACK_RIGHT_LEG], [1000, 1000])

        turn_degrees(-90) # turn left
        #turn_degrees(90)  # turn right, seems to be working well but currently halts back at the end of the turn
        #turn_degrees(180)  # turn around
        break

        # sleep to enforce loop rate
        r.sleep()

