#!/usr/bin/env python
import random

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
BACK_LEFT_LEG, BACK_RIGHT_LEG = 18, 6
HEAD = 1
IR_PORT, GYRO_PORT = 1, 2

# FOR REPRESENTING ROBOT POSITION
I, J, K = 0, 1, 2

# wrapper function to call service to set a motor mode
# 0 = set target positions, 1 = set wheel moving
def setMotorMode(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorMode', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# wrapper function to call service to get motor wheel speed
def getMotorWheelSpeed(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# wrapper function to call service to set motor wheel speed
def setMotorWheelSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorWheelSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# wrapper function to call service to set motor target speed
def setMotorTargetSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# wrapper function to call service to get sensor value
def getSensorValue(port):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetSensorValue', port, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# wrapper function to call service to set a motor target position
def setMotorTargetPositionCommand(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# wrapper function to call service to get a motor's current position
def getMotorPositionCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# wrapper function to call service to check if a motor is currently moving
def getIsMotorMovingCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetIsMotorMoving', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


#  wrapper function to call service to sync motor wheels
def syncMotorWheelSpeeds(number_ids, motor_ids, target_vals):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetWheelSpeedSync', 0, 0, number_ids, motor_ids, target_vals)
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# ******************************************************************
# ************************** LOCALIZATION **************************

# TODO: Turn by the specified degrees (-90, 90, and 180 for now)
def testing_gyro():
    reading = getSensorValue(GYRO_PORT)
    rospy.loginfo("Sensor value at port %d: %f", GYRO_PORT, reading)
    # syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG), (400,400,400,400))


def calibrate_gyro():
    c = 0
    err = 0
    while (c < 200):
        reading = getSensorValue(GYRO_PORT)
        err += reading
        c += 1

    err = err / 200
    return err


def gyro_turn(degrees, err):
    current_time = rospy.get_time()
    turned = False
    r = rospy.Rate(140)
    angle = 0
    i = 0
    if degrees > 0:
        while (angle < degrees):
            previous_time = current_time
            current_time = rospy.get_time()
            elapsed_time = current_time - previous_time
            accel = (0.4) * (getSensorValue(GYRO_PORT) - err)
            angle -= accel * elapsed_time
            # print(accel)
            print(angle)
            if i == 20:
                i = 0
                t_v = int(200 + (degrees - angle) / degrees * 100)
                target_vals = (t_v, t_v, t_v, t_v)
                syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG), target_vals)
            i += 1
            r.sleep()


def turn_degrees(degrees):
    # MOTOR WHEEL VALUES
    # COUNTER-CLOCKWISE: 0 to 1023
    # CLOCKWISE: 1024 to 2047

    if degrees == 90:
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < 2.55:  # for i in range(8):
            # testing_gyro()
            syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG),
                                 (400, 400, 400, 400))  # TODO maybe 500

        # STOP
        syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG), (0, 0, 1024, 1024))

    elif degrees == -90:
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < 2.5:  # for i in range(8):
            syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG),
                                 (1024 + 400, 1024 + 400, 1024 + 400, 1024 + 400))

        while 1 == getIsMotorMovingCommand(RIGHT_SHOULDER) == getIsMotorMovingCommand(BACK_RIGHT_LEG):
            print("right motors still moving")
            pass

        # STOP
        syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG), (1024, 1024, 0, 0))

    elif degrees == 180:
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < 5.05:  # for i in range(8):
            # testing_gyro()
            syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG),
                                 (400, 400, 400, 400))

        # STOP
        syncMotorWheelSpeeds(4, (LEFT_SHOULDER, BACK_LEFT_LEG, RIGHT_SHOULDER, BACK_RIGHT_LEG), (0, 0, 1024, 1024))


# Turn to the given direction and update heading accordingly, return new position?
def turn(dir, POSITION):
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
    return POSITION


# from map.py: DIRECTION = enum(North=1, East=2, South=3, West=4)
def walk_one_cell(dir, POSITION):
    # if robot is not already facing given direction, we need to turn in that direction
    if POSITION[K] != dir:
        POSITION = turn(dir)

    # walk forward (if there is no obstacle there, we can go)
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < 2.2:  # 1.52:
        # syncMotorWheelSpeeds(4, (LEFT_SHOULDER,RIGHT_SHOULDER, BACK_LEFT_LEG, BACK_RIGHT_LEG), (1023,2047,1023,2047))
        syncMotorWheelSpeeds(4, (LEFT_SHOULDER, RIGHT_SHOULDER, BACK_LEFT_LEG, BACK_RIGHT_LEG),
                             (910, 1024 + 800, 910, 1024 + 800))

    # STOP
    syncMotorWheelSpeeds(4, (LEFT_SHOULDER, RIGHT_SHOULDER, BACK_LEFT_LEG, BACK_RIGHT_LEG), (0, 1024, 0, 1024))

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
        print("walk_one_cell(): couldn't update position")

    return POSITION


def test_localization():
    # remember it matters which way we're looking at the robot & it should start in upper left corner facing south
    # (0, 0, 0) to (3, 0, 0)
    # POSITION = walk_one_cell(DIRECTION.South, POSITION)
    # POSITION = walk_one_cell(DIRECTION.South, POSITION)
    # POSITION = walk_one_cell(DIRECTION.South, POSITION)

    # (0, 0, 0) to (0, 3, 0)
    POSITION = walk_one_cell(DIRECTION.East, POSITION)
    POSITION = walk_one_cell(DIRECTION.East, POSITION)
    POSITION = walk_one_cell(DIRECTION.East, POSITION)

    # (0, 0, 0) to (2, 2, 0)
    # POSITION = walk_one_cell(DIRECTION.South, POSITION)
    # POSITION = walk_one_cell(DIRECTION.East, POSITION)
    # POSITION = walk_one_cell(DIRECTION.South, POSITION)
    # POSITION = walk_one_cell(DIRECTION.East, POSITION)


# ******************************************************************
# **************************** PLANNING ****************************

# Given a map and cell, finds neighboring cells that are not blocked and have not been visited
# Also sets their costs to 1 + curr cell's cost.
# Returns the list of valid neighbors
def find_neighbors(a_map, cell):
    neighbors = []
    i, j = cell[I], cell[J]
    cost = a_map.getCost(i, j)

    if a_map.getNeighborObstacle(i, j, DIRECTION.North) == 0 and a_map.getNeighborCost(i, j, DIRECTION.North) == 0:
        north_neighbor = [i - 1, j, DIRECTION.North]
        a_map.setNeighborCost(i, j, DIRECTION.North, cost + 1)
        neighbors.append(north_neighbor)

    if a_map.getNeighborObstacle(i, j, DIRECTION.South) == 0 and a_map.getNeighborCost(i, j, DIRECTION.South) == 0:
        south_neighbor = [i + 1, j, DIRECTION.South]
        a_map.setNeighborCost(i, j, DIRECTION.South, cost + 1)
        neighbors.append(south_neighbor)

    if a_map.getNeighborObstacle(i, j, DIRECTION.East) == 0 and a_map.getNeighborCost(i, j, DIRECTION.East) == 0:
        east_neighbor = [i, j + 1, DIRECTION.East]
        a_map.setNeighborCost(i, j, DIRECTION.East, cost + 1)
        neighbors.append(east_neighbor)

    if a_map.getNeighborObstacle(i, j, DIRECTION.West) == 0 and a_map.getNeighborCost(i, j, DIRECTION.West) == 0:
        west_neighbor = [i, j - 1, DIRECTION.West]
        a_map.setNeighborCost(i, j, DIRECTION.West, cost + 1)
        neighbors.append(west_neighbor)

    return neighbors


def generate_path(a_map, start, goal):
    path = [goal]
    i, j = goal[I], goal[J]
    cost = a_map.getCost(i, j)

    while True:
        if a_map.getNeighborObstacle(i, j, DIRECTION.North) == 0 and a_map.getNeighborCost(i, j,
                                                                                           DIRECTION.North) == cost - 1:
            next_step = [i - 1, j, DIRECTION.North]
        elif a_map.getNeighborObstacle(i, j, DIRECTION.South) == 0 and a_map.getNeighborCost(i, j,
                                                                                             DIRECTION.South) == cost - 1:
            next_step = [i + 1, j, DIRECTION.South]
        elif a_map.getNeighborObstacle(i, j, DIRECTION.East) == 0 and a_map.getNeighborCost(i, j,
                                                                                            DIRECTION.East) == cost - 1:
            next_step = [i, j + 1, DIRECTION.East]
        elif a_map.getNeighborObstacle(i, j, DIRECTION.West) == 0 and a_map.getNeighborCost(i, j,
                                                                                            DIRECTION.West) == cost - 1:
            next_step = [i, j - 1, DIRECTION.West]
        else:
            print("generate_path(): could not find next step")
            return

        i, j = next_step[I], next_step[J]
        cost = a_map.getCost(i, j)

        # avoid adding start to path; we know we're done backtracking if the next step is start
        if i == start[I] and j == start[J]:
            break

        # add next step to front of list to maintain order
        path.insert(0, next_step)

    return path


# Main algorithm for filling out cost map, returns a path
def wavefront(a_map, start, goal):
    queue = [start]  # list of cells that need to be visited
    a_map.setCost(start[I], start[J], 1)  # have to set origin cost to 1 bc 0 is used for unvisited cells

    while queue:
        cell = queue.pop(0)

        # if at goal, stop and backtrack
        if cell[I] == goal[I] and cell[J] == goal[J]:
            path = generate_path(a_map, start, goal)

        # otherwise get neighbors, set their costs, and add them to list of cells to visit
        else:
            neighbors = find_neighbors(a_map, cell)
            queue += neighbors

    return path


# Given the path to reach a given goal, finds out the steps to take and performs them one by one
def generate_and_walk_commands(path, POSITION):
    print(path)

    for step in path:
        try:
            POSITION = walk_one_cell(direction_to_turn(POSITION, step), POSITION)
        except:
            print("generate_commands(): could not generate command for: " + str(step))
            return

    POSITION = turn(path[len(path) - 1][K])
    return POSITION


def planning(a_map):
    # 1. be able to accept starting and ending positions from command line
    start = raw_input("Enter start position: ")
    goal = raw_input("Enter goal position: ")

    # parse inputs from '0 0 0' into [0, 0, 0]
    try:
        start = list(map(int, start.split()))
        goal = list(map(int, goal.split()))
    except:
        print("planning: could not parse start and/or goal")
        return

    # 2. set the cost of each grid cell and
    # 3. generate path from start to goal
    path = wavefront(a_map, start, goal)

    # 4. generate command sequence and
    # 5. walk path
    generate_and_walk_commands(path, start)


# ******************************************************************
# **************************** MAPPING ****************************
def mapping():
    # always start at 0,0,0; world is always 8x8
    start = [0, 0, DIRECTION.South]
    new_map = EECSMap()
    new_map.clearObstacleMap()
    new_map.clearCostMap()

    # 1. need wandering algoirthm
    explore(new_map, start)

    # 2. need wall recording mechanism
    new_map.printObstacleMap()

def get_lfr_dictionary(curr_heading):
    NORTH_FACING = {"left": [I, J - 1], "front": [I - 1, J], "right": [I, J + 1]}
    SOUTH_FACING = {"left": [I, J + 1], "front": [I + 1, J], "right": [I, J - 1]}
    EAST_FACING = {"left": [I - 1, J], "front": [I, J + 1], "right": [I + 1, J]}
    WEST_FACING = {"left": [I + 1, J], "front": [I, J - 1], "right": [I - 1, J]}

    if curr_heading == DIRECTION.North:
        return NORTH_FACING
    elif curr_heading == DIRECTION.South:
        return SOUTH_FACING
    elif curr_heading == DIRECTION.East:
        return EAST_FACING
    elif curr_heading == DIRECTION.West:
        return WEST_FACING

def explore(a_map, POSITION):
    cells = [[0 for x in xrange(8)] for x in xrange(8)]

    # TODO Start some loop here (not sure of the terminating condition yet)
    # mark current cell as visited (1)
    curr_i, curr_j, curr_k = POSITION
    cells[curr_i][curr_j] = 1

    # scan left, front, and right for obstacles
    left, front, right = detect()
    print("Blocked directions (left, front, right): " + str(left_front_right_blocked))

    # TODO trying to find the best way to represent our left, front, and right neighbors based on our current heading
    # if there is a better way to do this, certainly feel free to change it
    lfr_dict = get_lfr_dictionary(curr_k)

    l_i, l_j = lfr_dict["left"]
    f_i, f_j = lfr_dict["front"]
    r_i, r_j = lfr_dict["right"]

    avail_cells = []

    # TODO MAKE SURE [__i, __j] IS NOT OUT OF BOUNDS OF cells MATRIX
    # if there is wall, mark the cell on the opposite side as -1. Also, set an obstacle there.
    if left == 1:
        cells[l_i][l_j] = -1
        a_map.setObstacle(curr_i, curr_j, 1, direction_to_turn(POSITION, lfr_dict["left"]))
    elif left == 0 and cells[l_i][l_j] != 1:   # if there is no wall and we haven't been there before, mark for later (2)
        cells[l_i][l_j] = 2
        avail_cells.append([l_i, l_j])

    # if there is wall, mark the cell on the opposite side as -1. Also, set an obstacle there.
    if front == 1:
        cells[f_i][f_j] = -1
        a_map.setObstacle(curr_i, curr_j, 1, direction_to_turn(POSITION, lfr_dict["front"]))
    elif front == 0 and cells[f_i][f_j] != 1:  # if there is no wall and we haven't been there before, mark for later (2)
        cells[f_i][f_j] = 2
        avail_cells.append([f_i, f_j])

    # if there is wall, mark the cell on the opposite side as -1. Also, set an obstacle there.
    if right == 1:
        cells[r_i][r_j] = -1
        a_map.setObstacle(curr_i, curr_j, 1, direction_to_turn(POSITION, lfr_dict["right"]))
    elif right == 0 and cells[r_i][r_j] != 1:   # if there is no wall and we haven't been there before, mark for later (2)
        cells[r_i][r_j] = 2
        avail_cells.append([r_i, r_j])


    # now out of our available neighbors (marked as 2), randomly pick one to visit
    # then move to that cell
    if avail_cells:
        next_cell = random.choice(avail_cells)
        POSITION = walk_one_cell(direction_to_turn(POSITION, next_cell), POSITION)

    # if we are only surrounded by visited cells and/or walls, move back to the nearest bookmarked cell (2)
    else:
        # TODO find nearest 2, then find path to it, then move there
        # will need a planning() function that can accept predefined start and goals for this (not just command line input)
        # start is current position, goal is nearest cell with 2 marked there.
        # maybe the nearest cell with a 2 can just be our most recently marked one? not sure how to track that though
        pass

    # see changes made to obstacle map
    a_map.printObstacleMap()

    # repeat
    # end loop here

def direction_to_turn(curr, step):
    if step[I] == curr[I] - 1:
        return DIRECTION.North
    elif step[I] == curr[I] + 1:
        return DIRECTION.South
    elif step[J] == curr[J] + 1:
        return DIRECTION.East
    elif step[J] == curr[J] - 1:
        return DIRECTION.West


""" FROM ASN1 PART 3: Obstacle detection 
Has the robot turn its head and scan from left to right looking for obstacles within 15cm.
Returns a binary list in form [left, front, right] denoting whether an obstacle was found. """


def move_head(direction):
    if direction == "LEFT":
        setMotorTargetPositionCommand(HEAD, 512 - 300)
    elif direction == "RIGHT":
        setMotorTargetPositionCommand(HEAD, 512 + 300)
    elif direction == "FRONT":
        setMotorTargetPositionCommand(HEAD, 512)
    else:
        print("invalid direction")


def detect():
    blocked = [0, 0, 0]

    # check left side
    move_head("LEFT")
    while getIsMotorMovingCommand(HEAD) == 1:
        print("head is still moving ...left?")
    blocked[0] = object_detected()

    # check infront
    move_head("FRONT")
    while getIsMotorMovingCommand(HEAD) == 1:
        print("head is still moving to center")
    blocked[1] = object_detected()

    # check right side
    move_head("RIGHT")
    while getIsMotorMovingCommand(HEAD) == 1:
        print("head is still moving ...right?")
    blocked[2] = object_detected()

    # print(blocked)
    return blocked


# Helper to determine if something is detected within 15cm.
# Currently takes readings for 1 second, determines if it was within 15cm.
# determining distance could probably be improved somehow?
# also IDK if 1 second is enough/too much time
def object_detected():
    end = time.time() + 1
    while time.time() < end:
        reading = getSensorValue(IR_PORT)
        rospy.loginfo("Sensor value at port %d: %f", IR_PORT, reading)
        if reading >= 35:
            return 1

    return 0



# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")

    setMotorMode(BACK_LEFT_LEG, 1)
    setMotorMode(BACK_RIGHT_LEG, 1)
    setMotorMode(LEFT_SHOULDER, 1)
    setMotorMode(RIGHT_SHOULDER, 1)

    # HARD-CODED MAP FOR PARTS I, II, & III
    starter_map = EECSMap()
    starter_map.printObstacleMap()

    # err = 1050 #calibrate_gyro()
    # print(err)

    # control loop running at 10hz
    r = rospy.Rate(140)  # 10hz
    while not rospy.is_shutdown():
        print("about to move...")

        # test_localization()
        # turn_degrees(90)
        # turn_degrees(180)
        # testing_gyro()
        # walk_one_cell(DIRECTION.South)
        planning(starter_map)
        # gyro_turn(90, err)

        # stop moving
        syncMotorWheelSpeeds(4, (LEFT_SHOULDER, RIGHT_SHOULDER, BACK_LEFT_LEG, BACK_RIGHT_LEG), (0, 1024, 0, 1024))
        break

        # sleep to enforce loop rate
        r.sleep()
