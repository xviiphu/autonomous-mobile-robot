# Import MyRobot Class
from fairis_tools.my_robot import MyRobot
# Create the robot instance.
robot = MyRobot()
import math
import random
from dataclasses import dataclass
from collections import deque

# Loads the environment from the maze file
maze_files = ['../../worlds/Spring26/maze0.xml',
              '../../worlds/Spring26/maze1.xml',
              '../../worlds/Spring26/maze2.xml',
              '../../worlds/Spring26/maze3.xml',
              '../../worlds/Spring26/maze4.xml',
              '../../worlds/Spring26/maze5.xml',
              '../../worlds/Spring26/maze6.xml',
              '../../worlds/Spring26/maze7.xml',
              '../../worlds/Spring26/maze8.xml'
              ]

robot.load_environment(maze_files[8])

#Move robot to a random staring position listed in maze file
robot.move_to_start()

dt = 0.032
counterclockwise = "counterclockwise"
clockwise = "clockwise"

goal = 15 #random.randint(1, 25)  #inclusively picks a random cell 1-25 to test different goals

#a saturation function to saturate the control signal between -12 and 12 rad/sec
def saturation(value):
    if value > 12:
        return 12
    elif value < -12:
        return -12
    else:
        return value


#key: [N,E,S,W]
#cell walls: 1 = wall, 0 = no wall
map = {
    1: [1, 0, 0, 1],
    2: [1, 0, 1, 0],
    3: [1, 0, 1, 0],
    4: [1, 0, 1, 0],
    5: [1, 1, 0, 0],
    6: [0, 0, 1, 1],
    7: [1, 0, 1, 0],
    8: [1, 1, 0, 0],
    9: [1, 0, 1, 1],
    10: [0, 1, 1, 0],
    11: [1, 0, 0, 1],
    12: [1, 1, 0, 0],
    13: [0, 0, 1, 1],
    14: [1, 0, 1, 0],
    15: [1, 1, 0, 0],
    16: [0, 1, 0, 1],
    17: [0, 0, 1, 1],
    18: [1, 0, 1, 0],
    19: [1, 0, 1, 0],
    20: [0, 1, 1, 0],
    21: [0, 0, 1, 1],
    22: [1, 0, 1, 0],
    23: [1, 0, 1, 0],
    24: [1, 0, 1, 0],
    25: [1, 1, 1, 0],
}

#robot moves forward for a given distance using encoder with the help of PID
def forward_encoder(target_distance, kp = 10, ki = 1, kd = 0.5):
    start_position = robot.get_left_motor_encoder_reading()
    prev_error = 0
    i = 0

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        #updates error
        current_position = robot.get_left_motor_encoder_reading()
        distance_traveled = (current_position - start_position) * robot.wheel_radius
        error = target_distance - distance_traveled

        #PID constants
        p = kp * error
        i = ki * ((error * dt) + i)
        d = kd * ((error - prev_error) / dt)
        prev_error = error

        #control output for both motors
        control_signal = saturation(p + i + d)
        robot.set_left_motor_velocity(control_signal)
        robot.set_right_motor_velocity(control_signal)
        #stops when error is 0
        if abs(error) < 0.01:
            robot.stop()
            #print("Distance traveled: ", distance_traveled, " meters.")
            #print("Distance away from the wall: ", robot.get_lidar_range_image()[180], " meters.")
            break


#robot rotates in place at a given degree and direction(cw or ccw) using imu/compass with PID
def rotate(degree, direction, kp = 1, ki = 0.6, kd = 0.1):
    #final direction after a turn
    final_direction = (robot.get_compass_reading() + degree) % 360    
    prev_error = 0
    i = 0

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        #gets current error
        error = (final_direction - robot.get_compass_reading() + 180) % 360 - 180

        #PID constants
        p = kp * error
        i = ki * ((error * dt) + i)
        d = kd * ((error - prev_error) / dt)
        prev_error = error

        control_signal = saturation(p + i + d)

        #rotates in place ccw
        if direction == "counterclockwise":
            if error > 0:
                robot.set_left_motor_velocity(-control_signal)
                robot.set_right_motor_velocity(control_signal)

            if error < 0:
                robot.set_left_motor_velocity(control_signal)
                robot.set_right_motor_velocity(-control_signal)

        #rotates in place cw
        if direction == "clockwise":
            if error > 0:
                robot.set_left_motor_velocity(control_signal)
                robot.set_right_motor_velocity(-control_signal)

            if error < 0:
                robot.set_left_motor_velocity(-control_signal)
                robot.set_right_motor_velocity(control_signal)

        if abs(error) < 0.0001:
            robot.stop()
            break


#converts the robot (x, y) coordinates into cell index
def get_cell_index():
    x = robot.starting_position.x
    y = robot.starting_position.y

    col = round(x + 2)  
    row = round(-y + 2)  
    return row * 5 + col + 1  #cell 1–25


#uses the map configurations to determine the adjacent cells the robot can move
def get_neighbors(cell, map):
    walls = map[cell]
    neighbors = []

    #order: NESW
    if walls[0] == 0: neighbors.append(cell - 5)  #-5 going north
    if walls[1] == 0: neighbors.append(cell + 1)  #+1 going east
    if walls[2] == 0: neighbors.append(cell + 5)  #+5 going south
    if walls[3] == 0: neighbors.append(cell - 1)  #-1 going west

    return neighbors


#fills the map grid with the cost value for each cell starting from the goal
def wavefront(map, goal):
    g = {}
    for i in range(1, 26):
        g[i] = float('inf')
    
    g[goal] = 0
    Q = deque([goal])

    while Q:
        c = Q.popleft()
        for n in get_neighbors(c, map):
            if g[n] == float('inf'):  #visits if no assigned cost
                g[n] = g[c] + 1
                Q.append(n)
    return g


#traces the path from start to goal by following the descending cost values
def recover_path(g, start, goal):
    path = [start]
    current = start

    while current != goal:
        best = None
        best_cost = float('inf')
        
        #checks neighbor cells to find the one closest to the goal(NESW order)
        for n in get_neighbors(current, map):
            if g[n] < best_cost:
                best_cost = g[n]
                best = n
        
        current = best
        path.append(current)

    return path


#gets the cell index the robot currently in
start_cell = get_cell_index()
#return a dictionary g, each cell is assigned with a cost value
g = wavefront(map, goal)
#a list of cells from start to goal(shortest path)
path = recover_path(g, start_cell, goal)

#displays start and goal cell index, shortest path as a list of cells from start
#to goal, and the path length in steps
print(f'Start: {start_cell}')
print(f'Goal: {goal}')
print(f'Path: {path}')
print(f'Path length: {len(path) - 1} steps')


for i in range(len(path) - 1):
    direction = 0
    #calculates the direction to rotate to the next cell
    step = path[i + 1] - path[i]
    if step == -5:
        direction = 90
    elif step == 1:
        direction = 0
    elif step == 5:
        direction = 270
    elif step == -1:
        direction = 180
    
    #gets the direction the robot currently facing
    front_direction = robot.get_compass_reading()
    #calculates the degree of rotation required
    diff = (direction - front_direction + 360) % 360

    #rotates if needed then moves forward to the next cell
    if diff != 0:
        #finds the shortest rotation(either cw or ccw)
        if diff <= 180:
            rotate(diff, counterclockwise, 0.03, 0.58, 0.01)
        else:
            rotate(diff, clockwise, 0.03, 0.58, 0.01)

    forward_encoder(1, 10, 1, 0.5)


