# Import MyRobot Class
from fairis_tools.my_robot import MyRobot
# Create the robot instance.
robot = MyRobot()
import math

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
robot.load_environment(maze_files[2])

#Move robot to a random staring position listed in maze file
robot.move_to_start()

#set dt, cw, and ccw
dt = 0.032
counterclockwise = "counterclockwise"
clockwise = "clockwise"

#a saturation function to saturate the control signal between -12 and 12 rad/sec
def saturation(value):
    if value > 12:
        return 12
    elif value < -12:
        return -12
    else:
        return value


#the robot ultilizes lidar sensor and PID to move toward the wall and stop at 
#a given distance away from the wall, the robot gradually decelerates when it
#approaches the wall
def forward_lidar(target_distance, kp = 10, ki = 0.5, kd = 0.5):
    #updates error and constant i
    prev_error = 0
    i = 0
    
    while robot.experiment_supervisor.step(robot.timestep) != -1:
        #calculates error
        error = robot.get_lidar_range_image()[180] - target_distance

        #calculates PID constants and update the current error
        p = kp * error
        i = ki * ((error * dt) + i)
        d = kd * ((error - prev_error) / dt)
        prev_error = error

        #calculates, saturates the control output, and sets velocity for both wheels
        control_signal = saturation(p + i + d)
        robot.set_left_motor_velocity(control_signal)
        robot.set_right_motor_velocity(control_signal)
        #stops the robot when error is 0
        if abs(error) < 0.01:
            robot.stop()
            #print("Distance away from the wall: ", robot.get_lidar_range_image()[180], " meters.")
            break
        

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


#task 1: 1m away from the wall
forward_lidar(1, 20, 0.2, 0.7)

#Task 2: move forward 0.5m
forward_encoder(0.5, 10, 1, 0.5)

##robot.stop()
#Task 3: move backward 0.5m (1m away from the wall)     
forward_lidar(1, 20, 0.7, 0.5)

#robot.stop()
#Task 4: rotate 180 degree in place cw
rotate(180, clockwise, 0.03, 0.58, 0.01)        #0.03, 0.58, 0.01     0.03, 0.7, 0.02
#print("direction: ", robot.get_compass_reading())

#robot.stop()
robot.set_left_motor_velocity(6)
robot.set_right_motor_velocity(6)

#Task 5: 1m away from the wall
forward_lidar(1, 20, 0.5, 0.5)
#print("direction: ", robot.get_compass_reading())

#robot.set_left_motor_velocity(2)
#robot.set_right_motor_velocity(2)

#Task 6: move forward 0.5m
forward_encoder(0.5, 10, 1, 0.5)
#print("direction: ", robot.get_compass_reading())

#robot.stop()
#Task: 7: move backward 0.5m (1m away from the wall)
forward_lidar(1, 20, 0.5, 0.5)              #1, 20, 0.5, 0.5
#print("direction: ", robot.get_compass_reading())

#robot.stop()
#Task 8: rotate 180 degree in place ccw 180                 0.03, 0.8, 0.01   0.025, 0.7, 0.03
rotate(180, counterclockwise, 0.03, 0.8, 0.01)
#print("direction: ", robot.get_compass_reading())

#robot.stop()
robot.set_left_motor_velocity(6)
robot.set_right_motor_velocity(6)

#robot.stop()
#Task 9: move forward 2.5m
forward_encoder(2.5, 10, 0.99, 0.75)
#print("direction: ", robot.get_compass_reading())


