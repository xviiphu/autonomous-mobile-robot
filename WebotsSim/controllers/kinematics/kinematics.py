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
robot.load_environment(maze_files[1])

# Move robot to a random staring position listed in maze file
robot.move_to_start()


#set values for rectangle, ccw and cw circle
length_rec = 3      
width_rec = 1       
radius_ccw = 1.5     
radius_cw = 0.5       
counterclockwise = "counterclockwise"
clockwise = "clockwise"



#robot moves foward for a given distance
def go_straight(distance):
    start_postion = robot.get_left_motor_encoder_reading()
    robot.set_left_motor_velocity(12)
    robot.set_right_motor_velocity(12)
    
    start_time = robot.experiment_supervisor.getTime()
    while robot.experiment_supervisor.step(robot.timestep) != -1:

        current_pos = robot.get_left_motor_encoder_reading()
        
        distance_traveled = robot.wheel_radius * (current_pos - start_postion)
        travel_time = robot.experiment_supervisor.getTime() - start_time
        
        if distance_traveled >= distance:
            robot.set_left_motor_velocity(0)
            robot.set_right_motor_velocity(0)
            robot.stop()
        
            print(f"Distance traveled: {distance_traveled} meters.")
            print(f"Travel time: {travel_time} seconds.")
            break



#Make a 90-degree right turn
def turn_right(degree):
    
    #print(f"Initial direction: {robot.get_compass_reading()}")
    final_direction = (robot.get_compass_reading() - degree) % 360
    
    #print(f"Final direction goal: {final_direction}" )

    robot.set_left_motor_velocity(2)
    robot.set_right_motor_velocity(-2)
    
    while robot.experiment_supervisor.step(robot.timestep) != -1:
        current_direction = robot.get_compass_reading()

        if current_direction >= final_direction:
            if current_direction - final_direction < 10:
                robot.set_left_motor_velocity(1)
                robot.set_right_motor_velocity(-1)
            if current_direction == final_direction:
                robot.stop()
                #print(f"Finished direction: {robot.get_compass_reading()}")
                break



#make a cirle with the given radius and direction
def make_a_circle(radius, direction):
    
    #velocity = robot.max_motor_velocity  * robot.wheel_radius  -> 0.9 m/s
    #distance required for the right wheel to travel to make a ccw circle or left wheel for cw circle
    distance_required_right_ccw = 2 * math.pi * (radius + (robot.axel_length/2))
    distance_required_left_cw = 2 * math.pi * (radius + (robot.axel_length/2))

    start_position_right_ccw = robot.get_right_motor_encoder_reading()
    start_position_left_cw = robot.get_left_motor_encoder_reading()
   
    #counterclockwise circle: calculate and set velocity for both wheels
    if direction == "counterclockwise":
        velocity_left = (0.45 * (1 - (robot.axel_length/(2 * radius)))) / robot.wheel_radius
        velocity_right = (0.45 * (1 + (robot.axel_length/(2 * radius)))) / robot.wheel_radius

        robot.set_left_motor_velocity(velocity_left)
        robot.set_right_motor_velocity(velocity_right)

    #clockwise circle: calculate and set velocity for both wheels
    if direction == "clockwise":
        velocity_left = (0.25 * (1 + (robot.axel_length/(2 * radius)))) / robot.wheel_radius    
        velocity_right = (0.25 * (1 - (robot.axel_length/(2 * radius)))) / robot.wheel_radius  

        robot.set_left_motor_velocity(velocity_left)
        robot.set_right_motor_velocity(velocity_right)


    #calculate the distance the robot has traveled and stop if a full circle is traveled
    while robot.experiment_supervisor.step(robot.timestep) != -1:
        
        if direction == "counterclockwise":
            distance_traveled = robot.wheel_radius * (robot.get_right_motor_encoder_reading() - start_position_right_ccw)   
            
        if direction == "clockwise":
            distance_traveled = robot.wheel_radius * (robot.get_left_motor_encoder_reading() - start_position_left_cw) 


        if direction == "counterclockwise":
            if distance_traveled > distance_required_right_ccw:
                robot.stop()
                break

        if direction == "clockwise":
            if distance_traveled > distance_required_left_cw:
                robot.stop()
                break



def turn90(direction):

    distance_re = (0.92)*(direction * math.pi / 180)

    start_pos = robot.get_left_motor_encoder_reading()
    
    robot.set_left_motor_velocity(2)
    robot.set_right_motor_velocity(-2)

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        distance_traveled = robot.wheel_radius * (robot.get_left_motor_encoder_reading() - start_pos)

        if distance_traveled > distance_re:
            robot.stop()
            break



#function calls to make a full rectangle
go_straight(width_rec/2)
turn_right(90)

go_straight(length_rec)
turn_right(90)

go_straight(width_rec)
turn_right(90)

go_straight(length_rec)
turn_right(90)

go_straight(width_rec/2)


