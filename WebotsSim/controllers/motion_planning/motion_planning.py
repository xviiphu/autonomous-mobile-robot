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

robot.load_environment(maze_files[6])

#Move robot to a random staring position listed in maze file
robot.move_to_start()

wall_side = "left"
state = "mg"

dt = 0.032
counterclockwise = "counterclockwise"
clockwise = "clockwise"


#a saturation function to saturate the control signal between -12 and 12 rad/sec
def saturation(value):
    if value > 15:
        return 15
    elif value < -15:
        return -15
    else:
        return value
    



def forward_lidar(target_distance = 0.25, kp = 10, ki = 0.5, kd = 0.5):

    forward_dis = robot.get_lidar_range_image()[180]
    forward_dis = forward_dis if not math.isinf(forward_dis) else 100
    error = forward_dis - target_distance
    p = error * kp

    return saturation(p)





#robot rotates in place at a given degree and direction(cw or ccw) using imu/compass with PID
def rotate(degree, direction, kp = 0.4, ki = 0.6, kd = 0.1):
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





def scan_goal(kp = 0.2):
    landmark_object = robot.camera.getRecognitionObjects()

    

    if len(landmark_object) > 0:
        velocity = forward_lidar()
        goal = landmark_object[0]
        goal_in_image = goal.getPositionOnImage()[0]
        center_image = robot.camera.getWidth() / 2

        error = goal_in_image - center_image

        p = error * kp

        diff = abs(p)

        if error < 0:
            diff_left = -diff
            diff_right = diff
        elif error > 0:
            diff_left = diff
            diff_right = -diff
        else:
            diff_left = 0
            diff_right = 0
        
        velocity_left = velocity + diff_left
        velocity_right = velocity + diff_right
    
    #else:
        #print("distance: ", robot.get_lidar_range_image()[180])
        #velocity_left = 5
        #velocity_right = -5
    
    return saturation(velocity_left), saturation(velocity_right)




def wall_follow(wall_side):
    target_distance = 0.45
    kp = 5
    forward_dis = robot.get_lidar_range_image()[180]
    forward_dis = forward_dis if not math.isinf(forward_dis) else 100
    velocity = forward_lidar()

    if wall_side == "left":
        left_dis = min(robot.get_lidar_range_image()[80:150])
        error = left_dis - target_distance

        p = error * kp

        diff = abs(p)

        if error < 0:
            diff_left = -diff
            diff_right = diff
        elif error > 0:
            diff_left = diff
            diff_right = -diff
        else:
            diff_left = 0
            diff_right = 0

      
        if left_dis < 0.5:
            rotate(20, clockwise)
        
        velocity_left = velocity + diff_left
        velocity_right = velocity + diff_right
    
    
    return saturation(velocity_left), saturation(velocity_right)
'''
    elif wall_side == "right":
        right_dis = min(robot.get_lidar_range_image()[245:280])
        error = right_dis - target_distance

        p = error * kp

        diff = abs(p)

        if error < 0:
            diff_left = diff
            diff_right = -diff
        elif error > 0:
            diff_left = -diff
            diff_right = diff
        else:
            diff_left = 0
            diff_right = 0
        
        #if forward_dis < 0.3:
            #rotate(45, counterclockwise)
        
        velocity_left = velocity + diff_left
        velocity_right = velocity + diff_right

    return saturation(velocity_left), saturation(velocity_right)

'''



while robot.experiment_supervisor.step(robot.timestep) != -1:
    robot.set_left_motor_velocity(5)
    robot.set_right_motor_velocity(-5)
    landmark = robot.camera.getRecognitionObjects()
    if len(landmark) > 0:
        robot.stop()
        break






while robot.experiment_supervisor.step(robot.timestep) != -1:
    
    landmark = robot.camera.getRecognitionObjects()
    forward_distance = robot.get_lidar_range_image()[180]
    forward_distance = forward_distance if not math.isinf(forward_distance) else 100
    
    if state == "mg":
        if forward_distance < 0.45:
            state = "wf"
            robot.stop()
        else:
            velocity_left, velocity_right = scan_goal()
            robot.set_left_motor_velocity(velocity_left)
            robot.set_right_motor_velocity(velocity_right)
    
    elif state == "wf":
        if len(landmark) > 0 and forward_distance > 0.75:
            state = "mg"
            robot.stop()
        else:
            velocity_left, velocity_right = wall_follow(wall_side)
            robot.set_left_motor_velocity(velocity_left)
            robot.set_right_motor_velocity(velocity_right)











'''




  if len(landmark) > 0:
        velocity_left, velocity_right = scan_goal()
        robot.set_left_motor_velocity(velocity_left)
        robot.set_right_motor_velocity(velocity_right)
    else:
        velocity_left, velocity_right = wall_follow(wall_side)
        robot.set_left_motor_velocity(velocity_left)
        robot.set_right_motor_velocity(velocity_right)








    if forward_distance > 0.82:
        if len(landmark) >=1 or len(landmark) <=0:
            velocity_left, velocity_right = scan_goal()
            robot.set_left_motor_velocity(velocity_left)
            robot.set_right_motor_velocity(velocity_right)

    if forward_distance < 0.82 and len(landmark) <=0:

        velocity_left, velocity_right = wall_follow(wall_side)
        robot.set_left_motor_velocity(velocity_left)
        robot.set_right_motor_velocity(velocity_right)
        




len(landmark) <= 0 and len(landmark) > 0 and
    velocity_left, velocity_right = wall_follow(wall_side)
    robot.set_left_motor_velocity(velocity_left)
    robot.set_right_motor_velocity(velocity_right)

    
    front = robot.get_lidar_range_image()[180]
    front = front if not math.isinf(front) else 100

    landmark = robot.camera.getRecognitionObjects()
    goal_visible = len(landmark) > 0

    # ===== STATE MACHINE =====
    if state == "MG":

        velocity_left, velocity_right = scan_goal()

        # hit obstacle → switch to wall-following
        if front < 0.5:
            state = "WF"

    elif state == "WF":

        velocity_left, velocity_right = wall_follow(wall_side)

        # goal visible again → go back to goal
        if goal_visible:
            state = "MG"

    # ===== APPLY MOTION =====
    robot.set_left_motor_velocity(velocity_left)
    robot.set_right_motor_velocity(velocity_right)



'''

'''

while robot.experiment_supervisor.step(robot.timestep) != -1:

    velocity_left, velocity_right = scan_goal()
    robot.set_left_motor_velocity(velocity_left)
    robot.set_right_motor_velocity(velocity_right)

    velocity_left, velocity_right = wall_follow(wall_side)
    robot.set_left_motor_velocity(velocity_left)
    robot.set_right_motor_velocity(velocity_right)

    
'''





'''

state = "m2g"



velocity_left, velocity_right = scan_goal()
    robot.set_left_motor_velocity(velocity_left)
    robot.set_right_motor_velocity(velocity_right)

while robot.experiment_supervisor.step(robot.timestep) != -1:
    landmark = robot.camera.getRecognitionObjects()
    forward_distance = robot.get_lidar_range_image()[180]

    if(state == "m2g"):
        if forward_distance < 0.4:
            state = "wl"
        else:
            velocity_left, velocity_right = scan_goal()
            robot.set_left_motor_velocity(velocity_left)
            robot.set_right_motor_velocity(velocity_right)
    
    elif state == "wl": #len(landmark) > 0 and (robot.get_lidar_range_image()[180] < 0.4)
        if len(landmark) > 0 and forward_distance > 0.4:
            state = "m2g"
    
        else:
            velocity_left, velocity_right = wall_follow(wall_side)

            robot.set_left_motor_velocity(velocity_left)
            robot.set_right_motor_velocity(velocity_right)


'''

'''





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
robot.load_environment(maze_files[6])

#Move robot to a random staring position listed in maze file
robot.move_to_start()

wall_side = "left"


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
    

def forward_lidar(target_distance = 0.25, kp = 10, ki = 0.5, kd = 0.5):

    forward_dis = robot.get_lidar_range_image()[180]
    forward_dis = forward_dis if not math.isinf(forward_dis) else 100
    error = forward_dis - target_distance
    p = error * kp

    return saturation(p)

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





def scan_goal(kp = 0.2):

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        landmark_object = robot.camera.getRecognitionObjects()

    

        if len(landmark_object) > 0:
            velocity = forward_lidar()
            goal = landmark_object[0]
            goal_in_image = goal.getPositionOnImage()[0]
            center_image = robot.camera.getWidth() / 2

            error = goal_in_image - center_image

            p = error * kp

            diff = abs(p)

            if error < 0:
                diff_left = -diff
                diff_right = diff
            elif error > 0:
                diff_left = diff
                diff_right = -diff
            else:
                diff_left = 0
                diff_right = 0
        
            velocity_left = velocity + diff_left
            velocity_right = velocity + diff_right
            robot.set_left_motor_velocity(saturation(velocity_left))
            robot.set_right_motor_velocity(saturation(velocity_right))
    
        elif len(landmark_object) == 0:
            robot.set_left_motor_velocity(5)
            robot.set_right_motor_velocity(-5)
            #velocity_left = 5
            #velocity_right = -5


        #robot.set_left_motor_velocity(saturation(velocity_left))
        #robot.set_right_motor_velocity(saturation(velocity_right))

        if len(landmark_object) < 1 and robot.get_lidar_range_image()[180] < 1:
            robot.stop()
            break
    
    
        #robot.set_left_motor_velocity(saturation(velocity_left))
        #robot.set_right_motor_velocity(saturation(velocity_right))
        #return saturation(velocity_left), saturation(velocity_right)




def wall_follow(wall_side):
    while robot.experiment_supervisor.step(robot.timestep) != -1:

        target_distance = 0.4
        kp = 8
        forward_dis = robot.get_lidar_range_image()[180]

        velocity = forward_lidar()

        if wall_side == "left":
            left_dis = min(robot.get_lidar_range_image()[80:115])
            error = left_dis - target_distance

            p = error * kp

            diff = abs(p)

            if error < 0:
                diff_left = diff
                diff_right = -diff
            elif error > 0:
                diff_left = -diff
                diff_right = diff
            else:
                diff_left = 0
                diff_right = 0

            if forward_dis < 0.3:
                rotate(45, clockwise)
        
            velocity_left = velocity + diff_left
            velocity_right = velocity + diff_right
            robot.set_left_motor_velocity(saturation(velocity_left))
            robot.set_right_motor_velocity(saturation(velocity_right))


        elif wall_side == "right":
            right_dis = min(robot.get_lidar_range_image()[245:280])
            error = right_dis - target_distance

            p = error * kp

            diff = abs(p)

            if error < 0:
                diff_left = diff
                diff_right = -diff
            elif error > 0:
                diff_left = -diff
                diff_right = diff
            else:
                diff_left = 0
                diff_right = 0
        
            if forward_dis < 0.3:
                rotate(45, counterclockwise)
        
            velocity_left = velocity + diff_left
            velocity_right = velocity + diff_right
            robot.set_left_motor_velocity(saturation(velocity_left))
            robot.set_right_motor_velocity(saturation(velocity_right))
    
        #return saturation(velocity_left), saturation(velocity_right)

state = "m2g"
landmark = robot.camera.getRecognitionObjects()
forward_distance = robot.get_lidar_range_image()[180]

if len(landmark_object) < 1 and robot.get_lidar_range_image()[180] < 1:
            robot.stop()
            break

if(state == "m2g"):
    if 

if(state == "m2g"):
    if forward_distance < 0.4:
        state = "wl"
    else:
        velocity_left, velocity_right = scan_goal()
        robot.set_left_motor_velocity(velocity_left)
        robot.set_right_motor_velocity(velocity_right)
    
elif state == "wl": #len(landmark) > 0 and (robot.get_lidar_range_image()[180] < 0.4)
    if len(landmark) > 0 and forward_distance > 0.4:
        state = "m2g"
    
    else:
        velocity_left, velocity_right = wall_follow(wall_side)

        robot.set_left_motor_velocity(velocity_left)
        robot.set_right_motor_velocity(velocity_right)


state = "m2g"

while robot.experiment_supervisor.step(robot.timestep) != -1:
    landmark = robot.camera.getRecognitionObjects()
    forward_distance = robot.get_lidar_range_image()[180]

    if(state == "m2g"):
        if forward_distance < 0.4:
            state = "wl"
        else:
            velocity_left, velocity_right = scan_goal()
            robot.set_left_motor_velocity(velocity_left)
            robot.set_right_motor_velocity(velocity_right)
    
    elif state == "wl": #len(landmark) > 0 and (robot.get_lidar_range_image()[180] < 0.4)
        if len(landmark) > 0 and forward_distance > 0.4:
            state = "m2g"
    
        else:
            velocity_left, velocity_right = wall_follow(wall_side)

            robot.set_left_motor_velocity(velocity_left)
            robot.set_right_motor_velocity(velocity_right)


'''