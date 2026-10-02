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

robot.load_environment(maze_files[7])

#Move robot to a random staring position listed in maze file
robot.move_to_start()

#create a dictionary to store known map coordinates for the four landmarks
map = {"red": (2.5, 2.5),
       "yellow": (-2.5, 2.5),
       "green": (-2.5, -2.5),
       "blue": (2.5, -2.5)
}


#rotates the robot in place to scan for landmarks and records their distances and positions
def scan_for_landmarks(map):
    #landmark_attributes{"color": [(x, y), landmark_distance]}
    landmark_attributes = {}

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        #rotates the robot in place
        robot.set_left_motor_velocity(-5)
        robot.set_right_motor_velocity(5)

        detected_landmark = robot.camera.getRecognitionObjects()

        #checks the object's rgb values and determine its color
        for obj in detected_landmark:
            if obj.getColors()[0] == 1 and obj.getColors()[1] == 1:
                color = "yellow"
            elif obj.getColors()[0] == 1:
                color = "red"
            elif obj.getColors()[1] == 1:
                color = "green"
            elif obj.getColors()[2] == 1:
                color = "blue"

            #stores the obj if first found
            if color not in landmark_attributes:
                #gets the coordinates of the obj
                x_coordinate = obj.getPosition()[0]
                z_coordinate = obj.getPosition()[2]
                #calculates the distance between the robot and the obj
                landmark_distance = math.sqrt((x_coordinate**2) + (z_coordinate**2)) + 0.25

                #stores the known coordinates (x, y) and the calculated distance
                grid_position = map[color]
                landmark_attributes[color] = [grid_position, landmark_distance]

        #Scans at least 3 landmarks for trilateration or 4 for averaging then stop and return
        if len(landmark_attributes) >= 4:
            robot.stop()
            return landmark_attributes


#estimates the robot position (x,y) by converting circle equations into linear equations
def trilateration(recognized_landmarks):
    #converts the keys into a list of keys for indexing
    key = list(recognized_landmarks.keys())

    #assigns values for the circle equations
    (x1, y1), r1 = recognized_landmarks[key[0]]
    (x2, y2), r2 = recognized_landmarks[key[1]]
    (x3, y3), r3 = recognized_landmarks[key[2]]
    (x4, y4), r4 = recognized_landmarks[key[3]]

    #calculates the first line from circle 1 - circle 2
    a = (-2 * x1) + (2 * x2)
    b = (-2 * y1) + (2 * y2)
    c = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2

    #calculates the second line from circle 2 - circle 3
    d = (-2 * x2) + (2 * x3)
    e = (-2 * y2) + (2 * y3)
    f = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2

    #calculates the intersection of the two lines (x,y)
    x = (c*e - f*b) / (e*a - b*d)
    y = (c*d - a*f) / (b*d - a*e)


    #calculates the first line from circle 2 - circle 3
    a2 = (-2 * x2) + (2 * x3)
    b2 = (-2 * y2) + (2 * y3)
    c2 = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2

    #calculates the second line from circle 3 - circle 4
    d2 = (-2 * x3) + (2 * x4)
    e2 = (-2 * y3) + (2 * y4)
    f2 = r3**2 - r4**2 - x3**2 + x4**2 - y3**2 + y4**2

    #calculates the intersection of the two lines (x,y)
    x2 = (c2*e2 - f2*b2) / (e2*a2 - b2*d2)
    y2 = (c2*d2 - a2*f2) / (b2*d2 - a2*e2)

    #calculates the average of two estimates to reduce error and sensor noise
    x_final = (x + x2) / 2
    y_final = (y + y2) / 2

    #return the estimated position of the robot (x, y)
    return (x_final, y_final)


#this function calculates the cell index that the robot is in using
#the given position of the robot (x, y), this function is from Google Gemini
def find_cell_index(x, y):
    #keeps the values within the grid boundaries
    x = max(-2.49, min(2.49, x))
    y = max(-2.49, min(2.49, y))
    #x increases east
    column = math.floor(x + 2.5)
    #y increases north
    row = math.floor(2.5 - y)
    #converts (x,y) coordinates to index
    index = (row * 5) + column + 1

    return int(index)


#scan and detect landmarks
recognized_landmarks = scan_for_landmarks(map)
#print(recognized_landmarks)

#perform trilateration and print the robot position (x,y)
location = trilateration(recognized_landmarks)
print(f'Position(x, y): {location}')

#determine and print cell index
coordinate = list(location)
cell_index = find_cell_index(coordinate[0], coordinate[1])
print(f'Cell index: {cell_index}')


