# Import MyRobot Class
from fairis_tools.my_robot import MyRobot
# Create the robot instance.
robot = MyRobot()
import math
import random
from dataclasses import dataclass
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


#scans if there are walls in the NESW directions 
def scan_wall():
    #get the direction that the robot is currently facing
    front_direction = robot.get_compass_reading()
    direction = []
    wall = []
    
    #checks for walls in the order of NESW direction based on the direction
    #of the robot currently facing to
    if 85 <= front_direction <= 95:
        direction.append(robot.get_lidar_range_image()[180])
        direction.append(robot.get_lidar_range_image()[270])
        direction.append(robot.get_lidar_range_image()[0])
        direction.append(robot.get_lidar_range_image()[90])
    elif front_direction <= 5 or front_direction >= 355:
        direction.append(robot.get_lidar_range_image()[90])
        direction.append(robot.get_lidar_range_image()[180])
        direction.append(robot.get_lidar_range_image()[270])
        direction.append(robot.get_lidar_range_image()[0])
    elif 265 <= front_direction <= 275:
        direction.append(robot.get_lidar_range_image()[0])
        direction.append(robot.get_lidar_range_image()[90])
        direction.append(robot.get_lidar_range_image()[180])
        direction.append(robot.get_lidar_range_image()[270])
    elif 175 <= front_direction <= 185:
        direction.append(robot.get_lidar_range_image()[270])
        direction.append(robot.get_lidar_range_image()[0])
        direction.append(robot.get_lidar_range_image()[90])
        direction.append(robot.get_lidar_range_image()[180])

    #1 = wall detected within 0.55m, 0 = no wall
    for i in direction:
        if i < 0.55:
            wall.append(1)
        else:
            wall.append(0)

    return wall


#selects a random direction (NESW) that does not have a wall
def get_action(walls):
    #current direction that the robot is facing
    current_heading = robot.get_compass_reading()

    world_coordinate = [90, 0, 270, 180]
    #remaining directions that robot can move foward
    available_direction = [i for i, val in enumerate(walls) if val == 0]
    #print(available_direction)
    
    #picks a random direction
    chosen_index = random.choice(available_direction)

    #calculates the degree required to rotate the robot to face the chosen direction
    target_heading = world_coordinate[chosen_index]
    diff = target_heading - current_heading

    return chosen_index, diff

 
#particle filter class
@dataclass
class Particle:
    cell: int       #cell index from 1 to 25
    weight: float   #weight of the particle being in a cell

class ParticleFilter:
    #predefined probabilistic sensor model with 250 particles 
    def __init__(self, map,
                n_particles = 250,
                z0_s0 = 0.6,
                z1_s0 = 0.4,
                z1_s1 = 0.8,
                z0_s1 = 0.2  
                ):

        self.map = map
        self.n_particles = n_particles
        self.z0_s0 = z0_s0
        self.z1_s0 = z1_s0
        self.z1_s1 = z1_s1
        self.z0_s1 = z0_s1

        self.particles = []
        self.initial_filter()


    #distributes particles equally across all cells
    def initial_filter(self):
        self.particles = []
        #each particle is assigned an equal initial weight
        uni_weight = 1.0 / self.n_particles

        for i in range(self.n_particles):
            cell_index = (i % 25) + 1
            self.particles.append(Particle(cell = cell_index, weight = uni_weight))


    #returns the probability of a wall observation z given the actual state of the map s
    def sensor(self, observed_wall, map_wall):
        #set binary values after an observation and the true state
        z = 1 if observed_wall else 0
        s = 1 if map_wall else 0

        #return the predefined probability based on z and s
        if z == 0 and s == 0:
            return self.z0_s0
        if z == 0 and s == 1:
            return self.z0_s1
        if z == 1 and s == 0:
            return self.z1_s0
        if z == 1 and s == 1:
            return self.z1_s1
    

    #calculates the total likelihood of the robot being in a specific cell
    def likelihood_cell(self, walls_observed, cell_index):
        #gets the wall layout NESW for the given cell 
        map_walls = self.map[cell_index]
        #calculates the probabilities for each direction
        prop_n = self.sensor(walls_observed[0], map_walls[0])
        prop_e = self.sensor(walls_observed[1], map_walls[1])
        prop_s = self.sensor(walls_observed[2], map_walls[2])
        prop_w = self.sensor(walls_observed[3], map_walls[3])
        #Returns the total particle weight update
        return prop_n * prop_e * prop_s * prop_w


    #calculates the new weight for each particle
    def update(self, walls_observed):
        unnormal = []

        for i in self.particles:
            #how well particle's location matches robot's sensor 
            probability = self.likelihood_cell(walls_observed, i.cell)
            new_weight = i.weight * probability
            unnormal.append(new_weight)
        #sum of all weights to normalize later
        final_total = sum(unnormal)

        #checks for error, reset to uniform distribution if no particles match the observation
        if final_total <= 0:
            uni_weight = 1 / self.n_particles

            for i in self.particles:
                i.weight = uni_weight
        else:
            #normalize weights so the sum of all particle weights equals 1    
            for j,k in zip(self.particles, unnormal):
                j.weight = k / final_total
        
    
    #selects new particles based on their weights, consistent particles are duplicated, while
    #inconsistent ones are removed
    def resampling(self):
        weights = [i.weight for i in self.particles]

        #higher weight, higher probability of selection
        selected_particles = random.choices(population = self.particles, weights = weights, k = self.n_particles)

        #after resampling, all weights are reset to 1 / N
        uni_weight = 1 / self.n_particles

        '''
        self.particles = []
        for i in selected_particles:
            self.particles.append((Particle(i.cell, uni_weight))
        '''
        #reinitializes the particle list
        self.particles = [Particle(cell = i.cell, weight = uni_weight) for i in selected_particles]


    #based on the robot movement, updates each particle cell, particles move 1 cell forward if
    #not blocked by a wall
    def motion(self, cell_index, action_made):
        #wall layout of the current particle's cell
        wall_n, wall_e, wall_s, wall_w = self.map[cell_index]

        if action_made == 0:    #moves north one cell
            return cell_index if wall_n == 1 else cell_index - 5
        if action_made == 1:    #moves east one cell
            return cell_index if wall_e == 1 else cell_index + 1
        if action_made == 2:    #moves south one cell
            return cell_index if wall_s == 1 else cell_index  + 5
        if action_made == 3:    #moves west one cell
            return cell_index if wall_w == 1 else cell_index - 1
    

    #update the state of every particle based on the robot's recent action 
    def prediction(self, action_made):
        for i in self.particles:
            #shifts each particle to its predicted cell
            i.cell = self.motion(i.cell, action_made)

    #triggers resampling if needed
    def sample_size(self):
        total = 0
        for i in self.particles:
            total += i.weight**2
        if total == 0:
            return 0
        return 1 / total
    

    #full filter cycle, executes each step in sequence
    def transition(self, wall_observed, action_made):
        #update weights based on wall observations
        self.update(walls_observed = wall_observed)
        #resample only when needed (if less than 80%)
        samp_size = self.sample_size()
        if samp_size < 0.8 * self.n_particles:
            self.resampling()
        #move particles based on the robot's action
        self.prediction(action_made = action_made)


    #identifies the cell with the highest (particles) probability,
    #uses to guess the robot's location
    def densed_cell(self):
        #a density map for all 25 cells
        density = {i:0.0 for i in range(1, 26)}
        #accumulate weights for every particle in their cells
        for j in self.particles:
            density[j.cell] += j.weight
        #return the cell index with the max weight (most particles)
        return max(density, key = density.get)
    

#visualizes a 5x5 grid with particle distribution
def print_particle_grid(particle_filter):
    count = {i: 0 for i in range(1, 26)}
    #counts how many particles in each cell
    for j in particle_filter.particles:
        count[j.cell] += 1
    
    #finds the cell with the most particles and calculates the percentage of total
    #particles in that cell
    densed_cell = max(count, key = count.get)
    ratio_80 = count[densed_cell] / particle_filter.n_particles
    
    print("\n")
    #prints the 5x5 grid with number of particles in each cell
    for row in range(5):
        string = ""
        for column in range(5):
            cell = (row * 5) + column + 1
            string += f"{count[cell]}\t"
        print(string)
    print("\n")

    #returns 1 if at least 80% of particles in one cell
    if ratio_80 >= 0.8:
        return 1
    else:
        return 0



#particle filter with the given map and 250 particles
cell_particle_filter = ParticleFilter(map = map, n_particles = 250)

#displays initial 5x5 grid
print_particle_grid(cell_particle_filter)
print(f'cell index: {cell_particle_filter.densed_cell()}\n')
print("\n")


while robot.experiment_supervisor.step(robot.timestep) != -1:
    #scans for walls around a cell and picks a random available direction to move forward
    walls = scan_wall()
    chosen_index, rotate_angle = get_action(walls)

    #starts particle filter cycle
    cell_particle_filter.transition(wall_observed = walls, action_made = chosen_index)

    #rotates the robot in place and move forward a cell if not blocked by a wall
    rotate(rotate_angle, counterclockwise, 0.03, 0.58, 0.01)
    forward_encoder(1, 10, 1, 0.5)

    #displays 5x5 grid
    localization_met = print_particle_grid(cell_particle_filter)
    print(f'cell index: {cell_particle_filter.densed_cell()}\n')
    print("\n")

    #stops the particle filter and the robot when condition is met (at least 80% of particles are in one cell).
    if localization_met == 1:
        robot.stop()
        print("The localization condition has been met! At least 80% of the particles are in a single cell.")
        break


