import sys
import numpy as np
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS = 7
CELLCOLS = 14

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = 0
    
    def update(self, error, dt):
        # Cálculo do termo integral
        self.integral += error * dt
        # Cálculo do termo derivativo
        derivative = (error - self.previous_error) / dt
        # PID Output
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output
    
class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.lap_time = 0
        self.readSensors()

        # Controladores PID para os eixos X e Y
        self.pid_x = PIDController(Kp=0.1, Ki=0.01, Kd=0.01)
        self.pid_y = PIDController(Kp=0.1, Ki=0.01, Kd=0.01)

        global MAP
        global current_MAP_x
        global current_MAP_y
        global initial_MAP_x
        global initial_MAP_y

        MAP = []
        for i in range(27):
            rows = 55 * [10]
            MAP.append(rows)

        current_MAP_x = current_MAP_y = []

        global initial_GPS_x
        global initial_GPS_y
        global start_GPS_x
        global start_GPS_y
        global current_GPS_x
        global current_GPS_y
        initial_GPS_x = self.measures.x
        initial_GPS_y = self.measures.y
        start_GPS_x = self.measures.x
        start_GPS_y = self.measures.y

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            self.measures.gpsReady = True
            self.measures.gpsDirReady = True

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()

    def determineQuadrant(self):
        self.readSensors()
        compass = self.measures.compass
        Quadrant = [False, False, False, False]

        if abs(compass) <= 45:
            Quadrant[0] = True  # Facing Right
        elif compass > 45 and compass <= 135:
            Quadrant[1] = True  # Facing Up
        elif abs(compass) >= 135:
            Quadrant[2] = True  # Facing Left
        elif compass <= -45 and compass >= -135:
            Quadrant[3] = True  # Facing Down

        return Quadrant

    def mapSurroundings(self, quadrant, y, x):
        center_id = 0
        left_id = 1
        right_id = 2
        dist_tolerance = 1.15

        self.readSensors()
        center_sensor = self.measures.irSensor[center_id]
        left_sensor = self.measures.irSensor[left_id]
        right_sensor = self.measures.irSensor[right_id]

        def update_map(sensor_value, threshold, map_coords, wall_value, empty_value, explore_value):
            if sensor_value >= threshold:
                MAP[map_coords[0]][map_coords[1]] = wall_value
            else:
                MAP[map_coords[0]][map_coords[1]] = empty_value
                if MAP[map_coords[2]][map_coords[3]] != 80:
                    MAP[map_coords[2]][map_coords[3]] = explore_value

        if quadrant[0]:  # Facing Right
            update_map(center_sensor, dist_tolerance, (y, x + 1, y, x + 2), 30, 20, 60)
            update_map(left_sensor, dist_tolerance, (y - 1, x, y - 2, x), 40, 20, 60)
            update_map(right_sensor, dist_tolerance, (y + 1, x, y + 2, x), 40, 20, 60)
            return MAP[y][x + 1], MAP[y][x + 2], MAP[y + 1][x], MAP[y + 2][x], MAP[y - 1][x], MAP[y - 2][x]

        elif quadrant[1]:  # Facing Up
            update_map(center_sensor, dist_tolerance, (y - 1, x, y - 2, x), 40, 20, 60)
            update_map(left_sensor, dist_tolerance, (y, x - 1, y, x - 2), 30, 20, 60)
            update_map(right_sensor, dist_tolerance, (y, x + 1, y, x + 2), 30, 20, 60)
            return MAP[y - 1][x], MAP[y - 2][x], MAP[y][x + 1], MAP[y][x + 2], MAP[y][x - 1], MAP[y][x - 2]

        elif quadrant[2]:  # Facing Left
            update_map(center_sensor, dist_tolerance, (y, x - 1, y, x - 2), 30, 20, 60)
            update_map(left_sensor, dist_tolerance, (y + 1, x, y + 2, x), 40, 20, 60)
            update_map(right_sensor, dist_tolerance, (y - 1, x, y - 2, x), 40, 20, 60)
            return MAP[y][x - 1], MAP[y][x - 2], MAP[y - 1][x], MAP[y - 2][x], MAP[y + 1][x], MAP[y + 2][x]

        elif quadrant[3]:  # Facing Down
            update_map(center_sensor, dist_tolerance, (y + 1, x, y + 2, x), 40, 20, 60)
            update_map(left_sensor, dist_tolerance, (y, x + 1, y, x + 2), 30, 20, 60)
            update_map(right_sensor, dist_tolerance, (y, x - 1, y, x - 2), 30, 20, 60)
            return MAP[y + 1][x], MAP[y + 2][x], MAP[y][x - 1], MAP[y][x - 2], MAP[y][x + 1], MAP[y + 2][x]

    def move(self, directional_states):
        self.readSensors()
        
        GPS_x = self.measures.x - initial_GPS_x
        GPS_y = self.measures.y - initial_GPS_y

        x_positions = list(range(-26, 28, 2))
        current_GPS_x = x_positions[find_next_cell(x_positions, GPS_x)]

        y_positions = list(range(-12, 14, 2))
        current_GPS_y = y_positions[find_next_cell(y_positions, GPS_y)]
        
        error_x = error_y = 100
        lin = 0.15
        tolerance = 0.225
    
        while any(directional_states) and (error_x > tolerance or error_y > tolerance):
            self.readSensors()
            GPS_x = self.measures.x - initial_GPS_x
            GPS_y = self.measures.y - initial_GPS_y

            if directional_states[0]:  # Right
                error_x = (current_GPS_x + 2) - GPS_x
                error_y = current_GPS_y - GPS_y
                rot = self.pid_y.update(error_y, dt=0.1)
                right_rotation = lin + rot
                left_rotation = lin - rot
                self.driveMotors(left_rotation, right_rotation)

            if directional_states[1]:  # Up
                error_x = current_GPS_x - GPS_x
                error_y = (current_GPS_y + 2) - GPS_y
                rot = self.pid_x.update(error_x, dt=0.1)
                right_rotation = lin - rot
                left_rotation = lin + rot
                self.driveMotors(left_rotation, right_rotation)

            if directional_states[2]:  # Left
                error_x = GPS_x - (current_GPS_x - 2)
                error_y = GPS_y - current_GPS_y
                rot = self.pid_y.update(error_y, dt=0.1)
                right_rotation = lin + rot
                left_rotation = lin - rot
                self.driveMotors(left_rotation, right_rotation)

            if directional_states[3]:  # Down
                error_x = GPS_x - current_GPS_x
                error_y = GPS_y - (current_GPS_y - 2)
                rot = self.pid_x.update(error_x, dt=0.1)
                right_rotation = lin - rot
                left_rotation = lin + rot
                self.driveMotors(left_rotation, right_rotation)
             
    def turn_left_90(self):
        def adjust_compass(compass):
            compass_options = [0, 90, -180, -90, 180]
            calibrated_compass = compass_options[find_next_cell(compass_options, compass)]
            return -180 if calibrated_compass == 180 else calibrated_compass

        def calculate_rotation_error(current_compass, target_compass):
            rotation_error = target_compass - current_compass
            if rotation_error > 120:
                rotation_error -= 360
            return rotation_error

        def apply_rotation(rotation_error):
            Kd_angle = 0.005
            rotation = Kd_angle * rotation_error
            self.driveMotors(-rotation, rotation)

        target_compass = adjust_compass(self.measures.compass) + 90
        rotation_error = 100

        while abs(rotation_error) >= 1:
            self.readSensors()
            current_compass = self.measures.compass
            rotation_error = calculate_rotation_error(current_compass, target_compass)
            apply_rotation(rotation_error)

    def turn_right_90(self):
        def adjust_compass(compass):
            compass_options = [0, 90, -180, -90, 180]
            calibrated_compass = compass_options[find_next_cell(compass_options, compass)]
            return -180 if calibrated_compass == 180 else calibrated_compass

        def calculate_rotation_error(current_compass, target_compass):
            rotation_error = target_compass - current_compass
            if rotation_error < -120:
                rotation_error += 360
            return rotation_error

        def apply_rotation(rotation_error):
            Kd_angle = 0.005
            rotation = Kd_angle * rotation_error
            self.driveMotors(-rotation, rotation)

        target_compass = adjust_compass(self.measures.compass) - 90
        rotation_error = 100

        while abs(rotation_error) >= 1:
            self.readSensors()
            current_compass = self.measures.compass
            rotation_error = calculate_rotation_error(current_compass, target_compass)
            apply_rotation(rotation_error)

    def find_path(self, map_array, unvisited_x, unvisited_y, current_x, current_y):
        try:
            unvisited_positions = list(zip(unvisited_y, unvisited_x))
            linear_moves = []
            all_moves = []

            for target_y, target_x in unvisited_positions:
                path_array = np.zeros_like(map_array)
                path_array[current_y, current_x] = 1

                while path_array[target_y, target_x] == 0:
                    max_value = np.amax(path_array)
                    possible_moves = np.argwhere(path_array == max_value)

                    for j, i in possible_moves:
                        if map_array[j, i] in [20, 80, 90]:
                            for dj, di in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                                if path_array[j + dj, i + di] == 0 and map_array[j + dj, i + di] in [20, 60, 80]:
                                    path_array[j + dj, i + di] = max_value + 1

                moves = []
                max_value = np.amax(path_array)
                current_y, current_x = target_y, target_x

                for _ in range(max_value - 1):
                    surrounding = np.array([[0, path_array[current_y - 1, current_x], 0],
                                            [path_array[current_y, current_x - 1], path_array[current_y, current_x], path_array[current_y, current_x + 1]],
                                            [0, path_array[current_y + 1, current_x], 0]])
                    y, x = np.where(surrounding == max_value - 1)
                    if len(y) == 0 or len(x) == 0:
                        break
                    yi = (y[0], x[0])
                    if yi == (0, 1):
                        moves.append('DOWN')
                        current_y -= 1
                    elif yi == (1, 0):
                        moves.append('RIGHT')
                        current_x -= 1
                    elif yi == (1, 2):
                        moves.append('LEFT')
                        current_x += 1
                    elif yi == (2, 1):
                        moves.append('UP')
                        current_y += 1

                    max_value -= 1

                moves = moves[1::2][::-1]
                all_moves.append(moves)
                num_rotations = sum(1 for i in range(1, len(moves)) if moves[i] != moves[i - 1])
                total_moves = num_rotations + len(moves)
                linear_moves.append(total_moves)

            return all_moves[linear_moves.index(min(linear_moves))]
        except:
            print("Maze Completed. Exiting.")
            quit()
            
    def find_closest_index(arr, val):
        differences = [abs(val - i) for i in arr]
        return differences.index(min(differences))

    def navigate_path(self, next_moves):
        def adjust_orientation(quadrant, direction_index):
            while not quadrant[direction_index]:
                self.turn_left_90()
                quadrant = self.determineQuadrant()
            return quadrant
    
        def move_and_update_quadrant(quadrant):
            self.move(quadrant)
            return self.determineQuadrant()
    
        def get_rotation_action(prev_direction, next_direction):
            rotation_map = {
                ('LEFT', 'DOWN'): self.turn_left_90,
                ('LEFT', 'UP'): self.turn_right_90,
                ('RIGHT', 'DOWN'): self.turn_right_90,
                ('RIGHT', 'UP'): self.turn_left_90,
                ('UP', 'LEFT'): self.turn_left_90,
                ('UP', 'RIGHT'): self.turn_right_90,
                ('DOWN', 'LEFT'): self.turn_right_90,
                ('DOWN', 'RIGHT'): self.turn_left_90
            }
            return  rotation_map.get((prev_direction, next_direction))
    
        quadrant = self.determineQuadrant()
        direction_map = {
            'LEFT': 2,
            'RIGHT': 0,
            'UP': 1,
            'DOWN': 3
        }
    
        if next_moves[0] in direction_map:
            quadrant = adjust_orientation(quadrant, direction_map[next_moves[0]])
    
        quadrant = move_and_update_quadrant(quadrant)
    
        for i in range(1, len(next_moves)):
            if next_moves[i] == next_moves[i - 1]:
                quadrant = move_and_update_quadrant(quadrant)
            else:
                rotation_function = get_rotation_action(next_moves[i - 1], next_moves[i])
                if rotation_function:
                    rotation_function()
                    quadrant = self.determineQuadrant()
                    quadrant = move_and_update_quadrant(quadrant)

    def save_map(self, MAP, start_x, start_y):
        def map_symbol(cell):
            symbol_map = {
                80: 'X',
                90: 'X',
                20: 'X',
                60: 'X',
                30: '|',
                40: '-',
                10: ' ',
                50: 'I'
            }
            return symbol_map.get(cell, ' ')
    
        MAP[start_y][start_x] = 50
    
        # Convert the map to a string representation
        MAP_str = "\n".join("".join(map_symbol(cell) for cell in row) for row in MAP)
        
        # Write the map to a file
        with open('mymap.txt', 'w') as f:
            f.write(MAP_str)

    def wander(self):
        global initial_GPS_x, initial_GPS_y, start_GPS_x, start_GPS_y
        global current_GPS_x, current_GPS_y, MAP
        global current_MAP_x, current_MAP_y, initial_MAP_x, initial_MAP_y

        self.readSensors()

        compass = self.measures.compass
        GPS_x = self.measures.x - initial_GPS_x
        GPS_y = self.measures.y - initial_GPS_y

        current_GPS_x = self.get_current_position(GPS_x, -26, 28, 2)
        current_GPS_y = self.get_current_position(GPS_y, -12, 14, 2)

        initial_MAP_x, initial_MAP_y = 27, 13
        current_MAP_x = initial_MAP_x + current_GPS_x
        current_MAP_y = initial_MAP_y - current_GPS_y

        current_MAP_x = current_MAP_x if current_MAP_x else initial_MAP_x
        current_MAP_y = current_MAP_y if current_MAP_y else initial_MAP_y

        quadrant = self.determineQuadrant()

        MAP[initial_MAP_y][initial_MAP_x] = 80
        front, front_next, right, right_next, left, left_next = self.mapSurroundings(quadrant, current_MAP_y, current_MAP_x)
        
        MAP[current_MAP_y][current_MAP_x] = 90
        MAP_array = np.array(MAP)
        unvisited_y, unvisited_x = np.where(MAP_array == 60)
        current_y, current_x = np.where(MAP_array == 90)

        MAP[current_MAP_y][current_MAP_x] = 80
        
        if self.should_turn_right(right_next, right, front_next):
            self.turn_right_90()
        elif self.should_turn_left(left_next, left, front_next):
            self.turn_left_90()
        elif front == 20 and front_next != 80:
            self.move(quadrant)
        else:
            if not unvisited_y.size or not unvisited_x.size:
                print("No more cells to explore. Exiting.")
                self.save_map(MAP, initial_MAP_x, initial_MAP_y)
                quit()
        
            list_movements = self.find_path(MAP_array, unvisited_x, unvisited_y, current_x, current_y)
            self.navigate_path(list_movements)
        
        self.driveMotors(0, 0)
        self.save_map(MAP, initial_MAP_x, initial_MAP_y)


    def get_current_position(self, GPS, start, end, step):
        gps_grid = [i for i in range(start, end, step)]
        return gps_grid[find_next_cell(gps_grid, GPS)]

    def should_turn_right(self, right_next, right, front_next):
        return right_next == 60 and right == 20 and front_next != 60

    def should_turn_left(self, left_next, left, front_next):
        return left_next == 60 and left == 20 and front_next != 60

def find_next_cell(list, N):
    cells = []
    for i in list:
        cells.append(abs(N - i))
    return cells.index(min(cells))

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horijontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)

    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
