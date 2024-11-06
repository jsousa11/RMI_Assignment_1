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
    
    # Atualiza o controlador PID com o erro e o tempo decorrido
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
        self.pid_x = PIDController(Kp=0.01, Ki=0.01, Kd=0.1)
        self.pid_y = PIDController(Kp=0.01, Ki=0.01, Kd=0.1)

        global GRID_MAP, gps_start_x, gps_start_y
        global initial_map_x, initial_map_y, map_current_x, map_current_y
        
        # Inicializa o mapa do labirinto a ser explorado com valores padrão
        GRID_MAP = [[10] * 55 for _ in range(27)]
        gps_start_x, gps_start_y = self.measures.x, self.measures.y

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

    # Saber a orientação do robô
    def determine_orientation(self):
        self.readSensors()
        compass_dir = self.measures.compass
        dir = [False, False, False, False]

        if abs(compass_dir) <= 45:
            dir[0] = True
        elif compass_dir > 45 and compass_dir <= 135:
            dir[1] = True
        elif abs(compass_dir) >= 135:
            dir[2] = True
        elif compass_dir <= -45 and compass_dir >= -135:
            dir[3] = True
        return dir

    # Mapear as células laterais ao robô
    def mapSurroundings(self, dir, y, x):
        center_id = 0
        left_id = 1
        right_id = 2
        dist_tolerance = 1.15

        self.readSensors()
        center_sensor = self.measures.irSensor[center_id]
        left_sensor = self.measures.irSensor[left_id]
        right_sensor = self.measures.irSensor[right_id]

        # Atualiza o mapa com as paredes detectadas
        def update_map(sensor_value, threshold, coords, wall_value, empty_value, explore_value):
            if sensor_value >= threshold:
                GRID_MAP[coords[0]][coords[1]] = wall_value
            else:
                GRID_MAP[coords[0]][coords[1]] = empty_value
                if GRID_MAP[coords[2]][coords[3]] != 80:
                    GRID_MAP[coords[2]][coords[3]] = explore_value

        if dir[0]:  # Virado para a direita
            update_map(center_sensor, dist_tolerance, (y, x + 1, y, x + 2), 30, 20, 60)
            update_map(left_sensor, dist_tolerance, (y - 1, x, y - 2, x), 40, 20, 60)
            update_map(right_sensor, dist_tolerance, (y + 1, x, y + 2, x), 40, 20, 60)
            return GRID_MAP[y][x + 1], GRID_MAP[y][x + 2], GRID_MAP[y + 1][x], GRID_MAP[y + 2][x], GRID_MAP[y - 1][x], GRID_MAP[y - 2][x]

        elif dir[1]:  # Virado para cima
            update_map(center_sensor, dist_tolerance, (y - 1, x, y - 2, x), 40, 20, 60)
            update_map(left_sensor, dist_tolerance, (y, x - 1, y, x - 2), 30, 20, 60)
            update_map(right_sensor, dist_tolerance, (y, x + 1, y, x + 2), 30, 20, 60)
            return GRID_MAP[y - 1][x], GRID_MAP[y - 2][x], GRID_MAP[y][x + 1], GRID_MAP[y][x + 2], GRID_MAP[y][x - 1], GRID_MAP[y][x - 2]

        elif dir[2]:  # Virado para a esquerda
            update_map(center_sensor, dist_tolerance, (y, x - 1, y, x - 2), 30, 20, 60)
            update_map(left_sensor, dist_tolerance, (y + 1, x, y + 2, x), 40, 20, 60)
            update_map(right_sensor, dist_tolerance, (y - 1, x, y - 2, x), 40, 20, 60)
            return GRID_MAP[y][x - 1], GRID_MAP[y][x - 2], GRID_MAP[y - 1][x], GRID_MAP[y - 2][x], GRID_MAP[y + 1][x], GRID_MAP[y + 2][x]

        elif dir[3]:  # Virado para baixo
            update_map(center_sensor, dist_tolerance, (y + 1, x, y + 2, x), 40, 20, 60)
            update_map(left_sensor, dist_tolerance, (y, x + 1, y, x + 2), 30, 20, 60)
            update_map(right_sensor, dist_tolerance, (y, x - 1, y, x - 2), 30, 20, 60)
            return GRID_MAP[y + 1][x], GRID_MAP[y + 2][x], GRID_MAP[y][x - 1], GRID_MAP[y][x - 2], GRID_MAP[y][x + 1], GRID_MAP[y + 2][x]

    # Mover o robô para a próxima célula
    def move(self, directional_states):
        self.readSensors()
        
        gps_x = self.measures.x - gps_start_x
        gps_y = self.measures.y - gps_start_y

        x_positions = list(range(-26, 28, 2))
        map_gps_x = x_positions[find_next_cell(x_positions, gps_x)]

        y_positions = list(range(-12, 14, 2))
        map_gps_y = y_positions[find_next_cell(y_positions, gps_y)]
        
        error_x = error_y = 100
        lin_speed = 0.13
        tolerance = 0.225

        while any(directional_states) and (error_x > tolerance or error_y > tolerance):
            self.readSensors()
            gps_x = self.measures.x - gps_start_x
            gps_y = self.measures.y - gps_start_y

            if directional_states[0]:  # Direita
                error_x = (map_gps_x + 2) - gps_x
                error_y = map_gps_y - gps_y
                rot_correction = self.pid_y.update(error_y, dt=0.05)
                right_rotation = lin_speed + rot_correction
                left_rotation = lin_speed - rot_correction
                self.driveMotors(left_rotation, right_rotation)

            if directional_states[1]:  # Cima
                error_x = map_gps_x - gps_x
                error_y = (map_gps_y + 2) - gps_y
                rot_correction = self.pid_x.update(error_x, dt=0.1)
                right_rotation = lin_speed - rot_correction
                left_rotation = lin_speed + rot_correction
                self.driveMotors(left_rotation, right_rotation)

            if directional_states[2]:  # Esquerda
                error_x = gps_x - (map_gps_x - 2)
                error_y = gps_y - map_gps_y
                rot_correction = self.pid_y.update(error_y, dt=0.1)
                right_rotation = lin_speed + rot_correction
                left_rotation = lin_speed - rot_correction
                self.driveMotors(left_rotation, right_rotation)

            if directional_states[3]:  # Baixo
                error_x = gps_x - map_gps_x
                error_y = gps_y - (map_gps_y - 2)
                rot_correction = self.pid_x.update(error_x, dt=0.1)
                right_rotation = lin_speed - rot_correction
                left_rotation = lin_speed + rot_correction
                self.driveMotors(left_rotation, right_rotation)
    
    # Encontrar o caminho mais curto para a próxima célula (Algoritmo de busca em largura - BFS)
    def find_path(self, map_array, unvisited_x, unvisited_y, current_x, current_y):
        try:
            unvisited_positions = list(zip(unvisited_y, unvisited_x))
            unvisited_positions.sort(key=lambda pos: abs(pos[0] - current_y) + abs(pos[1] - current_x))  # Ordena pela proximidade
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

    # Direcionar o robô para a próxima célula
    def navigate_path(self, next_moves):
        def adjust_orientation(dir, direction_index):
            while not dir[direction_index]:
                self.rotate_90("left")
                dir = self.determine_orientation()
            return dir

        def move_and_update_quadrant(dir):
            self.move(dir)
            return self.determine_orientation()

        def get_rotation_action(prev_direction, next_direction):
            rotation_map = {
                ('LEFT', 'DOWN'): lambda: self.rotate_90("left"),
                ('LEFT', 'UP'): lambda: self.rotate_90("right"),
                ('RIGHT', 'DOWN'): lambda: self.rotate_90("right"),
                ('RIGHT', 'UP'): lambda: self.rotate_90("left"),
                ('UP', 'LEFT'): lambda: self.rotate_90("left"),
                ('UP', 'RIGHT'): lambda: self.rotate_90("right"),
                ('DOWN', 'LEFT'): lambda: self.rotate_90("right"),
                ('DOWN', 'RIGHT'): lambda: self.rotate_90("left")
            }
            return rotation_map.get((prev_direction, next_direction))

        dir = self.determine_orientation()
        direction_map = {
            'LEFT': 2,
            'RIGHT': 0,
            'UP': 1,
            'DOWN': 3
        }

        if next_moves[0] in direction_map:
            dir = adjust_orientation(dir, direction_map[next_moves[0]])

        dir = move_and_update_quadrant(dir)

        for i in range(1, len(next_moves)):
            if next_moves[i] == next_moves[i - 1]:
                dir = move_and_update_quadrant(dir)
            else:
                rotation_function = get_rotation_action(next_moves[i - 1], next_moves[i])
                if rotation_function:
                    rotation_function()
                    dir = self.determine_orientation()
                    dir = move_and_update_quadrant(dir)

    # Salvar o mapa em um arquivo de texto
    def save_map(self, GRID_MAP, start_x, start_y):
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

        # Define a posição inicial
        GRID_MAP[start_y][start_x] = 50

        MAP_str = "\n".join("".join(map_symbol(cell) for cell in row) for row in GRID_MAP)

        with open('mymap.txt', 'w') as f:
            f.write(MAP_str)

    # Função principal para explorar o labirinto
    def wander(self):
        global gps_start_x, gps_start_y, map_gps_x, map_gps_y, GRID_MAP
        global map_current_y, current_MAP_y, initial_map_x, initial_map_y

        self.readSensors()

        compass = self.measures.compass
        gps_x = self.measures.x - gps_start_x
        gps_y = self.measures.y - gps_start_y

        map_gps_x = self.get_current_position(gps_x, -26, 28, 2)
        map_gps_y = self.get_current_position(gps_y, -12, 14, 2)

        initial_map_x, initial_map_y = 27, 13
        map_current_y = initial_map_x + map_gps_x
        current_MAP_y = initial_map_y - map_gps_y

        map_current_y = map_current_y if map_current_y else initial_map_x
        current_MAP_y = current_MAP_y if current_MAP_y else initial_map_y

        dir = self.determine_orientation()

        GRID_MAP[initial_map_y][initial_map_x] = 80
        front, front_next, right, right_next, left, left_next = self.mapSurroundings(dir, current_MAP_y, map_current_y)
        
        GRID_MAP[current_MAP_y][map_current_y] = 90
        MAP_array = np.array(GRID_MAP)
        unvisited_y, unvisited_x = np.where(MAP_array == 60)
        current_y, current_x = np.where(MAP_array == 90)

        GRID_MAP[current_MAP_y][map_current_y] = 80
        
        if self.should_turn(right_next, right, front_next, "right"):
            self.rotate_90("right")
        elif self.should_turn(left_next, left, front_next, "left"):
            self.rotate_90("left")
        elif front == 20 and front_next != 80:
            self.move(dir)
        else:
            if not unvisited_y.size or not unvisited_x.size:
                print("No more cells to explore. Exiting.")
                self.save_map(GRID_MAP, initial_map_x, initial_map_y)
                quit()
        
            list_movements = self.find_path(MAP_array, unvisited_x, unvisited_y, current_x, current_y)
            self.navigate_path(list_movements)
        
        self.driveMotors(0, 0)
        self.save_map(GRID_MAP, initial_map_x, initial_map_y)

    # Verificar se o robô deve virar à esquerda ou à direita
    def should_turn(self, direction_next, direction_current, front_next, turn_direction):
        if turn_direction == "right":
            return direction_next == 60 and direction_current == 20 and front_next != 60
        elif turn_direction == "left":
            return direction_next == 60 and direction_current == 20 and front_next != 60
        return False

    # Rotação de 90 graus
    def rotate_90(self, direction="right"):
        # Define o ângulo alvo baseado na direção
        adjustment_angle = -90 if direction == "right" else 90
        target_compass = self.adjust_to_nearest_90(self.measures.compass) + adjustment_angle

        while True:
            self.readSensors()
            current_compass = self.measures.compass
            rotation_error = self.calculate_rotation_difference(current_compass, target_compass)

            if abs(rotation_error) > 5:
                # Corrige a rotação e ajusta a intensidade da rotação
                self.apply_rotation_error(rotation_error * 0.5)  
            else:
                rotation_error = 0
                break

    # Ajustar o ângulo da bússola para o múltiplo de 90 mais próximo
    def adjust_to_nearest_90(self, compass_angle):
        compass_options = [0, 90, -180, -90, 180]
        closest_angle = compass_options[find_next_cell(compass_options, compass_angle)]
        return -180 if closest_angle == 180 else closest_angle

    # Calcular a diferença de rotação entre o ângulo atual e o alvo
    def calculate_rotation_difference(self, current, target):
        rotation_error = target - current
        if rotation_error > 180:
            rotation_error -= 360
        elif rotation_error < -180:
            rotation_error += 360
        return rotation_error

    # Aplicar a correção de rotação aos motores
    def apply_rotation_error(self, rotation_error):
        rotation_correction = 0.004 * rotation_error
        self.driveMotors(-rotation_correction, rotation_correction)

    def get_current_position(self, GPS, start, end, step):
        gps_grid = [i for i in range(start, end, step)]
        return gps_grid[find_next_cell(gps_grid, GPS)]

# Encontrar a célula mais próxima
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
