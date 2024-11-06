import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = 0
    
    # Atualiza o controlador PID com o erro e o intervalo de tempo
    def update(self, error, dt):
        # Cálculo do termo integral
        self.integral += error * dt
        # Cálculo do termo derivativo
        derivative = (error - self.previous_error) / dt
        # PID Output
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

class SensorFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = []

    def update(self, new_value):
        self.values.append(new_value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return sum(self.values) / len(self.values)

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.at_intersection = False
        self.intersection_timer = 0

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
                    self.setVisitingLed(True);
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

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2

        safe_distance_front = 1.5
        dead_zone = 1.1

        base_speed = 0.13
        turn_speed = 0.15

        # Controladores PID para ajuste lateral e curvas
        lateral_pid = PIDController(Kp=0.15, Ki=0.02, Kd=0.02)
        turn_pid = PIDController(Kp=0.5, Ki=0.02, Kd=0.03)

        # Leitura dos sensores
        left_distance = self.measures.irSensor[left_id]
        right_distance = self.measures.irSensor[right_id]
        front_distance = self.measures.irSensor[center_id]

        # Limitar ajustes laterais com base em max_adjustment
        max_adjustment = 0.02

        if left_distance < dead_zone and right_distance < dead_zone:
            # Intersecção detectada
            adjusted_left_distance = 2.2
            adjusted_right_distance = 2.2
            error = adjusted_left_distance - adjusted_right_distance
            lateral_output = lateral_pid.update(error, dt=0.1)
            lateral_output = max(min(lateral_output, max_adjustment), -max_adjustment)
            self.driveMotors(base_speed + lateral_output, base_speed - lateral_output)
        else:
            if front_distance > safe_distance_front:
                # Obstáculo à frente, fazer curva
                error = left_distance - right_distance
                turn_output = turn_pid.update(error, dt=0.1)
                if right_distance <= dead_zone or left_distance < right_distance:
                    self.driveMotors(turn_output, -turn_output)  # Curva à esquerda
                elif left_distance <= dead_zone or right_distance < left_distance:
                    self.driveMotors(-turn_output, turn_output)  # Curva à direita
                else:
                    self.driveMotors(base_speed, base_speed)
            else:
                # Caminho livre, continuar reto
                error = left_distance - right_distance
                lateral_output = lateral_pid.update(error, dt=0.1)
                lateral_output = max(min(lateral_output, max_adjustment), -max_adjustment)
                self.driveMotors(base_speed + lateral_output, base_speed - lateral_output)

        # Tentar recuperar de colisões
        if self.measures.collision:
            self.driveMotors(-0.1, -0.1)
            if left_distance > right_distance:
                self.driveMotors(turn_speed, -turn_speed)
            else:
                self.driveMotors(-turn_speed, turn_speed)

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "agent"
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
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()