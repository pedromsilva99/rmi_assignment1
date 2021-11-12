
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

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

        # Our variables
        self.init_val = 0
        self.offset_x = 0
        self.offset_y = 0
        self.last_pos = (0, 0)
        self.next_pos = (0, 0)
        self.walk = 0
        self.first_call = 1

        self.go_left = False
        self.go_right = False
        self.go_back = False
        self.go_front = False

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
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


    def stop_movement(self, next_pos):
        if self.next_pos[0] >= self.measures.x - 0.25 and self.next_pos[0] <= self.measures.x + 0.25 \
        and self.next_pos[1] >= self.measures.y - 0.25 and self.next_pos[1] <= self.measures.y + 0.25:
            return True
        else:
            return False

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        # print(self.measures.compass)
        if self.init_val == 0:
            self.init_val = 1
            self.offset_x = self.measures.x
            self.offset_y = self.measures.y
            self.last_pos = (self.offset_x, self.offset_y)
            # print('Initial x: ' + str(self.offset_x))
            # print('Initial y: ' + str(self.offset_y))
            # print(self.measures.irSensor[center_id])
            # print(self.measures.irSensor[right_id])
            # print(self.measures.irSensor[left_id])
            # print(self.measures.irSensor[back_id])

        if self.next_pos == (0, 0):
            # print(self.measures.irSensor[center_id])
            # print(self.measures.irSensor[right_id])
            # print(self.measures.irSensor[left_id])
            # print(self.measures.irSensor[back_id])
            print(self.measures.compass)
            if self.measures.irSensor[left_id] < 1.2:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                print('Next position: ' + str(self.next_pos))
                self.go_front = False
                self.go_left = True
                self.go_right = False
                self.go_back = False
            elif self.measures.irSensor[center_id] < 1.2:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
                print('Next position: ' + str(self.next_pos))
                self.go_front = True
                self.go_left = False
                self.go_right = False
                self.go_back = False
            elif self.measures.irSensor[right_id] < 1.2:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
                print('Next position: ' + str(self.next_pos))
                self.go_front = False
                self.go_left = False
                self.go_right = True
                self.go_back = False
            else:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                print('Next position: ' + str(self.next_pos))
                self.go_front = False
                self.go_left = False
                self.go_right = False
                self.go_back = True



        if self.go_left:
            if self.next_pos[0] > self.last_pos[0]:
                if self.first_call:
                    if self.turn(0, 'left') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            elif self.next_pos[0] < self.last_pos[0]:
                if self.first_call:
                    if self.turn(-180, 'left') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            elif self.next_pos[1] > self.last_pos[1]:
                if self.first_call:
                    if self.turn(90, 'left') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            elif self.next_pos[1] < self.last_pos[1]:
                if self.first_call:
                    if self.turn(-90, 'left') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            if self.stop_movement(self.next_pos):
                print('Stopped on position: ' + str(self.next_pos))
                self.first_call = 1
                self.last_pos = self.next_pos
                self.next_pos = (0, 0)
        if self.go_front:
            if self.next_pos[0] > self.last_pos[0]:
                if self.first_call:
                    if self.turn(0, 'left') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            elif self.next_pos[0] < self.last_pos[0]:
                if self.first_call:
                    if self.turn(-180, 'left') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            elif self.next_pos[1] > self.last_pos[1]:
                if self.first_call:
                    if self.turn(90, 'left') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            elif self.next_pos[1] < self.last_pos[1]:
                if self.first_call:
                    if self.turn(-90, 'left') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            if self.stop_movement(self.next_pos):
                print('Stopped on position: ' + str(self.next_pos))
                self.first_call = 1
                self.last_pos = self.next_pos
                self.next_pos = (0, 0)
        if self.go_right:
            if self.next_pos[0] > self.last_pos[0]:
                if self.first_call:
                    if self.turn(0, 'right') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            elif self.next_pos[0] < self.last_pos[0]:
                if self.first_call:
                    if self.turn(-180, 'right') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            elif self.next_pos[1] > self.last_pos[1]:
                if self.first_call:
                    if self.turn(90, 'right') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            elif self.next_pos[1] < self.last_pos[1]:
                if self.first_call:
                    if self.turn(-90, 'right') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            if self.stop_movement(self.next_pos):
                print('Stopped on position: ' + str(self.next_pos))
                self.first_call = 1
                self.last_pos = self.next_pos
                self.next_pos = (0, 0)
        if self.go_back:
            if self.next_pos[0] > self.last_pos[0]:
                if self.first_call:
                    if self.turn(0, 'left') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            elif self.next_pos[0] < self.last_pos[0]:
                if self.first_call:
                    if self.turn(-180, 'left') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            elif self.next_pos[1] > self.last_pos[1]:
                if self.first_call:
                    if self.turn(90, 'left') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            elif self.next_pos[1] < self.last_pos[1]:
                if self.first_call:
                    if self.turn(-90, 'left') == 1:
                        self.driveMotors(0.12, 0.12)
                        self.first_call = 0
            if self.stop_movement(self.next_pos):
                print('Stopped on position: ' + str(self.next_pos))
                self.first_call = 1
                self.last_pos = self.next_pos
                self.next_pos = (0, 0)


        # if (self.measures.x < (self.offset_x + 2)):
        #     self.driveMotors(0.12, 0.12)
        # else:
        #     print(self.measures.compass)
        #     if self.measures.compass < 100 and self.measures.compass > 80:
        #         self.driveMotors(0.12, 0.12)
        #     else:
        #         self.driveMotors(-0.05, 0.05)


        # print('x: ' + str(self.measures.x))
        # print('y: ' + str(self.measures.y))
        # print('compass: ' + str(self.measures.compass))

    def turn(self, degrees, direction):
        if(degrees == -180 or degrees == 180):
            if self.walk == 4:
                # print("PRONTO PARA SEGUIR")
                self.walk = 0
                return 1
                #return True
            elif (self.measures.compass<(180-15) and self.measures.compass>(-180+15) and direction == 'left'):
                self.driveMotors(-0.10, 0.10)
            elif (self.measures.compass<(180-15) and self.measures.compass>(-180+15) and direction == 'right'):
                self.driveMotors(0.10, -0.10)
            elif (self.measures.compass>(180-10) and self.measures.compass<(180-2)):
                self.driveMotors(-0.05, 0.05)
            elif (self.measures.compass>(-180+2) and self.measures.compass<(-180+10)):
                self.driveMotors(0.05, -0.05)
            elif (self.measures.compass>(180-6) and self.measures.compass<(180-1)):
                self.driveMotors(-0.005,0.005)
            elif (self.measures.compass<(-180+6) and self.measures.compass>(-180+1)):
                self.driveMotors(0.005,-0.005)
            elif(self.measures.compass<=(-180+1) and self.measures.compass>=(-180)) or (self.measures.compass>=(180-1) and self.measures.compass<=(180)):
                if(self.measures.compass==(-180+1)):
                    self.driveMotors(0.004,-0.004)
                elif(self.measures.compass==(180-1)):
                    self.driveMotors(-0.004,0.004)
                else:
                    self.walk += 1
                    self.driveMotors(0,0)
        elif self.walk == 4:
            # print("PRONTO PARA SEGUIR")
            self.walk = 0
            return 1
            #return True
        elif (self.measures.compass<(degrees-15) or self.measures.compass>(degrees+15) and direction == 'left'):
            self.driveMotors(-0.10, 0.10)
        elif (self.measures.compass<(degrees-15) or self.measures.compass>(degrees+15) and direction == 'right'):
            self.driveMotors(0.10, -0.10)
        elif (self.measures.compass>(degrees-10) and self.measures.compass<(degrees-2)):
            self.driveMotors(-0.05, 0.05)
        elif (self.measures.compass>(degrees+2) and self.measures.compass<(degrees+10)):
            self.driveMotors(0.05, -0.05)
        elif (self.measures.compass>(degrees-6) and self.measures.compass<(degrees-1)):
            self.driveMotors(-0.005,0.005)
        elif (self.measures.compass<(degrees+6) and self.measures.compass>(degrees+1)):
            self.driveMotors(0.005,-0.005)
        elif(self.measures.compass<=(degrees+1) and self.measures.compass>=(degrees-1)):
            if(self.measures.compass==(degrees+1)):
                self.driveMotors(0.004,-0.004)
            elif(self.measures.compass==(degrees-1)):
                self.driveMotors(-0.004,0.004)
            else:
                self.walk += 1
                self.driveMotors(0,0)
        else:
            pass

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
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None

           i=i+1


rob_name = "pClient2"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
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
