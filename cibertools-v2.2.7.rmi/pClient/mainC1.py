
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
        self.background_flag = False
        self.checkpoint = 10
        self.back_s = 0
        self.front_s = 0
        self.i = 0
        self.first_ground_value = True

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


    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        # Sees if the center sensor is close to a wall
        if self.measures.irSensor[center_id] > 1.5:
            # Decides where to go based on the distance to the walls on the side vectors
            if self.measures.irSensor[right_id] > self.measures.irSensor[left_id]:
                self.driveMotors(-0.15, 0.15)
            else:
                self.driveMotors(0.15, -0.15)
        # Adjust the position to not collide with side walls
        elif self.measures.irSensor[right_id] > 1.7 and self.measures.irSensor[right_id] < 2.7:
            self.driveMotors(0.11, 0.15)
        elif self.measures.irSensor[left_id] > 1.7 and self.measures.irSensor[left_id] < 2.7:
            self.driveMotors(0.15, 0.11)
        elif self.measures.irSensor[right_id] >= 2.7 and self.measures.irSensor[right_id] <= 3.7:
            self.driveMotors(0.09, 0.15)
        elif self.measures.irSensor[left_id] >= 2.7 and self.measures.irSensor[left_id] <= 3.7:
            self.driveMotors(0.15, 0.09)
        elif self.measures.irSensor[right_id] > 3.7:
            self.driveMotors(-0.04, 0.10)
        elif self.measures.irSensor[left_id] > 3.7:
            self.driveMotors(0.10, -0.04)
        # Go forward
        else:
            self.driveMotors(0.15, 0.15)

        # Verifies if the robot is going backwards
        if self.measures.ground != -1:
            if self.measures.ground != self.checkpoint and self.first_ground_value:
                print("Checkpoint: " + str(self.measures.ground))
                self.checkpoint = self.measures.ground
                self.first_ground_value = False
            elif self.measures.ground == self.checkpoint and self.first_ground_value:
                self.background_flag = True
                self.first_ground_value = False
        else:
            self.first_ground_value = True

        # When the robot is going in the wrong direction
        if self.background_flag:
            if self.i == 0:
                self.i = 1
                self.back_s = self.measures.irSensor[back_id]
                self.front_s = self.measures.irSensor[center_id]
                self.driveMotors(-0.15,0.15)
            elif self.i <=3:
                self.i += 1
                self.driveMotors(-0.15,0.15)
            else:
                if self.compare(self.measures.irSensor[back_id],self.measures.irSensor[center_id]):
                    self.background_flag = False
                else:
                    self.driveMotors(-0.03,0.03)

    # Compares the values stored from the center and back sensors with their previous values
    def compare(self, back,front):
        if front > self.back_s - 0.2 and front < self.back_s + 0.2 and back > self.front_s - 0.2 and back < self.front_s + 0.2:
            return True
        else:
            return False

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
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
