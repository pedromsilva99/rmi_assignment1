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
        self.turn_left_signal = 0
        self.turn_right_signal = 0
        self.emergency_signal = 0

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
        self.center_id = 0
        self.left_id = 1
        self.right_id = 2
        self.back_id = 3
        
        print("x: " + str(self.measures.x))
        print("y: " + str(self.measures.y))
        print("bussola: " + str(self.measures.compass))

    def turn_right(self):
        print("virando à direita\n")
        self.driveMotors(0.05,-0.05)
        if self.measures.irSensor[self.center_id] < 0.8:# and self.measures.irSensor[self.left_id] > 1.3:
            self.turn_right_signal = 0

    def turn_left(self):
        print("virando à esquerda\n")
        self.driveMotors(-0.05,0.05)
        if self.measures.irSensor[self.center_id] < 0.8: # and self.measures.irSensor[self.right_id] > 1.3:
            self.turn_left_signal = 0

    def turn_slight_left(self):
        print("ajustando à esquerda\n")
        self.driveMotors(-0.05,0.05)

    def turn_slight_right(self):
        print("ajustando à direita\n")
        self.driveMotors(0.05,-0.05)

    def emergency(self):
        print("Emergencia\n")

        if self.measures.irSensor[self.center_id] < 2.0:
            self.emergency_signal = 0
        elif self.measures.irSensor[self.left_id] < 1.5:
            print("emeregencia à esquerda\n")
            self.driveMotors(-0.05,0.05)
        elif self.measures.irSensor[self.left_id] < 1.5:
            print("emergencia à direita\n")
            self.driveMotors(0.05,-0.05)
        else:
            self.driveMotors(0.05,-0.05)
            print("Sem saída")




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
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
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
