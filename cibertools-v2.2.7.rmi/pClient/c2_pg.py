
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
        self.count_intersection = 0
        self.intersections_ls = []
        self.visited_squares = []
        self.squares_to_visit = []
        self.do_astar = True
        # w, h = 55, 27
        # self.matrix = [[' ' for x in range(w)] for y in range(h)]
        # for i in self.matrix:
        #     print(i)
        w, h = 55, 27
        self.matrix = [[' ' for x in range(w)] for y in range(h)]
        self.maze = [['1' for x in range(w)] for y in range(h)]
        print(len(self.matrix[0]))




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

        # if self.do_astar:
        #     maze = [
        #     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
        #
        #     # (y, x) for some reason
        #     start = (0, 0)
        #     end = (3, 5)
        #
        #     path = astar(maze, start, end)
        #     print(path)
        #     return

        # print(self.measures.compass)
        if self.init_val == 0:
            self.init_val = 1
            self.offset_x = self.measures.x
            self.offset_y = self.measures.y
            self.last_pos = (self.offset_x, self.offset_y)
            self.matrix[13][27] = 'O'
            self.maze[13][27] = '0'
            self.visited_squares.append((13, 27))

            if self.measures.irSensor[center_id] < 1.2:
                self.matrix[13][28] = 'X'
                self.maze[13][28] = '0'
                self.squares_to_visit.append((13, 29))
            else:
                self.matrix[13][28] = '|'
            if self.measures.irSensor[right_id] < 1.2:
                self.matrix[27 - 13][27] = 'X'
                self.maze[27 - 13][27] = '0'
                self.squares_to_visit.append((27 - 13, 27))
            else:
                self.matrix[27 - 13][27] = '-'
            if self.measures.irSensor[left_id] < 1.2:
                self.matrix[25 - 13][27] = 'X'
                self.maze[25 - 13][27] = '0'
                self.squares_to_visit.append((25 - 13, 27))
            else:
                self.matrix[25 - 13][27] = '-'
            if self.measures.irSensor[back_id] < 1.2:
                self.matrix[13][26] = 'X'
                self.maze[13][26] = '0'
                self.squares_to_visit.append((13, 25))
            else:
                self.matrix[13][26] = '|'
            print(self.squares_to_visit)
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
            if self.measures.irSensor[left_id] < 1.2:
                self.count_intersection += 1
            if self.measures.irSensor[center_id] < 1.2:
                self.count_intersection += 1
            if self.measures.irSensor[right_id] < 1.2:
                self.count_intersection += 1
            if self.measures.irSensor[back_id] < 1.2:
                self.count_intersection += 1
            print(self.measures.compass)

            for t in self.visited_squares:
                if t in self.squares_to_visit:
                    self.squares_to_visit.remove(t)

            # Mark the walls
            if self.measures.irSensor[left_id] > 1.2:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[25 - self.pos[1]][self.pos[0]] = '-'
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] - 1] = '|'
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[27 - self.pos[1]][self.pos[0]] = '-'
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] + 1] = '|'
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
            if self.measures.irSensor[right_id] > 1.2:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[27 - self.pos[1]][self.pos[0]] = '-'
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] + 1] = '|'
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[25 - self.pos[1]][self.pos[0]] = '-'
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] - 1] = '|'
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
            if self.measures.irSensor[center_id] > 1.2:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] + 1] = '|'
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[25 - self.pos[1]][self.pos[0]] = '-'
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] - 1] = '|'
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[27 - self.pos[1]][self.pos[0]] = '-'
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
            # Mark the squares
            if self.measures.irSensor[center_id] < 1.2:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] + 1] = 'X'
                    self.maze[26 - self.pos[1]][self.pos[0] + 1] = '0'
                    if (26 - self.pos[1], self.pos[0] + 2) not in self.squares_to_visit and (26 - self.pos[1], self.pos[0] + 2) not in self.visited_squares:
                        self.squares_to_visit.append((26 - self.pos[1], self.pos[0] + 2))
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[25 - self.pos[1]][self.pos[0]] = 'X'
                    self.maze[25 - self.pos[1]][self.pos[0]] = '0'
                    if (24 - self.pos[1], self.pos[0]) not in self.squares_to_visit and (24 - self.pos[1], self.pos[0]) not in self.visited_squares:
                        self.squares_to_visit.append((24 - self.pos[1], self.pos[0]))
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] - 1] = 'X'
                    self.maze[26 - self.pos[1]][self.pos[0] - 1] = '0'
                    if (26 - self.pos[1], self.pos[0] - 2) not in self.squares_to_visit and (26 - self.pos[1], self.pos[0] - 2) not in self.visited_squares:
                        self.squares_to_visit.append((26 - self.pos[1], self.pos[0] - 2))
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[27 - self.pos[1]][self.pos[0]] = 'X'
                    self.maze[27 - self.pos[1]][self.pos[0]] = '0'
                    if (28 - self.pos[1], self.pos[0]) not in self.squares_to_visit and (28 - self.pos[1], self.pos[0]) not in self.visited_squares:
                        self.squares_to_visit.append((28 - self.pos[1], self.pos[0]))
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
            if self.measures.irSensor[right_id] < 1.2:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[27 - self.pos[1]][self.pos[0]] = 'X'
                    self.maze[27 - self.pos[1]][self.pos[0]] = '0'
                    if (28 - self.pos[1], self.pos[0]) not in self.squares_to_visit and (28 - self.pos[1], self.pos[0]) not in self.visited_squares:
                        self.squares_to_visit.append((28 - self.pos[1], self.pos[0]))
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] + 1] = 'X'
                    self.maze[26 - self.pos[1]][self.pos[0] + 1] = '0'
                    if (26 - self.pos[1], self.pos[0] + 2) not in self.squares_to_visit and (26 - self.pos[1], self.pos[0] + 2) not in self.visited_squares:
                        self.squares_to_visit.append((26 - self.pos[1], self.pos[0] + 2))
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[25 - self.pos[1]][self.pos[0]] = 'X'
                    self.maze[25 - self.pos[1]][self.pos[0]] = '0'
                    if (24 - self.pos[1], self.pos[0]) not in self.squares_to_visit and (24 - self.pos[1], self.pos[0]) not in self.visited_squares:
                        self.squares_to_visit.append((24 - self.pos[1], self.pos[0]))
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] - 1] = 'X'
                    self.maze[26 - self.pos[1]][self.pos[0] - 1] = '0'
                    if (26 - self.pos[1], self.pos[0] - 2) not in self.squares_to_visit and (26 - self.pos[1], self.pos[0] - 2) not in self.visited_squares:
                        self.squares_to_visit.append((26 - self.pos[1], self.pos[0] - 2))
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
            if self.measures.irSensor[left_id] < 1.2:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[25 - self.pos[1]][self.pos[0]] = 'X'
                    self.maze[25 - self.pos[1]][self.pos[0]] = '0'
                    if (24 - self.pos[1], self.pos[0]) not in self.squares_to_visit and (24 - self.pos[1], self.pos[0]) not in self.visited_squares:
                        self.squares_to_visit.append((24 - self.pos[1], self.pos[0]))
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] - 1] = 'X'
                    self.maze[26 - self.pos[1]][self.pos[0] - 1] = '0'
                    if (26 - self.pos[1], self.pos[0] - 2) not in self.squares_to_visit and (26 - self.pos[1], self.pos[0] - 2) not in self.visited_squares:
                        self.squares_to_visit.append((26 - self.pos[1], self.pos[0] - 2))
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[27 - self.pos[1]][self.pos[0]] = 'X'
                    self.maze[27 - self.pos[1]][self.pos[0]] = '0'
                    if (28 - self.pos[1], self.pos[0]) not in self.squares_to_visit and (28 - self.pos[1], self.pos[0]) not in self.visited_squares:
                        self.squares_to_visit.append((28 - self.pos[1], self.pos[0]))
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] + 1] = 'X'
                    self.maze[26 - self.pos[1]][self.pos[0] + 1] = '0'
                    if (26 - self.pos[1], self.pos[0] + 2) not in self.squares_to_visit and (26 - self.pos[1], self.pos[0] + 2) not in self.visited_squares:
                        self.squares_to_visit.append((26 - self.pos[1], self.pos[0] + 2))
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                print('Next position: ' + str(self.next_pos))
                self.go_front = False
                self.go_left = True
                self.go_right = False
                self.go_back = False
            elif self.measures.irSensor[center_id] < 1.2:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] + 1] = 'X'
                    self.maze[26 - self.pos[1]][self.pos[0] + 1] = '0'
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[25 - self.pos[1]][self.pos[0]] = 'X'
                    self.maze[25 - self.pos[1]][self.pos[0]] = '0'
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] - 1] = 'X'
                    self.maze[26 - self.pos[1]][self.pos[0] - 1] = '0'
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[27 - self.pos[1]][self.pos[0]] = 'X'
                    self.maze[27 - self.pos[1]][self.pos[0]] = '0'
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
                print('Next position: ' + str(self.next_pos))
                self.go_front = True
                self.go_left = False
                self.go_right = False
                self.go_back = False
            elif self.measures.irSensor[right_id] < 1.2:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[27 - self.pos[1]][self.pos[0]] = 'X'
                    self.maze[27 - self.pos[1]][self.pos[0]] = '0'
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] + 1] = 'X'
                    self.maze[26 - self.pos[1]][self.pos[0] + 1] = '0'
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[25 - self.pos[1]][self.pos[0]] = 'X'
                    self.maze[25 - self.pos[1]][self.pos[0]] = '0'
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] - 1] = 'X'
                    self.maze[26 - self.pos[1]][self.pos[0] - 1] = '0'
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
            if self.count_intersection >= 3:
                if (self.last_pos[0], self.last_pos[1]) not in self.intersections_ls:
                    self.intersections_ls.append((self.last_pos[0], self.last_pos[1]))
                # print("Intersection on: " + str(self.last_pos[0]) + ', ' + str(self.last_pos[1]))
                print('Intersections: ' + str(self.intersections_ls))
            print('To visit' + str(self.squares_to_visit))

        self.count_intersection = 0
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

                self.pos = ((int(self.next_pos[0]) - int(self.offset_x) + 27), int(self.next_pos[1]) - int(self.offset_y) + 13)

                # print('Stopped on position: ' + str(self.pos))
                #
                # print(self.pos[0])
                # print(self.pos[1])
                self.matrix[26 - self.pos[1]][self.pos[0]] = 'X'
                self.maze[26 - self.pos[1]][self.pos[0]] = '0'

                if (26 - self.pos[1], self.pos[0]) not in self.visited_squares:
                    self.visited_squares.append((26 - self.pos[1], self.pos[0]))

                # for i in self.maze:
                #     print(''.join(i))

                # print(self.visited_squares)

                # for i in self.maze:
                #     print(i)


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
                self.pos = ((int(self.next_pos[0]) - int(self.offset_x) + 27), int(self.next_pos[1]) - int(self.offset_y) + 13)

                # print('Stopped on position: ' + str(self.pos))
                #
                # print(self.pos[0])
                # print(self.pos[1])
                self.matrix[26 - self.pos[1]][self.pos[0]] = 'X'
                self.maze[26 - self.pos[1]][self.pos[0]] = '0'

                if (26 - self.pos[1], self.pos[0]) not in self.visited_squares:
                    self.visited_squares.append((26 - self.pos[1], self.pos[0]))

                # print(self.visited_squares)
                # for i in self.matrix:
                #     print(''.join(i))

                # for i in self.maze:
                #     print(i)

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
                self.pos = ((int(self.next_pos[0]) - int(self.offset_x) + 27), int(self.next_pos[1]) - int(self.offset_y) + 13)

                # print('Stopped on position: ' + str(self.pos))
                #
                # print(self.pos[0])
                # print(self.pos[1])
                self.matrix[26 - self.pos[1]][self.pos[0]] = 'X'
                self.maze[26 - self.pos[1]][self.pos[0]] = '0'

                if (26 - self.pos[1], self.pos[0]) not in self.visited_squares:
                    self.visited_squares.append((26 - self.pos[1], self.pos[0]))

                # print(self.visited_squares)

                # for i in self.matrix:
                #     print(''.join(i))

                # for i in self.maze:
                #     print(i)

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
                self.pos = ((int(self.next_pos[0]) - int(self.offset_x) + 27), int(self.next_pos[1]) - int(self.offset_y) + 13)

                # print('Stopped on position: ' + str(self.pos))
                #
                # print(self.pos[0])
                # print(self.pos[1])
                self.matrix[26 - self.pos[1]][self.pos[0]] = 'X'
                self.maze[26 - self.pos[1]][self.pos[0]] = '0'

                if (26 - self.pos[1], self.pos[0]) not in self.visited_squares:
                    self.visited_squares.append((26 - self.pos[1], self.pos[0]))

                # print(self.visited_squares)

                # for i in self.maze:
                #     print(''.join(i))

                # for i in self.maze:
                #     print(i)

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
            elif (self.measures.compass<(180-15) and self.measures.compass>(-180+15)):
                if(direction == 'left'):
                    self.driveMotors(-0.10, 0.10)
                else:
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
        elif (self.measures.compass<(degrees-15) or self.measures.compass>(degrees+15)):
            if(direction == 'left'):
                self.driveMotors(-0.10, 0.10)
            else:
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

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)


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
