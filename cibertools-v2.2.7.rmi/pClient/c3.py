
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import heapq

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
        self.visited_squares = []
        self.squares_to_visit = []
        self.beacons_ls = []
        self.walls = []
        self.do_astar = False
        self.previous = 0
        self.previous_pos = (100,100)
        self.flag = 0
        self.ls = []
        self.go_to_ls = False
        self.complete_astar = False
        self.checkpoints = True
        self.i = 1
        w, h = 55, 27
        self.matrix = [[' ' for x in range(w)] for y in range(h)]
        self.maze = [[1 for x in range(w)] for y in range(h)]

        for i in range(27):
            for j in range(55):
                self.walls.append((j, i))

        self.go_left = False
        self.go_right = False
        self.go_back = False
        self.go_front = False

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


        if self.do_astar:
            print('ENTROU NO DO ASTAR')

            min = 1000

            start = (self.pos[0], 26 - self.pos[1])
            print("Start: " + str(start))

            for i in self.squares_to_visit:
                a = AStar()
                end = (i[1], i[0])
                print("End: " + str(end))
                a.init_grid(55, 27, self.walls, start, end)

                path = a.solve()
                #print(path)

                try:
                    if len(path) < min:
                        min = len(path)
                        self.ls = path[::2]
                except:
                    pass
            print(self.ls)
            # print(path)
            #for i in self.matrix:
            #    print(''.join(i))
            #print(self.pos)
            #self.go_to_this_pos(ls)
            self.do_astar=False
            self.go_to_ls = True

            # start = (26 - self.pos[1], self.pos[0])
            # end = self.squares_to_visit[0]
            # # end = (13, 29)
            # maze = self.maze
            # print('START: ' + str(start))
            # print('END: ' + str(end))
            # print('MAZE: ' + str(maze))
            # path = astar(maze, start, end)
            # print('Caminho do labirinto: ' + str(path))
            #exit()

        if self.go_to_ls:
            print('i: ' + str(self.i) + ' Len: ' + str(len(self.ls)))

            if self.i == len(self.ls):
                self.i = 1
                self.complete_astar = False
                self.next_pos = (0, 0)
                self.go_to_ls = False
            else:
                print('Entra ')
                self.complete_astar = True
                if (self.ls[self.i-1][0] < self.ls[self.i][0]):
                    self.next_pos = (self.last_pos[0]+2, self.last_pos[1])
                elif (self.ls[self.i-1][0] > self.ls[self.i][0]):
                    self.next_pos = (self.last_pos[0]-2, self.last_pos[1])
                elif (self.ls[self.i-1][1] > self.ls[self.i][1]):
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif (self.ls[self.i-1][1] < self.ls[self.i][1]):
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)

                print("Next pos: " + str(self.next_pos))

                self.go_front = False
                self.go_left = False
                self.go_right = False
                self.go_back = False

                if self.pos[0]>self.ls[self.i][0] and (26-self.pos[1])==self.ls[self.i][1]:
                    if self.measures.compass > 80 and self.measures.compass<100:
                        self.go_left = True
                    elif self.measures.compass > -10 and self.measures.compass<10:
                        self.go_back = True
                    elif self.measures.compass > -100 and self.measures.compass<-80:
                        self.go_right = True
                    else:
                        self.go_front = True
                elif self.pos[0]<self.ls[self.i][0] and (26-self.pos[1])==self.ls[self.i][1]:
                    if self.measures.compass > 80 and self.measures.compass<100:
                        self.go_right = True
                    elif self.measures.compass > -10 and self.measures.compass<10:
                        self.go_front = True
                    elif self.measures.compass > -100 and self.measures.compass<-80:
                        self.go_left = True
                    else:
                        self.go_back = True
                elif self.pos[0]==self.ls[self.i][0] and (26-self.pos[1])>self.ls[self.i][1]:
                    if self.measures.compass > 80 and self.measures.compass<100:
                        self.go_front = True
                    elif self.measures.compass > -10 and self.measures.compass<10:
                        self.go_left = True
                    elif self.measures.compass > -100 and self.measures.compass<-80:
                        self.go_back = True
                    else:
                        self.go_right = True
                else:
                    if self.measures.compass > 80 and self.measures.compass<100:
                        self.go_back = True
                    if self.measures.compass > -10 and self.measures.compass<10:
                        self.go_right = True
                    if self.measures.compass > -100 and self.measures.compass<-80:
                        self.go_front = True
                    else:
                        self.go_left = True
                self.go_to_ls = False
                self.i += 1

            #exit()
        # print(self.measures.compass)
        # print("\n\n")
        # print(self.visited_squares)
        if self.init_val == 0:
            self.init_val = 1
            self.offset_x = self.measures.x
            self.offset_y = self.measures.y
            self.last_pos = (self.offset_x, self.offset_y)
            if (27, 13) in self.walls:
                self.walls.remove((27, 13))
            self.visited_squares.append((13, 27))

            if self.measures.irSensor[center_id] < 1.2:
                self.matrix[13][28] = 'X'
                # self.maze[13][28] = 0
                try:
                    self.walls.remove((28, 13))
                    self.walls.remove((29, 13))
                except:
                    pass
                self.squares_to_visit.append((13, 29))
            else:
                self.matrix[13][28] = '|'
            if self.measures.irSensor[right_id] < 1.2:
                self.matrix[27 - 13][27] = 'X'
                # self.maze[27 - 13][27] = 0
                try:
                    self.walls.remove((27, 27 - 13))
                    self.walls.remove((27, 28 - 13))
                except:
                    pass
                self.squares_to_visit.append((28 - 13, 27))
            else:
                self.matrix[27 - 13][27] = '-'
            if self.measures.irSensor[left_id] < 1.2:
                self.matrix[25 - 13][27] = 'X'
                # self.maze[25 - 13][27] = 0
                try:
                    self.walls.remove((27, 25 - 13))
                    self.walls.remove((27, 24 - 13))
                except:
                    pass
                self.squares_to_visit.append((24 - 13, 27))
            else:
                self.matrix[25 - 13][27] = '-'
            if self.measures.irSensor[back_id] < 1.2:
                self.matrix[13][26] = 'X'
                # self.maze[13][26] = 0
                try:
                    self.walls.remove((26, 13))
                    self.walls.remove((25, 13))
                except:
                    pass
                self.squares_to_visit.append((13, 25))
            else:
                self.matrix[13][26] = '|'

            print('Beacons: ' + str(self.nBeacons))
            print(self.measures.ground)
            if self.measures.ground == 0:
                self.beacons_ls.append((27, 13))



        # Save the path on file when the robot discovered the entire map
        if self.squares_to_visit == []:
            final_path = []
            for i in range(int(self.nBeacons)):
                start = self.beacons_ls[i]
                if i == int(self.nBeacons) - 1:
                    end = self.beacons_ls[0]
                else:
                    end = self.beacons_ls[i + 1]
                ls = self.path_to_beacon(start, end)
                if i == int(self.nBeacons) - 1:
                    for coor in ls:
                        final_path.append(coor)
                else:
                    for coor in ls[:-1]:
                        final_path.append(coor)
            # Fazer A star para todos os beacons e de volta ao 0 0
            print(final_path)

            with open('out_file3.txt', 'w') as out:
                for j in final_path:
                    out.write(str(j[0]) + ' ' + str(j[1]))
                    out.write('\n')
            self.finish()

        # Save on file when all beacons are discovered
        if len(self.beacons_ls) == int(self.nBeacons) and self.checkpoints:
            final_path = []
            self.checkpoints = False
            for i in range(int(self.nBeacons)):
                start = self.beacons_ls[i]
                if i == int(self.nBeacons) - 1:
                    end = self.beacons_ls[0]
                else:
                    end = self.beacons_ls[i + 1]
                ls = self.path_to_beacon(start, end)

                if i == int(self.nBeacons) - 1:
                    for coor in ls:
                        final_path.append(coor)
                else:
                    for coor in ls[:-1]:
                        final_path.append(coor)
            print(final_path)
            with open('out_file3.txt', 'w') as out:
                for j in final_path:
                    out.write(str(j[0]) + ' ' + str(j[1]))
                    out.write('\n')


        if self.next_pos == (0, 0):
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
                    # self.maze[26 - self.pos[1]][self.pos[0] + 1] = 0
                    try:
                        self.walls.remove((self.pos[0] + 1, 26 - self.pos[1]))
                        self.walls.remove((self.pos[0] + 2, 26 - self.pos[1]))
                    except:
                        pass
                    if (26 - self.pos[1], self.pos[0] + 2) not in self.squares_to_visit and (26 - self.pos[1], self.pos[0] + 2) not in self.visited_squares:
                        self.squares_to_visit.append((26 - self.pos[1], self.pos[0] + 2))
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[25 - self.pos[1]][self.pos[0]] = 'X'
                    # self.maze[25 - self.pos[1]][self.pos[0]] = 0
                    try:
                        self.walls.remove((self.pos[0], 25 - self.pos[1]))
                        self.walls.remove((self.pos[0], 24 - self.pos[1]))
                    except:
                        pass
                    if (24 - self.pos[1], self.pos[0]) not in self.squares_to_visit and (24 - self.pos[1], self.pos[0]) not in self.visited_squares:
                        self.squares_to_visit.append((24 - self.pos[1], self.pos[0]))
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] - 1] = 'X'
                    # self.maze[26 - self.pos[1]][self.pos[0] - 1] = 0
                    try:
                        self.walls.remove((self.pos[0] - 1, 26 - self.pos[1]))
                        self.walls.remove((self.pos[0] - 2, 26 - self.pos[1]))
                    except:
                        pass
                    if (26 - self.pos[1], self.pos[0] - 2) not in self.squares_to_visit and (26 - self.pos[1], self.pos[0] - 2) not in self.visited_squares:
                        self.squares_to_visit.append((26 - self.pos[1], self.pos[0] - 2))
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[27 - self.pos[1]][self.pos[0]] = 'X'
                    # self.maze[27 - self.pos[1]][self.pos[0]] = 0
                    try:
                        self.walls.remove((self.pos[0], 27 - self.pos[1]))
                        self.walls.remove((self.pos[0], 28 - self.pos[1]))
                    except:
                        pass
                    if (28 - self.pos[1], self.pos[0]) not in self.squares_to_visit and (28 - self.pos[1], self.pos[0]) not in self.visited_squares:
                        self.squares_to_visit.append((28 - self.pos[1], self.pos[0]))
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
            if self.measures.irSensor[right_id] < 1.2:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[27 - self.pos[1]][self.pos[0]] = 'X'
                    # self.maze[27 - self.pos[1]][self.pos[0]] = 0
                    try:
                        self.walls.remove((self.pos[0], 27 - self.pos[1]))
                        self.walls.remove((self.pos[0], 28 - self.pos[1]))
                    except:
                        pass
                    if (28 - self.pos[1], self.pos[0]) not in self.squares_to_visit and (28 - self.pos[1], self.pos[0]) not in self.visited_squares:
                        self.squares_to_visit.append((28 - self.pos[1], self.pos[0]))
                        #print('Direita ')
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] + 1] = 'X'
                    # self.maze[26 - self.pos[1]][self.pos[0] + 1] = 0
                    try:
                        self.walls.remove((self.pos[0] + 1, 26 - self.pos[1]))
                        self.walls.remove((self.pos[0] + 2, 26 - self.pos[1]))
                    except:
                        pass
                    if (26 - self.pos[1], self.pos[0] + 2) not in self.squares_to_visit and (26 - self.pos[1], self.pos[0] + 2) not in self.visited_squares:
                        self.squares_to_visit.append((26 - self.pos[1], self.pos[0] + 2))
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[25 - self.pos[1]][self.pos[0]] = 'X'
                    # self.maze[25 - self.pos[1]][self.pos[0]] = 0
                    try:
                        self.walls.remove((self.pos[0], 25 - self.pos[1]))
                        self.walls.remove((self.pos[0], 24 - self.pos[1]))
                    except:
                        pass
                    if (24 - self.pos[1], self.pos[0]) not in self.squares_to_visit and (24 - self.pos[1], self.pos[0]) not in self.visited_squares:
                        self.squares_to_visit.append((24 - self.pos[1], self.pos[0]))
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] - 1] = 'X'
                    # self.maze[26 - self.pos[1]][self.pos[0] - 1] = 0
                    try:
                        self.walls.remove((self.pos[0] - 1, 26 - self.pos[1]))
                        self.walls.remove((self.pos[0] - 2, 26 - self.pos[1]))
                    except:
                        pass
                    if (26 - self.pos[1], self.pos[0] - 2) not in self.squares_to_visit and (26 - self.pos[1], self.pos[0] - 2) not in self.visited_squares:
                        self.squares_to_visit.append((26 - self.pos[1], self.pos[0] - 2))
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
            if self.measures.irSensor[left_id] < 1.2:
                if self.measures.compass < 10 and self.measures.compass > -10:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[25 - self.pos[1]][self.pos[0]] = 'X'
                    # self.maze[25 - self.pos[1]][self.pos[0]] = 0
                    try:
                        self.walls.remove((self.pos[0], 25 - self.pos[1]))
                        self.walls.remove((self.pos[0], 24 - self.pos[1]))
                    except:
                        pass
                    if (24 - self.pos[1], self.pos[0]) not in self.squares_to_visit and (24 - self.pos[1], self.pos[0]) not in self.visited_squares:
                        self.squares_to_visit.append((24 - self.pos[1], self.pos[0]))
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] - 1] = 'X'
                    # self.maze[26 - self.pos[1]][self.pos[0] - 1] = 0
                    try:
                        self.walls.remove((self.pos[0] - 1, 26 - self.pos[1]))
                        self.walls.remove((self.pos[0] - 2, 26 - self.pos[1]))
                    except:
                        pass
                    if (26 - self.pos[1], self.pos[0] - 2) not in self.squares_to_visit and (26 - self.pos[1], self.pos[0] - 2) not in self.visited_squares:
                        self.squares_to_visit.append((26 - self.pos[1], self.pos[0] - 2))
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[27 - self.pos[1]][self.pos[0]] = 'X'
                    # self.maze[27 - self.pos[1]][self.pos[0]] = 0
                    try:
                        self.walls.remove((self.pos[0], 27 - self.pos[1]))
                        self.walls.remove((self.pos[0], 28 - self.pos[1]))
                    except:
                        pass
                    if (28 - self.pos[1], self.pos[0]) not in self.squares_to_visit and (28 - self.pos[1], self.pos[0]) not in self.visited_squares:
                        self.squares_to_visit.append((28 - self.pos[1], self.pos[0]))
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] + 1] = 'X'
                    # self.maze[26 - self.pos[1]][self.pos[0] + 1] = 0
                    try:
                        self.walls.remove((self.pos[0] + 1, 26 - self.pos[1]))
                        self.walls.remove((self.pos[0] + 2, 26 - self.pos[1]))
                    except:
                        pass
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
                    # self.maze[26 - self.pos[1]][self.pos[0] + 1] = 0
                    try:
                        self.walls.remove((self.pos[0] + 1, 26 - self.pos[1]))
                        self.walls.remove((self.pos[0] + 2, 26 - self.pos[1]))
                    except:
                        pass
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[25 - self.pos[1]][self.pos[0]] = 'X'
                    # self.maze[25 - self.pos[1]][self.pos[0]] = 0
                    try:
                        self.walls.remove((self.pos[0], 25 - self.pos[1]))
                        self.walls.remove((self.pos[0], 24 - self.pos[1]))
                    except:
                        pass
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] - 1] = 'X'
                    # self.maze[26 - self.pos[1]][self.pos[0] - 1] = 0
                    try:
                        self.walls.remove((self.pos[0] - 1, 26 - self.pos[1]))
                        self.walls.remove((self.pos[0] - 2, 26 - self.pos[1]))
                    except:
                        pass
                    self.next_pos = (self.last_pos[0] - 2, self.last_pos[1])
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[27 - self.pos[1]][self.pos[0]] = 'X'
                    # self.maze[27 - self.pos[1]][self.pos[0]] = 0
                    try:
                        self.walls.remove((self.pos[0], 27 - self.pos[1]))
                        self.walls.remove((self.pos[0], 28 - self.pos[1]))
                    except:
                        pass
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
                    # self.maze[27 - self.pos[1]][self.pos[0]] = 0
                    try:
                        self.walls.remove((self.pos[0], 27 - self.pos[1]))
                        self.walls.remove((self.pos[0], 28 - self.pos[1]))
                    except:
                        pass
                    self.next_pos = (self.last_pos[0], self.last_pos[1] - 2)
                elif self.measures.compass < 100 and self.measures.compass > 80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] + 1] = 'X'
                    # self.maze[26 - self.pos[1]][self.pos[0] + 1] = 0
                    try:
                        self.walls.remove((self.pos[0] + 1, 26 - self.pos[1]))
                        self.walls.remove((self.pos[0] + 2, 26 - self.pos[1]))
                    except:
                        pass
                    self.next_pos = (self.last_pos[0] + 2, self.last_pos[1])
                elif self.measures.compass > 170 or self.measures.compass < -170:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[25 - self.pos[1]][self.pos[0]] = 'X'
                    # self.maze[25 - self.pos[1]][self.pos[0]] = 0
                    try:
                        self.walls.remove((self.pos[0], 25 - self.pos[1]))
                        self.walls.remove((self.pos[0], 24 - self.pos[1]))
                    except:
                        pass
                    self.next_pos = (self.last_pos[0], self.last_pos[1] + 2)
                elif self.measures.compass > -100 and self.measures.compass < -80:
                    self.pos = ((int(self.last_pos[0]) - int(self.offset_x) + 27), int(self.last_pos[1]) - int(self.offset_y) + 13)
                    self.matrix[26 - self.pos[1]][self.pos[0] - 1] = 'X'
                    # self.maze[26 - self.pos[1]][self.pos[0] - 1] = 0
                    try:
                        self.walls.remove((self.pos[0] - 1, 26 - self.pos[1]))
                        self.walls.remove((self.pos[0] - 2, 26 - self.pos[1]))
                    except:
                        pass
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
            print('To visit' + str(self.squares_to_visit))

        next_position = ((int(self.next_pos[0]) - int(self.offset_x) + 27), int(self.next_pos[1]) - int(self.offset_y) + 13)
        if (26-next_position[1],next_position[0]) in self.visited_squares[:-1] and not self.complete_astar:
            if self.flag == 0:
                self.previous_pos=26-next_position[1],next_position[0]
                self.flag = 1
                self.previous += 1
                if self.previous == 1:
                    self.do_astar = True
            if self.previous_pos!=(26-next_position[1],next_position[0]):
                self.flag = 0
        else:
            self.previous = 0
            self.flag = 0

        if self.go_left:
            if self.next_pos[0] > self.last_pos[0]:
                if self.first_call:
                    if self.turn(0, 'left') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            elif self.next_pos[0] < self.last_pos[0]:
                if self.first_call:
                    if self.turn(-180, 'left') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            elif self.next_pos[1] > self.last_pos[1]:
                if self.first_call:
                    if self.turn(90, 'left') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            elif self.next_pos[1] < self.last_pos[1]:
                if self.first_call:
                    if self.turn(-90, 'left') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            if self.stop_movement(self.next_pos):

                self.pos = ((int(self.next_pos[0]) - int(self.offset_x) + 27), int(self.next_pos[1]) - int(self.offset_y) + 13)
                print(self.measures.ground)
                if self.measures.ground != -1 and self.measures.ground != 0 and (self.pos[0], 26 - self.pos[1]) not in self.beacons_ls:
                    self.beacons_ls.append((self.pos[0], 26 - self.pos[1]))
                    print('Beacon position: ' + str((self.pos[0], 26 - self.pos[1])))

                    if len(self.beacons_ls) > 1:
                        start = (self.beacons_ls[0][0], self.beacons_ls[0][1])
                        end = (self.beacons_ls[1][0], self.beacons_ls[1][1])
                        self.ls = self.path_to_beacon(start, end)
                        print(self.ls)

                        with open('out_file3.txt', 'w') as out:
                            for j in self.ls:
                                out.write(str(j[0]) + ' ' + str(j[1]))
                                out.write('\n')

                self.matrix[26 - self.pos[1]][self.pos[0]] = 'X'
                # self.maze[26 - self.pos[1]][self.pos[0]] = 0
                if (self.pos[0], 26 - self.pos[1]) in self.walls:
                    self.walls.remove((self.pos[0], 26 - self.pos[1]))

                if (26 - self.pos[1], self.pos[0]) not in self.visited_squares:
                    self.visited_squares.append((26 - self.pos[1], self.pos[0]))

                for i in self.matrix:
                    print(''.join(i))

                # print(self.visited_squares)

                # for i in self.maze:
                #     print(i)


                self.first_call = 1
                self.last_pos = self.next_pos
                if not self.complete_astar:
                    self.next_pos = (0, 0)
                else:
                    self.go_to_ls = True




        if self.go_front:
            if self.next_pos[0] > self.last_pos[0]:
                if self.first_call:
                    if self.turn(0, 'left') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            elif self.next_pos[0] < self.last_pos[0]:
                if self.first_call:
                    if self.turn(-180, 'left') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            elif self.next_pos[1] > self.last_pos[1]:
                if self.first_call:
                    if self.turn(90, 'left') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            elif self.next_pos[1] < self.last_pos[1]:
                if self.first_call:
                    if self.turn(-90, 'left') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            if self.stop_movement(self.next_pos):
                self.pos = ((int(self.next_pos[0]) - int(self.offset_x) + 27), int(self.next_pos[1]) - int(self.offset_y) + 13)
                print(self.measures.ground)
                if self.measures.ground != -1 and self.measures.ground != 0 and (self.pos[0], 26 - self.pos[1]) not in self.beacons_ls:
                    self.beacons_ls.append((self.pos[0], 26 - self.pos[1]))
                    print('Beacon position: ' + str((self.pos[0], 26 - self.pos[1])))

                    if len(self.beacons_ls) > 1:
                        start = (self.beacons_ls[0][0], self.beacons_ls[0][1])
                        end = (self.beacons_ls[1][0], self.beacons_ls[1][1])
                        self.ls = self.path_to_beacon(start, end)
                        print(self.ls)

                        with open('out_file3.txt', 'w') as out:
                            for j in self.ls:
                                out.write(str(j[0]) + ' ' + str(j[1]))
                                out.write('\n')

                self.matrix[26 - self.pos[1]][self.pos[0]] = 'X'
                # self.maze[26 - self.pos[1]][self.pos[0]] = 0
                if (self.pos[0], 26 - self.pos[1]) in self.walls:
                    self.walls.remove((self.pos[0], 26 - self.pos[1]))

                if (26 - self.pos[1], self.pos[0]) not in self.visited_squares:
                    self.visited_squares.append((26 - self.pos[1], self.pos[0]))

                # print(self.visited_squares)
                for i in self.matrix:
                    print(''.join(i))

                # for i in self.maze:
                #     print(i)

                self.first_call = 1
                self.last_pos = self.next_pos
                if not self.complete_astar:
                    self.next_pos = (0, 0)
                else:
                    self.go_to_ls = True
        if self.go_right:
            if self.next_pos[0] > self.last_pos[0]:
                if self.first_call:
                    if self.turn(0, 'right') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            elif self.next_pos[0] < self.last_pos[0]:
                if self.first_call:
                    if self.turn(-180, 'right') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            elif self.next_pos[1] > self.last_pos[1]:
                if self.first_call:
                    if self.turn(90, 'right') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            elif self.next_pos[1] < self.last_pos[1]:
                if self.first_call:
                    if self.turn(-90, 'right') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            if self.stop_movement(self.next_pos):
                self.pos = ((int(self.next_pos[0]) - int(self.offset_x) + 27), int(self.next_pos[1]) - int(self.offset_y) + 13)
                print(self.measures.ground)
                if self.measures.ground != -1 and self.measures.ground != 0 and (self.pos[0], 26 - self.pos[1]) not in self.beacons_ls:
                    self.beacons_ls.append((self.pos[0], 26 - self.pos[1]))
                    print('Beacon position: ' + str((self.pos[0], 26 - self.pos[1])))

                    if len(self.beacons_ls) > 1:
                        start = (self.beacons_ls[0][0], self.beacons_ls[0][1])
                        end = (self.beacons_ls[1][0], self.beacons_ls[1][1])
                        self.ls = self.path_to_beacon(start, end)
                        print(self.ls)

                        with open('out_file3.txt', 'w') as out:
                            for j in self.ls:
                                out.write(str(j[0]) + ' ' + str(j[1]))
                                out.write('\n')
                # print('Stopped on position: ' + str(self.pos))
                #
                # print(self.pos[0])
                # print(self.pos[1])
                self.matrix[26 - self.pos[1]][self.pos[0]] = 'X'
                # self.maze[26 - self.pos[1]][self.pos[0]] = 0
                if (self.pos[0], 26 - self.pos[1]) in self.walls:
                    self.walls.remove((self.pos[0], 26 - self.pos[1]))

                if (26 - self.pos[1], self.pos[0]) not in self.visited_squares:
                    self.visited_squares.append((26 - self.pos[1], self.pos[0]))

                # print(self.visited_squares)

                for i in self.matrix:
                    print(''.join(i))

                # for i in self.maze:
                #     print(i)

                self.first_call = 1
                self.last_pos = self.next_pos
                if not self.complete_astar:
                    self.next_pos = (0, 0)
                else:
                    self.go_to_ls = True
        if self.go_back:
            if self.next_pos[0] > self.last_pos[0]:
                if self.first_call:
                    if self.turn(0, 'left') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            elif self.next_pos[0] < self.last_pos[0]:
                if self.first_call:
                    if self.turn(-180, 'left') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            elif self.next_pos[1] > self.last_pos[1]:
                if self.first_call:
                    if self.turn(90, 'left') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            elif self.next_pos[1] < self.last_pos[1]:
                if self.first_call:
                    if self.turn(-90, 'left') == 1:
                        self.driveMotors(0.13, 0.13)
                        self.first_call = 0
            if self.stop_movement(self.next_pos):
                self.pos = ((int(self.next_pos[0]) - int(self.offset_x) + 27), int(self.next_pos[1]) - int(self.offset_y) + 13)

                if self.measures.ground != -1 and self.measures.ground != 0 and (self.pos[0], 26 - self.pos[1]) not in self.beacons_ls:
                    self.beacons_ls.append((self.pos[0], 26 - self.pos[1]))
                    print('Beacon position: ' + str((self.pos[0], 26 - self.pos[1])))

                    if len(self.beacons_ls) > 1:
                        start = (self.beacons_ls[0][0], self.beacons_ls[0][1])
                        end = (self.beacons_ls[1][0], self.beacons_ls[1][1])
                        self.ls = self.path_to_beacon(start, end)
                        print(self.ls)

                        with open('out_file3.txt', 'w') as out:
                            for j in self.ls:
                                out.write(str(j[0]) + ' ' + str(j[1]))
                                out.write('\n')



                # print('Stopped on position: ' + str(self.pos))
                #
                # print(self.pos[0])
                # print(self.pos[1])
                self.matrix[26 - self.pos[1]][self.pos[0]] = 'X'
                # self.maze[26 - self.pos[1]][self.pos[0]] = 0
                if (self.pos[0], 26 - self.pos[1]) in self.walls:
                    self.walls.remove((self.pos[0], 26 - self.pos[1]))

                if (26 - self.pos[1], self.pos[0]) not in self.visited_squares:
                    self.visited_squares.append((26 - self.pos[1], self.pos[0]))

                # print(self.visited_squares)

                for i in self.matrix:
                    print(''.join(i))

                # for i in self.maze:
                #     print(i)

                self.first_call = 1
                self.last_pos = self.next_pos

                if not self.complete_astar:
                    self.next_pos = (0, 0)
                else:
                    self.go_to_ls = True

    def path_to_beacon(self, start, end):
        a = AStar()
        a.init_grid(55, 27, self.walls, start, end)
        path = a.solve()
        path = path[::2]

        path_to_return = []
        for i in path:
            path_to_return.append((i[0] - 27, 13 - i[1]))
        return path_to_return

    def turn(self, degrees, direction):
        if(degrees == -180 or degrees == 180):
            if self.walk == 3:
                self.walk = 0
                return 1
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
        elif self.walk == 3:
            self.walk = 0
            return 1
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

class Cell(object):
    def __init__(self, x, y, reachable):
        """Initialize new cell.
        @param reachable is cell reachable? not a wall?
        @param x cell x coordinate
        @param y cell y coordinate
        @param g cost to move from the starting cell to this cell.
        @param h estimation of the cost to move from this cell
                 to the ending cell.
        @param f f = g + h
        """
        self.reachable = reachable
        self.x = x
        self.y = y
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = 0

    def __lt__(self, other):
        return self.f < other.f


class AStar(object):
    def __init__(self):
        # open list
        self.opened = []
        heapq.heapify(self.opened)
        # visited cells list
        self.closed = set()
        # grid cells
        self.cells = []
        self.grid_height = None
        self.grid_width = None

    def init_grid(self, width, height, walls, start, end):
        """Prepare grid cells, walls.
        @param width grid's width.
        @param height grid's height.
        @param walls list of wall x,y tuples.
        @param start grid starting point x,y tuple.
        @param end grid ending point x,y tuple.
        """
        self.grid_height = height
        self.grid_width = width
        for x in range(self.grid_width):
            for y in range(self.grid_height):
                if (x, y) in walls:
                    reachable = False
                else:
                    reachable = True
                self.cells.append(Cell(x, y, reachable))
        self.start = self.get_cell(*start)
        self.end = self.get_cell(*end)

    def get_heuristic(self, cell):
        """Compute the heuristic value H for a cell.
        Distance between this cell and the ending cell multiply by 10.
        @returns heuristic value H
        """
        return 10 * (abs(cell.x - self.end.x) + abs(cell.y - self.end.y))

    def get_cell(self, x, y):
        """Returns a cell from the cells list.
        @param x cell x coordinate
        @param y cell y coordinate
        @returns cell
        """
        return self.cells[x * self.grid_height + y]

    def get_adjacent_cells(self, cell):
        """Returns adjacent cells to a cell.
        Clockwise starting from the one on the right.
        @param cell get adjacent cells for this cell
        @returns adjacent cells list.
        """
        cells = []
        if cell.x < self.grid_width-1:
            cells.append(self.get_cell(cell.x+1, cell.y))
        if cell.y > 0:
            cells.append(self.get_cell(cell.x, cell.y-1))
        if cell.x > 0:
            cells.append(self.get_cell(cell.x-1, cell.y))
        if cell.y < self.grid_height-1:
            cells.append(self.get_cell(cell.x, cell.y+1))
        return cells

    def get_path(self):
        cell = self.end
        path = [(cell.x, cell.y)]
        while cell.parent is not self.start:
            cell = cell.parent
            path.append((cell.x, cell.y))

        path.append((self.start.x, self.start.y))
        path.reverse()
        return path

    def update_cell(self, adj, cell):
        """Update adjacent cell.
        @param adj adjacent cell to current cell
        @param cell current cell being processed
        """
        adj.g = cell.g + 10
        adj.h = self.get_heuristic(adj)
        adj.parent = cell
        adj.f = adj.h + adj.g

    def solve(self):
        """Solve maze, find path to ending cell.
        @returns path or None if not found.
        """
        # add starting cell to open heap queue
        heapq.heappush(self.opened, (self.start.f, self.start))
        while len(self.opened):
            # pop cell from heap queue
            f, cell = heapq.heappop(self.opened)
            # add cell to closed list so we don't process it twice
            self.closed.add(cell)
            # if ending cell, return found path
            if cell is self.end:
                return self.get_path()
            # get adjacent cells for cell
            adj_cells = self.get_adjacent_cells(cell)
            for adj_cell in adj_cells:
                if adj_cell.reachable and adj_cell not in self.closed:
                    if (adj_cell.f, adj_cell) in self.opened:
                        # if adj cell in open list, check if current path is
                        # better than the one previously found
                        # for this adj cell.
                        if adj_cell.g > cell.g + 10:
                            self.update_cell(adj_cell, cell)
                    else:
                        self.update_cell(adj_cell, cell)
                        # add adj cell to open list
                        heapq.heappush(self.opened, (adj_cell.f, adj_cell))


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
