#!/usr/bin/env python
from controller import Controller
from math import pi
from time import sleep
class LiveRun():
    def __init__(self, mouse):
        self.mouse = mouse
        self.controller = Controller()
        print("Preparing to start movement, sleeping for one second..."
        sleep(1)
        self.DIAGONAL_DISTANCE_MAX = 0.1

    def move(self):
        coordTuple = self.mouse.forwardCoordinates()
        if not self.mouse.facingWall() and self.mouse.inBounds(coordTuple[0], coordTuple[1]):
            (self.mouse.x, self.mouse.y) = self.mouse.forwardCoordinates()
            self.controller.straight(0.18)
            wallsAsExpected = self.straightMoveWaitForDetection()
            return wallsAsExpected
        elif self.mouse.facingWall():
            print "Mouse is facing wall"
        return False

        # move a certain distance and try to make sure that
        # mouse stays in the center
        # decrease or increase speed appropriately for next move
        # MAKE SURE TO UPDATE self.mouse.x and self.mouse.y when moving appropriately
    
    # def moveBack(self):
        #move backwards, similar to move() but reversed
        # MAKE SURE TO UPDATE self.mouse.x and self.mouse.y when moving appropriately
        # return -1

    def wait(self):
        count = 0
        while not self.controller.busy and count < 5:
            count += 1
            sleep(0.2)
            if count >= 5:
                print "Regular wait is taking longer than usual.."

        count = 0
        while self.controller.busy:
            sleep(0.2)
            count += 1
            if count > 40:
                print "this is taking longer than expected"

    def straightMoveWaitForDetection(self):
        count = 0
        while not self.controller.busy and count < 5:
            count += 1
            sleep(0.1)
            if count >= 5:
                print "waiting on straight movement to start"

        count = 0
        foundWall = False
        wallsAsExpected = True
        while self.controller.busy:
            sleep(0.2)
            count += 1

            if not foundWall and controller.dist_to_goal <= 0.09:
                print "finished moving straight 9 cm, preparing to use distance sensors" 
                foundWall = True
                wallsAsExpected = all((self.mouse.detectFrontWall(), self.mouse.detectFrontLeftWall(), self.mouse.detectFrontRightWall()))
                self.
            if count > 40:
                print "straight move taking longer than usual to complete..."

        return wallsAsExpected
    def turnRight(self):
        self.controller.turn(-pi/2)
        self.wait()
        self.mouse.direction = (self.mouse.direction + 1) % 4
        return 1
        # decrease speed appropriately for turn
        # MAKE SURE TO UPDATE self.mouse.direction
        # turn right

    def turnLeft(self):
        self.controller.turn(pi/2)
        self.wait()
        self.mouse.direction = (self.mouse.direction + 3) % 4
        return 1
        # decrease speed appropriately for turn
        # MAKE SURE TO UPDATE self.mouse.direction
        # turn left

    def facingWall(self):
        return self.mouse.board.boundaries[self.mouse.x][self.mouse.y][self.mouse.direction]
        # returns whether wall is directly in front of mouse


    """
    This method is called when mouse position in square is far enough to detect next wall ahead
    """
    def detectFrontRightWall(self):
        direction = (1 + self.mouse.direction)%4
        frontCoord = self.mouse.forwardCoordinates()
        hasRealWall = self.controller.right_dist < self.DIAGONAL_DISTANCE_MAX
        wallExpected = hasRealWall == self.mouse.board.boundaries[frontCoord[0]][frontCoord[1]][direction]
        self.mouse.board.boundaries[frontCoord[0]][frontCoord[1]][direction] = hasRealWall
        return wallExpected

        # hasWall = -1 # 1 or 0, set detection to check
        # forwardCoord = self.mouse.forwardCoordinates()
        # if(hasWall != self.mouse.boundaries[forwardCoord[0]][forwardCoord[1]][self.mouse.direction]):
            # somehow update self.mouse.boundaries to reflect appropriate changes
# 
            # return -1
        # else:
            # return 1 # wall found as expected

    
    def detectFrontLeftWall(self):
        direction = (3 + self.mouse.direction)%4
        frontCoord = self.mouse.forwardCoordinates()
        hasRealWall = self.controller.left_dist < self.DIAGONAL_DISTANCE_MAX
        wallExpected = hasRealWall == self.mouse.board.boundaries[frontCoord[0]][frontCoord[1]][direction]
        self.mouse.board.boundaries[frontCoord[0]][frontCoord[1]][direction] = hasRealWall

        return wallExpected


    def detectFrontWall(self):
        
        direction = (0 + self.mouse.direction)%4
        frontCoord = self.mouse.forwardCoordinates()
        hasRealWall = self.controller.front_dist > .15 and self.controller.front_dist < .23
        print str(self.controller.front_dist) + " distance from wall"
        wallExpected = hasRealWall == self.mouse.board.boundaries[frontCoord[0]][frontCoord[1]][direction]
        self.mouse.board.boundaries[frontCoord[0]][frontCoord[1]][direction] = hasRealWall
        return wallExpected

    def printBoard(self):

        line = "||"
        self.omniscientBoard = self.mouse.board
        for x in range(self.mouse.board.sideLength):
            if self.omniscientBoard.boundaries[x][0][1]:
                if self.mouse.board.boundaries[x][0][1]:
                    line += "===|"
                else:
                    line += "===%"
            else:
                line += "===="
        line = line + "|\n"
        for y in range(self.mouse.board.sideLength):
            for row in range(2):
                line += "||"
                for x in range(self.mouse.board.sideLength):
                    initial = ["   ", " "]
                    # should only print according to what is on bottom and right, since adding left
                    # and top is redundant with multiple boxes printed together.
                    box = self.mouse.board.boundaries[x][y]
                    omniscientBox = self.omniscientBoard.boundaries[x][y]
                    # print str(box) + " printing!!!! " + str(omniscientBox) + " " + str(box == omniscientBox)
                    if omniscientBox[1]:
                        if box[1]:
                            initial[1] = "|"
                        else:
                            initial[1] = "%"
                    if x in (7,8) and y in (7,8):
                        initial[0] = "GGG"
                        if(x == self.mouse.x and y == self.mouse.y):
                            initial[0] = "G{0}G".format(self.mouse.printDirection())
                        if x == 7:
                            initial[1] = "G" #override possiblity of vertical wall inside goal area
                    elif row == 0: 
                        if(x == self.mouse.x and y == self.mouse.y):
                            initial[0] = " {0} ".format(self.mouse.printDirection())
                        elif (x,y) in self.mouse.saved_path:
                            initial[0] = " * "
                    elif row == 1:
                        if omniscientBox[2]:
                            if box[2]:
                                if y == self.mouse.board.sideLength -1:
                                    initial[0] = "==="
                                    initial[1] = "="
                                else:
                                    initial[0]= "---"
                                    initial[1] = "-"
                            else:
                                if(y == self.mouse.board.sideLength -1):
                                    initial[0] = "%%%"
                                    initial[1] = "%"
                                else:
                                    initial[0]= "%%%"
                                    initial[1] = "%"

                        if omniscientBox[1]:
                            if box[1] == omniscientBox[1]:
                                initial[1] = "|"
                            else:
                                initial[1] = "%"

                    line += "".join(initial)
                line +="|\n"
        return line


