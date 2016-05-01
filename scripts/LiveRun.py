from controller import Controller
from math import pi
class LiveRun():
    def __init__(self, mouse):
        self.mouse = mouse
        self.controller = Controller()
        self.DIAGONAL_DISTANCE_MAX = 0.1

    def move(self):
        coordTuple = self.mouse.forwardCoordinates()
        wallsAsExpected = all((self.mouse.detectFrontWall(), self.mouse.detectFrontLeftWall(), self.mouse.detectFrontRightWall()))
        if not self.mouse.facingWall() and self.mouse.inBounds(coordTuple[0], coordTuple[1]):
            (self.mouse.x, self.mouse.y) = self.mouse.forwardCoordinates()
            self.controller.straight(0.18)
            self.wait()
            return wallsAsExpected
        if self.mouse.facingWall():
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

    def wait():
        count = 0
        while not self.controller.busy and count < 5:
            count += 1
            sleep(0.2)
        while self.controller.busy:
            sleep(0.2)

    def turnRight(self):
        self.controller.turn(-pi/2)
        self.wait()
        return 1
        # decrease speed appropriately for turn
        # MAKE SURE TO UPDATE self.mouse.direction
        # turn right

    def turnLeft(self):
        self.controller.turn(pi/2)
        self.wait()
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
        hasRealWall = self.controller.right_dist > self.DIAGONAL_DISTANCE_MAX
        wallExpected = hasRealWall == self.mouse.board.boundaries[frontCoord[0]][frontCoord[1]][direction]
        self.mouse.board.boundaries[frontCoord[0]][frontCoord[1]][direction] = hasRealWall
        return wallExpected

        # hasWall = -1 # 1 or 0, set detection to check
        # forwardCoord = self.mouse.forwardCoordinates()
        # if(hasWall != self.mouse.boundaries[forwardCoord[0]][forwardCoord[1]][self.mouse.direction]):
            somehow update self.mouse.boundaries to reflect appropriate changes
# 
            # return -1
        # else:
            # return 1 # wall found as expected

    
    def detectFrontLeftWall(self):
        direction = (3 + self.mouse.direction)%4
        frontCoord = self.mouse.forwardCoordinates()
        hasRealWall = self.controller.left_dist > self.DIAGONAL_DISTANCE_MAX
        wallExpected = hasRealWall == self.mouse.board.boundaries[frontCoord[0]][frontCoord[1]][direction]
        self.mouse.board.boundaries[frontCoord[0]][frontCoord[1]][direction] = hasRealWall

        return wallExpected


    def detectFrontWall(self):
        
        direction = (0 + self.mouse.direction)%4
        frontCoord = self.mouse.forwardCoordinates()
        hasRealWall = self.controller.front_dist > .15 and self.controller.first_dist < .23
        wallExpected = hasRealWall == self.mouse.board.boundaries[frontCoord[0]][frontCoord[1]][direction]
        self.mouse.board.boundaries[frontCoord[0]][frontCoord[1]][direction] = hasRealWall
        return wallExpected



    




