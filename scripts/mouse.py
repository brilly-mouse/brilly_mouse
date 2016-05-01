from Queue import PriorityQueue
from LiveRun import LiveRun
from SimRun import SimRun
from board import Board


class Mouse():
    """
        xi  -       initial x coordinate start
        yi  -       initial y coordinate start
        direction - initial direction of mouse {0:up, 1:right, 2:down, 3:left}
        board   -   board object to modify over time
        realRun -   1 for real run, 0 for simulation
    """
    def __init__(self, xi, yi, direction, board, realRun, pause=0):
        self.x = xi
        self.startX = xi
        self.startY = yi
        self.y = yi
        self.direction = direction
        self.saved_path = []
        self.pause = pause
        if(realRun):
            self.action = LiveRun(self)
            self.board = board
        else:
            self.action = SimRun(self, board, pause) # actual board will be provided to SimRun
            self.board = Board() # mouse will be given board with no information
            
    """
    Returns coordinates of moving forward one move in (x,y) form

    """
    def forwardCoordinates(self):
        if self.direction==0:
            return (self.x, self.y-1)
        elif self.direction ==1:
            return (self.x + 1, self.y)
        elif self.direction ==2:
            return (self.x, self.y + 1)
        else:
            return (self.x -1, self.y)

    def rightCoordinates(self):
        if self.direction == 0:
            return (self.x + 1, self.y)
        elif self.direction == 1:
            return (self.x, self.y + 1)
        elif self.direction == 2:
            return (self.x-1, self.y)
        else:
            return (self.x, self.y -1)

    def leftCoordinates(self):
        if self.direction==0:
            return (self.x - 1, self.y)
        elif self.direction ==1:
            return (self.x, self.y - 1)
        elif self.direction ==2:
            return (self.x + 1, self.y)
        else:
            return (self.x, self.y + 1)

    """
    Changes x and y if moving forward is open and in bounds
    """
    ##################### START SHARED METHOD CALLS TO LIVE/SIM RUNS #########################
    def move(self):
        return self.action.move() 
    
    def moveBack(self):
        return self.action.moveBack()

    def facingWall(self):
        return self.action.facingWall()

    def turnRight(self):
        return self.action.turnRight()

    def turnLeft(self):
        return self.action.turnLeft()

    def detectFrontRightWall(self):
        if not self.inBounds(self.forwardCoordinates()):
            return 0
        return self.action.detectFrontRightWall()
    
    def detectFrontLeftWall(self):
        if not self.inBounds(self.forwardCoordinates()):
            return 0
        return self.action.detectFrontLeftWall()

    def detectFrontWall(self):
        if not self.inBounds(self.forwardCoordinates()):
            return 0
        return self.action.detectFrontWall()

        
    ##########################################################################################

    # simulation function for mouse
    def inBounds(self, x,y=None):
        if y == None:
            return x[0] >= 0 and x[0] < self.board.sideLength and x[1] >= 0 and x[1] < self.board.sideLength
        return x >= 0 and x < self.board.sideLength and y >= 0 and y < self.board.sideLength

    def inGoal(self, x= None, y=None):
        if(x == None and y == None):
            x = self.x
            y = self.y
        middleCoord = (self.board.sideLength//2 -1, self.board.sideLength//2)
        return x in middleCoord and y in middleCoord
    
    def manhattanDistance(self, x= None, y=None):
        if(x == None and y == None):
            x = self.x
            y = self.y
        lowMiddle = self.board.sideLength//2 -1
        highMiddle = self.board.sideLength//2

        xD = min(abs(lowMiddle - x), abs(highMiddle - x))
        yD = min(abs(lowMiddle - y), abs(highMiddle - y))
        return xD + yD

    def euclideanDistance(self, x= None, y=None):
        if(x == None and y == None):
            x = self.x
            y = self.y
        lowMiddle = self.board.sideLength//2 -1
        highMiddle = self.board.sideLength//2
        xD = min(abs(lowMiddle - x), abs(highMiddle - x))
        yD = min(abs(lowMiddle - y), abs(highMiddle - y))
        return sqrt(xD**2 + yD**2)


    """
    Returns a list of (x,y) coordinates to take to reach the goal
    """
    def AStarSearch(self):
        nodesToSearch = PriorityQueue()
        first=  Node(self.x,self.y, self)
        nodesToSearch.put(first)
        visited = [[0 for _x in range(self.board.sideLength)] for _y in range(self.board.sideLength)]
        visited[first.x][first.y] = 1
        directions = [0,1,2,3]

        solution = first # temp value, will be replaced when reached the end
        while not nodesToSearch.empty():
            parent = nodesToSearch.get() # first node pulled out
            # print "parent " + str(parent.x) + " " + str(parent.y) + " priority " + str(parent.priority)

            visited[parent.x][parent.y] = 1
            if self.board.inGoal(parent.x, parent.y): # reached goal
                # print "solution reached"
                s = str(parent.x) + " " + str(parent.y)
                # print s
                solution = parent
                break
            neighbors = zip(directions,self.board.neighbors(parent.x, parent.y))
            # print neighbors
            for d,coord, in neighbors:
                n = (Node(coord[0],coord[1],self,parent.distance, parent))
                if self.board.inBounds(coord[0],coord[1]) and self.board.boundaries[parent.x][parent.y][d] == 0 and visited[coord[0]][coord[1]] == 0:
                    n = (Node(coord[0],coord[1],self,parent.distance, parent))
                    nodesToSearch.put(n)
            # toVisit = [Node(coord[0],coord[1],self,parent.distance, parent) for d,coord in neighbors if self.board.inBounds(coord[0],coord[1]) and self.board.boundaries[coord[0]][coord[1]][d] == 0 and visited[coord[0]][coord[1]] == 0]
            # toVisit is the neighbor spaces that are unvisited, have no boundaries, in bounds
            # for n in toVisit:
                # nodesToSearch.put(n)
                    # print "possible " + str(n.x) + " " + str(n.y) + " " + str(n.priority) + " " + str(d) 
                    # print str(self.board.inBounds(coord[0],coord[1])) +  str(self.board.boundaries[parent.x][parent.y][d]) +  str(visited[coord[0]][coord[1]] == 0)

        path = []

        while solution is not  None:
            path = [(solution.x, solution.y)] + path
            solution = solution.parent
        self.saved_path  = path
        return path[1:]





    def printDirection(self):
        if self.direction == 0:
            return "^"
        elif self.direction == 1:
            return ">"
        elif self.direction ==2:
            return "V"
        else:
            return "<"


    def printBoard(self):
        return self.action.printBoard()

    ####################################### FLOODFILL ##################################################

    """ Floodfill algorithm """
    def floodFillToGoal(self):
        while(not self.inGoal()):
            self.path = self.AStarSearch()
            self.followPath()
            # will follow path to completion if walls appropriate
            # recalculates path if followpath returns

    """
    Keeps moving mouse to through path. Returns if path completed or walls are updated
    """
    def followPath(self):
        while(self.path):
            path = self.path[0]
            moveSuccess = self.moveToSquare(path[0], path[1])
            if moveSuccess:
                self.path = self.path[1:] # truncate after sucessful move
                self.saved_path = self.saved_path[1:]
            else:
                return # when found walls unexpected
        return # when finished following the path
    
    """ Moves to an adjacent square """
    def moveToSquare(self, x, y):
        if self.pause:
            print "movetoSquare " + str((x,y))
            print "current situation in " + str((self.x, self.y))
            print "existing " +  str(self.board.boundaries[self.x][self.y]) + " actual " +str(self.action.omniscientBoard.boundaries[self.x][self.y])
            command = raw_input("press n to continue")
            if command == "q":
                exit()

        turn = 1
        if (x,y) == self.leftCoordinates() and (self.board.boundaries[self.x][self.y][(self.direction + 3)%4]==0):
            print "left turning!!"
            turn = self.turnLeft()
        elif (x,y) == self.rightCoordinates() and (self.board.boundaries[self.x][self.y][(self.direction + 1)%4]==0):
            print "right turning!!!!!"
            turn = self.turnRight()
        elif (x,y) ==  self.forwardCoordinates() and (self.board.boundaries[self.x][self.y][self.direction] == 0):
            turn = 1
            print "move forward"
        elif self.board.boundaries[x][y][(self.direction + 2)%4] == 0:
            # print "backward"
            # print self.direction
            # print (self.x, self.y)
            # print (x,y)
            # print self.forwardCoordinates()
            # means it is behind
            turn = all((self.turnRight(), self.turnRight()))
        print "board realization: " + str(self.board.boundaries[x][y]) + " " + str(self.action.omniscientBoard.boundaries[x][y])+ "  " + str(x) + " " + str(y)
        print self.printBoard()
        return turn and self.move() #returns 1 if move is successful, 0 if move if move unexpected

    
class Node():
    def __init__(self, x,y, mouse, distance=-1, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.mouse = mouse
        self.distance = distance + 1
        self.priority = mouse.manhattanDistance(x,y) + self.distance

    def __cmp__(self, other):
        return cmp(self.priority, other.priority)



