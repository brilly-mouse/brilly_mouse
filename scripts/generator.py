from mouse import Mouse
from board import Board

b = Board()
mouse = Mouse(0,0,3,b,0)
altfaceWallFunc = lambda self: False
mouse.facingWall = altfaceWallFunc.__get__(mouse, Mouse) # replace facingWall func so mouse can go through walls
mouse.detectFrontWall = altfaceWallFunc.__get__(mouse, Mouse) 
mouse.detectRightWall = altfaceWallFunc.__get__(mouse, Mouse)
mouse.detectLeftWall = altfaceWallFunc.__get__(mouse, Mouse)
originalBoard= mouse.board
mouse.action.omniscientBoard = mouse.board

def altMove(self):
    position = self.forwardCoordinates()
    self.x = position[0]
    self.y = position[1]

print mouse.printBoard()
commands = ('w','a','s','d','b', 'r', 'q','p', 'export','xy')
while(True):
    command = raw_input("b for bottom wall, r for right wall, export for export, read for read in:")
    if command == 'q':
        break
    else:
        if command == 'w':
            mouse.move()
        elif command == 'a':
            mouse.turnLeft()
        elif command == 's':
            mouse.moveBack()
        elif command == 'd':
            mouse.turnRight()
        elif command == 'p':
            print mouse.AStarSearch()
        elif command == 'b':
            mouse.board.addBound(mouse.x, mouse.y, 2)
        elif command == 'r':
            mouse.board.addBound(mouse.x, mouse.y, 1)
        elif command == 'export':
            mouse.board.save()
        elif command == "read":
            mouse.board.read()
        elif command == "xy":
            print str(mouse.board.boundaries[mouse.x][mouse.y]) + " " + str(originalBoard.boundaries[mouse.x][mouse.y])
       
        elif len(command.split()) > 0:
            command = command.split()
            if(command[0] == "read"):
                mouse.board.read(command[1])
        
    print mouse.printBoard()

