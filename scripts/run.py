from mouse import Mouse
from board import Board
import sys

x = 0
y = 0
direction = 0

# can run using python 1 2 3 where it starts at (x,y), direction is left
if(len(sys.argv) >= 4 ):
    x = int(sys.argv[-3])
    y = int(sys.argv[-2])
    direction = int(sys.argv[-1])


board = Board()
mouse = Mouse(x,y,direction,board,0)

# 
for i in range(0,16,3):
    for j in range(15):
        if(i%2 ==0):
            increment = 1
        else:
            increment = 0
        board.addBound(i,j+increment,1)




print mouse.printBoard()
commands = ('w','a','s','d','q','p')
while(True):
    command = raw_input("wasd to move, q to quit, p to print path:")
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
        
    print mouse.printBoard()
    if( mouse.inGoal()):
        print "You win!!"

