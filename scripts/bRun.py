from mouse import Mouse
from board import Board
import sys

x = 0
y = 0
direction = 0

# can run using python 1 2 3 where it starts at (x,y), direction is left
# if(len(sys.argv) == 1 ):
    # file = sys.arv[0]



board = Board()
board.read()
mouse = Mouse(x,y,direction,board,0, 1)




raw_input('press n to start')
mouse.floodFillToGoal()
