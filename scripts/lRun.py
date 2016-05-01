#!/usr/bin/env python
from mouse import Mouse
from board import Board
from time import sleep
from controller import Controller
import sys
import rospy

x = 0
y = 0
direction = 1

# can run using python 1 2 3 where it starts at (x,y), direction is left
# if(len(sys.argv) == 1 ):
    # file = sys.arv[0]

rospy.init_node('maze_solver')
board = Board()
mouse = Mouse(x,y,direction,board,1)

mouse.printBoard()



mouse.floodFillToGoal()
