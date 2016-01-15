#!/usr/bin/env python
from infrared import IR
from time import sleep

IR.setup()

while True:
    print "left: ", IR.get_left()
    print "right: ", IR.get_right()
    print "front: ", IR.get_front()
    print('-')
    sleep(0.5)
