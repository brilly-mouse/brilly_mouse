class Board():
    def __init__(self):

        self.sideLength = 16;
        self.boundaries = [[[0 for i in range(4)] for i in range(self.sideLength)] for i in range(self.sideLength)]
        self.mouseLocation = [0,0]
        #boundaries[x][y][direction]
        for i in range(16):
            # mark outside boundaries
            self.boundaries[i][0][0] = 1 #top side
            self.boundaries[i][self.sideLength-1][2] = 1 #bottom side
            self.boundaries[0][i][3] = 1 #left side
            self.boundaries[self.sideLength-1][i][1] = 1 #right side
    def __str__(self):
        line = "||"

        for x in range(self.sideLength):
            if self.boundaries[x][0][1]:
                line += "===|"
            else:
                line += "===="
        line = line + "|\n"
        for y in range(self.sideLength):
            for row in range(2):
                line += "||"
                for x in range(self.sideLength):
                    initial = ["   ", " "]
                    # should only print according to what is on bottom and right, since adding left
                    # and top is redundant with multiple boxes printed together.
                    box = self.boundaries[x][y]
                    if box[1]:
                        initial[1] = "|"

                    if x in (7,8) and y in (7,8):
                        initial[0] = "GGG"
                        if x == 7:
                            initial[1] = "G"
                    elif row == 0: 
                        # if box[0]:
                            # if([x,y] == self.mouseLocation):
                                # initial[0] = " M "
                        # if box[1]:
                            # initial[1] = "|"
                        # elif row == 1:

                        if( box[2]):
                            if(y == self.sideLength -1):
                                initial[0] = "==="
                                initial[1] = "="
                            else:
                                initial[0]= "---"
                                initial[1] = "-"
                        # if(box[1]):
                            # initial[1] = "|"
                    line += "".join(initial)
                line +="|\n"
        return line

    def addBound(self, x, y, direction):

        # case where adding bad bounds within goal
        if x== 7 and y == 7 and direction in (1,2):
            return
        if x== 7 and y == 8 and direction in (1,0):
            return
        if x== 8 and y == 7 and direction in (3,2):
            return
        if x== 8 and y == 8 and direction in (0,3):
            return

        if(not self.inBounds(x,y)):
            return
        self.boundaries[x][y][direction] = 1;
        if(direction == 0 and self.inBounds(x, y-1)):
            self.boundaries[x][y-1][2] = 1
        elif( direction ==1 and self.inBounds(x+1, y)):
            self.boundaries[x+1][y][3] = 1
        elif( direction == 2 and self.inBounds(x, y+1)):
            self.boundaries[x][y+1][0] = 1
        elif( direction == 3 and self.inBounds(x-1, y)):
            self.boundaries[x-1][y][1] = 1

    def inBounds(self,x,y):
        return x >= 0 and x < self.sideLength and y >= 0 and y < self.sideLength
    
    def neighbors(self, x, y):
        return [[x,y-1],[x+1,y],[x,y+1],[x-1,y]]

    def inGoal(self,x,y):
        return x in (7,8) and y in (7,8)

    def save(self, filename="boardSave.txt"):
        file = open(filename, "w")
        for i in range(self.sideLength):
            for j in range(self.sideLength):
                for direction in range(4):
                    file.write(str(self.boundaries[i][j][direction]))
        file.close()
    
    def read(self, filename="boardSave.txt"):
        file = open(filename, "r")
        readVal = file.read()
        file.close()
        counter = 0
        for i in range(self.sideLength):
            for j in range(self.sideLength):
                for direction in range(4):
                    self.boundaries[i][j][direction] = int(readVal[counter])
                    counter = counter + 1


        



# b =  Board()

# b.addBound(7,7,1);
# b.addBound(7,7,2);
# b.addBound(7,7,3);

# for i in range(15):
    # for j in range(76):
        # b.addBound(i,j,1)    


# print b

                        


