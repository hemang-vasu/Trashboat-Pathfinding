import sys; args = sys.argv[1:]
import cv2, math
#Ex Input
#
# newAngle(double), startposx(double), startposy(double)
#
#Needs to be an input for the camera output classification, LIDAR output, IMU output, and a use conveyor belt arg
#For the output classification, should be a string of the exact name; if no classification then should be an empty string
#Output of this code will be to the pixhawk to determine how much to rotate and to move (newAngle of rotation, how many feet to move)
#Output should also be position in an (x,y) like grid to input into path.py, as well as the bool that determines if the boat is allowed to veer in order to collect trash
#Goal can be to possibly return time so that distance is implied given a constant velocity
global grid 
global startposx 
global startposy
global startnewAngle 
unitOfMeasure = 1 #1 meter for each grid unit
velocity = 0.4 #meters/second
#startposx = float(args[1])
#startposy = float(args[2])
#startnewAngle = float(args[0])
startposx = 1.5
startposy = 3.5
startnewAngle = 326.02
#creating the initial path
grid = [[0 for i in range(15)] for j in range (15)]

#setting up the path, integer "1" will be the path on the grid
boolTop = True
for row in range(len(grid)):
    for col in range(len(grid[row])):
        if col % 2 != 0:
            grid[row][col] = 1
        elif row == 0:
            if col % 4 == 0:
                grid[row][col] = 1
        elif row == len(grid) - 1:
            if (col - 2) % 4 == 0:
                grid[row][col] = 1

def pathOverlay(): #overlay the lake with grid, currently not working on this
    return 0

def detectingCamObject(): #returns an (x,y) double type tuple referring to any detected object's position, else return 0, currently not working on this
    return 0

def findClosestPointOnGrid(): #will return (x,y) int tuple for the closest point on grid
    #camera will detect trash in an assumed radius, in this case it will be 1 meter to the left, 1 meter to the right, 1 meter up and down.
    xCoordinateLeft = math.floor(startposx)
    xCoordinateRight = math.ceil(startposx)
    yCoordinateDown = math.ceil(startposy)
    yCoordinateUp = math.floor(startposy)
    currXCoordinate = 0
    currYCoordinate = 0

    if grid[1][xCoordinateRight] == 1:
        currXCoordinate = xCoordinateRight
    else:
        currXCoordinate = xCoordinateLeft

    if currXCoordinate-1 >= 0 and grid[0][currXCoordinate-1] == 1: #i.e the path is moving downwards
        currYCoordinate = yCoordinateDown
    else:
        currYCoordinate = yCoordinateUp
    
    return (currXCoordinate, currYCoordinate)

def pathChange(): #Returns the newAngle that the boat needs to shift to along with the distance IN TERMS OF GRID UNITS
    global startposx
    global startposy
    global grid
    distance = 0
    newAngle = 0
    currPoint = (0,0)
    if startposx.is_integer() and startposy.is_integer():
        startposx = int(startposx)
        startposy = int(startposy)
        trashPos = detectingCamObject() #detects trash
        if trashPos != 0: #i.e trash was detected
            newAngle = math.atan((trashPos[1] - startposy) / (trashPos[0] - startposx)) * (180 / math.pi) #degrees
            if trashPos[0]-1 >= 0 and grid[0][trashPos[0]-1] == 1: #i.e the path is moving downwards
                newAngle = 180-newAngle #flips over the y-axis
                if newAngle < 0: #make newAngle positive
                    newAngle += 360
            else: #i.e path is moving upwards
                newAngle = newAngle * -1 #negate the newAngle, flips over the x-axis
                if newAngle < 0: #make newAngle positive
                    newAngle += 360
            distance = math.sqrt(pow((startposx - trashPos[0]) , 2) + pow((startposy - trashPos[1]) , 2)) #distance formula
            currPoint = trashPos #sets the destination point to the trash
        else: #meaning the boat is currently on the path
            if startposx == len(grid[0]) - 1: #end of path
                return startnewAngle, 0, (startposx, startposy) #returns a distance of 0, program will know to end
            if startposx-1 >= 0 and grid[0][startposx-1] == 1: #i.e the boat is moving downwards
                if startposy == len(grid): #reaches bottom of grid
                    if grid[startposy][startposx+1] == 1: #path is on the right
                        return 0, 1, (startposx + 1, startposy) #1 unit to the right
                    else: #else move the point upwards
                        return 90, 1, (startposx, startposy - 1)
                else:
                    return 270, 1, (startposx, startposy+1) #1 unit down
            else: #i.e the boat is moving upwards
                if startposy == 0:
                    if grid[startposy][startposx+1] == 1: #path is on the right
                        return 0, 1, (startposx + 1, startposy)
                    else: #boat moves downwards
                        return 270, 1, (startposx, startposy + 1)
                else: #move upwards
                    return 90, 1, (startposx, startposy - 1)
    else: #meaning startX and startY are not ints, go to closest point
        currPoint = findClosestPointOnGrid() #finds closest point
        newAngle = math.atan((currPoint[1] - startposy) / (currPoint[0] - startposx)) * (180 / math.pi) #degrees
        #if decreasing, newAngle must be flipped over y-axis, ie (180-newAngle) and make that positive
        
        if currPoint[0]-1 >= 0 and grid[0][currPoint[0]-1] == 1: #i.e the path is moving downwards
            newAngle = 180-newAngle #flips over the y-axis
            if newAngle < 0: #makes newAngle positive
                newAngle += 360
        else: #i.e path is moving upwards
            newAngle = newAngle * -1 #negate the newAngle, flips over the x-axis
            if newAngle < 0: #makes newAngle positive
                newAngle += 360
        
        """
        if currPoint[0] < startposx:
            newAngle = newAngle * -1
            newAngle = newAngle + 180
        else:
            newAngle = newAngle * -1
        """
        distance = math.sqrt(pow((startposx - currPoint[0]) , 2) + pow((startposy - currPoint[1]) , 2)) #distance formula
    return newAngle, distance, currPoint

print("Path:")
for row in range(len(grid)):
    for col in range(len(grid[row])):
        print(str(grid[row][col]) + " ", end='')
    print()
print("-" * len(grid[0]) * 2)

newnewAngle, gridPathDistance, newPoint = pathChange()
print("Starting Position: (" + str(startposx) + ", " + str(startposy) + ")")
print("New Position: (" + str(newPoint[0]) + ", " + str(newPoint[1]) + ")")
print("Initial newAngle: " + str(startnewAngle) + " degrees")
print()
print("Distance Needed to Travel: " + str(gridPathDistance*unitOfMeasure) + " meters at " + str(newnewAngle) + " degrees")
print("Time at Current Velocity: " + str((gridPathDistance*unitOfMeasure) / velocity) + "s ")
