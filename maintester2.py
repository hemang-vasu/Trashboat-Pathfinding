import sys; args = sys.argv[1:]
import cv2, math
#Ex Input: startposx(double), startposy(double), angle(double), velocity(m/s), unitOFMeasure
#2.6, 7.4, 326.02, 0.4, 1

#Needs to be an input for the camera output classification, LIDAR output, IMU output, and a use conveyor belt arg
#For the output classification, should be a string of the exact name; if no classification then should be an empty string
#Output of this code will be to the pixhawk to determine how much to rotate and to move (angle of rotation, how many feet to move)
#Output should also be position in an (x,y) like grid to input into path.py, as well as the bool that determines if the boat is allowed to veer in order to collect trash
#Goal can be to possibly return time so that distance is implied given a constant velocity
 #meters/second
print(args)
startPoint = (float(args[0]), float(args[1]))
startAngle = float(args[2])
velocity = float(args[3])
unitOfMeasure = float(args[4]) #1 meter for each grid unit

#creating the initial path
grid = [[0 for i in range(15)] for j in range (15)]

#setting up the path, integer "1" will be the path on the grid
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

def detectingTrashObject(): #returns an (x,y) double type tuple referring to any detected object's position, else return 0, currently not working on this
    return 0

def checkObstacleHindersPath(): #checks by image logic if the obstacle is in the way of the path
    return True

def detectingObstacleObject(): #returns an (x,y) double type referring to the obstacle position & absolute radius of that obstacle & if required to travel right or left
    return 0
    #return (4,5), 0.2, "left"
#returns angle on the grid between two array points
def getGridAngle(startPoint, endPoint):
    angle = math.atan((endPoint[1] - startPoint[1]) / (endPoint[0] - startPoint[0])) * (180 / math.pi) #degrees
    if endPoint[0] < startPoint[0]: #left case angle
            angle = (angle * -1) +180
    else: #right case angle
        angle = angle * -1
        if angle < 0: angle += 360
    return angle

#IMPORTANT: This case assumes that the inputs do not detect the trash and obstacle at the same time. This means that the case from path to trash is N/A in this scenario.
#This algorithm accounts for returning to path cases and following path cases
def obstacleAvoidance(startPoint, startAngle): #sets position to avoid obstacle and angle to get to that position
    obstaclePosition, radiusLen, direction = detectingObstacleObject()
    error = 0.5 #degree error
    angle = getGridAngle(startPoint, obstaclePosition)
    #check for error between start angle and angle derived from trash point and start position
    if angle >= startAngle + error or angle <= startAngle - error: #break case if error is not matched
        print("Error Thrown: Center not Found for Avoiding Trash")
        return 
    
    #convert obstacle point to cartesian coordinate via flipping over x-axis
    yDifference = math.abs(obstaclePosition[1] - startPoint[1])
    if startPoint[1] > obstaclePosition[1]:
        obstaclePosition[1] = obstaclePosition[1] + (2*yDifference)
    else:
        obstaclePosition[1] = obstaclePosition[1] - (2*yDifference)

    #math to find the point radiusLen distance away from the obstacle center point
    m = -1 / ((obstaclePosition[1] - startPoint[1]) / (obstaclePosition[0] - startPoint[0])) #perpendicular bisector slope
    a = radiusLen * (1/(math.sqrt(1+pow(m,2)))) #x change
    b = radiusLen * (m/(math.sqrt(1+pow(m,2)))) #y change
    newPoint = (0,0) #store value for the possible position to move to outside of obstacle

    #go through all 8 cases for possible position to move to outside of the obstacle
    if obstaclePosition[0] > startPoint[0]:
        if obstaclePosition[1] > startPoint[1]:
            if direction == "left": #Case 1
                newPoint = (obstaclePosition[0] - a, obstaclePosition[1] - b)
            else: #Case 2
                newPoint = (obstaclePosition[0] + a, obstaclePosition[1] + b)
        else:
            if direction == "left": #Case 3
                newPoint = (obstaclePosition[0] + a, obstaclePosition[1] + b)
            else: #Case 4
                newPoint = (obstaclePosition[0] - a, obstaclePosition[1] - b)
    else:
        if obstaclePosition[1] > startPoint[1]:
            if direction == "right": #Case 5
                newPoint = (obstaclePosition[0] + a, obstaclePosition[1] + b)
            else: #Case 6
                newPoint = (obstaclePosition[0] - a, obstaclePosition[1] - b)
        else:
            if direction == "right": #Case 7
                newPoint = (obstaclePosition[0] - a, obstaclePosition[1] - b)
            else: #Case 8
                newPoint = (obstaclePosition[0] + a, obstaclePosition[1] + b)

    #Convert newPoint back to array grid
    yDifference = math.abs(newPoint[1] - startPoint[1])
    if startPoint[1] > newPoint[1]:
        newPoint[1] = newPoint[1] + (2*yDifference)
    else:
        newPoint[1] = newPoint[1] - (2*yDifference)
    
    #find angle
    angle = getGridAngle(startPoint, newPoint)
    distance = math.sqrt(pow((startPoint[0] - newPoint[0]) , 2) + pow((startPoint[1] - newPoint[1]) , 2)) #distance formula

    return angle, distance, newPoint

def findClosestPointOnGrid(startPoint, grid): #will return (x,y) int tuple for the closest point on grid
    #camera will detect trash in an assumed radius, in this case it will be 1 meter to the left, 1 meter to the right, 1 meter up and down.
    xCoordinateLeft = math.floor(startPoint[0])
    xCoordinateRight = math.ceil(startPoint[0])
    yCoordinateDown = math.ceil(startPoint[1])
    yCoordinateUp = math.floor(startPoint[1])
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

def pathChange(startPoint, currAngle, grid): #Returns the angle that the boat needs to shift to along with the distance IN TERMS OF GRID UNITS
    distance = 0
    angle = 0
    currPoint = (0,0)
    if detectingObstacleObject() != 0:
        if checkObstacleHindersPath() == True:
            angle, distance, startPoint = obstacleAvoidance(startPoint, currAngle, grid)
    if startPoint[0].is_integer() and startPoint[1].is_integer(): #i.e starting on the path
        startPoint = (int(startPoint[0]), int(startPoint[1]))
        trashPos = detectingTrashObject() #detects trash
        if trashPos != 0: #i.e trash was detected
            #angle = getGridAngle(startPoint, trashPos)
            #distance = math.sqrt(pow((startPoint[0] - trashPos[0]) , 2) + pow((startPoint[1] - trashPos[1]) , 2)) #distance formula
            currPoint = trashPos #sets the destination point to the trash
        else: #meaning the boat is currently on the path
            if startPoint[0] == len(grid[0]) - 1: #end of path
                return startAngle, 0, (startPoint[0], startPoint[1]) #returns a distance of 0, program will know to end
            if startPoint[0]-1 >= 0 and grid[0][startPoint[0]-1] == 1: #i.e the boat is moving downwards
                if startPoint[1] == len(grid): #reaches bottom of grid
                    if grid[startPoint[1]][startPoint[0]+1] == 1: #path is on the right
                        return 0, 1, (startPoint[0] + 1, startPoint[1]) #1 unit to the right
                    else: #else move the point upwards
                        return 90, 1, (startPoint[0], startPoint[1] - 1)
                else:
                    return 270, 1, (startPoint[0], startPoint[1]+1) #1 unit down
            else: #i.e the boat is moving upwards
                if startPoint[1] == 0:
                    if grid[startPoint[1]][startPoint[0]+1] == 1: #path is on the right
                        return 0, 1, (startPoint[0] + 1, startPoint[1])
                    else: #boat moves downwards
                        return 270, 1, (startPoint[0], startPoint[1] + 1)
                else: #move upwards
                    return 90, 1, (startPoint[0], startPoint[1] - 1)
    else: #meaning startX and startY are not ints, go to closest point
        currPoint = findClosestPointOnGrid(startPoint, grid) #finds closest point

    angle = getGridAngle(startPoint, currPoint)
    distance = math.sqrt(pow((startPoint[0] - currPoint[0]) , 2) + pow((startPoint[1] - currPoint[1]) , 2)) #distance formula
    return angle, distance, currPoint

print("Path:")
for row in range(len(grid)):
    for col in range(len(grid[row])):
        print(str(grid[row][col]) + " ", end='')
    print()
print("-" * len(grid[0]) * 2)

newAngle, gridPathDistance, newPoint = pathChange(startPoint, startAngle, grid)
print("Starting Position: (" + str(startPoint[0]) + ", " + str(startPoint[1]) + ")")
print("New Position: (" + str(newPoint[0]) + ", " + str(newPoint[1]) + ")")
print("Initial Angle: " + str(startAngle) + " degrees")
print()
print("Distance Needed to Travel: " + str(gridPathDistance*unitOfMeasure) + " meters at " + str(newAngle) + " degrees")
print("Time at Current Velocity: " + str((gridPathDistance*unitOfMeasure) / velocity) + "s ")
