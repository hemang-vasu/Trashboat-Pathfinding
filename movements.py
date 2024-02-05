# BASIC METHODS TO STEER THE BOAT
# UNIT CONVENTIONS FOR THIS CODE!!!!!!!!!!!!
# DISTANCES: FEET.
# ANGLES: RADIANS, 0 is ahead, pi/2 is right, -pi/2 is left (watch out for absolute vs relative to boat).
# POINTS: LAT & LONG (DEGREES) - negative values for south or west (check that this works for my code).
# ^ check that these conventions actually work in the code ^
from pymavlink import mavutil
import time, math


# UNCOMMENT WHEN PIXHAWK IS CONNECTED
the_connection = None
the_connection = mavutil.mavlink_connection("/dev/ttyACM0")
the_connection.armed   = True


#servo numbers for thrusters
leftThruster = 5
rightThruster = 7


# default pwm vals
defaultForwardLeft = 1600
defaultForwardRight = 1600
defaultBackwardsLeft = 1400
defaultBackwardsRight = 1400
leftStop = 1500
rightStop = 1500


#tolerances
rad_tolerance = 5 * math.pi/180 # 5 degrees is max tolerance in rotation
dist_tolerance = 2 # 2 foot tolerance


# BOAT MOTION & ROTATION METHODS:
def moveToPoint(myLongX,myLatY):
	#point is lat/long desired
	#turn angle
	#move forward
	lat, long = getCoords()
	dist, bearingRads = distance_bearing(lat, long, myLatY,myLongX)
	rotateToXRadians(bearingRads)
	moveXFeet(dist)


def moveXFeet(feet): # move forward
	global dist_tolerance
	gpsData = getGPSInfo()
	startLat = gpsData.lat
	startLong = gpsData.long
	#start forward motion
    
	setThrusterVal(rightThruster,defaultForwardRight)
	setThrusterVal(leftThruster, defaultForwardLeft)
	distanceTraveled = 0
	while math.abs(feet-distanceTraveled)>dist_tolerance: # some leeway
    	#measure amount traveled
        time.sleep(.1)
    	gpsData = getGPSInfo()
    	newLat = gpsData.lat
    	newLong = gpsData.long
    	distanceTraveled = distance_bearing(startLong, startLat, newLong, newLat)[0]
    	print("You have moved",distanceTraveled,"feet so far")
   	 

	print("Stop moving, You have moved about",feet,"feet")
	stopMoving()


def rotateXRadians(amtRads): # takes just an amount to rotate by
	myYaw = getAttitude().yaw
	rotateToXRadians(myYaw+amtRads)


def rotateToXRadians(goalYaw): # takes absolute angle (rads) that you want the boat to end up facing
	global rad_tolerance
    
	#angles are in radians
	#keep center of mass the same, so like move one thruster forward, the other backwards
	myYaw = getAttitude().yaw # -pi to pi
	print("Currently facing",radToDeg(myYaw),"degrees and need to turn to",radToDeg(goalYaw),"deg");
	# if my yaw is bigger than desiered, turn to the left
	# if my yaw is smaller, turn right
	amountRotate = myYaw - goalYaw
	sleepIncr = .1
	while math.abs(amountRotate)>rad_tolerance: # within an error
    	if amountRotate<0:
        	setThrusterVal(leftThruster,defaultForwardLeft)
        	setThrusterVal(rightThruster, defaultBackwardsRight)
        	time.sleep(sleepIncr)
    	else:
        	setThrusterVal(rightThruster,defaultForwardRight)
        	setThrusterVal(leftThruster, defaultBackwardsLeft)
        	time.sleep(sleepIncr)
    	#stopMoving()
    	amountRotate = getAttitude().yaw - goalYaw
    	print("you still need to rotate",radToDeg(amountRotate),"deg")
	stopMoving()
	print("Stop rotating, you have reached the desired angle!")




# PIXHAWK COMMUNICATION METHODS (GENERAL):
def getGPSInfo():
	msg = the_connection.mav.command_long_encode(1,0,
	mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,
	33, 0,0,0,0,0,0) # or maybe GLOBAL_POSITION_INT not data_stream_position
	the_connection.mav.send(msg)
	# hdg gives -- Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	return the_connection.recv_match(type="GLOBAL_POSITION_INT")


def getIMUInfo():
	msg = the_connection.mav.command_long_encode(1,0,
	mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,
	27,0,0,0,0,0,0)
	the_connection.mav.send(msg)
	return the_connection.recv_match(type="RAW_IMU")


def getAttitude():
	msg = the_connection.mav.command_long_encode(1,0,
	mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,
	30,0,0,0,0,0,0)
	the_connection.mav.send(msg)
	return the_connection.recv_match(type="ATTITUDE")




# SPECIFIC PIXHAWK RETURN METHODS:
def getCoords(): # returns lat,long
	gpsData = getGPSInfo()
	lat = gpsData.lat
	long = gpsData.long
	return (lat,long)

def getRC():
	msg = the_connection.mav.command_long_encode(1,0,
	mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,
	65,0,0,0,0,0,0)
	the_connection.mav.send(msg)
	return the_connection.recv_match(type="RC_CHANNELS")

def getR():
	msg = the_connection.mav.command_long_encode(1,0,
	mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,
	109,0,0,0,0,0,0)
	the_connection.mav.send(msg)
	return the_connection.recv_match(type="RADIO_STATUS")

# BASIC MOTION FUNCTIONS:
def setThrusterVal(thruster, pwm): # servo #5 for example for thruster one
	msg1 = the_connection.mav.command_long_encode(
	1,0, # target system, target component
	mavutil.mavlink.MAV_CMD_DO_SET_SERVO,0, # command, confirmation
	thruster,pwm,0,0,0,0,0 # servo num, pwm val, zeros...
	)
	the_connection.mav.send(msg1)


def stopMoving():
	setThrusterVal(leftThruster,leftStop)
	setThrusterVal(rightThruster,rightStop)    




# CALIBRATION METHODS:
def findStopValue(thruster):
	for i in range(1300,1800,10):
    	setThrusterVal(thruster, i)
    	print("ON PWM:",i)
    	time.sleep(3)




# CALCULATION METHODS:
def distance_bearing(homeLatitude, homeLongitude, destinationLatitude, destinationLongitude): # bearing is absolute # negative for west or south works!!


	"""
	Simple function which returns the distance and bearing between two geographic location


	Inputs:
    	1.  homeLatitude        	-   Latitude of home location
    	2.  homeLongitude       	-   Longitude of home location
    	3.  destinationLatitude 	-   Latitude of Destination
    	4.  destinationLongitude	-   Longitude of Destination


	Outputs:
    	1. [Distance, Bearing]  	-   Distance (in metres) and Bearing angle (in degrees)
                                    	between home and destination


	Source:
    	https://github.com/TechnicalVillager/distance-bearing-calculation
	"""
	R = 6371e3
	rlat1   =   homeLatitude * (math.pi/180)
	rlat2   =   destinationLatitude * (math.pi/180)
	rlon1   =   homeLongitude * (math.pi/180)
	rlon2   =   destinationLongitude * (math.pi/180)
	dlat	=   (destinationLatitude - homeLatitude) * (math.pi/180)
	dlon	=   (destinationLongitude - homeLongitude) * (math.pi/180)


	# Haversine formula to find distance
	a = (math.sin(dlat/2) * math.sin(dlat/2)) + (math.cos(rlat1) * math.cos(rlat2) * (math.sin(dlon/2) * math.sin(dlon/2)))
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


	# Distance in feet
	distance = R * c * 3.28084


	# Formula for bearing
	y = math.sin(rlon2 - rlon1) * math.cos(rlat2)
	x = math.cos(rlat1) * math.sin(rlat2) - math.sin(rlat1) * math.cos(rlat2) * math.cos(rlon2 - rlon1)
   
	# Bearing in radians
	bearing = math.atan2(y, x)
	bearingDegrees = bearing * (180/math.pi)
	out = [distance, bearing]


	return out


def degToRad(myDeg):
	return myDeg * math.pi/180


def radToDeg(myRad):
	return myRad * 180/math.pi
# test the various methods to fine tune their values
# also make sure your stop values and default speeds are good
# find default vals that are equivilent for right and left motors

moveXFeet(10)

rotateToXRadians(math.pi)





