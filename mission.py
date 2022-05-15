#This is a script for a simple delivery mission , it arms and flys a drone to a given location while also checking for obstacles
#It drops off a package at the given location
#Author #AJ ABISOLA

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import argparse
import math
from pymavlink import mavutil
import cv2
from cv2 import aruco
import RPi.GPIO as GPIO

TRIG = 13
ECHO1 = 21
ECHO2 = 20
ECHO3 = 12
ECHO4 = 16

Drop = 6
Pick = 5
y = 0

GPIO.setmode(GPIO.BCM)

cap = cv2.VideoCapture(0)


#Connectcopter links the raspberry pi to the drone using serial connection
def connectcopter():
	parser = argparse.ArgumentParser(description ='commands')
	parser.add_argument('--connect',default = '127.0.0.1:14550')
	args = parser.parse_args()

	connection_string = args.connect
	baud_rate = 112500

	vehicle = connect(connection_string,baud = baud_rate, wait_ready = True)
	return vehicle


#arming_and_takeoff arms the drone and flies vetically to a specified height
def arming_and_takeoff(target):
	while not drone.is_armable:
		print('Waiting for drone to be armed')
		time.sleep(1)

	drone.armed = True
	while drone.armed == False:
		print('Drone yet to be armed')
		time.sleep(1)
	print ('Armed')
	print ('Taking off!')
	drone.simple_takeoff(target)

	while True:
		print('Current altitude: %s' %drone.location.global_relative_frame.alt)
		if drone.location.global_relative_frame.alt >= 0.95*target:
			break
		time.sleep(1)

	print('Target altitude reached')
	return None


#get_distance_metres gets the distance between two waypoints 
def get_distance_metres(aLocation1, aLocation2):
	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


#send_ned_velocity sends the desired direction to the drone during obstacle avoidance
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
	msg = drone.message_factory.set_position_target_local_ned_encode(
		0,       
		0, 0,    
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
		0b0000111111000111,
		0, 0, 0, 
		velocity_x, velocity_y, velocity_z, 
		0, 0, 0, 
		0, 0)    
	for x in range(0,duration):
		drone.send_mavlink(msg)
		time.sleep(1)

#obstaclecheck looks for obstacle
def obstaclecheck(ECHO):
	GPIO.setmode(GPIO.BCM)
	while True:
		GPIO.setup(TRIG,GPIO.OUT)
		GPIO.setup(ECHO,GPIO.IN)
		GPIO.output(TRIG,False)
		time.sleep(0.2)
		GPIO.output(TRIG,True)
		time.sleep(0.00001)
		GPIO.output(TRIG,False)
		while GPIO.input(ECHO)==0:
			pulse_start=time.time()
		while GPIO.input(ECHO)==1:
			pulse_end=time.time()
		pulse_duration=pulse_end-pulse_start
		distance=pulse_duration*17150
		distance=round(distance,2)
		return distance


#dropparcel sends command to the delivery system on the drone to drop parcel
def dropparcel():
	GPIO.setup(Drop, GPIO.OUT)
	GPIO.setup(Pick, GPIO.OUT)

	GPIO.output(Drop, True)
	time.sleep(1)
	GPIO.output(Drop, False)
	time.sleep(40)
	GPIO.output(Drop, True)

	time.sleep(10)

	GPIO.output(Pick, True)
	time.sleep(1)
	GPIO.output(Pick, False)
	time.sleep(40)
	GPIO.output(Pick, True)

	print ('Done')
	GPIO.cleanup()


#avoidance works with obstaclecheck function and send_ned_velocity
def avoidance():
	present = drone.location.global_frame.lat
	direction = present - present
	no1 = obstaclecheck(ECHO1)
	no2 = obstaclecheck(ECHO2)
	no3 = obstaclecheck(ECHO3)
	no4 = obstaclecheck(ECHO4)
	NORTH = 2
	SOUTH = -2
	EAST = -2
	WEST = 2

	#move away for 5seconds

	if no1 <= 30.0 and direction == 0:
		send_ned_velocity(SOUTH,0,0,5)

	elif no1 <= 30.0 and direction != 0:
		send_ned_velocity(NORTH,0,0,5)

	elif no2 <= 30.0 and direction == 0:
		send_ned_velocity(0,WEST,0,5)

	elif no2 <= 30.0 and direction != 0:
		send_ned_velocity(0,EAST,0,5)

	elif no3 <= 30.0 and direction == 0:
		send_ned_velocity(0,EAST,0,5)

	elif no3 <= 30.0 and direction != 0:
		send_ned_velocity(0,WEST,0,5)

	elif no4 <= 30.0 and direction == 0:
		send_ned_velocity(NORTH,0,0,5)

	elif no4 <= 30.0 and direction != 0:
		send_ned_velocity(SOUTH,0,0,5)
	else:
		print ('No obstacle to be avoided')


#markercheck looks for a specified marker before dropping parcel
def markercheck():
	while(True):
		ret, frame = cap.read()

		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)



		grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
		parameters =  aruco.DetectorParameters_create()
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)


		for rejectedPolygons in rejectedImgPoints:
			for points in rejectedPolygons:
				newp0 =[int(points[0][0]),int(points[0][1])]
				newp1 =[int(points[1][0]),int(points[1][1])]
				newp2 =[int(points[2][0]),int(points[2][1])]
				newp3 =[int(points[3][0]),int(points[3][1])]
				cv2.line(frame_markers, tuple(newp0), tuple(newp1), [100, 0, 100])
				cv2.line(frame_markers, tuple(newp2), tuple(newp1), [100, 0, 100])
				cv2.line(frame_markers, tuple(newp2), tuple(newp3), [100, 0, 100])
				cv2.line(frame_markers, tuple(newp0), tuple(newp3), [100, 0, 100])
		#  "53" is the index id of that particular Aruco Marker ( tetsed with 4X4 matrix marker)
		if (ids==53):
			y = 0
			break
		else:
			y = 1
			break

	return y


end = 0
drone = connectcopter()
drone.mode = VehicleMode('GUIDED')
#Wait for GPS to be active and for drone to get full parameters
while not drone.home_location:
	cmds = drone.commands
	cmds.download()
	cmds.wait_ready()
	if not drone.home_location:
		print(" Waiting for home location ..")
time.sleep(2)
home = drone.home_location
print(home)
missionend = 0
count = 0
#7.382015,3.875379
location1 = LocationGlobalRelative(7.381748,3.875265,6)
locationparcel = LocationGlobalRelative(7.381748,3.875265,2)
locationreturn = LocationGlobalRelative(7.381748,3.875265,6)
while missionend == 0:

	if count == 0:

		arming_and_takeoff(6)
		print('Picking up parcel..')
#		dropparcel()
		time.sleep(3)
#		drone.simple_goto(LocationGlobalRelative(home.lat,home.lon,6))
		targetDistance1=get_distance_metres(home, location1)
		print ('distance to target is: %s metres' %targetDistance1)
		time.sleep(2)
		while end == 0 :
#			drone.simple_goto(location1)
			while get_distance_metres(drone.location.global_frame, location1)> targetDistance1*0.02:
				print ('distance to target is: %s metres' %get_distance_metres(drone.location.global_frame, location1))
				drone.simple_goto(location1)
				if get_distance_metres(drone.location.global_frame, location1)<= targetDistance1*0.02:
					print('Reached target')
				time.sleep(3)
			print('Reached target')
			end =1
			break
		time.sleep(1)
		count = 1

	elif count == 1:
		#Drop to lower altitude to check for marker

		drone.simple_goto(locationparcel)
		time.sleep(7)
		checkingcv = 0

		#Searches for the marker for 45seconds before leaving.
		while checkingcv < 45:
			if markercheck() == 0:
				print('landing to drop parcel')
#				dropparcel()
#				time.sleep(20)
				drone.mode = VehicleMode('LAND')
				time.sleep(25)
				print('Parcel Delivered')
				drone.mode = VehicleMode('GUIDED')
				arming_and_takeoff(6)
				count = 2
				break
			print('Searching for marker..')
			time.sleep(1)
			checkingcv += 1
		else:
			print ('marker not found')
			print ('leaving target location')
			drone.simple_goto(locationreturn)
			time.sleep(5)
			count = 2

	elif count == 2:

		end = 0
		targetDistance2=get_distance_metres(location1, home)
		print('distance to AJs home is: %s metres' %targetDistance2)
		time.sleep(2)
		while end == 0 :
			while get_distance_metres(drone.location.global_frame, home)> targetDistance2*0.02:
				print('On the way home..')
				print ('distance to home is: %s metres' %get_distance_metres(drone.location.global_frame, home))
				drone.simple_goto(LocationGlobalRelative(home.lat,home.lon,6))
				time.sleep(3)
			if get_distance_metres(drone.location.global_frame, home)<= targetDistance2*0.02:
				print('Reached Home')
				drone.mode = VehicleMode('LAND')
				print ('landing..')
				time.sleep(5)
				end = 1
				break
		count = 3

	else:

		print('mission complete')
		missionend = 1
		break

print ('mission successful')
