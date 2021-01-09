#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty

import time
import math
import matplotlib.pyplot as plt

P0 = [0,0]
P1 = [50.0,250.0]
P2 = [100.0,0.0]
Qy = []
Qx = []
Qy1 = []
Qx1 = []
m = []
n = []
#________________________________________Global Variables__________________________________#
robotYaw = BotX = BotY = prevBotX = prevBotY = offest = startTime = 0
x1 = x2 = y1 = y2 = flag =  0
orient = 90 
flag = 1
check1 = 1
freq = 1000
pidOut = []
timeT = []
#_________________________________________Classes__________________________________________#
class PID:
	kp = 0
	kd = 0
	ki = 0
	required = 0 
	error = 0 
	prevError = 0
	derivative = 0
	integral = 0
	output = 0
	minControl = 0
	maxControl = 0

	def __init__(self,kp,kd,ki,required,minControl,maxControl):
		self.kp = kp
		self.kd = kd
		self.ki = ki
		self.required = required
		self.maxControl = maxControl
		self.minControl = minControl

	def pidControl(self,actual):
		if actual*self.required < 0 and abs(actual)>90 :
			self.error = -self.required + actual
		else:	 
			self.error = self.required - actual
		self.derivative = (self.error - self.prevError)*freq
		self.integral = self.integral + self.error
		self.prevError = self.error
		self.output = self.kp*self.error + self.kd*self.derivative + self.ki*self.integral
		if self.output > self.maxControl:
			self.output = self.maxControl
		elif self.output < self.minControl:
			self.output = self.minControl
		return self.output

ppidOmega = PID(2.35,0.0,0,0,-180,180) 
#_______________________________________________Functions_______________________________________#
def initSystem():
	rospy.init_node('Systemv1',anonymous='True')	

	rospy.Subscriber("/odom",Odometry,getXY)
	rospy.Subscriber("/mobile_base/sensors/imu_data",Imu,getYaw)
	

	
	
def getXY(PoseWithCovariance):	
	global BotX, BotY # My Axis
	BotY =  -(PoseWithCovariance.pose.pose.position.x - 0)*100 #cm
	BotX =  (PoseWithCovariance.pose.pose.position.y - 0)*100  #cm


def getYaw(Quaternion):	
	global robotYaw	
	qx = Quaternion.orientation.x
	qy = Quaternion.orientation.y
	qz = Quaternion.orientation.z
	qw = Quaternion.orientation.w
	
	robotYaw = -(math.atan2(2*(qz*qw + qy*qx),1- 2*(qz**2 + qy**2))*180/math.pi) 
	#x = int(abs(robotYaw)/robotYaw)	
	
	if robotYaw < -90.00 :
		robotYaw = -90.00 - (robotYaw + 180.00)	
	else:
		robotYaw= -robotYaw + 90.00	

	
def traceLine(x1,y1,x2,y2,linearV):
	
	global pidOut,timeT,check1	
	angleFix = Angle(BotX,BotY,x2,y2)
	ppidOmega.required = angleFix
			
	distLeft = dist(BotX,BotY,x2,y2)
	distMax = dist(x1,y1,x2,y2)

	angularV = ppidOmega.pidControl(robotYaw)*math.pi/180.0
	linearV = trapVel(distMax,distLeft,linearV)	
	
	distNow = distMax - distLeft
	pub.publish(Twist(Vector3(-linearV,0,0),Vector3(0,0,angularV)))	
	
	if int(distLeft) <= 1 :
		return 0
	return 1	
		

def dist(x1,y1,x2,y2):
	return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def Angle(x1,y1,x2,y2):
	angle = math.atan2((y2-y1),(x2-x1))/(math.pi/180)
	return angle	

def traceCircle(radius,linearV):

	angle = math.atan2(-(BotX-radius),BotY)/(math.pi/180)
	ppidOmega.required = angle
	
	angularV = ppidOmega.pidControl(robotYaw)*math.pi/180.0
	pub.publish(Twist(Vector3(-linearV,0,0),Vector3(0,0,angularV)))	
	print("angle= ",angle," yaw= ",robotYaw)
	
def trapVel(distMax,distLeft,vMax):
	distNow = int(distMax) - int(distLeft)
	
	if distMax == 0:
		return 0
		
	if distNow <= distMax/4.0:
		v = 4*(vMax - 0.1)*distNow/distMax + 0.1
	elif distNow > distMax/4.0 and distNow < 3.0*distMax/8.0:
		v = vMax
	else:
		v = 4*(-0.1)*distNow/(distMax) + 0.4
	
	return v 
				
	
def traceSquare(side,vel):
	global x1,y1,x2,y2,flag

	x1 = int(BotX)
	y1 = int(BotY)
	if x1 == 0 and y1 == 0:
		y2 = y1 + side
	elif y1 >= side and x1 <= 0:
		x2 = x1 + side
	elif y1 >= side and x1 >= side:
		y2 = 0
	elif y1 <= 0 and x1 >= side:
		x2 = 0
	traceLine(BotX,BotY,x2,y2,vel)
		
	
def getGoal():
	global x1,x2,y1,y2, flag, orient	

	if flag:
		flag = traceLine(x1,y1,x2,y2,0.5)
		print " X= ", int(BotX)," Y= ",int(BotY)," Yaw= ",int(robotYaw)
	else :
		ppidOmega.required = orient

		angularV = ppidOmega.pidControl(robotYaw)*math.pi/180.0
		pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,angularV))) 
		print " X= ", BotX," Y= ",BotY," Yaw= ",int(robotYaw)
		if abs(angularV) <= 0.0005:	
			print("__________ENTER POSE____________")		
			x1 = BotX
			y1 = BotY
			x2 = input("Enter goal (x) = ")
			y2 = input("Enter goal (y) = ")
			orient = input("Enter final yaw =")
			flag = 1	
	return 0

def generateCurve():
	global Qy,Qx,P0,P1,P2,Qx1,Qy1,m,n
	A = [P1[0]-P0[0] , P1[1]-P0[1]]
	B = [P2[0]-P1[0] , P2[1]-P1[1]]
	for T in range(100):
	    t = T/100.0
	   
	    Qy.append(((1-t)**2)*P0[1] + 2*(1-t)*t*P1[1] + (t**2)*P2[1])
	    Qx.append(((1-t)**2)*P0[0] + 2*(1-t)*t*P1[0] + (t**2)*P2[0])
	    Qx1.append(2*(1-t)*(P1[0]-P0[0]) + 2*t*(P2[0]-P1[0]))
	    Qy1.append(2*(1-t)*(P1[1]-P0[1]) + 2*t*(P2[1]-P1[1]))
	    m.append(5-10*t)
	    n.append(Qy1[T]/Qx1[T])
	plt.plot(n,m)
	plt.show()
	
def getAngle():
	angularV = ppidOmega.pidControl(robotYaw)*math.pi/180.0
	pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,angularV)))			
	if abs(angularV) <= 0.0008:
		ppidOmega.required = input("Enter angle = ")
	print " currentYaw = ",robotYaw," required =",ppidOmega.required
	
	
	
		
def actuate():
	global pub,startTime, orient
	orient = 90
	
	pub = rospy.Publisher('mobile_base/commands/velocity',Twist,queue_size=10)
	rate = rospy.Rate(freq)
	startTime = time.time()
	
	#print("__________GO TO GOAL____________")
	while not rospy.is_shutdown():				
		getGoal()
		#getAngle()			
		rate.sleep()		

if __name__ == '__main__':
           initSystem()
	   actuate()
	   rospy.spin()





           	

	





