#!/usr/bin/env python
#Developed by Xinyi Jiang, Jiaming Xu, Ben Winschel
#Oct 2017 
import rospy #import the ros python package
import math
import time
import sys

from geometry_msgs.msg import Twist #Message type for robot motion
from sensor_msgs.msg import LaserScan #Message type for laser data


class ScanTwistControlLoop:	
	def __init__(self, scan_topic, pub_topic):
		self.scan_topic_name = scan_topic
		self.pub_topic_name = pub_topic
		self.is_approching_wall = False
		self.wallDistance = 0.2 # Desired distance from the wall.
		self.e = 0.0            # Difference between desired distance from the wall and actual distance.
		self.diffE = 0.0        # Derivative element for PD controller
		self.maxSpeed = 0.2     # Maximum speed of robot.
		self.P = 10.0           # k_P Constant for PD controller.
		self.D = 5.0            # k_D Constant for PD controller.
		self.angleCoef = 1.0    # Coefficient for P controller.
		self.direction = 1      # 1 for LHR, -1 for RHR
		self.angleMin = 0.0     # Angle, at which was measured the shortest distance.
		self.distFront = 0.0    # Distance, measured by ranger in front of robot.

	
	def scanCallback(self,msg):
		ranges = [0 for i in range(360)]
		for i in range(180):
			if msg.ranges[i + 180] == 0:
				ranges[i] = 100
			else:
				ranges[i] = msg.ranges[i + 180]

			if msg.ranges[i] == 0:
				ranges[i+180] = 100
			else:
				ranges[i+180] = msg.ranges[i]
		size = len(ranges)
		minIdx = size * (self.direction + 1) / 4
		maxIdx =  size * (self.direction + 3) / 4
		half_ranges = ranges[minIdx : maxIdx]
		indexMin = ranges.index(min(half_ranges))
		self.angleMin = (indexMin - size / 2) * msg.angle_increment
		distMin = ranges[indexMin]
		self.distFront = ranges[size / 2]
		self.diffE = (distMin - self.wallDistance) - self.e
		self.e = distMin - self.wallDistance
		self.publishMsg()

	def publishMsg(self):
		twist = Twist()
		# print self.e
		# print self.diffE
		# print self.direction * (self.P * self.e + self.D * self.diffE)
		# print self.angleCoef * (self.angleMin - math.pi * self.direction / 2)
		twist.angular.z = self.direction * (self.P * self.e + self.D * self.diffE) + self.angleCoef * (self.angleMin - math.pi * self.direction / 2)  
		# PD controller
		# print twist.angular.z
		if (self.distFront < 2* self.wallDistance):
		    twist.angular.z = self.direction * - 2.0
		    print 1
		elif (self.distFront < self.wallDistance * 4):
		    twist.linear.x = 0.5 * self.maxSpeed
		    print 2
		elif (math.fabs(self.angleMin) > 1.75):
		    twist.linear.x = 0.4 * self.maxSpeed
		    print 3
		else:
		    print 4
		    twist.linear.x = self.maxSpeed
		self.cmd_vel_pub.publish(twist)
		print twist


	def start(self):
		self.sub = rospy.Subscriber(self.scan_topic_name,LaserScan,self.scanCallback)
		self.cmd_vel_pub = rospy.Publisher(self.pub_topic_name, Twist, queue_size=1)
		#Every time you receive a new message, the scanCallback will be called.
		#root.mainloop() #This just goes into an infinite loop and stops the program from exiting.
		while not rospy.is_shutdown():
		    continue
		twist = Twist()
		self.cmd_vel_pub.publish(twist)


def main():
	rospy.init_node('rosmaze_solver')
	#The first thing to do is always initialize the node.
	scan_monitor = ScanTwistControlLoop("/scan","cmd_vel")
	scan_monitor.start()

if __name__ == '__main__':
	main()
