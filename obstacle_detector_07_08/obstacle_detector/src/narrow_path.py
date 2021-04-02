#!/usr/bin/env python 

import rospy
import math, time

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
# from xycar_motor.msg import xycar_motor



class test:
	
	def __init__(self):
		rospy.init_node('test_path')

		self.pub = rospy.Publisher('ackermann',AckermannDriveStamped, queue_size=10)
		# self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)		
		self.obData = None
		self.center_x = None
		self.center_y = None
		self.xycar_angle = None
		self.xycar_angle_deg = None

	def get_center(self, obstacles):
		self.obData = obstacles	

	def circle(self):
		list_x=[]
		list_y=[]
		if len(self.obData.circles) == 0:
			return
		for circle in self.obData.circles:
			p = circle.center
			list_x.append(p.x)
			list_y.append(p.y)
		self.center_x = sum(list_x)/len(list_x)
		self.center_y = sum(list_y)/len(list_y)
		print(sum(list_x)/len(list_x))
		print(sum(list_y)/len(list_y))
		print(len(list_x))
		print("End! \n")
		self.calc_angle()
		self.publish_angle()

	def calc_angle(self):
		self.xycar_angle = math.atan(self.center_x/self.center_y)
		self.xycar_angle_deg = -self.xycar_angle*180/math.pi
		print(self.xycar_angle_deg)
		
	def publish_angle(self):
		acker_data = AckermannDriveStamped()
		acker_data.drive.speed = 3
		acker_data.drive.steering_angle = self.xycar_angle_deg
		self.pub.publish(acker_data)
		"""
		motor_msg = xycar_motor()
		motor_msg.header.stamp = rospy.Time.now()
		motor_msg.angle = self.xycar_angle_deg
		motor_msg.speed = 3
		self.pub.publish(motor_msg)"""



if __name__=='__main__':
	ob=test()
	rospy.Subscriber("/obstacles",Obstacles, ob.get_center, queue_size=1)
	

	time.sleep(3)
	while not rospy.is_shutdown():
		ob.circle()
		time.sleep(1)


	print('Done')
