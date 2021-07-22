#!/usr/bin/env python

import rospy
import math, time

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
# from xycar_motor.msg import xycar_motor


class point:
	def __init__(self):
		rospy.init_node('test_path')
		self.obData = None
		self.center_x = None
		self.center_y = None
		self.xycar_angle = None
		self.xycar_angle_deg = None

	def get_center(self, obstacles):
		self.obData = obstacles	

	

	def segment(self):
		list_l_y=[]
		list_r_y=[]
		list_x=[]

		if len(self.obData.segments) == 0:
			self.xycar_angle_deg = 0
			#self.publish_angle()
			return
		for seg in self.obData.segments:
			fp = seg.first_point # first_point
			lp = seg.last_point # last_point
			if fp.y < -0.001 : 
				list_l_y.append(fp.y)
			elif fp.y > 0.001 :
				list_r_y.append(fp.y) 			
			list_x.append(fp.x)
			list_x.append(lp.x)

		if len(list_l_y) == 0 and len(list_r_y) == 0:
			self.xycar_angle_deg = 0
			#self.publish_angle()
			return

		elif len(list_l_y) == 0:
			list_l_y = [-i*1.5 for i in list_r_y]
		elif len(list_r_y) == 0:
			list_r_y = [-i*1.5 for i in list_l_y]
		

		self.center_y = (sum(list_l_y)/len(list_l_y) + sum(list_r_y)/len(list_r_y)) / 2.0
		# self.center_x = sum(list_x)/len(list_x)
		self.center_x = sum(list_x)/len(list_x)

		print("L_y : ", sum(list_l_y)/len(list_l_y))
		print("R_y : ", sum(list_r_y)/len(list_r_y))
		print("x : ", sum(list_x)/len(list_x))
		print("CNT : ", len(self.obData.segments))
		print("End! \n")
		self.calc_angle()
		#self.publish_angle()
		

	def calc_angle(self):
		self.xycar_angle = math.atan(self.center_y/self.center_x)
		self.xycar_angle_deg = self.xycar_angle*180/math.pi #obstacles chase
		if (self.xycar_angle_deg > 26): self.xycar_angle_deg = 26
		elif (self.xycar_angle_deg < -26): self.xycar_angle_deg = -26
		print("Angle Rad : ", self.xycar_angle)


if __name__=='__main__':
	ob=point()
	rospy.Subscriber("/obstacles",Obstacles, ob.get_center, queue_size=1)
	
	time.sleep(3)
	while not rospy.is_shutdown():
		#ob.circle()
		ob.segment()
		time.sleep(1)


	print('Done')
	
	
