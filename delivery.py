#!/usr/bin/env python

import rospy
import math, time

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from race.msg import drive_values
from math import atan2, degrees


drive_values_pub = rospy.Publisher('control_value', drive_values, queue_size=1)



def cal_distance(x,y):
	return (math.sqrt(x**2+y**2))

))
			


def drive(angle, speed):
	global drive_values_pub
	
	drive_value = drive_values()
	drive_value.steering = angle
	standard_angle=18
	if (drive_value.steering>standard_angle):
		drive_value.throttle=3

	elif (drive_value.steering<-standard_angle):
		drive_value.throttle=3

	else:
		drive_value.throttle = speed

	drive_values_pub.publish(drive_value)

	print("steer : ", drive_value.steering)
	#print("throttle : ", drive_value.throttle)
class point:
	def __init__(self):
		self.distance_min=None
		self.obData = None
		self.center_x = None
		self.center_y = None
		self.xycar_angle = None
		self.xycar_angle_deg = None
		self.angle=None
		self.normal_speed=6
		self.slow_speed=3

	def cal_distance_two_circle(self,x1,y1,x2,y2):
		distance=math.sqrt((x2-x1)**2+(y2-y1)**2)
		return distance

			

	def circles_4_center(self,sorted_list):
		sorted_2_list=None
		sorted_2_list=sorted(sorted_list,key= lambda circle:circle.center.y)
		return sorted_2_list

	def get_center(self, obstacles):
		self.obData = obstacles

	def calc_dismin(self,p1, p2, p3, p4):
		dis1=math.sqrt(p1.x**2+p1.y**2)
		dis2=math.sqrt(p2.x**2+p2.y**2)
		dis3=math.sqrt(p3.x**2+p3.y**2)
		dis4=math.sqrt(p4.x**2+p4.y**2)
		temp_list=[[dis1,p1],[dis2,p2],[dis3,p3],[dis4,p4]]
		sorted_list=sorted(temp_list,key=lambda sub_list:(sub_list[0]))
		min_list=sorted_list[0]
		return min_list

	


	def calc_angle(self):
		self.xycar_angle = math.atan2(self.center_y,self.center_x)
		self.xycar_angle_deg = self.xycar_angle*180/math.pi
		if (self.xycar_angle_deg > 30): self.xycar_angle_deg = 30
		elif (self.xycar_angle_deg < -30): self.xycar_angle_deg = -30
		#print("Angle deg : ", self.xycar_angle_deg)

	def circle(self):
		
		print("CNT:",len(self.obData.circles))
		if len(self.obData.circles) == 0:
			print("zero obstacle")
			drive(0,7)

		elif len(self.obData.circles) == 1:
			circles=self.obData.circles
			if len(yellow_cone)==1 : 
			
				drive(25,3)
			 	print("one obstacle detected && right steering")
			else: 
				drive(-25,3)
				print("one obstacle detected && left steering")
				




		else:
			circles=self.obData.circles # List of Circles Data
			#print("Circle:",self.obData.circles[0].center.x)
			left_circle=None
			right_circle=None
			sorted_list=[1,2,3,4]
			sorted_list_3=[1,2,3]
			a=1.4
	        	b=0.1
			'''
			if len(blue_cone) == 0:
				print("blue cone is not detected!!")
				drive(30,self.normal_speed)
			if len(yellow_cone) == 0:
				print("yellow cone is not detected!!")
				drive(-30,self.normal_speed)'''
 			if(len(circles)==2):
		
		
				if(circles[0].center.y<circles[1].center.y):  #judgement left & right
					left_circle=circles[0]
					right_circle=circles[1]

				else:
					left_circle=circles[1]
					right_circle=circles[0]
	
				self.two_circle_distance=self.cal_distance_two_circle(left_circle.center.x,left_circle.center.y,right_circle.center.x,right_circle.center.y)
				self.two_between = self.angle_between(right_circle.center.x,left_circle.center.x,0,right_circle.center.y,left_circle.center.y,0)

				if (len(yellow_cone)==2 or len(yellow_cone)==1) and len(blue_cone)==0 : #(self.two_circle_distance>3.0 and left_circle.center.x + 0.4 < right_circle.center.x and self.two_between > 90):
					print("two_obstacle_right")
					drive(30,self.normal_speed)
				else:

					if(right_circle.center.y<0):
	 					drive(20,self.normal_speed)
						return

					elif(left_circle.center.y>0):
						drive(-20,self.normal_speed)
						return
					else:

						#print("left_circle:",left_circle.center.y,"right_circle:",right_circle.center.y)
						left_point=left_circle.center
						right_point=right_circle.center
						self.center_x=(left_point.x+right_point.x)/2
						self.center_y=(left_point.y+right_point.y)/2
						self.calc_angle()
						drive(self.xycar_angle_deg,self.normal_speed)


			elif (len(circles)==3):
				sorted_first_list=sorted(circles,key= lambda circle:circle.center.x)
				sorted_list_3[0]=sorted_first_list[0]
				sorted_list_3[1]=sorted_first_list[1]
				sorted_list_3[2]=sorted_first_list[2]

				sorted_2_list=self.circles_4_center(sorted_list_3)

				filter_point1=sorted_2_list[0].center
				filter_point2=sorted_2_list[1].center
				filter_point3=sorted_2_list[2].center

				self.center_x,self.center_y=self.calcEquidistance(filter_point1.x,filter_point2.x,filter_point3.x,filter_point1.y,filter_point2.y,filter_point3.y)
				self.calc_angle()
				if len(yellow_cone)>=2 and len(blue_cone)==0:
					self.xycar_angle_deg+=5
					drive(self.xycar_angle_deg,self.normal_speed)
					print(" yellow_cone==2")
				else:								self.three_detected_obstacles=self.angle_between(filter_point1.x,filter_point2.x,filter_point3.x,filter_point1.y,filter_point2.y,filter_point3.y)
					#print("angle: ",self.three_detected_obstacles)

			else:
				sorted_first_list=sorted(circles,key= lambda circle:circle.center.x)
				sorted_list[0]=sorted_first_list[0]
				sorted_list[1]=sorted_first_list[1]
				sorted_list[2]=sorted_first_list[2]
				sorted_list[3]=sorted_first_list[3]

				sorted_2_list=self.circles_4_center(sorted_list)

				left_circle_1=sorted_2_list[0]
				left_circle_2=sorted_2_list[1]
				right_circle_1=sorted_2_list[2]
				right_circle_2=sorted_2_list[3]
				left_point1=left_circle_1.center
				left_point2=left_circle_2.center
				right_point1=right_circle_1.center
				right_point2=right_circle_2.center

				self.center_x=(left_point1.x+left_point2.x+right_point1.x+right_point2.x)/4
				self.center_y=(left_point1.y+left_point2.y+right_point1.y+right_point2.y)/4
				self.calc_angle()
				if len(yellow_cone)>=2 and len(blue_cone)==0:
					self.xycar_angle_deg+=5
					drive(self.xycar_angle_deg,self.slow_speed)
					min_list=self.calc_dismin(left_point1,left_point2,right_point1,right_point2)
					#print("avoid:",min_list[1].y)
					print('center_y:' ,self.center_y)
					if(min_list[0]<1.3 and abs(min_list[1].y)<1.1):
						self.avoid_collision(min_list)
						print('front_avoid')
					else:
						drive(self.xycar_angle_deg,self.normal_speed)
						#print("normal path")
						return
				else:
						min_list=self.calc_dismin(left_point1,left_point2,right_point1,right_point2)
					#print("avoid:",min_list[1].y)
					print('center_y:' ,self.center_y)
					if(min_list[0]<1.3 and abs(min_list[1].y)<1.1):
						self.avoid_collision(min_list)
						print('front_avoid')

				
					else:
						drive(self.xycar_angle_deg,self.normal_speed)
						#print("normal path")
						return



if __name__=='__main__':

	ob=point()
	rospy.Subscriber("/obstacles",Obstacles, ob.get_center, queue_size=1)
	rospy.init_node('test3')
	time.sleep(1)
	while not rospy.is_shutdown():
		time.sleep(0.1)
		ob.circle()
		
	print('Done')
