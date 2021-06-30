#!/usr/bin/env python


import time
import enum

class Position(enum.Enum):
    NO = 0
    LEFT = 1
    RIGHT = 2

class ObstacleDetector:
    def __init__(self):
        self.mode = Position.NO
        self.previous_time = 0
        self.cnt = 0

    # return EnumClass Position
    def check(self, obstacles):
        for circle in obstacles.circles:
            p = circle.center
            if -1.0 < p.y < 0:
                if abs(p.x) < 0.3:
                    if p.x >= 0.1:
                        self.mode = Position.RIGHT
                    else:
                        self.mode = Position.LEFT
                        self.cnt += 1
                    break
            else:
                self.mode = Position.NO
        return self.mode

obstacles = None
def calibration(obstacles):
    list_x=[]
    list_y=[]
    for circle in obstacles.circles:
    	p = circle.center
	list_x.append(p.x)
	list_y.append(p.y)
    print(sum(list_x)/len(list_x))
    print(sum(list_y)/len(list_y))
    print("End! \n")
      

def obstacle_callback(data):
    global obstacles
    obstacles = data
    

if __name__ == '__main__':
    from obstacle_detector.msg import Obstacles
    import rospy
    rospy.init_node('TEST')
    rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size = 1)
    ob = ObstacleDetector()

    time.sleep(3)

    while not rospy.is_shutdown():
        #ob.check(obstacles)
        #print(ob.mode)
	calibration(obstacles)
        time.sleep(0.1)

    print('Done')

