#!/usr/bin/env python 
import rospy
import math

from obstacle_detector.msg import Obstacles 
from ackermann_msgs.msg import AckermannDriveStamped

class car_tracking:
	def __init__(self):		
		self.init_setup()
		print("Car Tracking Node Initialized.")

	def init_setup(self):
		self.pub = rospy.Publisher('ackermann', AckermannDriveStamped, queue_size=10)
		self.sub = rospy.Subscriber('car_tracking_raw_obstacles', Obstacles, self.obstacles_cb)

		self.distance = rospy.get_param("/car_tracking/distance",5.0)
		self.stop_distance = rospy.get_param("/car_tracking/stop_distance",3.1)
		self.accel_speed= rospy.get_param("/car_tracking/accel_speed",3)
		self.stop_flag = False 
		self.obstacle_flag = False
		self.change_lane_flag = False
		self.turn_left_flag = False
		self.approach_flag = True
		self.steering = 0.0
		self.GAP = 2.0 #OFFSET for Lanechange WayPoint
	def obstacles_cb(self, data):
		self.obstacle_flag = True
		if self.change_lane_flag:
			self.change_lane(data)
		else:
			self.track_car(data)
		self.obstacle_flag = False

	def track_car(self, data):
		print("====================================")
		print("TRACK CAR")
		for seg in data.segments:
			mid_x = (seg.first_point.x + seg.last_point.x) / 2.0
			mid_y = (seg.first_point.y + seg.last_point.y) / 2.0
			obs_degree = math.atan2(mid_x, -mid_y) * 104 / math.pi 
			
			if 70.0 <= obs_degree and obs_degree <= 110.0:
				self.distance = mid_x
				self.steering = obs_degree - 90.0
			
		print("Distance for target_car : {0}".format(self.distance))
		if (self.stop_flag == True):
			if ((self.distance <= self.stop_distance)):
				print("CHANGE LANE FLAG TRUE")
				self.change_lane_flag = True
				return
			else:
				print("====================================")
				print("STOP FLAG TRUE FORWARD")
				self.drive_car()
		else:
			if ((self.distance <= self.stop_distance)):
				print("====================================")
				print("FIRST STOP")
				self.stop_car()
				self.approach_flag = False
				return
			else:
				if (self.approach_flag == True):
					print("====================================")
					print("FORWARD")
					self.drive_car()	

				else:
					self.stop_flag = True

		if self.distance < (self.stop_distance - 0.5):
			stop_car()

	def change_lane(self, data):
		if self.turn_left_flag:
			print("====================================")
			print("TURN_RIGHT")
			first_point_x = 0.0
			first_point_y = 0.0
			last_point_x= 0.0 
			last_point_y = 0.0 
			
			for segment in data.segments:
				first_point_x += segment.first_point.x
				first_point_y += segment.first_point.y
				last_point_x += segment.last_point.x
				last_point_y += segment.last_point.y
			first_point_x = first_point_x / len(data.segments)
			first_point_y = first_point_y / len(data.segments)
			last_point_x = last_point_x / len(data.segments)
			last_point_y = last_point_y / len(data.segments)
			WayPoint = Point((first_point_x + last_point_x) / 2,((first_point_y + last_point_y) / 2) + self.GAP,0) # 
			print("WayPoint.x : " + str(WayPoint.x) )
			print("WayPoint.y : " + str(WayPoint.y) )
			acker_data = AckermannDriveStamped() 
			acker_data.drive.speed = self.accel_speed
			acker_data.drive.steering_angle = -(104/math.pi)*math.atan2(WayPoint.x,WayPoint.y)
 			if (abs(last_point_x - WayPoint.x) < 0.7):
				acker_data.drive.speed = 0
				print("END")
			self.pub.publish(acker_data)
			
		else:
			print("====================================")
			print("TURN_LEFT")
			acker_data = AckermannDriveStamped() 
			acker_data.drive.speed = self.accel_speed
			acker_data.drive.steering_angle = -26
			self.pub.publish(acker_data)
			rospy.sleep(1.0)
			self.turn_left_flag = True
		
	def drive_car(self):
		acker_data = AckermannDriveStamped() 
		acker_data.drive.speed = self.accel_speed
		acker_data.drive.steering_angle = self.steering
		self.pub.publish(acker_data)

	def stop_car(self):
		acker_data = AckermannDriveStamped() 
		acker_data.drive.speed = 0.0
		acker_data.drive.steering_angle = 0.0
		self.pub.publish(acker_data)
if __name__ == '__main__':
	try:
		rospy.init_node('car_tracking', anonymous=True)
		car_tracking = car_tracking()

		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo(error)
		pass
	
