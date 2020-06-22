#!/usr/bin/env python
import rospy
import tf
import actionlib
import time
import roslaunch

import math
import numpy as np
import random

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion  
from ar_track_alvar_msgs.msg import AlvarMarkers
from datmo.msg import TrackArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan

from tf import TransformListener

class TagClass:
	def __init__(self,ID,x_tag,y_tag,x_box,y_box,yaw,circle):
		self.ID = ID
		self.x_tag = x_tag
		self.y_tag = y_tag
		self.x_box = x_box
		self.y_box = y_box
		self.yaw = yaw
		self.circle = circle

class TagMatcher:

	def __init__ (self):
		self.tf = TransformListener()
		self.move = rospy.Publisher("/husky_velocity_controller/cmd_vel",Twist, queue_size = 10 )
		self.base = rospy.Publisher("/base_joint_position_controller/command",Float64,queue_size=10)
		self.shoulder = rospy.Publisher("/shoulder_joint_position_controller/command",Float64,queue_size=10)
		self.elbow = rospy.Publisher("/elbow_joint_position_controller/command",Float64,queue_size=10)
		self.wrist = rospy.Publisher("/wri_joint_position_controller/command",Float64,queue_size=10)
		self.markerlist = []
		self.readedmarkers = []
		self.readedmarkers_tf = []
		self.taglist = [0,1,2,3]
		rospy.loginfo("TagMatcher Initalized!!!!!!")	

	def move_robot_arm(self,base_angle,elbow_angle,shoulder_angle,wrist_angle):
		self.base.publish(base_angle)
		self.shoulder.publish(shoulder_angle)
		self.elbow.publish(elbow_angle)
		self.wrist.publish(wrist_angle)

	def explore_n_catch(self):
		rospy.sleep(0.5)
		rate = rospy.Rate(5)
		vision(True)
		if self.check_readed_markers():
			self.walk()
			rate.sleep()
		else:
			print self.readedmarkers_tf
			raw_input('ne yapucuik')
		vision(False)
		
	def check_readed_markers(self):
		# check readedmarkers_tf
		if len(self.readedmarkers) >= 4:
			return False
		else:
			return True	


	def laser_classification(self):
		msg = rospy.wait_for_message("/scan",LaserScan)
		regions = {
			'right':  min(min(msg.ranges[0:143]), 10),
        	'fright': min(min(msg.ranges[144:287]), 10),
        	'front':  min(min(msg.ranges[288:431]), 10),
        	'fleft':  min(min(msg.ranges[432:575]), 10),
        	'left':   min(min(msg.ranges[576:713]), 10),
		}
		return regions

	def random_walk(self,point):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = point[0]
		goal.target_pose.pose.position.y = point[1]
		orientation = tf.transformations.quaternion_from_euler(0, 0, point[2])
		goal.target_pose.pose.orientation.x = orientation[0]
		goal.target_pose.pose.orientation.y = orientation[1]
		goal.target_pose.pose.orientation.z = orientation[2]
		goal.target_pose.pose.orientation.w = orientation[3]
		result = movebase_client(goal)
		if result:
			rospy.loginfo("Goal Execution Done!")

	def walk(self):
		waypoints = [(-1,-1.65,0),(3,3.77,3.13),(1,-1.86,3.13),(1,3.05,0),(0,0,0)]
		front = 0
		for point in waypoints:
			self.random_walk(point)
			distance = self.laser_classification()
			front = min(distance['front'],min(distance['fleft'],distance['fright']))-0.2
			self.moveforward(front)
			self.moveforward(-front)
			self.Fusion()
			yaw = self.getHuskyPose()[3][2]
			self.rotateccw(yaw+0.45)
			laser = self.laser_classification()
			distance = min(min(laser['fleft'],laser['fright']),laser['front'])
			self.moveforward(front)
			self.moveforward(-front)
			self.Fusion()
			self.rotateccw(yaw-0.45)
			laser = self.laser_classification()
			distance = min(min(laser['fleft'],laser['fright']),laser['front'])
			self.moveforward(front)
			self.moveforward(-front)
			self.Fusion()

	def calibrate_dance(self):
		rospy.loginfo("AMCL calibration")
		self.moveforward(2)
		self.moveforward(-2)
		
	def moveforward(self,distance,rotation=0.0):
		speed = Twist()
		laser_msg = rospy.wait_for_message("/scan",LaserScan)
		initial_range = laser_msg.ranges[360]
		if (distance > 0 and initial_range > distance) or distance < 0: 
			print distance
			while round(((laser_msg.ranges[360] - initial_range)+distance),1)!= 0:
				#print("%.2f ---> %.2f"%(-1*distance,(laser_msg.ranges[360] - initial_range)))
				laser_msg = rospy.wait_for_message("/scan",LaserScan)
				if min(laser_msg.ranges[144:575]) < 0.1:
					break
				speed.linear.x = (distance+(laser_msg.ranges[360] - initial_range))
				speed.angular.z = rotation
				self.move.publish(speed)
				#print(speed)
			
	def rotateccw(self,target_heading):
		direction = "ccw"
		if target_heading > 3.14:
			target_heading = -3.14 + (target_heading-3.14) 
			direction = "ccw"
		elif target_heading < -3.14:
			target_heading = 3.14 + (target_heading + 3.14)
			direction = "cw"
		speed = Twist()
		heading = tf.transformations.euler_from_quaternion(self.getHuskyPose()[1])[2]
		if (target_heading - heading) >= 0:
			direction = "ccw"
		else:
			direction = "cw"
		print("%.2f\t%s"%(target_heading,direction))
		while not (round(target_heading-heading,1) <= 0.15 and round(target_heading-heading,1) >= -0.15): 
			#print("%.2f ---> %.2f"%(heading,target_heading))
			heading = tf.transformations.euler_from_quaternion(self.getHuskyPose()[1])[2]
			speed.linear.x = 0.0
			if direction == "ccw":
				speed.angular.z = 1.0
			else:
				speed.angular.z = -1.0
			self.move.publish(speed)

	def getMessages(self):
		L_msg = rospy.wait_for_message("/datmo/box_kf",TrackArray,timeout=None)
		T_msg = rospy.wait_for_message("/ar_pose_marker",AlvarMarkers,timeout=None)
		return L_msg, T_msg

	def Fusion(self):
		rospy.loginfo("Fusion!!!")
		L_msg, T_msg = self.getMessages()
		if len(T_msg.markers) >= 1:
			for i in range(0,len(T_msg.markers)):
				for k in range(0, len(L_msg.tracks)):
					# Tag
					self.tag_frame_name = "/ar_marker_" + str(T_msg.markers[i].id)
					# self.tag_trans, self.tag_orient, self.tag_matrix = self.getTagFrame(self.tag_frame_name)
					self.tag_matrix = self.getTagFrame_wrt_map(self.tag_frame_name)
					self.tag_trans = (self.tag_matrix[0][3],self.tag_matrix[1][3],self.tag_matrix[2][3])
					# Box
					box_x = L_msg.tracks[k].odom.pose.pose.position.x
					box_y = L_msg.tracks[k].odom.pose.pose.position.y
					box_orientation = (
						L_msg.tracks[k].odom.pose.pose.orientation.x,
						L_msg.tracks[k].odom.pose.pose.orientation.y,
						L_msg.tracks[k].odom.pose.pose.orientation.z,
						L_msg.tracks[k].odom.pose.pose.orientation.w
					)
					box_euler = tf.transformations.euler_from_quaternion(box_orientation)
					circle = math.sqrt(math.pow(self.tag_trans[0] - box_x,2)+math.pow(self.tag_trans[1] - box_y,2))
					if circle <= 2 :
						print("-!-!--!-!---!--!---!--!--!-!--!-!--\nID: %d\nCamera:\t%.2f\t%.2f\nLidar:\t%.2f\t%.2f\nCircle: %.2f\t%.2f"%(T_msg.markers[i].id,self.tag_trans[0],self.tag_trans[1],box_x,box_y,circle,box_euler[2]))
						if not T_msg.markers[i].id in self.readedmarkers:
							self.readedmarkers.append(T_msg.markers[i].id)
						self.readedmarkers_tf.append(TagClass(T_msg.markers[i].id,self.tag_trans[0],self.tag_trans[1],box_x,box_y,box_euler[2],circle))

	def Rotation(self,translation,orientation):
		self.rotation_matrix =  np.array([1,0,0,0,0,-1,0,1,0]).reshape(3,3) #-90 degree about the x axis
		self.converted_trans = np.dot(self.rotation_matrix,np.asarray(translation).reshape(3,1))
		self.converted_euler = np.dot(self.rotation_matrix,np.asarray(tf.transformations.euler_from_quaternion(orientation)).reshape(3,1))
		self.converted_quaternion = tf.transformations.quaternion_from_euler(self.converted_euler[0], self.converted_euler[1],self.converted_euler[2])
		return self.converted_trans, self.converted_quaternion

	def getHuskyPose(self):	
		try:
			rospy.sleep(1.0)
			(trans,rot) = self.tf.lookupTransform('/map', '/base_footprint', rospy.Time(0))
			#print("%s\tHusky\t%s"%("------------","---------------"))
			homo_matrix = tf.transformations.quaternion_matrix(rot)
			homo_matrix[0][3] = trans[0]
			homo_matrix[1][3] = trans[1]
			homo_matrix[2][3] = trans[2]
			#print(homo_matrix)
			yaw = tf.transformations.euler_from_quaternion(rot)
			#print("%s\tHusky\t%s"%("------------","---------------"))
			return (trans,rot,homo_matrix,yaw)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print("Error Occured! - Detection of Husky Pose wrt map frame")		

	def getTagFrame(self,tagFrame):	
		try:
			(trans,rot) = self.tf.lookupTransform('/base_footprint', tagFrame, rospy.Time(0))
			new_trans, new_rot = self.Rotation(trans,rot)
			new_euler = tf.transformations.euler_from_quaternion(new_rot)
			new_rot = tf.transformations.quaternion_from_euler(0, 0, 0) #new_euler[2]
			homo_matrix = tf.transformations.quaternion_matrix(new_rot)
			homo_matrix[0][3] = new_trans[0]
			homo_matrix[1][3] = new_trans[1]
			homo_matrix[2][3] = new_trans[2]
			# print("%s\tTag\t%s"%("------------","---------------"))
			# print(homo_matrix)
			# print("%s\tTag\t%s"%("------------","---------------"))
			return (new_trans,new_rot,homo_matrix)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print("Error Occured! - Detection of Tag Pose wrt base_footprint frame")	
		
	def getTagFrame_wrt_map(self,tagframe):
		result = self.getHuskyPose()[2] #np.dot(self.getHuskyPose()[2],self.getTagFrame(tagframe)[2])	
		tag_offset = self.getTagFrame(tagframe)[0]
		result[0][3] += tag_offset[0]
		result[1][3] += tag_offset[1]
		result[2][3] += tag_offset[2]
		# print("%s\tMapper\t%s"%("------------","---------------"))
		# print(result)
		# print("%s\tMapper\t%s"%("------------","---------------"))
		return result

def movebase_client(target_goal):
	# Create an action client called "move_base" with action definition file "MoveBaseAction"
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()
	
	client.send_goal(target_goal)
	print("Goal sended")
	wait = client.wait_for_result()

	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	else:
		return client.get_result()

def vision(status):
	if status:
		launch_datmo.start()
		rospy.loginfo("Datmo is online")
		launch_ar.start()
		rospy.loginfo("AR Track Alvar is online")
	else:
		launch_datmo.shutdown()
		rospy.loginfo("Datmo is online")
		launch_ar.shutdown()
		rospy.loginfo("AR Track Alvar is online")

uuid_datmo = roslaunch.rlutil.get_or_generate_uuid(None,False)
uuid_ar = roslaunch.rlutil.get_or_generate_uuid(None,False)
roslaunch.configure_logging(uuid_datmo)
roslaunch.configure_logging(uuid_ar)
launch_datmo = roslaunch.parent.ROSLaunchParent(uuid_datmo,["/home/grid/husky_ws/src/datmo/launch/datmo.launch"])
launch_ar = roslaunch.parent.ROSLaunchParent(uuid_ar,["/home/grid/husky_ws/src/ar_track_alvar/ar_track_alvar/launch/husky_track.launch"])

if __name__ == '__main__':
	np.set_printoptions(precision=3,suppress=True)
	rospy.init_node("fusion")
	r = rospy.Rate(5)
	matcher = TagMatcher()
	choice = raw_input("Press any key to continue ...")
	matcher.move_robot_arm(0,1,0.5,2)
	matcher.calibrate_dance()
	matcher.explore_n_catch()
	choice2 = raw_input("Press any key to continue ...")
	matcher.move_robot_arm(0,0,0,0)