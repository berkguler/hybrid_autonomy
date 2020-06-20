#!/usr/bin/env python
import rospy
import tf
import actionlib
import time

import math
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion  
from ar_track_alvar_msgs.msg import AlvarMarkers
from datmo.msg import TrackArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan

from tf import TransformListener


class TagMatcher:

	def __init__ (self):
		self.tf = TransformListener()
		self.move = rospy.Publisher("/husky_velocity_controller/cmd_vel",Twist, queue_size = 10 )
		self.base = rospy.Publisher("/base_joint_position_controller/command",Float64,queue_size=10)
		self.shoulder = rospy.Publisher("/shoulder_joint_position_controller/command",Float64,queue_size=10)
		self.elbow = rospy.Publisher("/elbow_joint_position_controller/command",Float64,queue_size=10)
		self.wrist = rospy.Publisher("/wri_joint_position_controller/command",Float64,queue_size=10)
		rospy.loginfo("TagMatcher Initalized!!!!!!")	

	def move_robot_arm(self,base_angle,elbow_angle,shoulder_angle,wrist_angle):
		self.base.publish(base_angle)
		self.shoulder.publish(shoulder_angle)
		self.elbow.publish(elbow_angle)
		self.wrist.publish(wrist_angle)


	def calibrate_dance(self):
		print("abracadabra")
		self.rotateccw(1.57)
		self.moveforward(2)
		self.rotateccw(3.14)
		self.moveforward(1)
		self.rotateccw(-1.57)
		self.moveforward(2)
		self.rotateccw(0)
		self.moveforward(1)
		print("ne sandin yarragim")
		
	def moveforward(self,distance):
		print("yuruuuu")
		speed = Twist()
		laser_msg = rospy.wait_for_message("/scan",LaserScan)
		initial_range = laser_msg.ranges[360]
		while round(((laser_msg.ranges[360] - initial_range)+distance),1)!= 0:
			print("%.2f ---> %.2f"%(-1*distance,(laser_msg.ranges[360] - initial_range)))
			laser_msg = rospy.wait_for_message("/scan",LaserScan)
			speed.linear.x = (distance+(laser_msg.ranges[360] - initial_range))
			speed.angular.z = 0.0
			self.move.publish(speed)
			print(speed)
			

	def rotateccw(self,target_heading):
		speed = Twist()
		heading = tf.transformations.euler_from_quaternion(self.getHuskyPose()[1])[2]
		while not (round(target_heading-heading,1) <= 0.2 and round(target_heading-heading,1) >= -0.2): 
			print("%.2f ---> %.2f"%(heading,target_heading))
			heading = tf.transformations.euler_from_quaternion(self.getHuskyPose()[1])[2]
			speed.linear.x = 0.0
			speed.angular.z = 0.5
			self.move.publish(speed)


	def getMessages(self):
		L_msg = rospy.wait_for_message("/datmo/box_kf",TrackArray,timeout=None)
		T_msg = rospy.wait_for_message("/ar_pose_marker",AlvarMarkers,timeout=None)
		return L_msg, T_msg

	def Fusion(self):
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
					# else:
					# 	print("%s\tOlmadi\t%s"%("o o o o o o ","o o o o o o ")) # -----------------------\nID: %d\nCamera:\t%.2f\t%.2f\nLidar:\t%.2f\t%.2f\nCircle: %.2f"%(T_msg.markers[i].id,self.tag_trans[0],self.tag_trans[1],box_x,box_y,circle))
					# 	print("%.2f\t%.2f\n%.2f\t%.2f"%(self.tag_trans[0],self.tag_trans[1],box_x,box_y))
					# 	print("%s\tOlmadi\t%s"%("o o o o o o ","o o o o o o "))


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
			print("%s\tHusky\t%s"%("------------","---------------"))
			homo_matrix = tf.transformations.quaternion_matrix(rot)
			homo_matrix[0][3] = trans[0]
			homo_matrix[1][3] = trans[1]
			homo_matrix[2][3] = trans[2]
			print(homo_matrix)
			print(tf.transformations.euler_from_quaternion(rot))
			print("%s\tHusky\t%s"%("------------","---------------"))
			return (trans,rot,homo_matrix)
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
		result = self.getHuskyPose(True)[2] #np.dot(self.getHuskyPose()[2],self.getTagFrame(tagframe)[2])	
		tag_offset = self.getTagFrame(tagframe)[0]
		result[0][3] += tag_offset[0]
		result[1][3] += tag_offset[1]
		result[2][3] += tag_offset[2]
		# print("%s\tMapper\t%s"%("------------","---------------"))
		# print(result)
		# print("%s\tMapper\t%s"%("------------","---------------"))

		return result

if __name__ == '__main__':
	np.set_printoptions(precision=3,suppress=True)
	rospy.init_node("fusion")
	r = rospy.Rate(5)
	matcher = TagMatcher()
	choice = int(raw_input("bok:"))
	matcher.move_robot_arm(0,1,0.5,2)
	matcher.calibrate_dance()
	matcher.move_robot_arm(0,0,0,0)
	while not rospy.is_shutdown():
	 	matcher.move_robot_arm(0,1,0.5,2)
		matcher.calibrate_dance()
		matcher.move_robot_arm(0,0,0,0)
	 	r.sleep()
 

