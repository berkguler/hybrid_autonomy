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
	def __init__(self,ID,x_tag,y_tag,x_box,y_box,circle,yaw):
		self.ID = ID
		self.x_tag = x_tag
		self.y_tag = y_tag
		self.x_box = x_box
		self.y_box = y_box
		self.yaw = yaw
		self.circle = circle
	def display(self):
		text = ("%s %d %s\nTag:\t%.2f\t%.2f\nBox:\t%.2f\t%.2f\nCircle:\t%.2f\tYaw:\t%.2f"%("--------------------",self.ID,"--------------------",self.x_tag,self.y_tag,self.x_box,self.y_box,self.circle,self.yaw))
		print(text)
class TagMatcher:

	def __init__ (self):
		self.tf = TransformListener()
		self.move = rospy.Publisher("/husky_velocity_controller/cmd_vel",Twist, queue_size = 10 )
		self.base = rospy.Publisher("/base_joint_position_controller/command",Float64,queue_size=10)
		self.shoulder = rospy.Publisher("/shoulder_joint_position_controller/command",Float64,queue_size=10)
		self.elbow = rospy.Publisher("/elbow_joint_position_controller/command",Float64,queue_size=10)
		self.wrist = rospy.Publisher("/wri_joint_position_controller/command",Float64,queue_size=10)
		self.readedmarkers_tf = []
		self.selectedmarkers = []
		self.is_mf_blocked = False
		rospy.loginfo("TagMatcher Initalized!!!!!!")	

	def debug(self):
		self.readedmarkers_tf.append(TagClass(0,4.62,-2.11,4.69,-1.95,0.18,-0.02))
		self.readedmarkers_tf.append(TagClass(1,4.94,2.21,4.94,2.33,0.11,0.00))
		self.readedmarkers_tf.append(TagClass(2,-2.36,-2.39,-2.42,-2.45,0.10,-0.06))
		self.readedmarkers_tf.append(TagClass(3,-2.07,3.25,-2.10,3.04,0.21,1.53))


	def move_robot_arm(self,base_angle,elbow_angle,shoulder_angle,wrist_angle):
		self.base.publish(base_angle)
		self.shoulder.publish(shoulder_angle)
		self.elbow.publish(elbow_angle)
		self.wrist.publish(wrist_angle)

	def explore_n_catch(self):
		rospy.sleep(0.5)
		vision(True)
		self.walk()

	def box_hunter(self,source,target):
		vision(True,True)
		rospy.loginfo(str(source) + " -> " + str(target))
		self.approach(source)

		# pick
		self.approach(target)
		
		#place
		vision(False,True)
		self.random_walk((0,0,0))


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
		waypoints = [(-0.50,-1.65,0),(3,3.6,3.1399),(1,-1.86,3.13),(1.45,3,-0.05),(0,0,0)]
		front = 0
		self.is_mf_blocked = False
		for point in waypoints:
			self.random_walk(point)
			if point != waypoints[len(waypoints)-1]:
				distance = self.laser_classification()
				front = min(distance['front'],min(distance['fleft'],distance['fright']))-0.2
				self.moveforward(front)
				self.moveforward(-front)
				self.Fusion()
				yaw = self.getHuskyPose()[3][2]
				self.rotateccw(yaw+0.3)
				laser = self.laser_classification()
				front = min(min(laser['fleft'],laser['fright']),laser['front']) -0.2
				self.moveforward(front)
				self.moveforward(-front)
				self.Fusion()
				self.rotateccw(yaw-0.3)
				laser = self.laser_classification()
				front = min(min(laser['fleft'],laser['fright']),laser['front'])-0.2
				self.moveforward(front)
				self.moveforward(-front)
				self.Fusion()
			else:
				self.moveforward(4)
				self.moveforward(-4)

	def calibrate_dance(self):
		rospy.loginfo("AMCL calibration")
		self.moveforward(2)
		self.moveforward(-2)
		
	def moveforward(self,distance,rotation=0.0):
		rospy.sleep(0.5)
		speed = Twist()
		laser_msg = rospy.wait_for_message("/scan",LaserScan)
		initial_range = laser_msg.ranges[360]
		last_range = initial_range
		replacement = 0
		if (distance > 0 and initial_range > distance) or distance < 0: 
			print "Target: " + str(distance)
			while round(((laser_msg.ranges[360] - initial_range)+distance),1) != 0:
				#print("%.2f ---> %.2f"%(-1*distance,(laser_msg.ranges[360] - initial_range)))
				laser_msg = rospy.wait_for_message("/scan",LaserScan)
				linear_speed = (distance+(laser_msg.ranges[360] - initial_range))
				if abs(last_range - laser_msg.ranges[360]) >= 0.2:	
					rospy.loginfo("Blocked 0.2 linearization error old:" + str(last_range) + " new:" + str(laser_msg.ranges[360]))
					replacement = -initial_range + last_range
					self.moveforward(replacement)
					rospy.loginfo("REVERSE")
					self.is_mf_blocked = True
					break
				if distance < 0 and self.is_mf_blocked:
					rospy.loginfo("It was Blocked")
					self.is_mf_blocked = False
					break 
				if min(laser_msg.ranges[260:460]) < 0.1:
					rospy.loginfo("Blocked 260-460  <0.1 ")
					replacement = -initial_range + last_range
					self.moveforward(replacement)
					rospy.loginfo("REVERSE")
					self.is_mf_blocked = True
					break
				else:
					self.is_mf_blocked = False
				
				if distance > 0 and linear_speed < 0:
					rospy.loginfo("Blocked inverse speed v:" + str(linear_speed) + " d:" + str(distance))
					print("Initial: %.2f Front: %.2f LR: %.2f distance: %.2f speed: %.2f"%(initial_range,laser_msg.ranges[360],min(laser_msg.ranges[260:460]),distance,linear_speed))
					linear_speed = 0
					self.is_mf_blocked = True
					
					break
				elif distance < 0 and linear_speed > 0:
					rospy.loginfo("Blocked inverse speed v:" + str(linear_speed) + " d:" + str(distance))
					print("Initial: %.2f Front: %.2f LR: %.2f distance: %.2f speed: %.2f"%(initial_range,laser_msg.ranges[360],min(laser_msg.ranges[260:460]),distance,linear_speed))
					linear_speed = 0
					self.is_mf_blocked = True
					break
				print("Initial: %.2f Front: %.2f LR: %.2f distance: %.2f speed: %.2f"%(initial_range,laser_msg.ranges[360],min(laser_msg.ranges[260:460]),distance,linear_speed))
				last_range = laser_msg.ranges[360]
				speed.linear.x = linear_speed
				speed.angular.z = rotation
				self.move.publish(speed)
				#print(speed)
			
	def rotateccw(self,target_heading):
		speed = Twist()
		heading = self.getHuskyPose()[3][2]
		direction = "ccw"
		if target_heading > 3.14:
			target_heading = -3.14 + (target_heading-3.14) 
			direction = "ccw"
		elif target_heading < -3.14:
			target_heading = 3.14 + (target_heading + 3.14)
			direction = "cw"
		elif (target_heading - heading) >= 0:
			direction = "ccw"
		elif (target_heading - heading) < 0:
			direction = "cw"
		print("%.2f\t%.2f\t%s"%(heading,target_heading,direction))
		while not (round(target_heading-heading,1) <= 0.15 and round(target_heading-heading,1) >= -0.15): 
			#print("%.2f ---> %.2f"%(heading,target_heading))
			heading = self.getHuskyPose()[3][2]
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

	def approach(self,_ID):
		rospy.loginfo("Destination: " + str(_ID))
		""" Move_Base"""
		print("Move Base")
		index = self.readedmarkers.index(_ID)
		tag = self.selectedmarkers[index]
		x = 0.0
		y = 0.0
		yaw = 0.0
		print("Before: x = %.2f y = %.2f yaw = %.2f "%(tag.x_box,tag.y_box,tag.yaw))
		if tag.x_box > 0:
			x = tag.x_box - 2.42
			y = tag.y_box 
			yaw = tag.yaw
			if yaw >= 1 and yaw <= 2:
				yaw = yaw-1.57
			elif yaw <= -1 and yaw >= -2:
				yaw = yaw + 1.57
			elif yaw >= 2 and yaw <= 3.15:
				yaw = yaw - 3.14
			elif yaw <= -2 and yaw >= -3.15:
				yaw = yaw + 3.14
		else:
			x = tag.x_box + 2.42
			y = tag.y_box 
			yaw = tag.yaw
			if yaw >= 1 and yaw <= 2:
				yaw = yaw+1.57
			elif yaw <= -1 and yaw >= -2:
				yaw = yaw - 1.57
			elif yaw >= 2 and yaw <= 3.15:
				yaw = yaw + 3.14
			elif yaw <= -2 and yaw >= -3.15:
				yaw = yaw - 3.14
		print("After: x = %.2f y = %.2f yaw = %.2f "%(x,y,yaw))
		self.random_walk((x,y,yaw))
		"""" Traditional """
		print("Traditional")
		rospy.sleep(0.5)
		r = rospy.Rate(5)
		speed = Twist()
		L_msg, T_msg = self.getMessages()
		if len(T_msg.markers) >= 1:
			for i in range(0,len(T_msg.markers)):
				if T_msg.markers[i].id == _ID: 
					while not len(T_msg.markers)  < 1:
						T_msg = self.getMessages()[1]
						#box_x = L_msg.tracks[k].odom.pose.pose.position.x
						if len(T_msg.markers)  < 1:
							print "Out of the vision"
							break
						#Can see the tag
						old_quaternion = (
									T_msg.markers[i].pose.pose.orientation.x,
									T_msg.markers[i].pose.pose.orientation.y,
									T_msg.markers[i].pose.pose.orientation.z,
									T_msg.markers[i].pose.pose.orientation.w )
						old_offset = (
							T_msg.markers[i].pose.pose.position.x,
							T_msg.markers[i].pose.pose.position.y,
							T_msg.markers[i].pose.pose.position.z )
						new_translation, new_quaternion =  self.Rotation(old_offset,old_quaternion)

						#(roll, pitch, yaw_tag) = euler_from_quaternion(new_quaternion)
						heading = math.atan2(new_translation[1],new_translation[0])	
						print("Ben headingim : %.4f"%heading)
						if abs(heading) > 0.01:
							speed.angular.z = heading + new_translation[1]
							speed.linear.x = new_translation[0]/15
						else:
							speed.linear.x = new_translation[0]/10
							speed.angular.z = 0
						print speed
						self.move.publish(speed)
						r.sleep()
					rospy.loginfo("Manual Approaching ...")
					#camera cannot see If it doesn't work we can continue with datmo
					laser_msg = rospy.wait_for_message("/scan",LaserScan)
					front = min(laser_msg.ranges[350:370])
					while front > 0.105:
						print front
						laser_msg = rospy.wait_for_message("/scan",LaserScan)
						front = min(laser_msg.ranges[350:370])
						speed.linear.x = front/5
						speed.angular.z = 0
						#print speed
						self.move.publish(speed)
						r.sleep()
					rospy.loginfo("!!! Done !!!")
					rospy.sleep(1.0)
					matcher.move_robot_arm(0,-1.2,0.5,2)
					rospy.sleep(0.5)
					matcher.move_robot_arm(0.1,-1.2,-0.5,2)
					rospy.sleep(0.5)
					matcher.move_robot_arm(-0.1,-1.2,0.5,2)
					rospy.sleep(0.5)
					matcher.move_robot_arm(0.1,-1.2,-0.5,2)
					rospy.sleep(0.5)
					matcher.move_robot_arm(-0.1,-1.2,0.5,2)
					rospy.sleep(5.0)
					matcher.move_robot_arm(0,1,0.5,2)
					self.moveforward(-2)
		


	def Fusion(self):
		rospy.loginfo("Fusion!!!")
		L_msg, T_msg = self.getMessages()
		if len(T_msg.markers) >= 1:
			for i in range(0,len(T_msg.markers)):
				near_box = (0,0,999,0) #initial box config _x _y _circle _yaw
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
						L_msg.tracks[k].odom.pose.pose.orientation.w )
					box_euler = tf.transformations.euler_from_quaternion(box_orientation)
					circle = math.sqrt(math.pow(self.tag_trans[0] - box_x,2)+math.pow(self.tag_trans[1] - box_y,2))
					if near_box[2] > circle:
						near_box = (box_x,box_y,circle,box_euler[2])
					# if circle < 1:
					# 	text = ("%s %d %s\nTag:\t%.2f\t%.2f\nBox:\t%.2f\t%.2f\nCircle:\t%.2f\tYaw:\t%.2f"%("--------------------",T_msg.markers[i].id,"--------------------",self.tag_trans[0],self.tag_trans[1],box_x,box_y,circle,box_euler[2]))
					# 	print(text)
					rospy.loginfo(str(T_msg.markers[i].id) + " is readed ...")
				self.readedmarkers_tf.append(TagClass(T_msg.markers[i].id,self.tag_trans[0],self.tag_trans[1],near_box[0],near_box[1],near_box[2],near_box[3]))

	def select_best_pose(self):
		self.readedmarkers = []
		for tag in self.readedmarkers_tf:
			if tag.ID not in self.readedmarkers:
				self.readedmarkers.append(tag.ID)
				self.selectedmarkers.append(tag)
			else:
				index = self.readedmarkers.index(tag.ID)
				old_tag = self.selectedmarkers[index]
				if tag.circle < old_tag.circle:
					self.selectedmarkers[index] = tag
		return self.selectedmarkers
			
	def Rotation(self,translation,orientation):
		self.rotation_matrix =  np.array([1,0,0,0,0,1,0,-1,0]).reshape(3,3) #-90 degree about the x axis
		self.converted_trans = np.dot(self.rotation_matrix,np.asarray(translation).reshape(3,1))
		self.converted_euler = np.dot(self.rotation_matrix,np.asarray(tf.transformations.euler_from_quaternion(orientation)).reshape(3,1))
		self.converted_quaternion = tf.transformations.quaternion_from_euler(self.converted_euler[0], self.converted_euler[1],self.converted_euler[2])
		return self.converted_trans, self.converted_quaternion

	def getHuskyPose(self):	
		try:
			rospy.sleep(1.0)
			(trans,rot) = self.tf.lookupTransform('/map', '/base_footprint', rospy.Time(0))
			# print("%s\tHusky\t%s"%("------------","---------------"))
			homo_matrix = tf.transformations.quaternion_matrix(rot)
			homo_matrix[0][3] = trans[0]
			homo_matrix[1][3] = trans[1]
			homo_matrix[2][3] = trans[2]
			# print(homo_matrix)
			yaw = tf.transformations.euler_from_quaternion(rot)
			# print("%s\tHusky\t%s"%("------------","---------------"))
			return (trans,rot,homo_matrix,yaw)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print("Error Occured! - Detection of Husky Pose wrt map frame")		

	def getTagFrame(self,tagFrame):	
		try:
			(trans,rot) = self.tf.lookupTransform('/base_footprint', tagFrame, rospy.Time(0))
			new_trans, new_rot = self.Rotation(trans,rot)
			new_euler = tf.transformations.euler_from_quaternion(new_rot)
			new_rot = tf.transformations.quaternion_from_euler(0, 0, new_euler[2])
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
		result = np.dot(self.getHuskyPose()[2],self.getTagFrame(tagframe)[2]) #self.getHuskyPose()[2]	
		# tag_offset = self.getTagFrame(tagframe)[0]
		# result[0][3] += tag_offset[0]
		# result[1][3] += tag_offset[1]
		# result[2][3] += tag_offset[2]
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

def vision(status,from_basefootprint=False):
	if not from_basefootprint:
		if status:
			launch_datmo.start()
			rospy.loginfo("Datmo is online")
			launch_ar.start()
			rospy.loginfo("AR Track Alvar is online")
		else:
			launch_datmo.shutdown()
			rospy.loginfo("Datmo is offline")
			launch_ar.shutdown()
			rospy.loginfo("AR Track Alvar is offline")
	else:
		if status:
			launch_datmo_bf.start()
			rospy.loginfo("Datmo from base_footprint is online")
			launch_ar.start()
			rospy.loginfo("AR Track Alvar is online")			
		else:
			launch_datmo_bf.shutdown()
			rospy.loginfo("Datmo from base_footprint is offline")
			launch_ar.shutdown()
			rospy.loginfo("AR Track Alvar is offline")			
	

uuid_datmo = roslaunch.rlutil.get_or_generate_uuid(None,False)
uuid_datmo_bf = roslaunch.rlutil.get_or_generate_uuid(None,False)
uuid_ar = roslaunch.rlutil.get_or_generate_uuid(None,False)
roslaunch.configure_logging(uuid_datmo)
roslaunch.configure_logging(uuid_datmo_bf)
roslaunch.configure_logging(uuid_ar)
launch_datmo = roslaunch.parent.ROSLaunchParent(uuid_datmo,["/home/grid/husky_ws/src/datmo/launch/datmo.launch"])
launch_datmo_bf = roslaunch.parent.ROSLaunchParent(uuid_datmo_bf,["/home/grid/husky_ws/src/datmo/launch/datmo_bf.launch"])
launch_ar = roslaunch.parent.ROSLaunchParent(uuid_ar,["/home/grid/husky_ws/src/ar_track_alvar/ar_track_alvar/launch/husky_track.launch"])

if __name__ == '__main__':
	np.set_printoptions(precision=3,suppress=True)
	rospy.init_node("fusion")
	r = rospy.Rate(5)
	matcher = TagMatcher()
	choice = raw_input("Press e to recognize the tags, any key to debug ...")
	matcher.move_robot_arm(0,1,0.5,2)
	matcher.calibrate_dance()
	if choice == "e" or choice == "E":	
		matcher.explore_n_catch()
	else:
		matcher.debug()
	for tag_message in matcher.readedmarkers_tf:
		tag_message.display()	
	matcher.select_best_pose()
	rospy.loginfo("Readed tags were selected wrt circle value")
	for tag_message in matcher.selectedmarkers:
		tag_message.display()	
	print("Available markers: ")
	print matcher.readedmarkers
	choice = raw_input("Press x to exit, any key to continue...")
	if choice != "x" or choice!= "X":
		source_tag = int(raw_input("Select the source box from list: "))
		target_tag = int(raw_input("Select the target box from list: "))
		if not source_tag in matcher.readedmarkers and not target_tag in matcher.readedmarkers:
			print("Wrong Selection")
		else:
			matcher.box_hunter(source_tag,target_tag)
	matcher.move_robot_arm(0,0,0,0)
	raw_input("any key to exit...")