#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import Twist, Point, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from math import pow,sqrt



class TagClass:
	def __init__(self,ID,x,y,z,r,p,yaw,w):
		self.ID = ID
		self.x = x
		self.y = y
		self.z = z
		self.r = r
		self.p = p
		self.yaw = yaw
		self.w = w
		self.item_list = [self.r, self.p, self.yaw, self.w]
		
		


def getBox(msg):

	global tag
	global item
	global x
	global marker_list
	marker_list = []
	readed_markers = []

	if len(msg.markers) >= 1:
		# tag = msg.markers[0].pose.pose.position
		for i in range(0,len(msg.markers)):
			print("%d - %.2f - %.2f - %.2f // %.2f - %.2f - %.2f - %.2f"%(msg.markers[i].id,msg.markers[i].pose.pose.position.x,msg.markers[i].pose.pose.position.y,msg.markers[i].pose.pose.position.z,msg.markers[i].pose.pose.orientation.x,msg.markers[i].pose.pose.orientation.y,msg.markers[i].pose.pose.orientation.z,msg.markers[i].pose.pose.orientation.w))
			#if msg.markers[i].id not in readed_markers:
				# print("%d bir tene var abey"%msg.markers[i].id)
			readed_markers.append(msg.markers[i].id)
			marker_list.append(TagClass(msg.markers[i].id,msg.markers[i].pose.pose.position.x,msg.markers[i].pose.pose.position.y,msg.markers[i].pose.pose.position.z,msg.markers[i].pose.pose.orientation.x,msg.markers[i].pose.pose.orientation.y,msg.markers[i].pose.pose.orientation.z,msg.markers[i].pose.pose.orientation.w))
#			else:
#				index_ = readed_markers.index(msg.markers[i].id)
#				if marker_list[index_].ID == msg.markers[i].id:
#					# print("%d iki tene var abey"%msg.markers[i].id)
#					# print("%.2f + %.2f "%(marker_list[index_].x, msg.markers[i].pose.pose.position.x))
#					marker_list[index_].x = (marker_list[index_].x + msg.markers[i].pose.pose.position.x)/2
#					marker_list[index_].y = (marker_list[index_].y + msg.markers[i].pose.pose.position.y)/2
#					marker_list[index_].z = (marker_list[index_].z + msg.markers[i].pose.pose.position.z)/2
#					marker_list[index_].r = (marker_list[index_].r + msg.markers[i].pose.pose.orientation.x)/2
#					marker_list[index_].p = (marker_list[index_].p + msg.markers[i].pose.pose.orientation.y)/2
#					marker_list[index_].yaw = (marker_list[index_].yaw + msg.markers[i].pose.pose.orientation.z)/2
#					marker_list[index_].z = (marker_list[index_].w + msg.markers[i].pose.pose.orientation.w)/2
#					#print("Son durum %.2f "%(marker_list[index_].x))
#			
			if 1 in readed_markers:
				target_index = readed_markers.index(1)
				tag.x = marker_list[target_index].x
				tag.y = marker_list[target_index].y
				tag.z = marker_list[target_index].z
				x =  (roll, pitch, yaw) = euler_from_quaternion (marker_list[target_index].item_list)
			else: 
				tag = Point()
	else:
		tag.x = 0
		tag.y = 0
		tag.z = 0.5
	#for every_marker in marker_list:
	#	print("%d + %.2f "%(every_marker.ID, every_marker.x))



if __name__ == '__main__':
	rospy.init_node ("tag_follower")
	marker = rospy.Subscriber("/ar_pose_marker",AlvarMarkers, getBox)
	move = rospy.Publisher("/husky_velocity_controller/cmd_vel",Twist, queue_size = 10 )
	rate = rospy.Rate(100)

	marker_list = []
	x = (0,0,0)
	y = [0,0,0]
	roll = pitch = yaw = 0
	Euc_Dis = 0.845

	speed = Twist()
	break_ = Twist()
	tag = Point()
	item = Quaternion()

	while not rospy.is_shutdown():
		#print("------------")
		
		#print(x)
		y = x[0]
		speed.linear.x = 0.04*(sqrt(pow((tag.x),2)+pow((tag.z),2)))
		speed.angular.z = 0.1*tag.x*tag.z+0.4*y
		break_.linear.x = 0.0
		break_.angular.z = 0.0

		if tag.x >= 0.6:
			move.publish(speed)
		else:
			print("Target is spotted")
			move.publish(break_)

		rate.sleep()




