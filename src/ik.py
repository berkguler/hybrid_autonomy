#!/usr/bin/env python

import rospy
from math import atan2,cos,sin,sqrt,acos,radians,pi
from std_msgs.msg import Float64

def ik(x,y,z,parorperp):
    xc = x - 0.421
    yc = y
    zc = z
    l = [0.055,0.2,0.109,0.058]
    h0 = 0.404
    theta_1 = atan2(yc,xc)
    D = sqrt(xc*xc+yc*yc)
    H = zc
    try:
        if parorperp == 1:
            D_ = D - l[3]
            H_ = H - h0
            L = sqrt(D_*D_+H_*H_)
            Z = (-L*L+l[1]*l[1]+l[2]*l[2])/(2*l[1]*l[2])
            print D, D_ , H, H_, L,Z, (1-Z*Z)
            if abs(Z) > 1.0:
                print "Out_of_Z"
                Z = 1.0
                print Z
            theta_3 = atan2(-sqrt(1-Z*Z),Z)
            phi = atan2(H_,D_)
            theta_2 = -pi/2 + (acos((L*L+l[1]*l[1]-l[2]*l[2])/(2*L*l[1]))+phi)
            if theta_2 < -1.5708:
                theta_2 = -1.5708
            if theta_3 < -1.5708:
                theta_3 = -1.5708    
            theta_4 = -pi/2-(theta_2 + theta_3)
        else:
            D_ = D 
            H_ = H + l[3] - h0
            L = sqrt(D_*D_+H_*H_)
            Z = (-L*L+l[1]*l[1]+l[2]*l[2])/(2*l[1]*l[2])
            print D, D_ , H, H_, L,Z, (1-Z*Z)
            if abs(Z) > 1.0:
                print "Out_of_Z"
                Z = 1.0
                print Z
            theta_3 = atan2(-sqrt(1-Z*Z),Z)
            phi = atan2(H_,D_)
            theta_2 = -pi/2 + (acos((L*L+l[1]*l[1]-l[2]*l[2])/(2*L*l[1]))+phi)
            if theta_2 < -1.5708:
                theta_2 = -1.5708
            if theta_3 < -1.5708:
                theta_3 = -1.5708  
            theta_4 = -pi-(theta_2 + theta_3)
    except:
        print "Out of WS"
        theta_1 = 0
        theta_2 = 0
        theta_3 = 0
        theta_4 = 0
    theta= [theta_1,theta_2,theta_3,theta_4]
    rospy.loginfo(str(theta))

    while not rospy.is_shutdown():
        base.publish(theta_1)
        shoulder.publish(theta_2)
        elbow.publish(theta_3)
        wrist.publish(theta_4)
        rospy.sleep(1.0)
        break
    


base = rospy.Publisher("/base_joint_position_controller/command",Float64,queue_size=10)
shoulder = rospy.Publisher("/shoulder_joint_position_controller/command",Float64,queue_size=10)
elbow = rospy.Publisher("/elbow_joint_position_controller/command",Float64,queue_size=10)
wrist = rospy.Publisher("/wrist_joint_position_controller/command",Float64,queue_size=10)

if __name__ == "__main__":
    rospy.init_node('ik_solver', anonymous=True)
    waypoints = [(0.65,0.001,0.2,1),(0.65,0.001,0.25,0),(0.65,0.001,0.3,0),(0.65,0.001,0.35,0),(0.65,0.001,0.4,0),(0.65,0.001,0.45,0),(0.65,0.001,0.5,0),(0.65,0.001,0.55,0),(0.6,0.001,0.4,1),(0.65,0.001,0.4,1),(0.7,0.001,0.4,1),(0.75,0.001,0.4,1),(0,0,0,0)]
    for wp in waypoints:
        rospy.sleep(1.0)
        ik(wp[0],wp[1],wp[2],wp[3])
        rospy.sleep(1.0)