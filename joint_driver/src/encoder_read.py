#!/usr/bin/env python
import sys
import os
import math
import string
import datetime
import rospy
import serial as ser
from nasa import Roboclaw
from sensor_msgs.msg import JointState

roboclaw = Roboclaw("/dev/ttyTHS0", 38400)

# baska topige yazilacak /roboclaw/feedback, /roboclaw/command burdan oku 

pub = 0
joints = JointState()

#######
ENC_RES = 1600
#######

#######
JOINT1_CONST = ENC_RES/83.0
JOINT2_CONST = ENC_RES/30.0

JOINT3_CONST = ENC_RES/30.0
JOINT4_CONST = 1153600/72.0

JOINT5_CONST = ENC_RES/90.0
JOINT6_CONST = ENC_RES/90.0
#######

def listener(lock , addr1 , addr2 , addr3):
#	rospy.init_node('encoder', anonymous=True)
	rate = rospy.Rate(50)
	while not rospy.is_shutdown(): 				#address check et

		lock.acquire()
		read_roboclaw1_encoder1 = roboclaw.ReadEncM1(addr1) #base 
		lock.release()

		lock.acquire()
		read_roboclaw1_encoder2 = roboclaw.ReadEncM2(addr1) #redline 1
		lock.release()

		lock.acquire()
		read_roboclaw2_encoder1 = roboclaw.ReadEncM1(addr2) #redline 2
		lock.release()

		lock.acquire()
		read_roboclaw2_encoder2 = roboclaw.ReadEncM2(addr2) #eski motor
		lock.release()
		
#		read_roboclaw3_encoder1 = roboclaw.ReadEncM1(131) #gripper sag
#		read_roboclaw3_encoder2 = roboclaw.ReadEncM2(131) #gripper sol

		print(read_roboclaw1_encoder1)
		print(read_roboclaw1_encoder2)
		print(read_roboclaw2_encoder1)
		print(read_roboclaw2_encoder2)
#		print(read_roboclaw3_encoder1)
#		print(read_roboclaw3_encoder2)
		print('---------------------------')

		joint1 = read_roboclaw1_encoder1[1]/180*math.pi/JOINT1_CONST
		joint2 = read_roboclaw1_encoder2[1]/180*math.pi/JOINT2_CONST
		joint3 = read_roboclaw2_encoder1[1]/180*math.pi/JOINT3_CONST
		joint4 = read_roboclaw2_encoder2[1]/180*math.pi/JOINT4_CONST
#		joint5 = read_roboclaw3_encoder1[1]/180*math.pi/JOINT5_CONST
#		joint6 = read_roboclaw3_encoder2[1]/180*math.pi/JOINT6_CONST

		joint5 = 0
		joint6 = 0


		joints.position = [joint1,joint2,joint3,joint4,joint5+joint6,joint5-joint6]
		rate.sleep()
		#pub.publish(joints)


def encoder_read(lock , addr1 , addr2 , addr3):
	PID = os.getpid()
	file_object = open('log.txt', 'a',1)

	msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe encoder reader has started !! add = '+ '\n')
	file_object.write(msg)

	if roboclaw.Open() == 0:
		msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\t port acilmadi  !! add = '+ '\n')
		sys.exit(1)
	
	rospy.init_node('encoderReader', anonymous=True)
	pub = rospy.Publisher('converted_joint_states', JointState, queue_size=100)
	listener(lock , addr1 , addr2 , addr3)



if __name__ == '__main__':

    PID = os.getpid()
    file_object = open('log.txt', 'a',1)

    msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe encoder reader has started !! add = '+ '\n')
    file_object.write(msg)

    if roboclaw.Open() == 0:
		msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\t port acilmadi  !! add = '+ '\n')
		sys.exit(1)
	
    rospy.init_node('encoderReader', anonymous=True)
    pub = rospy.Publisher('converted_joint_states', JointState, queue_size=100)


    #listener()