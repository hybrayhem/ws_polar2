#!/usr/bin/env python
import rospy
import os
import datetime
import string
import math
from sensor_msgs.msg import JointState
from roboclaw_nasa import Roboclaw

joints = JointState()
nasa = Roboclaw("/dev/ttyTHS0", 38400)
#######
ENC_RES = 1600
#######

#######
JOINT1_CONST = ENC_RES/63.0
JOINT2_CONST = ENC_RES/30.0

JOINT3_CONST = ENC_RES/30.0
JOINT4_CONST = 1153600/72.0

JOINT5_CONST = ENC_RES/90.0
JOINT6_CONST = ENC_RES/90.0
#######






def callback(data):
    '''
    convert the position and the velocity to encoder counts 
    '''
    # joint 1 ######
    '''
    joint1 = data.position[0]*180/math.pi  # convert rad to deg
    joint1_encoder_count = joint1 * JOINT1_CONST
    '''
    joint1 = data.position[0]*180/math.pi  # convert rad to deg
    joint1_encoder_count = joint1 * 63 * 1600 / 360
    
    ################

    # joint 2 ######
    '''
    joint2 = data.position[1]*180/math.pi  # convert rad to deg
    joint2_encoder_count = joint2 * JOINT2_CONST
    '''
    joint2 = data.position[1]*180/math.pi  # convert rad to deg
    joint2_encoder_count = joint2 * 30 * 1600 / 360

    ################

    # joint 3 ######
    joint3 = data.position[2]*180/math.pi  # convert rad to deg
    joint3_encoder_count = joint3  * 30 * 1600 / 360
    ################

    # joint 4 ######
    joint4 = data.position[3]*180/math.pi  # convert rad to deg
    joint4_encoder_count = joint4 * 663320 /360
		     
    ################

    # joint 5 ######
    joint5 = data.position[4]*180/math.pi  # convert rad to deg 
    joint5_encoder_count = round(joint5*JOINT5_CONST)
    ################
    
    # joint 6 ######
    joint6 = data.position[5]*180/math.pi  # convert rad to deg
    joint6_encoder_count = round(joint6*JOINT6_CONST)
    ################

    joints.position = [joint1_encoder_count,joint2_encoder_count,joint3_encoder_count,joint4_encoder_count,joint5_encoder_count,joint6_encoder_count]
    #pub.publish(joints)
    #print(joints)

    '''
    print("joint 1 encoder count = ",joint1_encoder_count)
    print("joint 2 encoder count = ",joint2_encoder_count)
    print("joint 3 encoder count = ",joint3_encoder_count)
    print("joint 4 encoder count = ",joint4_encoder_count)
    print("joint 5 encoder count = ",joint5_encoder_count)
    print("joint 6 encoder count = ",joint6_encoder_count)
    '''

    ###################################3
    #print(nasa.SpeedAccelDeccelPositionM1M2(add,1000,5000,100,int(data.position[index]+data.position[index+1]),1000,5000,100,int(data.position[index]-data.position[index+1]),1))
    print(nasa.SpeedAccelDeccelPositionM1M2(128,1000,5000,300,int(joint1_encoder_count),50,300,50,int(joint2_encoder_count),1))
    print(nasa.SpeedAccelDeccelPositionM1M2(129,50,300,50,int(joint3_encoder_count),10000,20000,5000,int(joint4_encoder_count*-1),1))
    read_roboclaw1_encoder1 = nasa.ReadEncM1(128)
    read_roboclaw1_encoder2 = nasa.ReadEncM2(128)
    read_roboclaw2_encoder1 = nasa.ReadEncM1(129) #redline 2
    read_roboclaw2_encoder2 = nasa.ReadEncM2(129) #eski motor

    print(read_roboclaw1_encoder1)
    print(read_roboclaw1_encoder2)
    print(read_roboclaw2_encoder1)
    print(read_roboclaw2_encoder2)
#		print(read_roboclaw3_encoder1)
#		print(read_roboclaw3_encoder2)
    print('---------------------------')


if __name__ == '__main__':
    PID = os.getpid()
    #file_object = open('logFile.txt', 'a',1)
    
    
    #msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe converter has started !!\n')
    
    #file_object.write(msg)
    #print(os.getcwd())
    
    rospy.init_node('motor_driver', anonymous=True)
    
    #pub = rospy.Publisher('converted_joint_states', JointState, queue_size=1)
    if nasa.Open() == 0:
	    sys.exit(1)
	    print("roboclaw open !!!!!")
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.spin()
    
    #msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe converter has stoped !!\n')
    
    #file_object.write(msg)
    #file_object.close()