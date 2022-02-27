#!/usr/bin/env python
import rospy
import os
import datetime
import string
import math
from sensor_msgs.msg import JointState

joints = JointState()
file_object = 0 


#######
ENC_RES = 1600
#######

#######
JOINT1_CONST = 0
JOINT2_CONST = 0

JOINT3_CONST = 0
JOINT4_CONST = 1153600/72.0

JOINT5_CONST = ENC_RES/90.0
JOINT6_CONST = ENC_RES/90.0
#######

def callback(data):
    '''
    convert the position and the velocity to encoder counts 
    '''
    # joint 1 ######
    ## to do 
    ################

    # joint 2 ######
    ## to do 
    ################

    # joint 3 ######
    ## to do 
    ################

    # joint 4 ######
    joint4 = data.position[3]*180/math.pi  # convert rad to deg
    joint4_encoder_count = joint4 * JOINT4_CONST
		     
    ################

    # joint 5 ######
    joint5 = data.position[4]*180/math.pi  # convert rad to deg 
    joint5_encoder_count = round(joint5*JOINT5_CONST)
    ################
    
    # joint 6 ######
    joint6 = data.position[5]*180/math.pi  # convert rad to deg
    joint6_encoder_count = round(joint6*JOINT6_CONST)
    ################

    joints.position = [0,0,0,joint4_encoder_count,joint5_encoder_count,joint6_encoder_count]
    pub.publish(joints)
    print("joint 4 encoder count = ",joint4_encoder_count)
    print("joint 5 encoder count = ",joint5_encoder_count)
    print("joint 6 encoder count = ",joint6_encoder_count)
    ###################################3
    
    
if __name__ == '__main__':
    PID = os.getpid()
    file_object = open('logFile.txt', 'a',1)
    
    
    msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe converter has started !!\n')
    
    file_object.write(msg)
    print(os.getcwd())
    rospy.init_node('converter', anonymous=True)
    pub = rospy.Publisher('converted_joint_states', JointState, queue_size=100)
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.spin()
    
    msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe converter has stoped !!\n')
    
    file_object.write(msg)
    file_object.close()
