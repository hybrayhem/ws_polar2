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

AXIS_1_RATIO  = 63
AXIS_2_RATIO  = 30
AXIS_3_RATIO  = 30




#######

#######
JOINT1_CONST = ENC_RES * AXIS_1_RATIO / 360.0

JOINT2_CONST = ENC_RES * AXIS_2_RATIO / 360.0

JOINT3_CONST = ENC_RES * AXIS_3_RATIO / 360.0

JOINT4_CONST = 1153600/72.0

JOINT5_CONST = ENC_RES/90.0
JOINT6_CONST = ENC_RES/90.0
#######

def callback(data , publisher):
    '''
    convert the position and the velocity to encoder counts 
    '''
    # joint 1 ######
    '''
    joint1 = data.position[0]*180/math.pi  # convert rad to deg
    joint1_encoder_count = joint1 * JOINT1_CONST
    '''
    joint1 = data.position[0]*180/math.pi  # convert rad to deg
    joint1_encoder_count = joint1 * JOINT1_CONST
    
    ################

    # joint 2 ######
    '''
    joint2 = data.position[1]*180/math.pi  # convert rad to deg
    joint2_encoder_count = joint2 * JOINT2_CONST
    '''
    joint2 = data.position[1]*180/math.pi  # convert rad to deg
    joint2_encoder_count = joint2 * JOINT2_CONST

    ################

    # joint 3 ######
    joint3 = data.position[2]*180/math.pi  # convert rad to deg
    joint3_encoder_count = joint3 * JOINT3_CONST
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

    joints.position = [joint1_encoder_count,joint2_encoder_count,joint3_encoder_count,0,joint5_encoder_count,joint6_encoder_count]
    
    publisher.publish(joints)
    print(joints.position)

    '''
    print("joint 1 encoder count = ",joint1_encoder_count)
    print("joint 2 encoder count = ",joint2_encoder_count)
    print("joint 3 encoder count = ",joint3_encoder_count)
    print("joint 4 encoder count = ",joint4_encoder_count)
    print("joint 5 encoder count = ",joint5_encoder_count)
    print("joint 6 encoder count = ",joint6_encoder_count)
    '''

    ###################################3


def converter():
    PID = os.getpid()
    file_object = open('log.txt', 'a',1)
    
    
    msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe converter has started !!\n')
    
    file_object.write(msg)
    print(os.getcwd())
    rospy.init_node('converter', anonymous=True)
    pub = rospy.Publisher('converted_joint_states', JointState, queue_size=1)
    rospy.Subscriber("joint_states", JointState, callback, pub)
    rospy.spin()
    
    msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe converter has stoped !!\n')
    
    file_object.write(msg)
    file_object.close()

    
if __name__ == '__main__':
    PID = os.getpid()
    file_object = open('log.txt', 'a',1)
    
    
    msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe converter has started !!\n')
    
    file_object.write(msg)
    print(os.getcwd())
    rospy.init_node('converter', anonymous=True)
    pub = rospy.Publisher('converted_joint_states', JointState, queue_size=1)
    rospy.Subscriber("joint_states", JointState, callback , pub)
    rospy.spin()
    
    msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe converter has stoped !!\n')
    
    file_object.write(msg)
    file_object.close()
