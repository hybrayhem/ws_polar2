#!/usr/bin/env python
import sys
import os
import string
import datetime
import rospy
from sensor_msgs.msg import JointState

from roboclaw_nasa import Roboclaw

add = 0 
index = 0

nasa = Roboclaw("/dev/ttyTHS0", 38400)

def print_usage():
    print('sdfsdfsdfsdf')

def callback(data):    
    #print('add = ',end="")
    #print(add ,data.position[index])
    print(data.position[index])
    print(nasa.SpeedAccelDeccelPositionM1M2(add,1000,5000,100,int(data.position[index]+data.position[index+1]),1000,5000,100,int(data.position[index]-data.position[index+1]),1))
    #print(nasa.driveM1M2(add,int(data.position[index]+data.position[index+1]),int(data.position[index]-data.position[index+1])))
    

if __name__ == '__main__':
    if len(sys.argv) == 3:
        add   = int(sys.argv[1])
        index = int(sys.argv[2])
    else:
        print_usage()
        sys.exit(1)
    
    if nasa.Open() == 0:
	sys.exit(1)

    PID = os.getpid()
    file_object = open('logFile.txt', 'a',1)

    msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe motor controller has started !! add = '+str(add)+ '\n')
    file_object.write(msg)
    
    rospy.init_node('motorControler', anonymous=True)
    rospy.Subscriber("converted_joint_states", JointState, callback)
    rospy.spin()
    
    msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe motor controller has stoped !! add = '+str(add)+ '\n')
    file_object.write(msg)
    file_object.close()
