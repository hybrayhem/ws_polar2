#!/usr/bin/env python3
import time
import rospy
import board
import busio
import sys
import os
import string
import datetime
from adafruit_servokit import ServoKit

i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
kit = ServoKit(channels=8, i2c=i2c_bus0)

def callback(data):
	pass

if __name__ == '__main__':
	PID = os.getpid()
	file_object = open('log.txt', 'a',1)
	
	msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe servo controller has started !! add = '+ '\n')
	file_object.write(msg)
	
	rospy.init_node('servoControler', anonymous=True)
	rospy.Subscriber("converted_joint_states", JointState, callback) #topic adini check et, converted_joint_states degisecek
	rospy.spin()
	
	msg = str('PID = '+str(PID)+"\t["+str(datetime.datetime.now())+']\tthe servo controller has stoped !! add = '+ '\n')
	file_object.write(msg)
	file_object.close()



""" 	
while True: 

	
	degree_from_halil = int(input('enter a value'))



	kit.servo[1].angle = 97-int(degree_from_halil) #sol servo derece kadar açık
	print("Sol servo derece kadar açık")
	kit.servo[0].angle = 68+int(degree_from_halil) #sag servo derece kadar açık
	print("Sag servo derece kadar açık")
	time.sleep(1.5) 
"""


'''

	kit.servo[1].angle = 15 #sol servo tam acik pozisyon
	print("Sol servo tam acik")
	kit.servo[0].angle = 150 #sag servo tam acik pozisyon
	print("Sag servo tam acik")
	time.sleep(1.5)
	
	kit.servo[1].angle = 97 #sol servo dik pozisyon (gripper marka gozuken taraftan sol)
	print("Sol servo dik")
	kit.servo[0].angle = 68 #sag servo dik pozisyon (gripper marka gozuken taraftan sag)
	print("Sag servo dik")
	time.sleep(1.5)
'''