#!/usr/bin/env python

import multiprocessing
import sys
from converter import converter
from encoder_read import encoder_read
from motorController import motor_controller

if __name__ == '__main__':
	#rospy.init_node('runAll', anonymous=True)

	lock = multiprocessing.Lock()
	P_converter = multiprocessing.Process(target=converter)
	P_encoderReader = multiprocessing.Process(target=encoder_read , args=(lock,128,129,130))
	P_motor_controller1 =  multiprocessing.Process(target=motor_controller , args=(128,0,lock,1000,1000,5000,50,50,300))
	P_motor_controller2 =  multiprocessing.Process(target=motor_controller , args=(129,2,lock,50,50,300,1000,1000,5000))




	P_converter.start()
	P_encoderReader.start()
	P_motor_controller1.start()
	P_motor_controller2.start()

	P_converter.join()
	P_encoderReader.join()
	P_motor_controller1.join()
	P_motor_controller2.join()
	