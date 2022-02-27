#!/bin/sh

rosrun joint_driver converter.py &
rosrun joint_driver motorController.py 130 0 &
rosrun joint_driver motorController.py 131 2 &
rosrun joint_driver motorController.py 132 4 &
