#!/usr/bin/env python
# -*- coding: utf-8 -*-

import imp
import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Joy
import math
from joystick_class import Joystick

max_speed = 0.2 #(m/s)  #默认最大运行速度
singlePressJudge = 1  #用于跳变判断
MAX_MAX_SPEED_SET = 1.00  #运行最大速度的最大限度
MIN_MAX_SPEED_SET = 0.10  #运行最大速度的最小限度

def update_APE_msg(joyclass):
    global APEVeloMsg
    global APEPumpMsg
    global APEChargeMsg
    global APEHinsonWorkModeMsg

    global max_speed
    global singlePressJudge
    if(joyclass.start): #按开始按键开始进行操控
        joyclass.sysstart = 1 if joyclass.sysstart == 0 else 0
        APE_msg_Init()
        joyclass.RightTrigger = 0

    if(joyclass.sysstart == 1):

        #左遥感，控制运动
        if(joyclass.LeftStick.len >=0.1):
            APEVeloMsg.linear.x = max_speed * joyclass.LeftStick.len
            APEVeloMsg.angular.z = joyclass.LeftStick.angle

        elif(joyclass.A):
            APEPumpMsg.data = 1
        elif(joyclass.B):    
            APEPumpMsg.data = 2
        elif(joyclass.X):
            APEChargeMsg.data = 1
        elif(joyclass.up and max_speed < MAX_MAX_SPEED_SET-0.05 and singlePressJudge):
            max_speed +=0.1
            singlePressJudge = 0
        elif(joyclass.down and max_speed > MIN_MAX_SPEED_SET+0.05 and singlePressJudge):
            max_speed -=0.1
            singlePressJudge = 0

        elif(not joyclass.up and not joyclass.down):
            APE_msg_Init()

def APE_msg_Init():
    global APEVeloMsg
    global APEPumpMsg
    global APEChargeMsg
    global APEHinsonWorkModeMsg
    global singlePressJudge
    
    APEVeloMsg.linear.x = 0
    APEVeloMsg.angular.z = 0

    WorkMode = [0,0]  #hinson的工作模式  第一个是hinson1，第二个是hinson2
    APEHinsonWorkModeMsg = UInt8MultiArray(data = WorkMode)

    APEPumpMsg.data = 0
    APEChargeMsg.data = 0
    
    singlePressJudge = 1

def Publish_topic():

    global APEVeloMsg
    global APEPumpMsg
    global APEChargeMsg
    global APEHinsonWorkModeMsg

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        APEVeloPub.publish(APEVeloMsg)
        APEPumpPub.publish(APEPumpMsg)
        APEChargePub.publish(APEChargeMsg)
        APEHinsonWorkModePub.publish(APEHinsonWorkModeMsg)
    
        rate.sleep()


def JoystickCallback(msg):
    global Xbox_joy
    Xbox_joy.update_action_msg(msg)
    Xbox_joy.print_Joymsg()
    update_APE_msg(Xbox_joy)



if __name__=="__main__":

    rospy.init_node('APE_JoystickControl')
    APEVeloPub = rospy.Publisher('/APE_Velo', Twist, queue_size=10)
    APEVeloMsg = Twist()
    APEPumpPub = rospy.Publisher('/APE_Pump', UInt8, queue_size=10)
    APEPumpMsg = UInt8()
    APEChargePub = rospy.Publisher('/APE_Charge', UInt8, queue_size=10)
    APEChargeMsg = UInt8()
    APEHinsonWorkModePub = rospy.Publisher('/APE_HinsonWorkMode', UInt8MultiArray, queue_size=10)
    APEHinsonWorkModeMsg = UInt8MultiArray()

    APE_msg_Init()

    Xbox_joy = Joystick()
    sub  = rospy.Subscriber("/joy", Joy, JoystickCallback)
    Publish_topic()

