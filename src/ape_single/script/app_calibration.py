#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
import math

from geometry_msgs.msg import Pose2D
from ape_single.srv import CalibrateStatus

import sys
sys.path.append('/home/ape/APE_Application/src/ape_single/script/')

from configs.config_path import *
from utils import tool

import pymongo

## data handle
myclient = pymongo.MongoClient("mongodb://localhost:27017/")
# database
apeDB = myclient["ape_db"]
# collection
statusCollection = apeDB["ape_status_collection"]
configCollection = apeDB["ape_config_collection"]
setCollection = apeDB["ape_set_collection"]

class Calibration():
    def __init__(self,x,y,distance,repeat_time):
        """标定变量初始化

        Args:
            x (_type_): x坐标
            y (_type_): y坐标
            distance (_type_): 设定标定移动距离
            repeat_time (_type_): 最大重复标定次数
        """
        self.x = x
        self.y = y
        self.start_record = False
        self.target_distance = distance
        self.distance = 0
        self.speed = 0
        self.repeat_time = repeat_time
        self.current_repeat_time = 0
        self.__is_calibration = False

        self.pose_sub = rospy.Subscriber("/APETrack/PoseData", Pose2D, self.Pose_Callback)

    def Start_Calibration(self, speed, distance):
        """开始执行标定流程

        Args:
            distance (_type_): 标定距离
            speed (_type_): 标定速度

        Raises:
            Exception: 1. ros service没有响应; 2. 服务节点处于关闭状态

        Returns:
            Bool: 是否开始标定
        """

        # 发送service
        print("speed is {}, distance is {}".format(speed,distance))
        self.target_distance = distance
        # self.current_repeat_time = 0
        self.speed = speed
        node_list = tool.Ros_Get_NodeList()
        try:
            if CONTROL_NODE_NAME in node_list:
                rospy.wait_for_service(CALIBRATION_SERVICE_NAME)
                CalibrationStart = rospy.ServiceProxy(CALIBRATION_SERVICE_NAME, CalibrateStatus)
                resp1 = CalibrationStart(True, speed)
                # 将开始标定的状态写到数据库中
                statusCollection.update_one({}, {"$set": {"cali_status":2}})
                print("---calibration start !!!")
                if not resp1.success:
                    # 将失败的标定状态写到数据库中
                    statusCollection.update_one({}, {"$set": {"cali_status":0}})
                    print("***calibration run failed!!!")
                    # return resp1.success
            else:
                raise Exception("service dead")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False
        except Exception as e:
            print("Service call failed: %s"%e)
            return False

        # 开始进行标定
        self.__is_calibration = True
        # 开始记录距离
        self.start_record = True
        print("---start callback")
        return True
    

    def Stop_Calibration(self):
        # 发送service
        node_list = tool.Ros_Get_NodeList()
        try:
            if CONTROL_NODE_NAME in node_list:
                rospy.wait_for_service(CALIBRATION_SERVICE_NAME)
                CalibrationStart = rospy.ServiceProxy(CALIBRATION_SERVICE_NAME, CalibrateStatus)
                resp1 = CalibrationStart(False, 0)
                print("calibration stop!!!")
                print("the calibration state is {}".format(resp1.success))
                # 记录标定状态
                if not resp1.success:
                    # 如果标定不成功，在重新标定次数内，则重新标定
                    print("the calibration failed!")
                    self.current_repeat_time += 1
                    print("current time is {}, repeat time is {}".format(self.current_repeat_time, self.repeat_time))
                    if self.current_repeat_time < self.repeat_time:
                        set_info = {
                            "calibration_start": True,
                            "calibration_speed": -self.speed,
                            "calibration_distance": self.target_distance
                        }
                        setCollection.update_one({}, {"$set": set_info})
                        # self.Start_Calibration(-self.speed, self.target_distance)
                    else:
                        self.current_repeat_time = 0
                        # 将失败的标定状态写入数据库中
                        statusCollection.update_one({}, {"$set": {"cali_status":0}})
                    return resp1.success
                else:
                    self.current_repeat_time = 0
                    # 将标定的数据写入数据库中
                    config_info = {"calibration_param_ready":{
                        "steeringAngleOffset": resp1.steeringAngleOffset,
                        "laserOffsetAngle": resp1.laserOffsetAngle,
                        "isLaserOffsetAngleValid": resp1.isLaserOffsetAngleValid,
                        "isSteeringAngleOffsetValid": resp1.isSteeringAngleOffsetValid
                    }}
                    configCollection.update_one({}, {"$set": config_info})
                    # 将成功的标定状态写入数据库中
                    statusCollection.update_one({}, {"$set": {"cali_status":1}})
            else:
                raise Exception("service dead")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        except Exception as e:
            print("Service call failed: %s"%e)

        return True
    
    def Cancel_Calibration(self):
        # 发送取消service
        node_list = tool.Ros_Get_NodeList()
        try:
            if CONTROL_NODE_NAME in node_list:
                rospy.wait_for_service(CALIBRATION_SERVICE_NAME)
                CalibrationStart = rospy.ServiceProxy(CALIBRATION_SERVICE_NAME, CalibrateStatus)
                resp1 = CalibrationStart(False, 0)
                self.__is_calibration = False
                print("calibration is canceled")
            else:
                raise Exception("service dead")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        except Exception as e:
            print("Service call failed: %s"%e)
        return True
    
    def Calculate(self, msg):
        self.distance = math.sqrt(math.pow(msg.x-self.x,2)+math.pow(msg.y-self.y,2))
        print(self.distance)
        return self.distance
    
    def Pose_Callback(self, msg):
        if self.__is_calibration:
            # 更新里程计
            if self.start_record:
                print("---start record!!!")
                self.x = msg.x
                self.y = msg.y
                self.start_record = False
                self.distance = 0
            
            if self.Calculate(msg) >= self.target_distance:
                print("the targer distance is {}".format(self.target_distance),"the calibration is done")
                self.__is_calibration = False
                self.Stop_Calibration()
                
        return True

if __name__ == '__main__':
    rospy.init_node("calibration_node")
    rate = rospy.Rate(1)
    APE_Calibration = Calibration(0,0,0,3)

    while not rospy.is_shutdown():
        status_Dict = statusCollection.find_one()
        set_Dict = setCollection.find_one()
        if set_Dict["calibration_start"]:
            APE_Calibration.Start_Calibration(set_Dict["calibration_speed"], set_Dict["calibration_distance"])
            setCollection.update_one({}, {"$set": {"calibration_start": False}})
        if set_Dict["calibration_cancel"]:
            APE_Calibration.Cancel_Calibration()
            setCollection.update_one({}, {"$set": {"calibration_cancel": False}})

        rate.sleep()

    print("app_calibration is shutdown!")

