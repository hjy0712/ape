
# -*- coding: utf-8 -*-
import time

class Middleinfo:

    def __init__(self):
        # control
        self.start_Ctrl = False  # connection status

        # motion
        self.velocityVel = 0
        self.wheelAngle = 0

        # position
        self.x = 0
        self.y = 0
        self.angle = 0
        self.confidence = 0
        # status
        self.collision = False
        self.emergency = False
        self.forkStatus = 0    #0 fork is low, 1 fork is high
        self.inPosition = False
        self.softLock = False    # 软件锁定，用于在错误状态时锁定车辆
        self.hisLockStatus = False

        # battery status
        self.batteryLevel = 0
        self.batteryTemp = 0
        self.voltage = 0
        self.current = 0
        self.chargeOn = False

        #time
        self.startTime = time.time()

        # init
        self.initStatus = True

    def Connect_Fun(self):
        self.start_Ctrl = True

    def Unconnect_Fun(self):
        if self.start_Ctrl:
            self.start_Ctrl = False
        else:
            raise("No Built Connection")

    def Status_Init(self):
        self.collision = False
        self.emergency = False
        self.forkStatus = 0
        self.softLock = False
