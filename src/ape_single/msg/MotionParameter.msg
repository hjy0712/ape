# MotionParameter.msg
float64 maxStraightForwardSpeed    #前进最大直线速度：mm/s
float64 maxStraightBackwardSpeed   #后退最大直线速度：mm/s
float64 maxTurningForwardSpeed     #前进最大转弯速度：mm/s
float64 maxTurningBackwardSpeed    #后退最大转弯速度：mm/s
float64 maxRotSpeed                #自旋的最大速度限制：mm/s
float64 maxRotSpeed_Cargo          #带货状态下的自旋最大速度限制：mm/s
float64 maxSpeedCargo              #带货状态下的最大速度限制：mm/s
float64 maxManualSpeed             #最大手动控制速度
float64 maxManualAngle             #最大手动打角 -> 转角限制
float64 accelRateHigh              #1级加速度 - 1200mm/s2
float64 accelRateLow               #2级加速度 - 400mm/s2
float64 decelRateHigh              #/级减速度 - 1800mm/s2
float64 decelRateLow               #2级减速度 - 400mm/s2
float64 preciseDiffAttitude        #小角度位姿调整，航向可允许误差
float64 steeringVel                #舵轮打角速度 °/s
float64 minResponseSpeed           #最小响应速度 - 40-50mm/s
float64 speedDelayTime             #舵轮响应延时 - ~0.5s