# PathSegment.msg
int64 sequenceID               #路线sequence
string index                   #路线ID
string startPointID            #起始点ID
PositionInfo startPointPosition#起始点坐标x,y,theta
string endPointID              #结束点ID
PositionInfo endPointPosition  #结束点坐标x,y,theta
float64 speedLimit             #路段最大限速
int16 lineType                 #路线类型：1直线 2圆弧 3Bspline（贝塞尔） 示教（平滑）4点
int32 controlPointsCnt         #控制点个数
PositionInfo[] controlPoints   #控制点
bool rotEnable                 #自旋使能
int16 rotDirection             #自旋方向
bool direction                 #路径方向, false正走 true倒走, 对于示教模式无用
bool ignoreDir                 #是否忽略目标站点方向, 对于示教模式无用