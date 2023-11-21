from flask import Blueprint, request
from flask import current_app

from utils.app_service.response import *

# 调用logger可以写作下式
# current_app.logger.error("{}".format(request.path))

ApeTest = Blueprint("ApeTest", __name__, url_prefix="/api/ApeTest")


# --------------- 测试服务器连接状态 ---------------- #
@ApeTest.route("/connectTest", methods=["GET"])
def ConnectTest():
    ##TODO：可以加入一些定时运行代码
    return Api_Result(success)
