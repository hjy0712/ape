from flask import Blueprint, request
from flask import current_app
from utils import tool

from utils.app_service.response import *


ApeTest = Blueprint("ApeTest", __name__, url_prefix="/api/ApeServer")


# --------------- 重启后端服务 ---------------- #
@ApeTest.route("/restartAGV", methods=["GET"])
def ConnectTest():
    ##TODO：执行重启命令脚本
    tool.Run_ShellCmd("cd && source restart.sh")
    return Api_Result(success)
