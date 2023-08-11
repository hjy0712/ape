from flask import Flask, jsonify, request, abort
import logging
from logging.handlers import TimedRotatingFileHandler

from utils.app_service.response import *
from utils import tool
from configs.config_path import *

import pymongo
from flask import current_app

from routes.ApeConfig import ApeConfig
from routes.ApeControl import ApeControl
from routes.ApeNavigation import ApeNavigation
from routes.ApeStatus import ApeStatus
from routes.ApeTest import ApeTest
bp_List = [ApeConfig, ApeControl, ApeNavigation, ApeStatus, ApeTest]

## data handle
myclient = pymongo.MongoClient("mongodb://localhost:27017/")
# database
apeDB = myclient["ape_db"]
# collection
configCollection = apeDB["ape_config_collection"]
statusCollection = apeDB["ape_status_collection"]
setCollection = apeDB["ape_set_collection"]

def create_app():
    app = Flask(__name__)
    # basic config, not use in this project
    # app.config.from_object(config[config_name])

    # change database
    statusDict = statusCollection.find_one()
    condition = {"_id": statusDict["_id"]}
    statusCollection.update_one(condition, {'$set' : {"start_Ctrl": False}})

    # 注册蓝图
    for bp in bp_List:
        app.register_blueprint(bp, url_prefix = "/api/{}".format(bp.name))

    # error handler
    handle_errors(app)

    # request handler
    handle_request(app)

    # logger config
    formatter = logging.Formatter(
        "[%(asctime)s][%(filename)s:%(lineno)d][%(levelname)s][%(thread)d] - %(message)s")
    handler = TimedRotatingFileHandler(
        "/home/ape/APE_Application/src/ape_single/script/logs/flask.log", when="D", interval=1, backupCount=15,
        encoding="UTF-8", delay=False, utc=True)
    app.logger.addHandler(handler)
    handler.setFormatter(formatter)

    time.sleep(5)
    tool.Run_ShellCmd("amixer -D pulse sset Master 100%+ on")
    time.sleep(1)
    tool.Run_ShellCmd("play "+ VOICE_FOLD + DIDI_NAME)

    return app

def handle_errors(app):
    """define error"""

    @app.errorhandler(403)
    def handle_403_error(err):
        """在进行连接之前，用户没有请求权限"""
        return Api_Result(noconnect_Error)

    @app.errorhandler(400)
    def handle_400_error(err):
        """用户的请求数据异常"""
        return Api_Result(request_Error)

    @app.errorhandler(404)
    def handle_404_error(err):
        """请求资源不存在"""
        return Api_Result(localFile_Error)

    @app.errorhandler(405)
    def handle_405_error(err):
        """请求方法错误"""
        return Api_Result(requestMethod_Error)

    @app.errorhandler(500)
    def handle_500_error(err):
        """自定义的处理错误方法"""
        return Api_Result(rosCom_Error)
    
    @app.errorhandler(Exception)
    def handler_exception(e):
        """ 异常捕捉处理，主要针对自行raise的部分"""
        # 地图构建异常
        if str(e) == "The map build is not complete!":
            error_Info = {
                "ret_code":1001,
                "create_on":time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
                "err_msg":str(e)
                }
        # 定位异常
        elif str(e) == "Map confidence is low!":
            error_Info = {
                "ret_code":1002,
                "create_on":time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
                "err_msg":str(e)
                }
        # 无定位的状态下发任务
        elif str(e) == "":
            error_Info = {
                "ret_code":1003,
                "create_on":time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
                "err_msg":str(e)
            }

        else:
            error_Info = {
                "ret_code":500,
                "create_on":time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
                "err_msg":str(e)
                }
                # code, status_code, error = 5000, 500, '%s(%s)' % (e.__class__.__name__, str(e))
        api_Response = jsonify(error_Info)
        api_Response.status_code = 200
        current_app.logger.error("{} : {}".format(request.path, str(e)))
        return api_Response

def handle_request(app):
    """define request"""
    
    @app.before_first_request
    def Middle_Init():
        # 判断该AGV是否处于初始化状态
        statusDict = statusCollection.find_one()
        condition = {"_id": statusDict["_id"]}
        statusCollection.update_one(condition, {'$set' : {"start_Ctrl": False}})

        # 判断是否初始化序列号
        configDict = configCollection.find_one()
        condition = {"_id": configDict["_id"]}
        try:
            if configDict["UUID"] == None:
                configCollection.update_one(condition, {"$set": {"UUID": int(time.time())}})
        except Exception as e:
            configCollection.update_one(condition, {"$set": {"UUID": int(time.time())}})


    # 下一步代码优化需要完成的几个钩子函数
    @app.before_request
    def beforeRequestJudge():
        if request.path == "/api/ApeConfig/lockAGV" or request.path == "/api/ApeConfig/webtest":
            return None
        # 屏蔽调试用API
        if request.path == "/api/ApeConfig/setAGVBodyParam":
            return None
        if request.path == "/api/ApeConfig/getAGVBodyParam":
            return None
        if request.path == "/api/ApeConfig/setAGVMotionParam":
            return None
        if request.path == "/api/ApeConfig/getAGVMotionParam":
            return None
            
        # 判断该AGV是否处于初始化状态
        statusDict = statusCollection.find_one()
        if statusDict["start_Ctrl"] == False:
            abort(403)
        else:
            return None

    @app.after_request
    def afterRequest(response):
        """每次正确请求/异常被handle接住后会执行的函数"""
        if response.get_json() == None:
            response = {"web_error":"unknow error"}
        else:
            response = response.get_json()
        responseInfo = {"requestID": request.path}
        response.update(responseInfo)
        response = jsonify(response)
        return response
