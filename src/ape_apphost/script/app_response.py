import time

from flask import jsonify

class Middleresponse:
    # response info
    request_Error = {
        "ret_code":400,
        "create_on":time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
        "err_msg":"There is no correct param"
    }

    # no connection info
    noconnect_Error = {
                    "ret_code":403,
                    "create_on":time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
                    "err_msg":"There is no connection"
                    }

    # local configure error info
    localFile_Error = {
                    "ret_code":404,
                    "create_on":time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
                    "err_msg":"the operation to local configure file occur error"
                    }
    
    # success info
    success = {
                "ret_code":200,
                "create_on":time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
                "err_msg":"None"
                }

    # ros message error info
    rosCom_Error = {
                "ret_code":500,
                "create_on":time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
                "err_msg":"the process of ros communication is error"
                }

    # request method error info
    requestMethod_Error = {
                "ret_code":405,
                "create_on":time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
                "err_msg":"the method of request is error"
                }

    def __init__(self):
        pass

    def Api_Result(self, response_type):
        api_Response = jsonify(response_type)
        api_Response.status_code = 200
        return api_Response

    def Api_Return_Param(self, response_content):
        response_content.update(self.success)
        api_Response = jsonify(response_content)
        api_Response.status_code = 200
        return api_Response