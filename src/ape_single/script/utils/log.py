#!/home/ape/tool/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import os, sys
from datetime import datetime


class Logger(object):
    """
    日志操作类
    """
    import logging.handlers
    import logging

    # 日志目录
    __LOG_PATH = os.path.abspath(os.path.dirname(os.path.dirname(__file__)) + os.sep + 'logs' + os.sep)
    # 日志大小（单位：M），超过后最老日志被自动覆盖
    __LOG_SIZE = 8
    # 日志保存个数
    __LOG_NUM = 1

    # 读取目录路径
    logpath = __LOG_PATH
    # 读取日志文件容量，转换为字节
    logsize = 1024 * 1024 * __LOG_SIZE
    # 读取日志文件保存个数
    lognum = __LOG_NUM

    # 判断目录路径是否存在，假如不存在，则创建
    if not os.path.exists(logpath):
        os.mkdir(logpath)
    # 日志文件名：由用例脚本的名称，结合日志保存路径，得到日志文件的绝对路径
    logname = os.path.join(logpath, sys.argv[0].split('/')[-1].split('.')[0] + '_' + datetime.now().strftime("%Y%m%d") + '.log')

    # 初始化logger
    log = logging.getLogger()
    # 日志格式，可以根据需要设置
    fmt = logging.Formatter('[%(asctime)s][%(levelname)s] %(message)s',
                            '%Y-%m-%d %H:%M:%S')

    # 日志输出到文件，这里用到了上面获取的日志名称，大小，保存个数
    handle1 = logging.handlers.RotatingFileHandler(logname, maxBytes=logsize, backupCount=lognum, encoding='utf-8')
    handle1.setFormatter(fmt)
    # 同时输出到屏幕，便于实施观察
    handle2 = logging.StreamHandler(stream=sys.stdout)
    handle2.setFormatter(fmt)
    log.addHandler(handle1)
    log.addHandler(handle2)

    # 设置日志基本，这里设置为INFO，表示只有INFO级别及以上的会打印
    log.setLevel(logging.INFO)

    # 日志接口，用户只需调用这里的接口即可，这里只定位了INFO, WARNING, ERROR三个级别的日志，可根据需要定义更多接口
    @classmethod
    def info(self, msg):
        self.log.info(msg)
        return

    @classmethod
    def warning(self, msg):
        self.log.warning(msg)
        return

    @classmethod
    def error(self, msg):
        self.log.error(msg)
        return