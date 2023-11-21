from configs.config_path import *
import rospy

import datetime
import logging


def getLogger(_fileName: str, _fileRoot: str, _level: int) -> logging.Logger:

    logger = logging.getLogger("logger")

    print(logger.handlers)

    if logger.handlers:
        return logger

    logger.setLevel(_level)
    formatter = logging.Formatter(
        "%(asctime)s - %(filename)s:%(lineno)s - %(levelname)s: %(message)s")

    file_handler = logging.FileHandler(_fileRoot + _fileName)
    file_handler.setLevel(_level)
    file_handler.setFormatter(formatter)

    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(_level)
    stream_handler.setFormatter(formatter)

    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)

    logger.info("Logger start!")
    return logger


# if rospy.get_param("/log_file_name_init") == False:
fileName = datetime.datetime.today().strftime("%Y_%m_%d_%H:%M")
# rospy.set_param("log_file_name", fileName)
# else:
# fileName = rospy.get_param("/log_file_name")
fileRootPath = ROOT_PATH + "script/logs/"

logger = getLogger(fileName, fileRootPath, logging.DEBUG)
