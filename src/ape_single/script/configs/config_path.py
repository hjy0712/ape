# root path
ROOT_PATH = "/home/ape/aiten-server-py/src/ape_single/"

# config json path
ORIGIN_CONFIG_FILE = ROOT_PATH + "script/configs/default_ape_config.json"

# status json path
STATUS_FILE = ROOT_PATH + "script/configs/default_status.json"

# set json path
SET_FILE = ROOT_PATH + "script/configs/default_set.json"


# voice path
# VOICE_FOLD = "/home/ape/ape_-android-app/src/ape_apphost/script/peripheral/voicefile/"
VOICE_FOLD = ROOT_PATH + "script/utils/voicefile/"
MANUAL_NAME = "manual.mp3"
ERROR_NAME = "error.mp3"
CHARGE_NAME = "charge.mp3"
DIDI_NAME = "didi.mp3"

# AGVpath json path
PATH_MAP = ROOT_PATH + "script/envs/path_map/"
PATH_MAP_NAME = "origin.json"
PATH_OPTIMAL_MAP_NAME = "optimal_origin.json"

USER_ORIGIN_PATH_MAP_NAME = "user_origin_path.json"
USER_PATH_MAP_NAME = "user_path.json"

# AGVmap json path
MAP = ROOT_PATH + "script/envs/map/"
MAP_NAME = "origin.json"

PATH_COMBINE = "path_combine.json"

# navigation task database

# task control status
TASK_NONE = 0
TASK_START = 1
TASK_STOP = 2
TASK_RESTART = 3
TASK_CANCELED = 4

# task run status
NONE = 0
MANAUL = 1
RUNNING = 2
SUSPENDED = 3
COMPLETED = 4
FAILED = 5
CANCELED = 6
IDLE = 7
CHARGING = 8

# localization status
LOC_FAILED = 0
LOC_SUCCESS = 1
LOC_RELOCING = 2
LOC_COMPLETED = 3


NAVTASK_INFO = {
    "task_control_status": TASK_NONE,
    "task_run_status": NONE,
    "station_list": [],
    "operation_list": [],
    "current_station_index": 0,
    "given_run_time": 0,
    "current_run_time": 0,
    "tracking_end": False
}

SMAPDATA = {
    "mapDirectory": "",
    "header": {
        "mapType": "2D-Map",
        "mapName": "test",
        "minPos": {},
        "maxPos": {},
        "resolution": 0,
        "version": "1.0.6"
    },
    "normalPosList": [],
    "advancedPointList": [],
    "demonstrationPathList": [],
}

# control node name
CONTROL_NODE_NAME = "/motionControl"
# control service name
CONTROL_TASK_SERVICE_NAME = "/APE_Task/taskStatus"
CONTROL_TRACK_SERVICE_NAME = "/APE_Task/trackStatus"
# control topic name
CONTROL_PATH_TOPIC_NAME = "/APE_Task/trackPath"
CONTROL_SEQ_TOPIC_NAME = "/APE_Task/seqID"
CONTROL_MODE_TOPIC_NAME = "/APE_Control/mode"
CONTROL_CMD_TOPIC_NAME = '/APE_Control/manualCmd'
# confidence topic name
TRUST_TOPIC_NAME = "/APE_Trust"
# calibration service
CALIBRATION_SERVICE_NAME = "/APE_Cali/caliCmd"
# serial
SERIAL_NODE_NAME = "web_serial_node"
SERIAL_PARAM_SERVICE_NAME = "/ape_srv/board_param_rs"
SERIAL_BASIC_SERVICE_NAME = "/ape_srv/basic_info_read"

# log file
LOG_FILE = ROOT_PATH + "script/logs/flask.log"
