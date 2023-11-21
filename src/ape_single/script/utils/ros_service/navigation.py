#!/home/ape/tool/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

from utils.tool import *
import time
import os


class Navigation:
    """
    This is a class for APE vehicle navigation and positioning

    workflow:
    slam
    1. Instantiate this class. if laser and tftree error, it will raise error.
    2. Start_Slam() to start getting map
    3. Stop_Slam() to stop getting map information. But the node still working
    4. Save_SlamMap(mapName) to save the map built
    5. Kill_Slam()  to kill the slam node

    localization
    6. Start_Localization().   if no slam before, pbstream file needed
    7. Kill_Localization().
    """

    __isRosLaserOpen = 1
    __isRosTFtreeOpen = 1
    __isRosSlamOpen = 0
    __isRosLocOpen = 0

    __RosTFtreeNode = ["/robot_state_publisher"]
    __RosLaserNode = ["/r2000_node"]
    __RosSlamNode = ["/cartographer_node", "/cartographer_occupancy_grid_node"]
    __RosLocNode = ["/cartographer_node_localization",
                    "/cartographer_occupancy_grid_node_localization"]

    __lastMapName = ""
    yaml_file = ""

    def __init__(self) -> None:
        if not (self.Check_Laser_Working() and self.Check_TFtree_Working()):
            raise Exception("Laser or TFtree is not Ready")
        else:
            pass

    def __del__(self) -> None:
        # if(self.__isRosSlamOpen):
        #     self.Kill_Slam()
        # if(self.__isRosLaserOpen):
        #     self.Kill_Laser()
        # if(self.__isRosTFtreeOpen):
        #     self.Kill_TFtree()
        pass

#################################################################################################
###########################   ros resource for slam and navigation    ############################
#################################################################################################
# ---------------------  start resource  -------------------
    def Start_TFtree(self) -> str:
        Run_ShellCmd("roslaunch ape_coordinate ape_tftree.launch")
        self.__RosTFtreeNode = 1

    def Start_Slam(self) -> str:
        """
        func: start slam in cartographer
        Returns: 
        {
            "succeed"
            "already started"
            "error"
        }
        """
        try:
            if (not self.__isRosSlamOpen):
                os.popen("roslaunch ape_coordinate carto_slam.launch")
                # Run_ShellCmd("roslaunch ape_coordinate carto_slam.launch")
                self.__isRosSlamOpen = 1
                return "succeed"
            else:
                return "already started"
        except:
            return "error"

# ---------------------  check if resource is working -------------------
    def Check_Laser_Working(self) -> bool:
        """
        func: check if Laser is working
        Returns: 
        {
            "result": 1:is working   0:not working
        }
        """
        rosNodeList = Ros_Get_NodeList()
        # 如果lasernode不在当前nodelist中的项是空的，则开着
        if [x for x in self.__RosLaserNode if x not in rosNodeList] == []:
            self.__isRosLaserOpen = 1
            return 1
        else:
            self.__isRosLaserOpen = 0
            return 0

    def Check_TFtree_Working(self) -> bool:
        """
        func: check if Laser is working
        Returns: 
        {
            "result": 1:is working   0:not working
        }
        """
        rosNodeList = Ros_Get_NodeList()
        # 如果TFnode不在当前nodelist中的项是空的，则开着
        if [x for x in self.__RosTFtreeNode if x not in rosNodeList] == []:
            self.__isRosTFtreeOpen = 1
            return 1
        else:
            self.__isRosTFtreeOpen = 0
            return 0

    def Check_Slam_Working(self) -> bool:
        """
        func: check if Laser is working
        Returns: 
        {
            "result": 1:is working   0:not working
        }
        """
        rosNodeList = Ros_Get_NodeList()
        # 如果Slamnode不在当前nodelist中的项是空的，则开着
        if [x for x in self.__RosSlamNode if x not in rosNodeList] == []:
            self.__isRosSlamOpen = 1
            return 1
        else:
            self.__isRosSlamOpen = 0
            return 0

    def Check_Localization_Working(self) -> bool:
        """
        func: check if Laser is working
        Returns: 
        {
            "result": 1:is working   0:not working
        }
        """
        rosNodeList = Ros_Get_NodeList()
        if [x for x in self.__RosLocNode if x not in rosNodeList] == []:  # 如果Locnode不在当前nodelist中的项是空的，则开着
            self.__isRosLocOpen = 1
            return 1
        else:
            self.__isRosLocOpen = 0
            return 0

# ---------------------  shut down resource or slam  -------------------
    def Kill_Slam(self) -> None:
        """
        func: if slam is working, kill it;  if it not, do nothing
        Returns: 
        {
            1:succeed  0:error
        }
        """
        for eachNode in self.__RosSlamNode:
            Run_ShellCmd("rosnode kill " + eachNode)
        self.__isRosSlamOpen = 0

# ---------------------  stop slam and save map  -------------------
    def Stop_Slam(self) -> None:
        """
        func: stop receiving slam message,but don't kill node
        """
        res = Run_ShellCmd("rosservice call /finish_trajectory 0", output=1)
        print("rosservice call /finish_trajectory 0 result if {}".format(res))

    def Save_SlamMap(self, mapName: str = "defaultMap", filepath: str = "") -> str:
        """
        func: save slam map 
        Inputs: 
        {
            filepath: the path you want to save maps. End with "/"
            mapName:the mapName you want to save with.
        }
        Return:
        {
            the pbstream file (Full path)
        }
        """
        if filepath == "":
            filepath = os.path.abspath('.')  # 默认地址是程序当前地址
            filepath = filepath + "/map/"
        elif not filepath[-1] == "/":
            raise ValueError("file path need to end with '/'")

        filepath = filepath + mapName + "/"
        file = filepath + mapName + ".pbstream"

        print("SAVE MAP TO:  " + file)

        folder = os.path.exists(filepath)

        if not folder:  # 判断是否存在文件夹如果不存在则创建为文件夹
            os.makedirs(filepath)  # makedirs 创建文件时如果路径不存在会创建这个路径

        # Run_ShellCmd("mkdir " + filepath)

        self.__lastMapName = file
        self.yaml_file = filepath + mapName + ".yaml"
        # 输出 pbstream
        Run_ShellCmd(
            "rosservice call /write_state  \"{filename: '" + file + "'}\"", output=1)
        # 输出 yaml and image:
        Run_ShellCmd("rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=" + filepath + mapName +
                     " -pbstream_filename=" + file + " -resolution=0.05", output=1)

# ---------------------  start and stop localization  -------------------
    def Start_Localization(self, PbstreamFile: str = "", YamlFile: str = "") -> None:
        """
        func: start only localization in cartographer
        Inputs: 
        {
            PbstreamFile: the map file used to localization.
                        default -- the last map you save
        }
        """
        if PbstreamFile == "":
            if self.__lastMapName == "":
                raise ValueError("NO Map input or no Slam history")
            else:
                PbstreamFile = self.__lastMapName

        if YamlFile == "":
            if self.yaml_file == "":
                raise ValueError("NO Map input or no Slam history")
            else:
                YamlFile = self.yaml_file
        # print("roslaunch ape_coordinate ape_localization.launch load_state_filename:=" + PbstreamFile)
        Run_ShellCmd(
            "roslaunch ape_coordinate ape_localization.launch load_state_filename:=" + PbstreamFile)
        time.sleep(3)
        Run_ShellCmd(
            "roslaunch ape_reflector_loc reflector_local.launch map_file:=" + YamlFile)
        self.__isRosLocOpen = 1

    def Kill_Localization(self) -> None:
        """
        func: if slam is working, kill it;  if it not, do nothing
        Returns: 
        {
            1:succeed  0:error
        }
        """
        for eachNode in self.__RosLocNode:
            Run_ShellCmd("rosnode kill " + eachNode)
        self.__isRosLocOpen = 0


if __name__ == "__main__":
    pass
    nav = Navigation()
    nav.Start_Localization(
        "/home/ape/ape_-android-app/src/ape_apphost/script/map/origin/origin.pbstream")
    # result.read()
    # result.close()
    # output = result.read()
    # result.close()
