#!/home/ape/tool/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-
import os
import subprocess

def Run_ShellCmd(cmd:str, output:bool = 0) -> str:
    """
    func: Run command in shell.
    Input:
    {
        "cmd": command to run in shell
        "output": if output message needed
                    if output = 1, function will return the output of command. But the cmd will be run Synchronous
                    if output = 0 (default), function will be run Asynchronous. But no output will be return
    }
    Returns: 
    {
        {command output} or "done" or "error"
    }
    """
    try:
        if(not output):
            os.popen(cmd)
            return "done"
        else:
            result = os.popen(cmd)
            resultmes = result.read()
            result.close()
            return resultmes
    except:
        return "error"
    # try:
    #     #result = os.popen(cmd)
    #     if(not output):
    #         subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    #         return "done"
    #     else:
    #         message = subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    #         mesOutput = str(message.communicate()[0],"utf-8")
    #         return mesOutput
    # except:
    #     return "error"

def Ros_Get_NodeList() -> list:
    """
    func: Get all the node running in ROS
    Input:
    {
        none
    }
    Returns: 
    {
        all the node , e.g. ["/rosout"]
    }
    """
    nodeList = Run_ShellCmd("rosnode list", output = 1)
    nodeList = nodeList.split("\n")[:-1]
    return nodeList

def Ros_Kill_Roscore() -> None:
    """
    func: kill ros master process
    Input:
    {
        none
    }
    Returns: 
    {
        none
    }
    """
    Run_ShellCmd("killall -9 roscore")
    Run_ShellCmd("killall -9 rosmaster")
