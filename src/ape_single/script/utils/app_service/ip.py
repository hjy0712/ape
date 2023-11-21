import os

''' 获取终端命令的输出 '''
def getCMD(cmd: str) -> str:
    result = os.popen(cmd)
    resultmes = result.read()
    result.close()
    return resultmes


''' 根据空行分段落 返回段落列表'''
def parseData(data):
    parsed_data = []
    new_line = ''
    data = [i for i in data.split('\n') if i]
    for line in data:
        if line[0].strip():
            parsed_data.append(new_line)
            new_line = line + '\n'
        else:
            new_line += line + '\n'
    parsed_data.append(new_line)
    return [i for i in parsed_data if i]


""" 获取无线网卡名称 """
def getWIFIName():
    result = getCMD("iwconfig")
    result_parse = parseData(result)
    wifi = [i for i in result_parse if not len(i.split('\n')) == 1]
    wifi_name = wifi[0].split('\n')[0].split()[0]
    return wifi_name


''' 根据输入的wifi网卡名称分析出它的ip、netmask、gateway等信息 '''
def parseIfconfig(wifi_name):
    dic = {}
    wifi_config = getCMD("ifconfig {}".format(wifi_name))
    line_list = wifi_config.split('\n')
    dic['IP_address']  = line_list[1].split()[1]
    dic["netmask"] = line_list[1].split()[3]
    gateway_config = getCMD("ip route show | grep {} | grep default".format(wifi_name))
    dic["gateway"] = gateway_config.split()[2]
    return dic


def modifyWIFIconfig(address: str, gateway: str):
    device = getCMD("nmcli d")
    device_list = device.split("\n")
    connection_name = None
    for i in device_list:
        if i.split()[2] == "connected":
            connection_name = i.split()[3]
            break
    getCMD("echo {} | sudo -S nmcli c m \'{}\' ipv4.method \"manual\" ipv4.addresses \"{}\" ipv4.gateway \"{}\" ipv4.dns \"{}\"".format(123123, connection_name, address, gateway, gateway))
    print("nmcli c m \'{}\' ipv4.method \"manual\" ipv4.addresses \"{}\" ipv4.gateway \"{}\" ipv4.dns \"{}\"".format(connection_name, address, gateway, gateway))
    getCMD("echo {} | sudo -S systemctl restart network-manager.service".format(123123))


if __name__ == '__main__':
    wifi_name = getWIFIName()
    net_config = parseIfconfig(wifi_name)
    print("wifi is {}, net_config is {}".format(wifi_name, net_config))
    modifyWIFIconfig("192.168.5.111/24", "192.168.5.1")