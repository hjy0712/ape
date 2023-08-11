#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

# Author: 雷文捷
# Function: flask服务器主程序
# Dependence: flask库
# Other Information: 安装方案:
                    # pip install flask
                    # 因为是在ros中运行，注意安装到ros可以运行的python环境下

from app import create_app

if __name__ == "__main__":
    app = create_app()
    app.run(host='0.0.0.0',port="5000",debug=False) #指定当前电脑ip地址