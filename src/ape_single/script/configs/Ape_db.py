import os
import sys

from flask import Flask

import sqlite3
import click
from flask import current_app, g
from flask.cli import with_appcontext

from flask_sqlalchemy import SQLAlchemy

db = SQLAlchemy()  # <--- The db object belonging to the blueprint

# create table

class Status(db.Model):
    __tablename__ = 'status'
    id = db.Column(db.Integer, primary_key=True)
    work_time = db.Column(db.Integer)
    task_number = db.Column(db.Integer)
    error_number = db.Column(db.Integer)
    task_list = db.Column(db.Integer)

class Errors(db.Model):
    __tablename__ = 'errors'
    id = db.Column(db.Integer, primary_key=True)
    error_list = db.Column(db.Integer)

class Tasks(db.Model):
    __tablename__ = 'tasks'
    id = db.Column(db.Integer, primary_key=True)
    task_list = db.Column(db.Integer)

# 1.创建数据库连接
def get_db():
    if 'db' not in g:
        g.db = sqlite3.connect(
            current_app.config['DATABASE'],
            detect_types=sqlite3.PARSE_DECLTYPES
        )
        g.db.row_factory = sqlite3.Row
    return g.db


# 2. 关闭数据库连接
def close_db():
    db = g.pop('db', None)
    if db is not None:
        db.close()


# 3. 初始化数据库
def init_db():
    db = get_db()
    with current_app.open_resource('schema.sql') as f:
        db.executescript(f.read().decode('utf8'))


@click.command('init-db')
@with_appcontext
# app context，应用上下文，存储的是应用级别的信息，比如数据库连接信息。
# request context，程序上下文，存储的是请求级别的信息，比如当前访问的url
def init_db_command():
    init_db()
    click.echo('数据库初始化成功。')


# 4. 在应用中注册`db.py`实现flask的调用
# 此处为注册函数，另一端在工厂函数（create_app）中,动态调用此函数，实现动态注册。
def init_app(app):
    # teardown_appcontext
    # 不管是否有异常，注册的函数都会在每次请求之后执行。
    # flask 为上下文提供了一个 teardown_appcontext 钩子，使用它注册的毁掉函数会在程序上下文被销毁时调用，通常也在请求上下文被销毁时调用。
    # 比如你需要在每个请求处理结束后销毁数据库连接：app.teardown_appcontext 装饰器注册的回调函数需要接收异常对象作为参数，当请求被正常处理时这个参数将是None，这个函数的返回值将被忽略。
    # 参考链接 https://www.cnblogs.com/Wu13241454771/p/15439350.html
    app.teardown_appcontext(close_db)
    app.cli.add_command(init_db_command)
