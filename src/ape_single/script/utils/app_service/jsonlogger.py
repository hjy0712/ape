import os
import json
import fcntl

class Logger:

    def __init__(self, file_name):
        log_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "logs"))
        print(log_path)
        if not os.path.exists(log_path):
            os.mkdir(log_path)
        self.logname = log_path + "/" +file_name + ".json"
        try:
            with open(self.logname, "r", encoding="utf8") as f:
                pass
        except Exception:
            with open(self.logname, "w", encoding="utf8") as f:
                pass

    # add content
    def Add_Content(self, add_dict):
        """
        func: add new item in log file
        Params: 
        {
            "add_dict": new item, type is dictionary
        }
        """
        try:
            with open(self.logname, "r", encoding="utf8") as f:
                try:
                    log_content = json.load(f)
                    log_content.update(add_dict)
                except Exception:
                    log_content = add_dict
            with open(self.logname, "w", encoding="utf8") as f:
                json.dump(log_content, f, indent=4, ensure_ascii=False)
            return True
        except Exception:
            return False

    # change content
    def Change_Content(self, change_dict):
        """
        func: change item in log file
        Params: 
        {
            "change_dict": change item, type is dictionary
        }
        """
        try:
            with open(self.logname, "r", encoding="utf8") as f:
                log_content = json.load(f)
                for i in change_dict:
                    log_content[i] = change_dict[i]
            with open(self.logname, "w", encoding="utf8") as f:
                json.dump(log_content, f, indent=4, ensure_ascii=False)
            return True
        except Exception:
            return False

    # delete content
    def delete_Content(self, delete_key):
        """
        func: delete item in log file
        Params: 
        {
            "delete_key": delete item, type is string
        }
        """
        try:
            with open(self.logname, "r", encoding="utf8") as f:
                log_content = json.load(f)
                log_content.pop(delete_key)
            with open(self.logname, "w", encoding="utf8") as f:
                json.dump(log_content, f, indent=4, ensure_ascii=False)
            return True
        except Exception:
            return False
    
    # find content
    def find_Content(self, find_key):
        """
        func: find value in log file
        Params: 
        {
            "find_key": find item, type is list
            
        }
        eg:
        params:["key1", "key2"]
        return dict["key1"]["key2"]
        """
        try:
            with open(self.logname, "r", encoding="utf8") as f:
                log_content = json.load(f)
                for i in find_key:
                    log_content = log_content[i]
                find_value = log_content
            return find_value
        except Exception:
            return False

    # clear file content
    def clear_Content(self):
        try:
            with open(self.logname, "w", encoding="utf8") as f:
                pass
            return True
        except Exception:
            return False

class JsonFile:
    def __init__(self):
        pass

    @staticmethod
    def jsonSafeSave(dir, data, mode = "", block = True):
        if mode == "":
            if os.path.exists(dir):
                mode = 'r+' #以r+模式打开避免with...open打开文件时自动清空内容同时被读取
            else:
                mode = 'w' #以w模式打开以新建文件
        with open(dir, mode=mode, encoding="utf8") as f:
            if block:
                # 当文件存在文件锁时，阻塞，等待锁取消后再执行
                fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                f.truncate() # 从文件指针位置开始截断，相当于删除文件指针以后的内容
                json.dump(data, f, indent=4, ensure_ascii=False)
            else:
                # 当文件存在文件锁时，不阻塞，继续运行，放弃本次文件保存操作
                fcntl.flock(f.fileno(), fcntl.LOCK_EX|fcntl.LOCK_NB)
                f.truncate()
                json.dump(data, f, indent=4, ensure_ascii=False)