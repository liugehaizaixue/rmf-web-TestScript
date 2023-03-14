import requests
import time
from Utils.logger import LoggingManager
class MyClass:
    def __init__(self):
        self.token = None
        self.retry_count = {}
        self.logger = LoggingManager('main', log_file='logs/mylogs.log').logger


    def CancelTask(self, task_id, req_id):
        status_code = 500
        # self.HandleCancelRespone(status_code=status_code, task_id=task_id,req_id=req_id retry_count=self.retry_count)
        self.HandleCancelRespone(status_code=status_code, task_id=task_id,req_id=req_id,retry_count=self.retry_count)

    def HandleCancelRespone(self, status_code, task_id, req_id, retry_count, max_retry_count=3, retry_interval=2):
        if req_id not in retry_count:
            retry_count[req_id] = 0
        if status_code == 200:
            print(f"Task {task_id} has been cancelled successfully.")
        elif status_code == 401 or status_code == 403:
            if retry_count[req_id] < max_retry_count:
                print(f"Retrying to cancel task {task_id} ({retry_count[req_id]}/{max_retry_count})...")
                retry_count[req_id] += 1
                time.sleep(retry_interval)
                time.sleep(2)
                self.CancelTask(task_id,req_id=req_id)
            else:
                print(f"Reached max retry count ({max_retry_count}), giving up.")
        else:
            if retry_count[req_id] < max_retry_count:
                print(f"Retrying to cancel task {task_id} ({retry_count[req_id]}/{max_retry_count})...")
                retry_count[req_id] += 1
                time.sleep(retry_interval)
                self.CancelTask(task_id,req_id=req_id)
            else:
                print(f"Reached max retry count ({max_retry_count}), giving up.")



my_class = MyClass()
my_class.logger.info('Debug message')
my_class.logger.debug('Debug message')  # 不会被记录
my_class.logger.info('Info message')  # 不会被记录
my_class.logger.warning('Warning message')  # 会被记录
my_class.logger.error('Error message')  # 会被记录
my_class.logger.critical('Critical message')  # 会被记录
my_class.CancelTask("my_task_id","@@@")
my_class.CancelTask("my_task_id","@@@")
my_class.CancelTask("my_task_id2","WWWWWW")