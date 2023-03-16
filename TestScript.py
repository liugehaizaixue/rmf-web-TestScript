""" 
切换大地图
任务 产生与取消 pickup/dropoff
dropoff根据地图waypoint由机器人随机生成
充电任务根据机器人电量自动生成———充电任务,则记录机器人在设定电量下,是否前去充电 ?

创建subscriber,监听机器人发起的dropoff任务,日志记录


测试充电任务能否被打断
先手动测试一趟,是否符合仿真要求
"""
import random 
import requests
import time
import json
import threading
import queue
from DropOffSub import DropOffSubscriber
from FleetState import FleetState
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import datetime
import yaml
import os
import uuid
from Utils.logger import LoggingManager

class TestScript:
    def __init__(self):      
        now = datetime.datetime.now()
        log_name=now.strftime('%Y-%m-%d_%H-%M-%S.log')
        self.logger = LoggingManager('TestScript', log_file='logs/'+log_name).logger
        # 总共生成3000个任务 或 运行时间大于 1h 
        self.total_tasks = 300
        self.total_run_time = 1

        # 初始化pickup任务类型
        self.pick_types = ["robot_task_request","dispatch_task_request"]
        # self.pick_types = ["robot_task_request"]
        self.init_time = time.time()
        self.current_task_num = 0
        self.cancel_task_num = 0
        self.robots = ["tinyRobot1"]
        self.GetWaypoints()
        self.PickList=[] #记录发起pickup任务id，执行完毕后删除
        self.DropList=[]
        self.FleetStateDict = {}
        # 创建线程同步对象和队列
        # self.lock = threading.Lock()
        # self.queue = queue.Queue()
        # self.lock_event = threading.Event()

        # 线程停止标志位
        self.create_task_stop_flag = False  
        self.cancel_task_stop_flag = False  
        self.refresh_token_stop_flag = False  
        self.get_task_state_stop_flag = False
        # 失败重试次数标志
        self.retry_count = {}

        # 监听dropoff部分
        self.drop_subscriber = DropOffSubscriber()
        self.drop_is_listening = True
        self.drop_sub_thread = threading.Thread(target=self.DropSubThread)

        # 监听FleetState部分
        self.fleet_state_getter = FleetState()
        self.fleet_state_running = True
        self.fleet_state_thread = threading.Thread(target=self.FleetStateRun)

        self.GetToken()

    def FleetStateRun(self):
        while self.fleet_state_running:
            value = self.fleet_state_getter.get_state()
            if value is None:  # 判断是否应该退出循环
                break
            self.FleetStateDict = value
            self.logger.info("Fleet State :"+str(self.FleetStateDict))
            
    def FleetStateStop(self):
        self.fleet_state_running = False
        self.fleet_state_getter.stop()

    def FleetStateStart(self):
        self.fleet_state_thread.start()           

    def DropSubThread(self):
        # 在这里执行其他操作，并等待停止监听的信号
        while self.drop_is_listening:
            rclpy.spin_once(self.drop_subscriber.node)
        # 停止ROS2消息的监听
        self.drop_subscriber.destroy_subscription(self.drop_subscriber.subscription)
        self.drop_subscriber.destroy_node()    

    def StartDropSub(self):
        # 启动监听线程
        self.drop_sub_thread.start()

    def StopDropSub(self):
        self.drop_is_listening = False

    def GetWaypoints(self):
        file_path = './Setting/lejushandong.building.yaml'
        with open(file_path, 'r') as f:
            yaml_content = f.read()
        # 解析YAML内容
        yaml_data = yaml.safe_load(yaml_content)
        points = yaml_data["levels"]["L1"]["vertices"]
        waypoints=[]
        for inner_arr in points:
            if inner_arr[3] != '':
                waypoints.append(inner_arr[3])
        # 获取地图所有waypoints
        self.waypoints=waypoints
        self.waypoints.remove("lift1")
        self.logger.info("获取Waypoints:")
        self.logger.info(self.waypoints)

    def RandomCreate(self):
        self.create_task_stop_flag = False
        while not self.create_task_stop_flag:
            interval = random.randint(5,50)
            random_tasks_num = random.randint(10,20)
            for i in range(random_tasks_num):
                self.current_task_num = self.current_task_num + 1
                self.CreateTask()
            #  每次随机生成 5-10个任务
            self.logger.info(f"睡眠{interval}生成个{random_tasks_num}任务")
            self.logger.info("current_task_num:"+str(self.current_task_num))
            time.sleep(interval)  # 每5-100s刷新创建一轮任务

    
    def CreateTask(self):
         # 随机生成pickup任务类型——指定机器人or不指定
        choice_type = random.choice(self.pick_types)
        # 随机生成pickup任务地点进行提交
        pickup_place = random.choice(self.waypoints)
        unix_millis = int(round(time.time() * 1000))
        if choice_type == "robot_task_request" :
            choice_robot = random.choice(self.robots)
            task_data = {
                "type":"robot_task_request",
                "payload":{
                    "type": "robot_task_request",
                    "robot": choice_robot,
                    "fleet": "tinyRobot",
                    "request": {
                        "unix_millis_earliest_start_time": unix_millis,
                        "category": "pickup",
                        "description": {
                        "pickup": {
                            "place": pickup_place,
                            "handler": "coke_dispenser",
                            "payload": []
                            }
                        }
                    }
                }
            }
        elif choice_type == "dispatch_task_request" :
            task_data = {
                "type":"dispatch_task_request",
                "payload":{
                    "type": "dispatch_task_request",
                    "request": {
                        "unix_millis_earliest_start_time": unix_millis,
                        "category": "pickup",
                        "description": {
                        "pickup": {
                            "place": pickup_place,
                            "handler": "coke_dispenser",
                            "payload": []
                            }
                        }
                    }
                }
            }
        self.logger.info("Create Task")
        req_id = str(uuid.uuid4())
        self.SendTask(task_data,req_id)
        
    
    def start_create_task(self):
        self.create_task_thread = threading.Thread(target=self.RandomCreate)
        self.create_task_thread.start()
    def stop_create_task(self):
        if self.create_task_thread is not None:
            self.create_task_stop_flag=True

    def SendTask(self,task_data,req_id):
        if task_data["type"] == "robot_task_request" :
            url = "http://127.0.0.1:8000/tasks/robot_task"
        elif task_data["type"] == "dispatch_task_request" :
            url = "http://127.0.0.1:8000/tasks/dispatch_task"        
        headers = {"Authorization": f"Bearer {self.token}"}
        payload = task_data["payload"]
        response = requests.post(url=url, json=payload,headers=headers)
        self.logger.info("send Task : "+req_id)
        self.HandleCreateRespone(response=response,req_id=req_id,task_data=task_data, retry_count=self.retry_count)
        # 机器人根据pick随机生成dropoff

    def start_cancel_task(self):
        self.cancel_task_thread = threading.Thread(target=self.RandomCancel)
        self.cancel_task_thread.start()
    def stop_cancel_task(self):
        if self.cancel_task_thread is not None:
            self.cancel_task_stop_flag = True

    # 从PickList中随机取消任务
    def RandomCancel(self):
        self.cancel_task_stop_flag = False  
        while not self.cancel_task_stop_flag:
            interval = random.randint(60,180)
            random_tasks_num = random.randint(1,5)
            self.logger.info(f"睡眠{interval}取消个{random_tasks_num}任务")
            time.sleep(interval)  # 每60-180s刷新一轮取消任务
            for i in range(random_tasks_num):
                if len(self.PickList) != 0 :
                    task_id = random.choice(self.PickList)
                    self.logger.info(f"申请取消{task_id}任务")
                    req_id = str(uuid.uuid4())
                    self.CancelTask(task_id=task_id,req_id=req_id)
                else:
                    self.logger.info("当前pickup队列为空")
            #  每次随机取消1-5个任务
    
    # 发送取消任务请求
    def CancelTask(self,task_id,req_id):
        url = "http://localhost:8000/tasks/cancel_task"
        headers = {"Authorization": f"Bearer {self.token}"}
        payload={
            "type":"cancel_task_request",
            "task_id":task_id
        }
        response = requests.post(url=url, json=payload,headers=headers)
        self.logger.info("CancelTask:"+task_id)
        self.HandleCancelRespone(response=response,task_id=task_id,req_id=req_id,retry_count=self.retry_count)
    
    def HandleCancelRespone(self,response,task_id,req_id,retry_count, max_retry_count=3, retry_interval=2):
        if req_id not in retry_count:
            retry_count[req_id] = 0
        if response.status_code == 200:
            self.logger.info("HandleCancelRespone:"+str(response.status_code))
            self.cancel_task_num = self.cancel_task_num + 1
            if task_id in self.PickList:
                self.PickList.remove(task_id)
                # self.lock.acquire()  # 加锁
                # self.queue.put(self.PickList)  # 将新数组添加到队列中
                # self.lock.release()  # 解锁
                 # 触发事件，通知其他线程有新数据可用
                # self.lock_event.set()
        # 如果response 是 401，未认证，则重新获取token再发送任务
        elif response.status_code == 401 or response.status_code == 403:
            self.logger.warning("HandleCancelRespone:"+str(response.status_code))
            if retry_count[req_id] < max_retry_count:
                self.logger.warning(f"Retrying to cancel task {task_id} ({retry_count[req_id]}/{max_retry_count})...")
                # 增加重试次数，并等待指定的时间间隔
                retry_count[req_id] += 1
                time.sleep(retry_interval)
                # 重新获取token并重新发送任务
                self.GetToken()
                time.sleep(2)
                self.CancelTask(task_id,req_id)
            else:
                # 达到最大重试次数，放弃处理
                self.logger.warning(f"Reached max retry count ({max_retry_count}), giving up.")
        else :
            self.logger.warning("HandleCancelRespone:"+str(response.status_code))
            # 处理其他响应失败的情况
            if retry_count[req_id] < max_retry_count:
                self.logger.warning(f"Retrying to cancel task {task_id} ({retry_count[req_id]}/{max_retry_count})...")
                # 增加重试次数，并等待指定的时间间隔
                retry_count[req_id] += 1
                time.sleep(retry_interval)
                # 重新发送任务
                self.CancelTask(task_id,req_id)
            else:
                # 达到最大重试次数，放弃处理
                self.logger.warning(f"Reached max retry count ({max_retry_count}), giving up.")

    def HandleCreateRespone(self,response,task_data,req_id,retry_count, max_retry_count=3, retry_interval=2):
        if req_id not in retry_count:
            retry_count[req_id] = 0
        if response.status_code == 200:
            self.logger.info("HandleCreateRespone:"+str(response.status_code))
            json_data = response.json()
            task_id = json_data["state"]["booking"]["id"]
            self.PickList.append(task_id)
            # self.lock.acquire()  # 加锁
            # self.queue.put(self.PickList)  # 将新数组添加到队列中
            # self.lock.release()  # 解锁
                # 触发事件，通知其他线程有新数据可用
            # self.lock_event.set()

            # self.current_task_num = self.current_task_num + 1
       # 如果response 是 401，未认证，则重新获取token再发送任务
        elif response.status_code == 401 or response.status_code == 403:
            self.logger.warning("HandleCreateRespone:"+str(response.status_code))
            if retry_count[req_id] < max_retry_count:
                self.logger.warning(f"Retrying to Create task request {req_id} ({retry_count[req_id]}/{max_retry_count})...")
                # 增加重试次数，并等待指定的时间间隔
                retry_count[req_id] += 1
                time.sleep(retry_interval)
                # 重新获取token并重新发送任务
                self.GetToken()
                time.sleep(2)
                self.SendTask(task_data,req_id)
            else:
                # 达到最大重试次数，放弃处理
                self.logger.warning(f"Reached max retry count ({max_retry_count}), giving up.")
        else :
            self.logger.warning("HandleCreateRespone:"+str(response.status_code))
            # 处理其他响应失败的情况
            if retry_count[req_id] < max_retry_count:
                self.logger.warning(f"Retrying to Create task request {req_id} ({retry_count[req_id]}/{max_retry_count})...")
                # 增加重试次数，并等待指定的时间间隔
                retry_count[req_id] += 1
                time.sleep(retry_interval)
                # 重新发送任务
                self.SendTask(task_data,req_id)
            else:
                # 达到最大重试次数，放弃处理
                self.logger.warning(f"Reached max retry count ({max_retry_count}), giving up.")
                

    def HandleGetTaskStateRespone(self,response,task_id):
        if response.status_code == 200:
            self.logger.info("HandleGetTaskStateRespone:"+str(response.status_code))
            pass
        # 如果response 是 401，未认证，则重新获取token再发送任务
        elif response.status_code == 401 or response.status_code == 403:
            self.logger.warning("HandleGetTaskStateRespone:"+str(response.status_code))
            # 认证失败，重新获取token
            self.GetToken()
            time.sleep(2)
            #    
        else :
            self.logger.warning("HandleGetTaskStateRespone:"+str(response.status_code))
            # response_json = response.json()
            # error = response_json['error']
            # self.logger.warning(f'请求失败：{response.status_code},{error}')
            # 

    # 登录，获取token
    def GetToken(self):
        self.logger.info("获取token")
        url = 'http://localhost:8080/realms/rmf/protocol/openid-connect/token'

        data = {
            'grant_type': 'password',
            'client_id': 'myclient',
            'username': 'admin',
            'password': 'admin'
        }
        headers = {
            'Content-Type': 'application/x-www-form-urlencoded',
        }
        response = requests.post(url, data=data, headers=headers)

        if response.status_code == 200:
            response_json = response.json()
            self.token = response_json['access_token']
            self.fleet_state_getter.token = self.token
            # self.logger.info('access_token:', self.token)
        else:
            self.logger.warning('获取token请求失败:', response.status_code)


    # 定义刷新token的函数
    def refresh_token(self):
        self.logger.info("定义刷新token的函数")
        self.refresh_token_stop_flag = False  
        while not self.refresh_token_stop_flag:
            time.sleep(1800)  # 每0.5小时刷新一次token
            self.GetToken()

    # 启动刷新token的线程
    def start_refresh_token(self):
        self.logger.info("启动刷新token的线程")
        self.refresh_thread = threading.Thread(target=self.refresh_token)
        self.refresh_thread.start()

    # 停止刷新token的线程
    def stop_refresh_token(self):
        self.logger.info("停止刷新token的线程")
        if self.refresh_thread is not None:
            self.refresh_token_stop_flag = True


    """ 定时获取当前rmf任务状态
        不断获取当前PickList中各个任务状态
        若任务完成或取消 则从List中删除
        若正在执行或queue则无视
        若异常则报告该任务id 
    """
    def GetTaskState(self):
        self.get_task_state_stop_flag = False
        while not self.get_task_state_stop_flag:
            self.logger.info("GetTaskState")
             # 等待事件的触发
            # self.lock_event.wait()
            # self.lock.acquire()  # 加锁
            # if not self.queue.empty():
                 # 从队列中获取新数组
                # self.PickList = self.queue.get()
            for task_id in self.PickList:
                url = "http://localhost:8000/tasks/"+task_id+"/state"
                headers = {"Authorization": f"Bearer {self.token}"}
                response = requests.get(url=url,headers=headers)
            # self.lock.release()  # 解锁
            # 清除事件，等待下一次触发
            # self.lock_event.clear()
            # 设定时间间隔，等待下一次发送请求
            time.sleep(10)

    def start_get_task_state(self):
        self.get_task_state_thread = threading.Thread(target=self.GetTaskState)
        self.get_task_state_thread.start()
    def stop_get_task_state(self):
        if self.get_task_state_thread is not None:
            self.get_task_state_stop_flag = True


# 启动机器人、rmf 、web
rclpy.init(args=None)
tscript = TestScript()
tscript.logger.info("init testscript")
tscript.StartDropSub()
tscript.logger.info("tscript.StartDropSub()")
# 获取token
tscript.start_refresh_token()
tscript.logger.info("tscript.start_refresh_token()")
# 开始随机发起任务
time.sleep(2)
while tscript.token is None:
    continue
tscript.FleetStateStart()
tscript.logger.info("tscript.FleetStateRun()")
tscript.start_create_task()
tscript.logger.info("tscript.start_create_task()")
tscript.start_cancel_task()
tscript.logger.info("tscript.start_cancel_task()")
tscript.start_get_task_state()
tscript.logger.info("tscript.start_get_task_state()")

while tscript.current_task_num < tscript.total_tasks \
    and (time.time() < tscript.init_time + (tscript.total_run_time*3600)):
    #不满足条件，继续循环
    pass
tscript.logger.info("结束,停止发送任务")
# 结束，停止发送任务
tscript.stop_create_task()
tscript.stop_cancel_task()
tscript.StopDropSub()
tscript.logger.info("tscript.StopDropSub()")
rclpy.shutdown()
tscript.logger.info("rclpy.shutdown()")
tscript.logger.info(".......")
tscript.logger.info("待当前所有任务执行完毕。")
# 待当前所有任务执行完毕。
tscript.FleetStateStop()
tscript.logger.info("tscript.FleetStateStop()")
tscript.stop_get_task_state()
tscript.logger.info("tscript.stop_get_task_state()")
tscript.stop_refresh_token()
tscript.logger.info("tscript.stop_refresh_token()")