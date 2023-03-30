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
        self.current_pick_num = 0 # 用发起的pickup任务数量 作为结束 标志
        self.dispatch_task_num = 0
        self.robot_task_num = 0
        self.cancel_task_num = 0 # 记录取消任务数量
        self.robots = ["tinyRobot_1","tinyRobot_2","tinyRobot_3","tinyRobot_4","tinyRobot_5","tinyRobot_6","tinyRobot_7","tinyRobot_8","tinyRobot_9","tinyRobot_10"]
        self.GetWaypoints()
        self.PickDict={} #记录发起pickup任务id，每个任务id对应不同的状态{“task_id_1”:"queue",“task_id_2”:"underway",}
        self.DropDict={}
        self.FleetStateDict = {}


        # 线程停止标志位
        self.create_task_stop_flag = False  
        self.cancel_task_stop_flag = False  
        self.refresh_token_stop_flag = False  
        self.get_task_state_stop_flag = False
        self.wait_for_finish = False
        # 失败重试次数标志
        self.retry_count = {}

        # 监听dropoff部分
        self.drop_subscriber = DropOffSubscriber(log_name=log_name)
        self.drop_is_listening = True
        self.drop_sub_thread = threading.Thread(target=self.DropSubThread)

        # 监听FleetState部分
        self.fleet_state_getter = FleetState(log_name=log_name)
        self.fleet_state_running = True
        self.fleet_state_thread = threading.Thread(target=self.FleetStateRun)

        # self.GetToken() # 暂时不需要登录

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
        file_path = './Setting/airport_terminal.building.yaml'
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
                self.CreateTask()
            #  每次随机生成 5-10个任务
            self.logger.info(f"睡眠{interval}生成个{random_tasks_num}任务")
            self.logger.info("current_pick_num:"+str(self.current_pick_num))
            time.sleep(interval)  # 每5-100s刷新创建一轮任务

    
    def CreateTask(self):
         # 随机生成pickup任务类型——指定机器人or不指定
        choice_type = random.choice(self.pick_types)
        # 随机生成pickup任务地点进行提交
        pickup_place = random.choice(self.waypoints)
        unix_millis = int(round(time.time() * 1000))
        if choice_type == "robot_task_request" :
            choice_robot = random.choice(self.robots)
            self.dispatch_task_num = self.dispatch_task_num + 1
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
            self.robot_task_num = self.robot_task_num + 1  
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
        # headers = {"Authorization": f"Bearer {self.token}"}
        payload = task_data["payload"]
        # response = requests.post(url=url, json=payload,headers=headers)
        response = requests.post(url=url, json=payload)
        self.logger.info("send Task : "+req_id)
        self.HandleCreateRespone(response=response,req_id=req_id,task_data=task_data, retry_count=self.retry_count)
        # 机器人根据pick随机生成dropoff

    def start_cancel_task(self):
        self.cancel_task_thread = threading.Thread(target=self.RandomCancel)
        self.cancel_task_thread.start()
    def stop_cancel_task(self):
        if self.cancel_task_thread is not None:
            self.cancel_task_stop_flag = True

    # 随机取消任务
    def RandomCancel(self):
        self.cancel_task_stop_flag = False  
        while not self.cancel_task_stop_flag:
            interval = random.randint(60,180)
            random_tasks_num = random.randint(1,5)
            self.logger.info(f"睡眠{interval}取消个{random_tasks_num}任务")
            time.sleep(interval)  # 每60-180s刷新一轮取消任务
            CancellableList = []  # 中间数组，保存可删除任务的列表
            for key, value in self.PickDict.items():
                if value != 'canceled' and value != 'completed':
                    CancellableList.append(key)
            # 将数组中的元素作为键添加到字典中
            for drop_id in self.drop_subscriber.get_dropoff():
                self.DropDict.setdefault(drop_id, "queue")
            for key, value in self.DropDict.items():
                if value != 'canceled' and value != 'completed':
                    CancellableList.append(key)
            for i in range(random_tasks_num):
                if len(CancellableList) != 0 :
                    task_id = random.choice(CancellableList)
                    self.logger.info(f"申请取消{task_id}任务")
                    req_id = str(uuid.uuid4())
                    self.CancelTask(task_id=task_id,req_id=req_id)
                else:
                    self.logger.info("当前CancellableList队列为空")
            #  每次随机取消1-5个任务
    
    # 发送取消任务请求
    def CancelTask(self,task_id,req_id):
        url = "http://localhost:8000/tasks/cancel_task"
        # headers = {"Authorization": f"Bearer {self.token}"}
        payload={
            "type":"cancel_task_request",
            "task_id":task_id
        }
        # response = requests.post(url=url, json=payload,headers=headers)
        response = requests.post(url=url, json=payload)
        self.logger.info("CancelTask:"+task_id)
        self.HandleCancelRespone(response=response,task_id=task_id,req_id=req_id,retry_count=self.retry_count)
    
    def HandleCancelRespone(self,response,task_id,req_id,retry_count, max_retry_count=3, retry_interval=2):
        if req_id not in retry_count:
            retry_count[req_id] = 0
        if response.status_code == 200:
            # 成功取消任务
            self.logger.info("HandleCancelRespone:"+str(response.status_code))
            self.cancel_task_num = self.cancel_task_num + 1
            if task_id in self.PickDict:
                self.PickDict[task_id]=="canceled"
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
            self.PickDict[task_id] = 'queue'
            self.current_pick_num = self.current_pick_num + 1

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
                

    def HandleGetPickTaskStateRespone(self,response,task_id):
        if response.status_code == 200:
            self.logger.info("GetPickTaskStateRespone:"+str(response.status_code))
            self.HandleGetTaskStateResponse(response,task_id,"pickup")
        else :
            self.logger.warning("GetPickTaskStateRespone:"+str(response.status_code))

    def HandleGetDropTaskStateRespone(self,response,task_id):
        if response.status_code == 200:
            self.logger.info("GetDropTaskStateRespone:"+str(response.status_code))
            self.HandleGetTaskStateResponse(response,task_id,"dropoff")
        else :
            self.logger.warning("GetDropTaskStateRespone:"+str(response.status_code))

    def HandleGetTaskStateResponse(self,response,task_id,task_type):
        # 获取结果并从任务队列中设置状态
        # uninitialized, blocked, error, failed, queued, standby, underway, delayed, skipped, canceled, killed, completed
        response_json = response.json()
        task_status = response_json["status"]
        if task_type =="pickup":
            self.PickDict[task_id]=task_status
        else :
            self.DropDict[task_id]=task_status
    
    def HandleGetTaskState(self):
        #判断所有pick与drop任务是否都是'canceled', 'completed'
        #如果满足要求则设置self.wait_for_finish = True
        # 判断字典中的值是否都是 target_values 中的值

        # 要判断的值
        target_values = ['canceled', 'completed']
        if all(value in target_values for value in self.PickDict.values()) and all(value in target_values for value in self.DropDict.values()):
            print('所有任务都是', target_values)
            self.wait_for_finish = True

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
        不断获取当前各个任务状态
    """
    def StartGetTaskState(self):
        self.get_task_state_stop_flag = False
        while not self.get_task_state_stop_flag:
            self.logger.info("StartGetTaskState")
            self.GetTaskState()
            self.HandleGetTaskState()
            time.sleep(10)

    def GetTaskState(self):
        PickList = []  # 中间数组，保存未结束任务的列表
        for key, value in self.PickDict.items():
            if value != 'canceled' and value != 'completed':
                PickList.append(key)

        for task_id in PickList:
            url = "http://localhost:8000/tasks/"+task_id+"/state"
            # headers = {"Authorization": f"Bearer {self.token}"}
            # response = requests.get(url=url,headers=headers)
            response = requests.get(url=url)
            self.HandleGetPickTaskStateRespone(response,task_id)
            time.sleep(0.05)
        

        # 将数组中的元素作为键添加到字典中
        for drop_id in self.drop_subscriber.get_dropoff():
            self.DropDict.setdefault(drop_id, "queue")
        DropList = [] # 中间数组，保存未结束任务的列表
        for key, value in self.DropDict.items():
            if value != 'canceled' and value != 'completed':
                DropList.append(key)

        for task_id in DropList:
            url = "http://localhost:8000/tasks/"+task_id+"/state"
            # headers = {"Authorization": f"Bearer {self.token}"}
            # response = requests.get(url=url,headers=headers)
            response = requests.get(url=url)
            self.HandleGetDropTaskStateRespone(response,task_id)
            time.sleep(0.05)

    def start_get_task_state(self):
        self.get_task_state_thread = threading.Thread(target=self.StartGetTaskState)
        self.get_task_state_thread.start()
    def stop_get_task_state(self):
        if self.get_task_state_thread is not None:
            self.get_task_state_stop_flag = True



start_time = time.time() #程序开始时间
# 启动机器人、rmf 、web
rclpy.init(args=None)
tscript = TestScript()
tscript.logger.info("init testscript")
tscript.StartDropSub()
tscript.logger.info("tscript.StartDropSub()")
# 获取token
# tscript.start_refresh_token()
# tscript.logger.info("tscript.start_refresh_token()")
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

while tscript.current_pick_num < tscript.total_tasks \
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

stop_time = time.time() # 停止发送任务时间
tscript.logger.info("待当前所有任务执行完毕。")
# 待当前所有任务执行完毕，或抛出异常
while tscript.wait_for_finish == False :
    pass

tscript.logger.info(".......")
tscript.logger.info("当前所有任务已执行完毕。")
tscript.FleetStateStop()
tscript.logger.info("tscript.FleetStateStop()")
tscript.stop_get_task_state()
tscript.logger.info("tscript.stop_get_task_state()")
tscript.stop_refresh_token()
tscript.logger.info("tscript.stop_refresh_token()")

end_time = time.time() # 程序结束时间
run_time = end_time - start_time
drop_num = len(tscript.DropDict)

total_num = drop_num + tscript.current_pick_num + tscript.cancel_task_num 
tscript.logger.info(f"本次共执行{total_num}个任务，其中，"
                    f"pickup任务{tscript.current_pick_num}个，pickup任务中有"
                        f"{tscript.dispatch_task_num}个dispatch任务，"
                        f"{tscript.robot_task_num}个robot任务"
                    f"dropoff任务{drop_num}个"
                    f"cancel任务{tscript.cancel_task_num}个"
                    f"此次脚本开始时间{start_time}"
                    f"停止发送任务时间{stop_time}"
                    f"程序结束时间{end_time}"
                    f"总运行时间{run_time}")