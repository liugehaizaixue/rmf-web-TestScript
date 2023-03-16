import requests
import threading
import time
import queue

"""
记录机器人状态 
"""
class FleetState:
    def __init__(self,interval=5,fleet_name="tinyRobot"):
        self.interval = interval
        self.state_value = None
        self.token=None
        self.queue = queue.Queue()
        self.stopped = False  # 添加一个变量，用于表示程序是否应该停止
        self.thread = threading.Thread(target=self.update_state)
        self.thread.start()
        self.fleet_name = fleet_name

    def update_state(self):
        while not self.stopped:  # 判断程序是否应该停止
            # 发送http请求获取FleetState
            if self.token is not None :
                fleet_state_url = "http://localhost:8000/fleets/"+self.fleet_name+"/state"
                headers = {"Authorization": f"Bearer {self.token}"}
                response = requests.get(url=fleet_state_url,headers=headers)
                if response.status_code == 200:
                    json_data = response.json()
                    self.state_value = json_data["robots"]
                    self.queue.put(self.state_value)
            time.sleep(self.interval)

    def get_state(self):
        return self.queue.get()

    def stop(self):
        self.stopped = True  # 设置程序应该停止
        self.queue.put(None)  # 添加一个None值，使得get_state方法能够返回None
        self.thread.join()


# class MyClass:
#     def __init__(self):
#         self.fleet_state_getter = FleetState()
#         self.fleet_state_running = True
#         self.fleet_state_threadd = threading.Thread(target=self.run)

#     def run(self):
#         while self.fleet_state_running:
#             value = self.fleet_state_getter.get_state()
#             if value is None:  # 判断是否应该退出循环
#                 break
#             print('最新的值为：%s' % value)

#     def stop(self):
#         self.fleet_state_running = False
#         self.fleet_state_getter.stop()

#     def start(self):
#         self.fleet_state_thread.start()


# def main(args=None):
#     my_class = MyClass()
#     my_class.start()

#     input("按回车键停止监听：")
#     my_class.stop()

