import random 
import requests
import time
import json
import threading
from Utils.logger import Logger
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
"""
监听topic,记录机器人发起的dropoff任务 
"""
class DropOffSubscriber(Node):

    def __init__(self):
        super().__init__('drop_off_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription  # 防止subscription被垃圾回收
        self.msg_event = threading.Event()
        self.dropoff_list = []
    def listener_callback(self, msg):
        self.get_logger().info('收到的消息: "%s"' % msg.data)
        # 判断dropoff-id，决定是否加入
        self.dropoff_list.append(msg.data)
        self.msg_event.set()
    def get_dropoff(self):
        self.msg_event.wait()
        self.msg_event.clear()
        return self.dropoff_list