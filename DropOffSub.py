import threading
from Utils.logger import LoggingManager
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
"""
监听topic,记录机器人发起的dropoff任务 
"""
class DropOffSubscriber(Node):

    def __init__(self):
        self.node = rclpy.create_node("drop_off_subscriber")
        self.subscription = self.node.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription  # 防止subscription被垃圾回收
        # self.msg_event = threading.Event()
        self.dropoff_list = []
    def listener_callback(self, msg):
        # self.get_logger().info('收到的消息: "%s"' % msg.data)
        print(msg)
        # 判断dropoff-id，决定是否加入
        self.dropoff_list.append(msg.data)
        # self.msg_event.set()

    def destroy_subscription(self, subscription):
        self.node.destroy_subscription(subscription)

    def destroy_node(self):
        self.node.destroy_node()

    def get_dropoff(self):
        # self.msg_event.wait()
        # self.msg_event.clear()
        return self.dropoff_list
    

""" 
终端测试命令
ros2 topic pub /chatter std_msgs/String "data: Hello world"
"""