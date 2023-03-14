import requests
import threading
import time

"""
记录机器人状态 
"""
class HttpGetter:
    def __init__(self, url, interval=1):
        self.url = url
        self.interval = interval
        self.value = None
        self.event = threading.Event()

        def update_value():
            while not self.event.wait(self.interval):
                response = requests.get(self.url)
                if response.status_code == 200:
                    value = response.json()['value']
                    self.value = value
                    self.event.set()

        self.thread = threading.Thread(target=update_value)
        self.thread.start()

    def get_value(self):
        self.event.wait()
        self.event.clear()
        return self.value

    def stop(self):
        self.event.set()
        self.thread.join()


class MyClass:
    def __init__(self):
        self.http_getter = HttpGetter('http://example.com/api/value')

    def run(self):
        while True:
            value = self.http_getter.get_value()
            print('最新的值为：%s' % value)
            time.sleep(1)  # 每1秒获取一次最新的值

    def stop(self):
        self.http_getter.stop()


def main(args=None):
    my_class = MyClass()
    my_class.run()
    my_class.stop()


if __name__ == '__main__':
    main()

"""
在上述代码中，我们定义了一个名为HttpGetter的类，用于异步获取HTTP接口的最新值。在__init__方法中，我们创建了一个名为update_value的线程函数
以每interval秒的时间间隔更新最新的值。在get_value方法中，我们等待event对象变为已触发状态，然后获取最新的值。

在MyClass类中，我们创建了一个HttpGetter对象，并在run方法中循环调用get_value方法以获取最新的值。
在程序结束时，我们需要先调用MyClass对象的stop方法以停止更新值的线程。 
"""