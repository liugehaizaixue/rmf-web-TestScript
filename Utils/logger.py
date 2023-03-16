import os
import logging
import threading

class LoggingManager:
    # 添加一个静态的锁对象，用于保证并发写入同一个文件时只有一个实例在写入
    file_lock = threading.Lock()

    def __init__(self, logger_name, log_file=None, level=logging.INFO):
        self.logger = logging.getLogger(logger_name)
        self.logger.setLevel(level)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

        # 添加控制台处理器
        ch = logging.StreamHandler()
        ch.setFormatter(formatter)
        self.logger.addHandler(ch)

        # 添加文件处理器
        if log_file:
            # 获取文件夹路径
            log_dir = os.path.dirname(log_file)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir)
            fh = logging.FileHandler(log_file)
            fh.setFormatter(formatter)
            self.logger.addHandler(fh)


    def debug(self, msg):
        with self.file_lock:
            self.logger.debug(msg)

    def info(self, msg):
        with self.file_lock:
            self.logger.info(msg)

    def warning(self, msg):
        with self.file_lock:
            self.logger.warning(msg)

    def error(self, msg):
        with self.file_lock:
            self.logger.error(msg)

    def critical(self, msg):
        with self.file_lock:
            self.logger.critical(msg)
