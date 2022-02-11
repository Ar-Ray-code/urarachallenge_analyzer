import PyQt5
import sys
import os
import cv2
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QFileDialog

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from bboxes_ex_msgs.msg import BoundingBoxes, BoundingBox

from subprocess import Popen, PIPE
import signal

class ExecuteROS2RUN():
    def __init__(self, node_name:str, pkg_name:str, custom_node_name:str=None, args:list=None):
        # self.node_str = ["ros2", "run", pkg_name, custom_node_name]
        self.node_str = ["ros2", "run", pkg_name, node_name]
        self.node_str.extend(args)

    def __del__(self):
        self.close_node()

    def open_node(self):
        self.p = Popen(self.node_str, stdout=PIPE, stderr=PIPE)
        self.p.wait()
    
    def close_node(self):
        self.p.kill()

class ExecuteROS2Launch():
    def __init__(self, launch_file_path:str, args:list=None):
        self.launch_file_path = launch_file_path
        self.launch_str = ["ros2", "launch", self.launch_file_path]
        self.launch_str.extend(args)

    def __del__(self):
        self.delete()

    def start(self):
        self.p = Popen(self.launch_str, stdout=PIPE, stderr=PIPE)
        # self.p.wait()
        # detach process
        self.p.detach()
    
    def delete(self):
        # self.p.kill()
        # Ctrl + C
        os.kill(self.p.pid, signal.SIGINT)

class test_data():
    def __init__(self) -> None:
        self.csv_points_xy = []
        self.target_points_xy = []
    
    def read_csv_point_xy(self, csv_path:str) -> None:
        with open(csv_path, 'r') as f:
            lines = f.readlines()
            lines = lines[1:]
            lines = [line.strip().split(',') for line in lines]
            lines = [[float(x) for x in line] for line in lines]
        self.csv_points_xy = lines
        print(self.csv_points_xy)
        return None
    
    def define_target_points_length(self, length:int) -> None:
        # fill -1
        for i in range(length):
            self.target_points_xy.append([-1, -1])
        return None
    

# ========================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.test_data_class = test_data()

        # ROS2 init ============================================================
        rclpy.init(args=None)
        self.node = Node('button_sub')
        self.cv_bridge = CvBridge()

        self.image_pub = self.node.create_publisher(Image, 'image', 10)
        self.sub = self.node.create_subscription(BoundingBoxes, 'bboxes', self.sub_bboxes, 10)

        # Qt init ============================================================

        self.frame_number = 1 # publish image frame number
        
        self.subscribe_ros2_flag = False

        self.number = 0

        self.title = 'PyQt test(Subscriber)'
        self.width = 400
        self.height = 200
        self.setWindowTitle(self.title)
        self.setGeometry(0, 0, self.width, self.height)
        
        self.create_widgets()

        # select csv file path
        home = os.path.expanduser('~')
        self.csv_file_path = QFileDialog.getOpenFileName(self, 'Open CSV file', home)[0]
        print(self.csv_file_path)
        # select target video file path
        self.video_file_path = QFileDialog.getOpenFileName(self, 'Open Targegt Video file', home)[0]
        print(self.video_file_path)

        self.test_data_class.read_csv_point_xy(self.csv_file_path)

        # create cache
        self.create_target_images()

        # start YOLOv5-ROS
        # get this script path
        self.script_path = os.path.dirname(os.path.abspath(__file__))
        self.target_detection = ExecuteROS2Launch(self.script_path + "/yolov5s.launch.py")
        self.target_detection.start()

        self.subscribe_ros2_flag = True


        # spin once
        rclpy.spin_once(self.node)

    def __del__(self):
        self.target_detection.delete()
        self.node.destroy_node()

    # Qt ============================================================
    def create_widgets(self):
        # create label
        self.label = QLabel(self)
        self.label.setText(str(self.number))
        self.label.move(100, 100)
        self.show()

        # create timer
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.timer_update)
        self.timer.start(10)
    
    # Qt timer update (subscribe)
    def timer_update(self):
        if self.subscribe_ros2_flag:
            rclpy.spin_once(self.node)
        
        # show
        self.update_label()
        self.show()
        # update after 1 second
        self.timer.start(10)

    def update_label(self):
        self.label.setText(str(self.number))
        # show
        self.show()
    
    # ROS2 ============================================================
    def publish_image(self):
        image = cv2.imread(self.video_file_path + "/frame%d.jpg" % self.frame_number)
        # get image
        self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8"))
        # spin once
        rclpy.spin_once(self.node)

    def sub_bboxes(self, msg:BoundingBoxes):
        # self.number = msg.data
        # rclpy.spin_once(self.node)
        # print(self.number)
        print(msg.xmin)
        self.update_label()
    


    # Tools ============================================================
    def create_target_images(self):
        cache_dir = self.video_file_path + "-cache"
        os.makedirs(cache_dir, exist_ok=True)

        # ffmpeg -i $VIDO_PATH  -vcodec png -r 2 -vf scale=640:360 $CACHE_PATH/image_%03d.png using popen
        process_ffmpeg = Popen(["ffmpeg", "-i", self.video_file_path, "-vcodec", "png", "-r", "2", "-vf", "scale=640:360", cache_dir + "/image_%03d.png"], stdout=PIPE, stderr=PIPE)
        # wait for process
        process_ffmpeg.wait()
        

        # get cache folder length
        self.cache_dir_length = len(os.listdir(cache_dir))

        # delete images if cache_length > csv_length
        if self.cache_dir_length > len(self.test_data_class.csv_points_xy):
            gap = self.cache_dir_length - len(self.test_data_class.csv_points_xy)
            print("cache images: " + str(self.cache_dir_length))
            print("csv points: " + str(len(self.test_data_class.csv_points_xy)))
            print("delete images: " + str(gap))
            
            for i in range(gap):
                os.remove(cache_dir + "/image_%03d.png" % (i+1))
            # rename 
            for i in range(len(self.test_data_class.csv_points_xy)):
                os.rename(cache_dir + "/image_%03d.png" % (i+1 + gap), cache_dir + "/image_%03d.png" % (i+1))

        # add images if cache_length < csv_length
        elif self.cache_dir_length < len(self.test_data_class.csv_points_xy):
            gap = len(self.test_data_class.csv_points_xy) - self.cache_dir_length
            print("cache images: " + str(self.cache_dir_length))
            print("csv points: " + str(len(self.test_data_class.csv_points_xy)))
            print("add images: " + str(gap))

            for i in range(self.cache_dir_length):
                os.rename(cache_dir + "/image_%03d.png" % (self.cache_dir_length - i), cache_dir + "/image_%03d.png" % (self.cache_dir_length - i + gap))
            # add none image
            for i in range(gap):
                image = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.imwrite(cache_dir + "/image_%03d.png" % (i+1), image)
        
        else:
            print("cache images: " + str(self.cache_dir_length))
            print("csv points: " + str(len(self.test_data_class.csv_points_xy)))
            print("no need to add or delete images")
        
        
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())