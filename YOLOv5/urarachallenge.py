from socket import timeout
import PyQt5
import sys
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from bboxes_ex_msgs.msg import BoundingBoxes, BoundingBox

from subprocess import Popen, PIPE
import argparse

class test_data():
    def __init__(self) -> None:
        self.csv_points_xy = []
        self.target_points_xy = []
        self.detection_result = []
        self.raw_bbox_ex_msg = []

    def read_csv_point_xy(self, csv_path:str) -> None:
        with open(csv_path, 'r') as f:
            lines = f.readlines()
            lines = lines[1:]
            lines = [line.strip().split(',') for line in lines]
            lines = [[float(x) for x in line] for line in lines]
        self.csv_points_xy = lines
        # print(self.csv_points_xy)
        return None

    def define_target_points_length(self, length:int) -> None:
        # fill -1
        for i in range(length):
            self.target_points_xy.append([-1, -1])
            self.detection_result.append("None")

        # length
        print("target_points_xy:", len(self.target_points_xy))
        return None


# ========================================================
class urarachallenge_main(Node):
    def __init__(self):
        super().__init__('urarachallenge')

        # select csv file path
        home = os.path.expanduser('~')
        # script dir
        script_dir = os.path.dirname(os.path.abspath(__file__))

        self.test_data_class = test_data()

        # arg (csv path)
        parser = argparse.ArgumentParser()
        parser.add_argument("--csv_path", type=str, default="")
        parser.add_argument("--video_folder", type=str, default=home + "/video")
        parser.add_argument("--video_file", type=str, default=home + "")
        args = parser.parse_args()
        self.csv_file_path = args.csv_path
        self.video_folder = args.video_folder
        self.video_file_path = args.video_file

        # ROS2 init =============================================
        self.cv_bridge = CvBridge()

        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.sub = self.create_subscription(BoundingBoxes, '/yolov5/bounding_boxes', self.sub_bboxes, 10)

        # Qt init ============================================================
        self.frame_number = 1 # publish image frame number

        # get path using select-folder-file-dialog.py
        if self.csv_file_path == "":
            self.csv_file_path = Popen(["python3", script_dir + "/select-folder-file-dialog/select-folder-file-dialog.py", "-f", "-t", "Open CSV", '-e', home], stdout=PIPE).communicate()[0].decode('utf-8').rstrip()
        print(self.csv_file_path)
        # select target video file path
        if self.video_file_path == "":
            self.video_file_path = Popen(["python3", script_dir + "/select-folder-file-dialog/select-folder-file-dialog.py", "-f", "-t", "Open Video", '-e', self.video_folder], stdout=PIPE).communicate()[0].decode('utf-8').rstrip()
        print(self.video_file_path)

        self.test_data_class.read_csv_point_xy(self.csv_file_path)

        # create cache
        self.test_data_class.define_target_points_length(len(self.test_data_class.csv_points_xy))
        self.create_target_images()

        self.publish_image()

    def __del__(self):
        pass

    # ROS2 ============================================================
    def publish_image(self):
        frame_name = str(self.frame_number).zfill(3)
        print_data = self.cache_dir + "/image_" + frame_name + ".png"
        print(print_data)

        image = cv2.imread(self.cache_dir + "/image_" + frame_name + ".png")
        # get image
        self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8"))

    def sub_bboxes(self, msg:BoundingBoxes):

        self.raw_bbox_ex_msg = msg

        csv_x = self.test_data_class.csv_points_xy[self.frame_number - 1][0]
        csv_y = self.test_data_class.csv_points_xy[self.frame_number - 1][1]

        # search bbox label in csv file
        for box in msg.bounding_boxes:
            if box.xmin < csv_x and box.xmax > csv_x and box.ymin < csv_y and box.ymax > csv_y:
                # print(str(csv_x), "," , str(csv_y) , ":", box.class_id)
                self.test_data_class.target_points_xy[self.frame_number - 1][0] = csv_x
                self.test_data_class.target_points_xy[self.frame_number - 1][1] = csv_y
                self.test_data_class.detection_result[self.frame_number - 1] = box.class_id
                # print(box.class_id)
                break

        self.frame_number += 1

        if self.frame_number > len(self.test_data_class.csv_points_xy):
            self.write_report_csv()
            # delete cache
            import shutil
            shutil.rmtree(self.cache_dir)

            # self.target_detection.delete()
            self.destroy_node()
            sys.exit()

        self.publish_image()

    def write_report_csv(self):
        write_csv_path = self.cache_dir + "/result_csv/report.csv"
        with open(write_csv_path, 'w') as f:
            f.write("frame,x,y\n")
            for i in range(len(self.test_data_class.target_points_xy)-1):
                # print(i, self.test_data_class.target_points_xy[i][0], self.test_data_class.target_points_xy[i][1], self.test_data_class.detection_result[i])
                f.write("%d,%f,%f,%s\n" % (i , self.test_data_class.target_points_xy[i][0], self.test_data_class.target_points_xy[i][1], self.test_data_class.detection_result[i]))
        self.plot_data()
        return None

    def plot_data(self):
        person_count = 0
        horse_count = 0
        teddy_count = 0
        kite_count = 0

        # delete list (-1)
        detection_data = []
        for i in range(len(self.test_data_class.detection_result)):
            if self.test_data_class.csv_points_xy[i][0] == -1:
                continue
            else:
                detection_data.append(self.test_data_class.detection_result[i])

        person_count = detection_data.count("person")
        horse_count = detection_data.count("horse")
        teddy_count = detection_data.count("teddy bear")
        kite_count = detection_data.count("kite")
        other = len(detection_data) - person_count - horse_count - teddy_count - kite_count

        print("total:", len(detection_data))
        print("person:", person_count)
        print("horse:", horse_count)
        print("teddy:", teddy_count)
        print("kite:", kite_count)
        print("other:", other)

        # pie chart
        labels = 'person', 'horse', 'teddy bear', 'kite', 'other'
        sizes = [person_count, horse_count, teddy_count, kite_count, other]
        plt.pie(sizes, labels=labels, autopct='%1.1f%%', shadow=False, startangle=90)
        # plt.show()
        # save
        plt.savefig(self.cache_dir + "/result_pichart/" + self.video_file_path.split("/")[-1].split(".")[0] + ".png")
        # save output to txt
        with open(self.cache_dir + "/result_text/" + self.video_file_path.split("/")[-1].split(".")[0] + ".txt", 'w') as f:
            f.write("total: " + str(len(detection_data)) + "\n")
            f.write("person: " + str(person_count) + "\n")
            f.write("horse: " + str(horse_count) + "\n")
            f.write("teddy: " + str(teddy_count) + "\n")
            f.write("kite: " + str(kite_count) + "\n")
            f.write("other: " + str(other) + "\n")

    # Tools ============================================================
    def create_target_images(self):
        self.cache_dir = self.video_file_path + "-cache"
        os.makedirs(self.cache_dir, exist_ok=True)
        os.makedirs(self.cache_dir + "/result_text", exist_ok=True)
        os.makedirs(self.cache_dir + "/result_pichart", exist_ok=True)
        os.makedirs(self.cache_dir + "/result_csv", exist_ok=True)

        print("encoding video to images")
        # ffmpeg -i $VIDO_PATH  -vcodec png -r 2 -vf scale=640:360 $CACHE_PATH/image_%03d.png using popen
        process_ffmpeg = Popen(["ffmpeg", "-i", self.video_file_path, "-vcodec", "png", "-r", "2", "-vf", "scale=640:360", self.cache_dir + "/image_%03d.png"], stdout=PIPE, stderr=PIPE)
        # wait for process
        process_ffmpeg.wait()
        print("encoding video to images done")

        # get cache folder length
        self.cache_dir_length = len(os.listdir(self.cache_dir))

        # delete images if cache_length > csv_length
        if self.cache_dir_length > len(self.test_data_class.csv_points_xy):
            gap = self.cache_dir_length - len(self.test_data_class.csv_points_xy)
            print("cache images: " + str(self.cache_dir_length))
            print("csv points: " + str(len(self.test_data_class.csv_points_xy)))
            print("delete images: " + str(gap))

            for i in range(gap):
                os.remove(self.cache_dir + "/image_%03d.png" % (i+1))
            # rename
            for i in range(len(self.test_data_class.csv_points_xy)):
                os.rename(self.cache_dir + "/image_%03d.png" % (i+1 + gap), self.cache_dir + "/image_%03d.png" % (i+1))

        # add images if cache_length < csv_length
        elif self.cache_dir_length < len(self.test_data_class.csv_points_xy):
            gap = len(self.test_data_class.csv_points_xy) - self.cache_dir_length
            print("cache images: " + str(self.cache_dir_length))
            print("csv points: " + str(len(self.test_data_class.csv_points_xy)))
            print("add images: " + str(gap))

            for i in range(self.cache_dir_length):
                os.rename(self.cache_dir + "/image_%03d.png" % (self.cache_dir_length - i), self.cache_dir + "/image_%03d.png" % (self.cache_dir_length - i + gap))
            # add none image
            for i in range(gap):
                image = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.imwrite(self.cache_dir + "/image_%03d.png" % (i+1), image)

        else:
            print("cache images: " + str(self.cache_dir_length))
            print("csv points: " + str(len(self.test_data_class.csv_points_xy)))
            print("no need to add or delete images")

def ros_main(args = None) -> None:
    rclpy.init(args=args)

    urarachallenge = urarachallenge_main()
    rclpy.spin(urarachallenge)

    urarachallenge.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    ros_main()