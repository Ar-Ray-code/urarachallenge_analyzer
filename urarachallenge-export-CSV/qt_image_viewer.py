import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QProgressBar, QSlider
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import Qt

import argparse
import os
import csv

class App(QWidget):

    def __init__(self):
        super().__init__()
        # get argment
        parser = argparse.ArgumentParser()
        parser.add_argument('-i', '--image-cache-path', help='image folder path')
        parser.add_argument('-c', '--csv-folder', help='csv file folder path')
        args = parser.parse_args()
        self.image_path = args.image_cache_path
        self.csv_path = args.csv_folder

        # not args
        if self.image_path is None:
            # exit
            sys.exit(0)

        # get image length
        self.image_length = len(os.listdir(self.image_path))
        print("image length:", self.image_length)
        
        self.title = 'Urarachallenge'
        self.left = 10
        self.top = 10
        self.width = 640
        self.height = 360
        self.image_count = 1

        # initialize point_list_xy by image_length definition
        self.point_list_xy = []
        for i in range(self.image_length):
            self.point_list_xy.append([-1,-1])


        self.initUI()
        self.load_image(self.get_image_path(self.image_path, self.image_count))
    
    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        # show empty
        self.label = QLabel(self)
        self.label.move(0,0)
        self.label.resize(self.width, self.height)

        # put slider
        self.slider = QSlider(QtCore.Qt.Horizontal, self)
        self.slider.setGeometry(0, self.height - 20, self.width, 20)

        # slider max value
        self.slider.setMaximum(self.image_length)
        self.slider.setMinimum(1)

        self.slider.setValue(self.image_count)

        # define keypress event
        self.setFocusPolicy(Qt.StrongFocus)
                
        self.show()
    
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            print("Escape key is pressed")
        
        if event.key() == Qt.Key_Space:
            # self.load_image(self.image_path)
            print('skip')
            self.image_count += 1
            self.load_image(self.get_image_path(self.image_path, self.image_count))
        
        if event.key() == Qt.Key_Left:
            if (self.image_count > 1):
                self.image_count -= 1
                self.load_image(self.get_image_path(self.image_path, self.image_count))
        
        if event.key() == Qt.Key_Right:
            if (self.image_count < self.image_length):
                self.image_count += 1
                self.load_image(self.get_image_path(self.image_path, self.image_count))

    # click event
    def mousePressEvent(self, event):
        print("Mouse is pressed")
        print(event.x())
        print(event.y())

        self.point_list_xy[self.image_count-1][0] = event.x()
        self.point_list_xy[self.image_count-1][1] = event.y()

        if self.image_count < self.image_length:
            self.image_count += 1
            self.load_image(self.get_image_path(self.image_path, self.image_count))
        
    def load_image(self, image_path):
        self.slider.setValue(self.image_count)
        
        pixmap = QPixmap(image_path)
        pixmap = pixmap.scaled(self.label.size(), QtCore.Qt.KeepAspectRatio)

        # draw circle on image by point_list_xy
        if self.point_list_xy[self.image_count-1][0] != -1:
            self.draw_circle(self.point_list_xy[self.image_count-1][0], self.point_list_xy[self.image_count-1][1], pixmap)
            return None
        
        else:
            self.label.setPixmap(pixmap)
            self.resize(pixmap.width(),pixmap.height())

        self.show()

    def draw_circle(self, x, y, pixmap):
        # draw circle
        painter = QtGui.QPainter(pixmap)
        painter.setBrush(Qt.red)
        painter.drawEllipse(x-5, y-5, 10, 10)
        
        # return pixmap
        self.label.setPixmap(pixmap)
        self.resize(pixmap.width(),pixmap.height())
        self.show()

    def get_image_path(self, folder_path, count) -> str:
        # 001, 002, ...
        return folder_path + "/image_" + str(count).zfill(3) + '.png'

    # close event
    def closeEvent(self, event):
        print("close")
        # save point_list_xy
        with open(self.csv_path + '/point_list_xy.csv', 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(self.point_list_xy)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())

