#!/bin/python3
from PyQt5.QtWidgets import QApplication, QWidget, QFileDialog
import sys
import argparse

class Application(QWidget):
    def __init__(self):
        super().__init__()

        parser = argparse.ArgumentParser(description='Select folder')
        parser.add_argument('-e', '--entry', help='entry point folder')
        parser.add_argument('-f', '--file', help='Using file dialog.', action='store_true')
        parser.add_argument('-t', '--title', help='title of dialog', default='Select folder or file')
        args = parser.parse_args()

        self.file_flag = args.file
        self.entry = args.entry
        self.title = args.title

        self.create_widgets()

    def create_widgets(self):

        if(self.file_flag==True):
            selected_usb_device = QFileDialog.getOpenFileName(self, self.title, self.entry)[0]
        else:
            selected_usb_device = QFileDialog.getExistingDirectory(self, self.title, self.entry)
        
        print(selected_usb_device)
        sys.exit(0)
        

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = Application()