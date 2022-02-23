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
        args = parser.parse_args()

        self.file_flag = args.file
        self.entry = args.entry

        self.create_widgets()

    def create_widgets(self):

        if(self.file_flag==True):
            selected_usb_device = QFileDialog.getOpenFileName(self, 'Open file', self.entry)[0]
        else:
            selected_usb_device = QFileDialog.getExistingDirectory(self, 'Open file', self.entry)
        
        print(selected_usb_device)
        sys.exit(0)
        

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = Application()