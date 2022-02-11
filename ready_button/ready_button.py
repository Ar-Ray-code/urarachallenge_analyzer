#!/bin/python3
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
import sys

class ready_button(QWidget):
    def __init__(self):
        super().__init__()
        self.title = 'BTN'
        self.left = 10
        self.top = 10
        self.width = 150
        self.height = 100
        self.initUI()
    
    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        # create button
        self.button = QPushButton('Ready', self)
        self.button.setToolTip('Ready')
        self.button.move(30, 20)
        self.button.clicked.connect(self.on_click)
        self.show()
    
    def on_click(self):
        print("1")
        sys.exit(0)
    
    # close button
    def closeEvent(self, event):
        print("0")
        sys.exit(0)
    
if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ready_button()
    sys.exit(app.exec_())
