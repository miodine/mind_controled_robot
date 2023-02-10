#!/usr/bin/env python3


# IMPORTS 

# general 
import sys

# QT
from PyQt5 import QtCore, QtGui, QtWidgets

#ROS 
import rospy

# ROS MCR
from mcr_gui_server import GUIDataServer
from mcr_messages.msg import mcr_intention, mcr_datastream, mcr_predictions,  mcr_control_monit



class Ui_MainWindow(object):

    def setupUi(self, MainWindow):

        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(473, 640)
        MainWindow.setStyleSheet("QWidget{\n"
                                 "    position: relative;\n"
                                 "    background: #3f496c;\n"
                                 "    font-family: \'Poppins\', sans-serif;\n"
                                 "    border-radius: 10px;\n"
                                 "    overflow: hidden;\n"
                                 "    box-shadow: 0 0 20px rgba(0, 0, 0, 0.2);\n"
                                 "\n"
                                 "}")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame.setObjectName("frame")
        self.header = QtWidgets.QLabel(self.frame)
        self.header.setGeometry(QtCore.QRect(30, 120, 221, 71))
        self.header.setStyleSheet("#header{\n"
                                  "    font-size: 20px;\n"
                                  "\n"
                                  "}")
        self.header.setTextFormat(QtCore.Qt.TextFormat.AutoText)
        self.header.setObjectName("header")
        self.verticalLayout.addWidget(self.frame)
        self.frame_2 = QtWidgets.QFrame(self.centralwidget)
        self.frame_2.setStyleSheet("#frame_2{\n"
                                   "background: qlineargradient(spread:pad, x1:0.960227, y1:0.528, x2:0, y2:0.522727, stop:0 rgba(63, 73, 108, 255), stop:1 rgba(115, 127, 170, 255));\n"
                                   "margin: 20px 20px 20px 20px;\n"
                                   "}")
        self.frame_2.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_2.setObjectName("frame_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.frame_2)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.frame_5 = QtWidgets.QFrame(self.frame_2)
        self.frame_5.setStyleSheet("#frame_5{\n"
                                   "    background: none;\n"
                                   "}")
        self.frame_5.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_5.setObjectName("frame_5")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.frame_5)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.frame_6 = QtWidgets.QFrame(self.frame_5)
        self.frame_6.setStyleSheet("#frame_6{\n"
                                   "    background: none;\n"
                                   "}")
        self.frame_6.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_6.setObjectName("frame_6")
        self.prediction = QtWidgets.QLabel(self.frame_6)
        self.prediction.setGeometry(QtCore.QRect(60, 0, 49, 16))
        self.prediction.setStyleSheet("#prediction {\n"
                                      "    background: none;\n"
                                      "}")
        self.prediction.setObjectName("prediction")
        self.command = QtWidgets.QLabel(self.frame_6)
        self.command.setGeometry(QtCore.QRect(60, 30, 49, 16))
        self.command.setStyleSheet("#command {\n"
                                   "    background: none;\n"
                                   "}")
        self.command.setObjectName("command")
        self.verticalLayout_2.addWidget(self.frame_6)
        self.frame_7 = QtWidgets.QFrame(self.frame_5)
        self.frame_7.setStyleSheet("#frame_7{\n"
                                   "    background: none;\n"
                                   "}")
        self.frame_7.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_7.setObjectName("frame_7")
        self.velocity = QtWidgets.QLabel(self.frame_7)
        self.velocity.setGeometry(QtCore.QRect(60, 10, 49, 16))
        self.velocity.setStyleSheet("#velocity {\n"
                                    "    background: none;\n"
                                    "}")
        self.velocity.setObjectName("velocity")
        self.verticalLayout_2.addWidget(self.frame_7)
        self.horizontalLayout.addWidget(self.frame_5)
        self.frame_4 = QtWidgets.QFrame(self.frame_2)
        self.frame_4.setStyleSheet("#frame_4{\n"
                                   "    background: none;\n"
                                   "}")
        self.frame_4.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_4.setObjectName("frame_4")
        self.horizontalLayout.addWidget(self.frame_4)
        self.verticalLayout.addWidget(self.frame_2)
        self.frame_3 = QtWidgets.QFrame(self.centralwidget)
        self.frame_3.setStyleSheet("#frame_3{\n"
                                   "background: qlineargradient(spread:pad, x1:0.960227, y1:0.528, x2:0, y2:0.522727, stop:0 rgba(63, 73, 108, 255), stop:1 rgba(115, 127, 170, 255));\n"
                                   "margin: 0px 20px 40px 20px;\n"
                                   "}")
        self.frame_3.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_3.setObjectName("frame_3")
        self.graph = QtWidgets.QGraphicsView(self.frame_3)
        self.graph.setGeometry(QtCore.QRect(160, 0, 271, 161))
        self.graph.setStyleSheet("#graph{\n"
                                 "    background: qlineargradient(spread:pad, x1:0.960227, y1:0.528, x2:0, y2:0.522727, stop:0 rgba(63, 73, 108, 255), stop:1 rgba(115, 127, 170, 255));\n"
                                 "}")
        self.graph.setObjectName("graph")
        self.verticalLayout.addWidget(self.frame_3)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.header.setText(_translate("MainWindow", "Mind Controlled Robot"))
        self.prediction.setText(_translate("MainWindow", "TextLabel"))
        self.command.setText(_translate("MainWindow", "TextLabel"))
        self.velocity.setText(_translate("MainWindow", "TextLabel"))

def main():

    # initialize node 
    rospy.init_node('mcr_gui', anonymous=True)

    # initialize UI
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)

    # initialize data server
    data_handle = GUIDataServer(ui)

    # schedule periodic ui update
    rospy.Timer(rospy.Duration(0.5), data_handle.update_ui)


    # run the UI
    MainWindow.show()
    sys.exit(app.exec())

    

    




if __name__ == "__main__":
    main()