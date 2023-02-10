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
        MainWindow.resize(460, 624)
        MainWindow.setStyleSheet("QWidget{\n"
"    position: relative;\n"
"    background: rgba(63,73,108,1);\n"
"    font-family: \'Poppins\', sans-serif;\n"
"    border-radius: 10px;\n"
"    overflow: hidden;\n"
"    box-shadow: 0 0 20px rgba(0, 0, 0, 0.2);\n"
"\n"
"\n"
"}")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem)
        self.frame_8 = QtWidgets.QFrame(self.frame)
        self.frame_8.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_8.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_8.setObjectName("frame_8")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.frame_8)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_5 = QtWidgets.QLabel(self.frame_8)
        self.label_5.setStyleSheet("#label_5 {\n"
"    font:  large \"Verdana\";\n"
"font-size: 20px;\n"
"color: #dae3ff\n"
"\n"
"\n"
"}")
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_4.addWidget(self.label_5)
        self.verticalLayout_3.addWidget(self.frame_8)
        self.label_4 = QtWidgets.QLabel(self.frame)
        self.label_4.setStyleSheet("#label_4 {\n"
"    font:  large \"Verdana\";\n"
"color: #dae3ff\n"
"}")
        self.label_4.setObjectName("label_4")
        self.verticalLayout_3.addWidget(self.label_4)
        self.verticalLayout.addWidget(self.frame)
        self.frame_2 = QtWidgets.QFrame(self.centralwidget)
        self.frame_2.setMinimumSize(QtCore.QSize(442, 146))
        self.frame_2.setMaximumSize(QtCore.QSize(442, 146))
        self.frame_2.setStyleSheet("#frame_2{\n"
"background: qlineargradient(spread:pad, x1:0.960227, y1:0.528, x2:0, y2:0.522727, stop:0 rgba(63, 73, 108, 255), stop:1 rgba(115, 127, 170, 255));\n"
"margin: 20px 20px 20px 20px;\n"
"}")
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.frame_2)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.frame_5 = QtWidgets.QFrame(self.frame_2)
        self.frame_5.setMinimumSize(QtCore.QSize(189, 0))
        self.frame_5.setStyleSheet("#frame_5{\n"
"    background: none;\n"
"}")
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.frame_5)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.frame_6 = QtWidgets.QFrame(self.frame_5)
        self.frame_6.setStyleSheet("#frame_6{\n"
"    background: none;\n"
"\n"
"}")
        self.frame_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.frame_6)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label = QtWidgets.QLabel(self.frame_6)
        self.label.setStyleSheet("#label {\n"
"    background: none;\n"
"    font:  large \"Verdana\";\n"
"    color: #dae3ff;\n"
"}")
        self.label.setObjectName("label")
        self.horizontalLayout_2.addWidget(self.label)
        self.command = QtWidgets.QLabel(self.frame_6)
        self.command.setStyleSheet("#command {\n"
"    background: none;\n"
"    font:  large \"Verdana\";\n"
"    color: #dae3ff;\n"
"}")
        self.command.setObjectName("command")
        self.horizontalLayout_2.addWidget(self.command)
        self.verticalLayout_2.addWidget(self.frame_6)
        self.frame_7 = QtWidgets.QFrame(self.frame_5)
        self.frame_7.setStyleSheet("#frame_7{\n"
"    background: none;\n"
"}")
        self.frame_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_7.setObjectName("frame_7")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.frame_7)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_2 = QtWidgets.QLabel(self.frame_7)
        self.label_2.setStyleSheet("#label_2 {\n"
"    background: none;\n"
"    font:  large \"Verdana\";\n"
"    color: #dae3ff;\n"
"}")
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_3.addWidget(self.label_2)
        self.prediction = QtWidgets.QLabel(self.frame_7)
        self.prediction.setStyleSheet("#prediction {\n"
"    background: none;\n"
"    font:  large \"Verdana\";\n"
"    color: #dae3ff;\n"
"}")
        self.prediction.setObjectName("prediction")
        self.horizontalLayout_3.addWidget(self.prediction)
        self.verticalLayout_2.addWidget(self.frame_7)
        self.horizontalLayout.addWidget(self.frame_5)
        self.frame_4 = QtWidgets.QFrame(self.frame_2)
        self.frame_4.setStyleSheet("#frame_4{\n"
"    background: none;\n"
"}")
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.frame_4)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_6 = QtWidgets.QLabel(self.frame_4)
        font = QtGui.QFont()
        font.setPointSize(7)
        font.setBold(False)
        font.setItalic(False)
        self.label_6.setFont(font)
        self.label_6.setStyleSheet("#label_6 {\n"
"    background: none;\n"
"    font:  large \"Verdana\";\n"
"    color: #dae3ff;\n"
"}")
        self.label_6.setObjectName("label_6")
        self.verticalLayout_4.addWidget(self.label_6)
        self.confidence = QtWidgets.QProgressBar(self.frame_4)
        self.confidence.setMinimumSize(QtCore.QSize(0, 30))
        self.confidence.setStyleSheet("QProgressBar {\n"
"\n"
"    border-style: solid;\n"
"    border-color: #dae3ff;\n"
"    border-radius: 7px;\n"
"    border-width: 2px;\n"
"    text-align:center;\n"
"\n"
"    font:  large \"Verdana\";\n"
"    color: #dae3ff;\n"
"\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"width: 2px;\n"
"background-color : #ee2244;\n"
"margin: 1px\n"
"}")
        self.confidence.setProperty("value", 24)
        self.confidence.setObjectName("confidence")
        self.verticalLayout_4.addWidget(self.confidence)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem1)
        self.frame_9 = QtWidgets.QFrame(self.frame_4)
        self.frame_9.setEnabled(True)
        self.frame_9.setMinimumSize(QtCore.QSize(0, 28))
        self.frame_9.setStyleSheet("#frame_9{\n"
"    background: none;\n"
"}")
        self.frame_9.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_9.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_9.setObjectName("frame_9")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.frame_9)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_7 = QtWidgets.QLabel(self.frame_9)
        font = QtGui.QFont()
        font.setPointSize(7)
        font.setBold(False)
        font.setItalic(False)
        self.label_7.setFont(font)
        self.label_7.setStyleSheet("#label_7 {\n"
"    background: none;\n"
"    font:  large \"Verdana\";\n"
"    color: #dae3ff;\n"
"}")
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_6.addWidget(self.label_7)
        self.accuracy = QtWidgets.QLabel(self.frame_9)
        font = QtGui.QFont()
        font.setPointSize(7)
        font.setBold(False)
        font.setItalic(False)
        self.accuracy.setFont(font)
        self.accuracy.setStyleSheet("#accuracy {\n"
"    background: none;\n"
"    font:  large \"Verdana\";\n"
"    color: #dae3ff;\n"
"}")
        self.accuracy.setObjectName("accuracy")
        self.horizontalLayout_6.addWidget(self.accuracy)
        self.verticalLayout_4.addWidget(self.frame_9)
        self.horizontalLayout.addWidget(self.frame_4)
        self.verticalLayout.addWidget(self.frame_2)
        self.frame_3 = QtWidgets.QFrame(self.centralwidget)
        self.frame_3.setMinimumSize(QtCore.QSize(442, 250))
        self.frame_3.setMaximumSize(QtCore.QSize(442, 250))
        self.frame_3.setStyleSheet("#frame_3{\n"
"background: qlineargradient(spread:pad, x1:0.960227, y1:0.528, x2:0, y2:0.522727, stop:0 rgba(63, 73, 108, 255), stop:1 rgba(115, 127, 170, 255));\n"
"margin: 0px 20px 40px 20px;\n"
"}")
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.frame_3)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_3 = QtWidgets.QLabel(self.frame_3)
        font = QtGui.QFont()
        font.setFamily("Verdana")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        self.label_3.setFont(font)
        self.label_3.setStyleSheet("#label_3 {\n"
"    background: none;\n"
"    font:  large \"Verdana\";\n"
"    color: #dae3ff;\n"
"}")
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_5.addWidget(self.label_3)
        self.graphicsView = QtWidgets.QGraphicsView(self.frame_3)
        self.graphicsView.setStyleSheet("#graphicsView{\n"
"    background: qlineargradient(spread:pad, x1:0.960227, y1:0.528, x2:0, y2:0.522727, stop:0 rgba(63, 73, 108, 255), stop:1 rgba(115, 127, 170, 255));\n"
"}")
        self.graphicsView.setObjectName("graphicsView")
        self.horizontalLayout_5.addWidget(self.graphicsView)
        self.verticalLayout.addWidget(self.frame_3)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_5.setText(_translate("MainWindow", "Mind Controlled Robot"))
        self.label_4.setText(_translate("MainWindow", "   ewelian x miodine"))
        self.label.setText(_translate("MainWindow", "Intention:"))
        self.command.setText(_translate("MainWindow", "None"))
        self.label_2.setText(_translate("MainWindow", "Prediction"))
        self.prediction.setText(_translate("MainWindow", "None"))
        self.label_6.setText(_translate("MainWindow", "  Confidence:"))
        self.label_7.setText(_translate("MainWindow", "Accuracy:"))
        self.accuracy.setText(_translate("MainWindow", "None"))
        self.label_3.setText(_translate("MainWindow", "Data"))
# import mcr_gui_rc

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
