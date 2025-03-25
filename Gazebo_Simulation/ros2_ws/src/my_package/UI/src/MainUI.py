# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MainUI.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        MainWindow.setStyleSheet("background-color: #2498db")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.VehicleDiagnostics = QtWidgets.QPushButton(self.frame)
        self.VehicleDiagnostics.setGeometry(QtCore.QRect(80, 60, 231, 121))
        self.VehicleDiagnostics.setObjectName("VehicleDiagnostics")
        self.Sensors = QtWidgets.QPushButton(self.frame)
        self.Sensors.setGeometry(QtCore.QRect(410, 60, 231, 121))
        self.Sensors.setObjectName("Sensors")
        self.Control = QtWidgets.QPushButton(self.frame)
        self.Control.setGeometry(QtCore.QRect(230, 250, 261, 111))
        self.Control.setObjectName("Control")
        self.verticalLayout.addWidget(self.frame)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 24))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.VehicleDiagnostics.setText(_translate("MainWindow", "Vehicle Diagnostics"))
        self.Sensors.setText(_translate("MainWindow", "Sensors"))
        self.Control.setText(_translate("MainWindow", "Control"))
