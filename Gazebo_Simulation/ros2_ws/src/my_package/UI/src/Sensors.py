# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'RPLIDAR_Data.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!

import sys
import os
import math
from math import ceil, trunc,isnan
import json
import jsonpickle
#import pandas as pd
from pandas import read_excel
import re
import time
import sqlite3
import openpyxl
from bs4 import BeautifulSoup as Soup
import paho.mqtt.client as mqtt
from PyQt5 import QtCore, uic
from PyQt5.QtCore import Qt, QRegExp, QThread, pyqtSignal
from PyQt5.QtGui import QRegExpValidator, QFont, QColor
from PyQt5.QtGui import QPainter, QPen, QBrush, QFont
from PyQt5.QtCore import Qt, QPointF, QTimer
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QMessageBox, QWidget, QTreeWidget, QTreeWidgetItem, QFileDialog,
    QComboBox, QSpinBox, QHeaderView, QDialog, QTextEdit, QLabel, QLCDNumber, QVBoxLayout
)
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer, pyqtSlot
import statistics
from PyQt5.QtGui import QPixmap, QImage

MAX_RANGE = 50  # Max range in centimeters
SWEEP_SPEED = 10  # Degrees per frame

fixed_objects = []

class Ui_Sensors(object):
    def setupUi(self, Sensors):
        Sensors.setObjectName("Sensors")
        Sensors.resize(760, 727)
        Sensors.setStyleSheet("background-color: #2498db")
        self.gridLayout_4 = QtWidgets.QGridLayout(Sensors)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.buttonBox = QtWidgets.QDialogButtonBox(Sensors)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Close)
        self.buttonBox.setObjectName("buttonBox")
        self.gridLayout_4.addWidget(self.buttonBox, 1, 0, 1, 1)
        self.TabWidget = QtWidgets.QTabWidget(Sensors)
        font = QtGui.QFont()
        font.setFamily("Noto Mono")
        self.TabWidget.setFont(font)
        self.TabWidget.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.TabWidget.setTabPosition(QtWidgets.QTabWidget.North)
        self.TabWidget.setObjectName("TabWidget")
        self.Velodyne = QtWidgets.QWidget()
        self.Velodyne.setObjectName("Velodyne")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.Velodyne)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.frame = QtWidgets.QFrame(self.Velodyne)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame.sizePolicy().hasHeightForWidth())
        self.frame.setSizePolicy(sizePolicy)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.treeWidget = QtWidgets.QTreeWidget(self.frame)
        self.treeWidget.setObjectName("treeWidget")
        self.treeWidget.headerItem().setTextAlignment(0, QtCore.Qt.AlignCenter)
        self.treeWidget.headerItem().setTextAlignment(1, QtCore.Qt.AlignCenter)
        self.treeWidget.headerItem().setTextAlignment(2, QtCore.Qt.AlignCenter)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        item_0 = QtWidgets.QTreeWidgetItem(self.treeWidget)
        self.horizontalLayout_3.addWidget(self.treeWidget)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.line = QtWidgets.QFrame(self.frame)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout.addWidget(self.line)
        self.Graph = QtWidgets.QWidget(self.frame)
        self.Graph.setObjectName("Graph")
        self.verticalLayout.addWidget(self.Graph)
        self.horizontalLayout_2.addWidget(self.frame)
        self.TabWidget.addTab(self.Velodyne, "")
        self.Operational_Mode = QtWidgets.QWidget()
        self.Operational_Mode.setObjectName("Operational_Mode")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.Operational_Mode)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.frame_2 = QtWidgets.QFrame(self.Operational_Mode)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_2.sizePolicy().hasHeightForWidth())
        self.frame_2.setSizePolicy(sizePolicy)
        self.frame_2.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.frame_2)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.line_2 = QtWidgets.QFrame(self.frame_2)
        self.line_2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.verticalLayout_6.addWidget(self.line_2)
        self.RPLIDAR_Plot = QtWidgets.QWidget(self.frame_2)
        self.RPLIDAR_Plot.setObjectName("RPLIDAR_Plot")
        self.verticalLayout_6.addWidget(self.RPLIDAR_Plot)
        self.verticalLayout_7.addLayout(self.verticalLayout_6)
        self.gridLayout_3.addWidget(self.frame_2, 0, 0, 1, 1)
        self.TabWidget.addTab(self.Operational_Mode, "")
        self.tab_3 = QtWidgets.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout(self.tab_3)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.frame_3 = QtWidgets.QFrame(self.tab_3)
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.frame_3)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.treeWidget_4 = QtWidgets.QTreeWidget(self.frame_3)
        self.treeWidget_4.setObjectName("treeWidget_4")
        self.treeWidget_4.headerItem().setTextAlignment(0, QtCore.Qt.AlignCenter)
        self.verticalLayout_2.addWidget(self.treeWidget_4)
        self.verticalLayout_3.addLayout(self.verticalLayout_2)
        self.line_3 = QtWidgets.QFrame(self.frame_3)
        self.line_3.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.verticalLayout_3.addWidget(self.line_3)
        self.Plot = QtWidgets.QWidget(self.frame_3)
        self.Plot.setObjectName("Plot")
        self.verticalLayout_3.addWidget(self.Plot)
        self.verticalLayout_8.addWidget(self.frame_3)
        self.TabWidget.addTab(self.tab_3, "")
        self.gridLayout_4.addWidget(self.TabWidget, 0, 0, 1, 1)

        self.retranslateUi(Sensors)
        self.TabWidget.setCurrentIndex(2)
        self.buttonBox.accepted.connect(Sensors.accept)
        self.buttonBox.rejected.connect(Sensors.reject)
        QtCore.QMetaObject.connectSlotsByName(Sensors)

    def retranslateUi(self, Sensors):
        _translate = QtCore.QCoreApplication.translate
        Sensors.setWindowTitle(_translate("Sensors", "Sensors"))
        self.treeWidget.headerItem().setText(0, _translate("Sensors", "Point ID"))
        self.treeWidget.headerItem().setText(1, _translate("Sensors", "Distance (m)"))
        self.treeWidget.headerItem().setText(2, _translate("Sensors", "Angle (°)"))
        __sortingEnabled = self.treeWidget.isSortingEnabled()
        self.treeWidget.setSortingEnabled(False)
        self.treeWidget.topLevelItem(0).setText(0, _translate("Sensors", "1"))
        self.treeWidget.topLevelItem(1).setText(0, _translate("Sensors", "2"))
        self.treeWidget.topLevelItem(2).setText(0, _translate("Sensors", "3"))
        self.treeWidget.topLevelItem(3).setText(0, _translate("Sensors", "4"))
        self.treeWidget.topLevelItem(4).setText(0, _translate("Sensors", "5"))
        self.treeWidget.topLevelItem(5).setText(0, _translate("Sensors", "6"))
        self.treeWidget.topLevelItem(6).setText(0, _translate("Sensors", "7"))
        self.treeWidget.topLevelItem(7).setText(0, _translate("Sensors", "8"))
        self.treeWidget.topLevelItem(8).setText(0, _translate("Sensors", "9"))
        self.treeWidget.topLevelItem(9).setText(0, _translate("Sensors", "10"))
        self.treeWidget.topLevelItem(10).setText(0, _translate("Sensors", "11"))
        self.treeWidget.topLevelItem(11).setText(0, _translate("Sensors", "12"))
        self.treeWidget.topLevelItem(12).setText(0, _translate("Sensors", "13"))
        self.treeWidget.topLevelItem(13).setText(0, _translate("Sensors", "14"))
        self.treeWidget.topLevelItem(14).setText(0, _translate("Sensors", "15"))
        self.treeWidget.topLevelItem(15).setText(0, _translate("Sensors", "16"))
        self.treeWidget.topLevelItem(16).setText(0, _translate("Sensors", "17"))
        self.treeWidget.topLevelItem(17).setText(0, _translate("Sensors", "18"))
        self.treeWidget.topLevelItem(18).setText(0, _translate("Sensors", "19"))
        self.treeWidget.topLevelItem(19).setText(0, _translate("Sensors", "20"))
        self.treeWidget.topLevelItem(20).setText(0, _translate("Sensors", "21"))
        self.treeWidget.topLevelItem(21).setText(0, _translate("Sensors", "22"))
        self.treeWidget.topLevelItem(22).setText(0, _translate("Sensors", "23"))
        self.treeWidget.topLevelItem(23).setText(0, _translate("Sensors", "24"))
        self.treeWidget.topLevelItem(24).setText(0, _translate("Sensors", "25"))
        self.treeWidget.topLevelItem(25).setText(0, _translate("Sensors", "26"))
        self.treeWidget.topLevelItem(26).setText(0, _translate("Sensors", "27"))
        self.treeWidget.topLevelItem(27).setText(0, _translate("Sensors", "28"))
        self.treeWidget.topLevelItem(28).setText(0, _translate("Sensors", "29"))
        self.treeWidget.topLevelItem(29).setText(0, _translate("Sensors", "30"))
        self.treeWidget.topLevelItem(30).setText(0, _translate("Sensors", "31"))
        self.treeWidget.topLevelItem(31).setText(0, _translate("Sensors", "32"))
        self.treeWidget.setSortingEnabled(__sortingEnabled)
        self.TabWidget.setTabText(self.TabWidget.indexOf(self.Velodyne), _translate("Sensors", "Velodyne LiDAR"))
        self.TabWidget.setTabText(self.TabWidget.indexOf(self.Operational_Mode), _translate("Sensors", "RPLIDAR"))
        self.treeWidget_4.headerItem().setText(0, _translate("Sensors", "Distance (cm)"))
        self.TabWidget.setTabText(self.TabWidget.indexOf(self.tab_3), _translate("Sensors", "Ultrasonic"))
        
        self.Velodyne_Plot = Velodyne_Plot(self.Graph)
        layout = QtWidgets.QVBoxLayout(self.Graph)
        layout.addWidget(self.Velodyne_Plot)
        
        self.RPLIDAR_Plot = RPLIDAR_Plot(self.RPLIDAR_Plot)
        layout2 = QtWidgets.QVBoxLayout(self.RPLIDAR_Plot)
        layout2.addWidget(self.RPLIDAR_Plot)
        
class Velodyne_Plot(FigureCanvas, QWidget):
    def __init__(self, parent=None):
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        super().__init__(self.fig)
        self.setParent(parent)  # Attach to the QWidget in UI

        self.ax.set_ylim(0, MAX_RANGE)
        self.ax.set_yticks([10, 20, 30, 40, 50])
        self.ax.set_yticklabels(['10m', '20m', '30m', '40m', '50m'])
        self.ax.set_xticklabels([])
        self.ax.grid(True, linestyle="--", alpha=0.5)

        # Sweep Line & Points
        self.sweep_line, = self.ax.plot([], [], 'g-', linewidth=2)
        self.points, = self.ax.plot([], [], 'go', markersize=6, alpha=0.7)
     

        # Dynamic Data Storage
        self.angles = np.array([])  # Store angles in radians
        self.distances = np.array([])  # Store distances

        # Start animation
        self.ani = animation.FuncAnimation(self.fig, self.animate, frames=360//SWEEP_SPEED, 
                                           interval=50, blit=True)
 
    def animate(self, frame):

        angle_sweep = np.radians(frame * SWEEP_SPEED)  # Current sweep angle
        sweep_range = np.radians(10)  # 10-degree window

        # Filter points within sweep range
        visible_indices = (self.angles >= angle_sweep - sweep_range) & (self.angles <= angle_sweep + sweep_range)
        
        # Update Sweep Line
        self.sweep_line.set_data([angle_sweep, angle_sweep], [0, MAX_RANGE])

        # Update Points
        self.points.set_data(self.angles[visible_indices], np.array(self.distances)[visible_indices])

        return self.sweep_line, self.points
        
    def update_Velodyne_points(self, angle, distance):
        angle_rad = np.radians(angle)  # Convert to radians

        # Append new data
        self.angles = np.append(self.angles, angle_rad)
        self.distances = np.append(self.distances, distance)
        
        if len(self.angles) > 100:  # Limit stored points to 100 for smooth animation
            self.angles = self.angles[-100:]
            self.distances = self.distances[-100:]
        
class RPLIDAR_Plot(FigureCanvas, QWidget):
    def __init__(self, parent=None):
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        super().__init__(self.fig)
        self.setParent(parent)  # Attach to the QWidget in UI
        
        self.ax.set_ylim(0, MAX_RANGE)
        self.ax.set_yticks([10, 20, 30, 40, 50])
        self.ax.set_yticklabels(['10cm', '20cm', '30cm', '40cm', '50cm'])
        self.ax.set_xticklabels([])
        self.ax.grid(True, linestyle="--", alpha=0.5)

        # Sweep Line & Points
        self.sweep_line, = self.ax.plot([], [], 'g-', linewidth=2)
        self.points, = self.ax.plot([], [], 'go', markersize=6, alpha=0.7)
     

        # Dynamic Data Storage
        self.angles = np.array([])  # Store angles in radians
        self.distances = np.array([])  # Store distances

        # Start animation
        self.ani = animation.FuncAnimation(self.fig, self.animate, frames=360//SWEEP_SPEED, 
                                           interval=20, blit=True)
 
    def animate(self, frame):

        angle_sweep = np.radians(frame * SWEEP_SPEED)  # Current sweep angle
        sweep_range = np.radians(10)  # 10-degree window

        # Filter points within sweep range
        visible_indices = (self.angles >= angle_sweep - sweep_range) & (self.angles <= angle_sweep + sweep_range)
        
        # Update Sweep Line
        self.sweep_line.set_data([angle_sweep, angle_sweep], [0, MAX_RANGE])

        # Update Points
        self.points.set_data(self.angles[visible_indices], np.array(self.distances)[visible_indices])

        return self.sweep_line, self.points
        
    def update_RPLIDAR_points(self, angle, distance):
    
        #angle_rad = np.radians(angle)  # Convert to radians
        angle_rad = np.radians(float(angle) % 360)

        # Append new data
        self.angles = np.append(self.angles, angle_rad)
        self.distances = np.append(self.distances, distance)
        
        if len(self.angles) > 100:  # Limit stored points to 100 for smooth animation
            self.angles = self.angles[-100:]
            self.distances = self.distances[-100:]

class Ultrasonic_Plot(FigureCanvas, QWidget):
    def __init__(self):
        super().__init__()

        # Create a label to display the plot
        self.plot_label = QLabel(self)
        self.layout = QVBoxLayout(self)
        self.layout.addWidget(self.plot_label)

        self.x_data, self.y_data = [], []
        self.fig, self.ax = plt.subplots(figsize=(7, 4))

        # Set up a QTimer to call update_plot every 200ms
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(200)  # Update every 200ms

        # Initial simulated start time
        self.start_time = time.time()

    def update_plot(self, dist=None):
        """Update the plot with the new data and display it."""
        if dist is not None:

            elapsed_time = time.time() - self.start_time
            self.x_data.append(elapsed_time)
            self.y_data.append(dist)

            # Plot the data
            self.ax.clear()
            self.ax.plot(self.x_data, self.y_data, 'b-', label="Distance (cm)")
            self.ax.set_xlabel("Time (s)")
            self.ax.set_ylabel("Distance (cm)")
            self.ax.set_ylim(0, 400)
            self.ax.set_title("Live Ultrasonic Distance Measurements")

            # Convert the plot to a QPixmap
            self.fig.canvas.draw()
            img = self.fig.canvas.tostring_rgb()
            img = QImage(img, self.fig.canvas.width(), self.fig.canvas.height(), QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(img)

            # Update the QLabel with the new plot image
            self.plot_label.setPixmap(pixmap)

    def get_filtered_distance(self, samples=5):
        """Get a stable distance reading using median filtering."""
        distances = [self.get_distance() for _ in range(samples) if self.get_distance() is not None]
        time.sleep(0.05 * samples)
        return round(statistics.median(distances), 2) if distances else None

    def get_distance(self):
    	fixed = 88
    	self.update_plot(fixed)
    	
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Sensors = QtWidgets.QDialog()
    ui = Ui_Sensors()
    ui.setupUi(Sensors)
    Sensors.show()
    sys.exit(app.exec_())
