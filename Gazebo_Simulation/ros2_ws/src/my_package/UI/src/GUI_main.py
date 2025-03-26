import sys
import os
from math import ceil, trunc,isnan
import json
import jsonpickle
from collections import deque
#import pandas as pd
from pandas import read_excel
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
import threading
import time
import sqlite3
import openpyxl
from bs4 import BeautifulSoup as Soup
import paho.mqtt.client as mqtt
from PyQt5 import QtCore, uic
from PyQt5.QtCore import Qt, QRegExp, QThread, pyqtSignal
from PyQt5.QtGui import QRegExpValidator, QFont, QColor
from PyQt5.QtGui import QPainter, QPen, QBrush, QFont
from PyQt5.QtCore import Qt, QPointF, QTimer, QPropertyAnimation
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QMessageBox, QWidget, QTreeWidget, QTreeWidgetItem, QFileDialog,
    QComboBox, QSpinBox, QHeaderView, QDialog, QTextEdit, QLabel, QLCDNumber, QVBoxLayout
)
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QVBoxLayout
from PyQt5.QtCore import pyqtSignal, QSettings

from datetime import datetime

from MainUI import Ui_MainWindow
from VehicleDiagnostics import Ui_Vehicle_Diagnostics, CompassWidget
from Sensors import Ui_Sensors, Velodyne_Plot, RPLIDAR_Plot, Ultrasonic_Plot
from Control import Ui_Control

#####################################################################
# UI control of the main window
#####################################################################


class MQTTClientThread(QThread):
    # Define a custom signal to send messages to the GUI thread
    message_received = pyqtSignal(str, float)

    def __init__(self, broker, port, topic):
        super().__init__()
        self.broker = broker
        self.port = port
        self.topic = topic
        self.client = mqtt.Client()

    def run(self):
        # Set up the MQTT callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # Connect to the broker and start the loop
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_forever()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.client.subscribe(self.topic)

    def on_message(self, client, userdata, msg):
        message = f"{msg.payload.decode()}"
        #print(message)
        #self.message_received.emit(message)  # Emit the signal to update the GUI
        try:
            value = float(msg.payload.decode("utf-8"))
            self.message_received.emit(self.topic, value)  # Emit (topic, value)
        except Exception as e:
            print(f"MQTT Parsing Error: {e}")
            
class RADARPlot_MQTTClientThread(QThread):
    # Define a custom signal to send messages to the GUI thread
    RADARPlot_message_received = pyqtSignal(str, float)

    def __init__(self, broker, port, topic):
        super().__init__()
        self.broker = broker
        self.port = port
        self.topic = topic
        self.client = mqtt.Client()

    def run(self):
        # Set up the MQTT callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # Connect to the broker and start the loop
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_forever()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.client.subscribe(self.topic)

    def on_message(self, client, userdata, msg):
        message = f"{msg.payload.decode()}"
        #print(message)
        #self.message_received.emit(message)  # Emit the signal to update the GUI
        try:
            value = float(msg.payload.decode("utf-8"))
            self.RADARPlot_message_received.emit(self.topic, value)  # Emit (topic, value)
        except Exception as e:
            print(f"MQTT Parsing Error: {e}")

class Diagnostics_MQTTClientThread(QThread):
    # Define a custom signal to send messages to the GUI thread
    message_received = pyqtSignal(str)

    def __init__(self, broker, port, topic):
        super().__init__()
        self.broker = broker
        self.port = port
        self.topic = topic
        self.client = mqtt.Client()

    def run(self):
        # Set up the MQTT callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # Connect to the broker and start the loop
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_forever()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.client.subscribe(self.topic)

    def on_message(self, client, userdata, msg):
        message3 = f"{msg.payload.decode()}"
        #print(message3)
        self.message_received.emit(message3)  # Emit the signal to update the GUI

class MQTTClient(QDialog):
   message_received = pyqtSignal(str)

   def __init__(self):
        super().__init__()
        uic.loadUi("Vehicle_Diagnostics.ui", self)  # Load your .ui file
        self.compass_placeholder = self.findChild(QWidget, "Compass")
            # Set layout for the CompassPlaceholder
        self.layout = QVBoxLayout(self.compass_placeholder)
        self.compass_widget = CompassWidget(self.compass_placeholder)
        self.layout.addWidget(self.compass_widget)
        
        self.XPosition = self.findChild(QLabel, "XPosition")  # Assuming you have a QTextEdit with name 'textEdit'
        self.YPosition = self.findChild(QLabel, "YPosition")
        self.MinRange = self.findChild(QLabel, "MinRange")
        self.MaxRange = self.findChild(QLabel, "MaxRange")
        self.Rotation = self.findChild(QLabel, "Rotation")
        self.lcd = self.findChild(QLCDNumber, "lcdNumber")
        self.ControlStatus = self.findChild(QLabel, "ControlStatus")
        self.settings = QSettings("MyApp", "MQTTClient")
        last_text = self.settings.value("label_text", "Waiting for update...")
        self.ControlStatus.setText(last_text)
        
        # Start the MQTT client threads
        self.xposition_thread = Diagnostics_MQTTClientThread("localhost", 1883, "vehicle_status/navigation/x_position")
        self.yposition_thread = Diagnostics_MQTTClientThread("localhost", 1883, "vehicle_status/navigation/y_position")
        self.minrange_thread = Diagnostics_MQTTClientThread("localhost", 1883, "sensors/Velodyne/diagnostics/min_range")
        self.maxrange_thread = Diagnostics_MQTTClientThread("localhost", 1883, "sensors/Velodyne/diagnostics/max_range")
        self.angle_thread = Diagnostics_MQTTClientThread("localhost", 1883, "vehicle_status/navigation/angle")
        self.velocity_thread = Diagnostics_MQTTClientThread("localhost", 1883, "vehicle_status/navigation/velocity")
        self.hsamples_thread = Diagnostics_MQTTClientThread("localhost", 1883, "sensors/Velodyne/diagnostics/hsamples")
        self.vsamples_thread = Diagnostics_MQTTClientThread("localhost", 1883, "sensors/Velodyne/diagnostics/vsamples")
        self.hresolution_thread = Diagnostics_MQTTClientThread("localhost", 1883, "sensors/Velodyne/diagnostics/hresolution")
        self.vresolution_thread = Diagnostics_MQTTClientThread("localhost", 1883, "sensors/Velodyne/diagnostics/vresolution")
        self.health_thread = Diagnostics_MQTTClientThread("localhost", 1883, "sensors/RPLIDAR/diagnostics/health_status")
        self.modelno_thread = Diagnostics_MQTTClientThread("localhost", 1883, "sensors/RPLIDAR/diagnostics/model_num")
        self.status_thread = Diagnostics_MQTTClientThread("localhost", 1883, "sensors/RPLIDAR/diagnostics/status")
        self.standard_thread = Diagnostics_MQTTClientThread("localhost", 1883, "sensors/RPLIDAR/diagnostics/samplerate_standard")
        self.express_thread = Diagnostics_MQTTClientThread("localhost", 1883, "sensors/RPLIDAR/diagnostics/samplerate_express")
        
        self.xposition_thread.message_received.connect(self.display_xposition)
        self.yposition_thread.message_received.connect(self.display_yposition)
        self.minrange_thread.message_received.connect(self.display_minrange)
        self.maxrange_thread.message_received.connect(self.display_maxrange)
        self.angle_thread.message_received.connect(self.display_angle)
        self.velocity_thread.message_received.connect(self.display_velocity)
        self.hsamples_thread.message_received.connect(self.display_hsamples)
        self.vsamples_thread.message_received.connect(self.display_vsamples)
        self.hresolution_thread.message_received.connect(self.display_hresolution)
        self.vresolution_thread.message_received.connect(self.display_vresolution)
        self.health_thread.message_received.connect(self.display_healthstatus)
        self.modelno_thread.message_received.connect(self.display_modelno)
        self.status_thread.message_received.connect(self.display_status)
        self.standard_thread.message_received.connect(self.display_standardrate)
        self.express_thread.message_received.connect(self.display_expressrate)
        
        self.xposition_thread.start()
        self.yposition_thread.start()
        self.minrange_thread.start()
        self.maxrange_thread.start()
        self.angle_thread.start()
        self.velocity_thread.start()
        self.hsamples_thread.start()
        self.vsamples_thread.start()
        self.hresolution_thread.start()
        self.vresolution_thread.start()
        self.health_thread.start()
        self.modelno_thread.start()
        self.status_thread.start()
        self.standard_thread.start()
        self.express_thread.start()
        
   def display_xposition(self, xposition):
   # Append the message to the QTextEdit
       self.XPosition.setText(xposition)
       now = datetime.now()
       formatted = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Trim microseconds to milliseconds
       #print("X Position: {} \t\t Timestamp: {}".format(d, formatted))

   def display_yposition(self, yposition):
   # Append the message to the QTextEdit
       self.YPosition.setText(yposition)
       
   def display_minrange(self, min_range):
   # Append the message to the QTextEdit
       float_min_range = float(min_range)
       rounded_min_range = f"{float_min_range:.2f}"
       self.MinRange.setText(rounded_min_range)
       
   def display_maxrange(self, max_range):
   # Append the message to the QTextEdit
       float_max_range = float(max_range)
       rounded_max_range = f"{float_max_range:.2f}"
       self.MaxRange.setText(rounded_max_range)
       
   def display_angle(self, angle):
   # Append the message to the QTextEdit
       float_angle = float(angle)
       rounded_angle = f"{float_angle:.2f}"
       self.Rotation.setText(rounded_angle + "°")
       self.compass_widget.setAngle(float(angle))
       
   def display_velocity(self, velocity):
   # Append the message to the QTextEdit
       float_velocity = float(velocity)
       rounded_velocity = f"{float_velocity:.3f}"
       self.Velocity.setText(rounded_velocity)
       
   def display_hsamples(self, hsamples):
   # Append the message to the QTextEdit
       self.Horiz_Samples.setText(hsamples)
   
   def display_vsamples(self, vsamples):
   # Append the message to the QTextEdit
       self.Vert_Samples.setText(vsamples)
       
   def display_hresolution(self, hresolution):
   # Append the message to the QTextEdit
       self.Horiz_Res.setText(hresolution)
   
   def display_vresolution(self, vresolution):
   # Append the message to the QTextEdit
       self.Vert_Res.setText(vresolution)
       
   def display_healthstatus(self, health):
   # Append the message to the QTextEdit
       self.HealthStatus.setText(health)
       
   def display_modelno(self, model_number):
   # Append the message to the QTextEdit
       self.ModelNo.setText(model_number)
       
   def display_status(self, status):
   # Append the message to the QTextEdit
       self.Status.setText(status)
       
   def display_standardrate(self, standard_rate):
   # Append the message to the QTextEdit
       self.Standard.setText(standard_rate + " µs/sample")
       
   def display_expressrate(self, express_rate):
   # Append the message to the QTextEdit
       self.Express.setText(express_rate + " µs/sample")
       
   def update_mode(self, text):
   	self.ControlStatus.setText(text)
   	self.settings.setValue("label_text", text)
       
class Sensor_Client(QDialog):
   message_received = pyqtSignal(str, float)

   def __init__(self):
        super().__init__()
        uic.loadUi("Sensors.ui", self)  # Load your .ui fil
        self.connectSignalsSlots()
        
        self.Velodyne_data = {}
        self.RPLIDAR_data = {}
        
        self.Velodyne_distance_threads = [
    Diagnostics_MQTTClientThread("localhost", 1883, f"sensors/Velodyne/navigation/distance{i}") for i in range(1, 33)
]

        self.Velodyne_angle_threads = [
    Diagnostics_MQTTClientThread("localhost", 1883, f"sensors/Velodyne/navigation/angle{i}") for i in range(1, 33)
]

        self.RPLIDAR_distance_threads = [
    Diagnostics_MQTTClientThread("localhost", 1883, f"sensors/RPLIDAR/navigation/dist{i}") for i in range(1, 33)
]

        self.RPLIDAR_angle_threads = [
    Diagnostics_MQTTClientThread("localhost", 1883, f"sensors/RPLIDAR/navigation/ang{i}") for i in range(1, 33)
]

        self.Ultrasonic_distance_thread = Diagnostics_MQTTClientThread("localhost", 1883, "sensors/Ultrasonic/navigation/distance")
        self.Ultrasonic_distance_thread.message_received.connect(self.display_UltrasonicDistance)
        self.Ultrasonic_distance_thread.start()
        
        ###As pyQT didnt have this option, its placed here
        self.treeWidget.setColumnWidth(0, 200)
        self.treeWidget.setColumnWidth(1, 200)
        self.treeWidget.setColumnWidth(2, 200)
        self.treeWidget.setColumnWidth(3, 200)
        self.Velodyne_radar_placeholder = self.findChild(QWidget, "Graph")
        self.Ultrasonic_Plot = self.findChild(QWidget, "Plot")

        for i, thread in enumerate(self.Velodyne_distance_threads):
        	thread.message_received.connect(lambda msg, i=i: self.display_VelodyneDistance(i, msg))
        	
        for i, thread in enumerate(self.Velodyne_angle_threads):
        	thread.message_received.connect(lambda msg, i=i: self.display_VelodyneAngle(i, msg))
        	
        for thread in self.Velodyne_distance_threads + self.Velodyne_angle_threads:
                thread.start()

        # Create radar plot and add it to the existing QWidget
        self.Velodyne_layout = QVBoxLayout(self.Velodyne_radar_placeholder)
        self.Velodyne_radar_plot = Velodyne_Plot(self.Velodyne_radar_placeholder)  # Attach to the QWidget in the UI
        self.Velodyne_layout.addWidget(self.Velodyne_radar_plot)
        self.VelodyneTable = self.findChild(QTreeWidget, "treeWidget")
        
        for i in range(0, 32):
        	self.VelodyneTable.topLevelItem(i).setTextAlignment(0, Qt.AlignCenter)
        
        self.UltrasonicTable = self.findChild(QTreeWidget, "treeWidget_4")
        
                # Create the LivePlotWidget and set it inside the `Ultrasonic_Plot` widget
        
        self.RPLIDAR_radar_placeholder = self.findChild(QWidget, "RPLIDAR_Plot")
        
        self.data_queue = deque()  # Store received (angle, distance) updates
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.process_data_queue)
        self.update_timer.start(100)
        	
        for i, thread in enumerate(self.RPLIDAR_distance_threads):
        	thread.message_received.connect(lambda msg, i=i: self.display_RPLIDARDistance(i, msg))
        	
        for i, thread in enumerate(self.RPLIDAR_angle_threads):
        	thread.message_received.connect(lambda msg, i=i: self.display_RPLIDARAngle(i, msg))
                
        for thread in self.RPLIDAR_distance_threads + self.RPLIDAR_angle_threads:
                thread.start()

        # Create radar plot and add it to the existing QWidget
        self.RPLIDAR_layout = QVBoxLayout(self.RPLIDAR_radar_placeholder)
        self.RPLIDAR_radar_plot = RPLIDAR_Plot(self.RPLIDAR_radar_placeholder)  # Attach to the QWidget in the UI
        self.RPLIDAR_layout.addWidget(self.RPLIDAR_radar_plot)
        
        # Create MQTT Threads for Angles & Distances
        self.Velodyne_mqtt_clients = []
        for i in range(23, 32):  # For angle1 to angle7
            VelodyneAngle_topic = f"sensors/Velodyne/navigation/angle{i}"
            VelodyneDistance_topic = f"sensors/Velodyne/navigation/distance{i}"

            VelodyneAngle_client = MQTTClientThread("localhost", 1883, VelodyneAngle_topic)
            VelodyneDistance_client = MQTTClientThread("localhost", 1883, VelodyneDistance_topic)

            # Connect the signals to a handler function
            VelodyneAngle_client.message_received.connect(self.Velodyne_angles)
            VelodyneDistance_client.message_received.connect(self.Velodyne_distances)

            # Start threads
            VelodyneAngle_client.start()
            VelodyneDistance_client.start()

            # Store in list
            self.Velodyne_mqtt_clients.extend([VelodyneAngle_client, VelodyneDistance_client])

        self.RPLIDAR_mqtt_clients = []
        for i in range(1, 6):  # For angle1 to angle7
            RPLIDARAngle_topic = f"sensors/RPLIDAR/navigation/ang{i}"
            RPLIDARDistance_topic = f"sensors/RPLIDAR/navigation/dist{i}"

            RPLIDARAngle_client = RADARPlot_MQTTClientThread("localhost", 1883, RPLIDARAngle_topic)
            RPLIDARDistance_client = RADARPlot_MQTTClientThread("localhost", 1883, RPLIDARDistance_topic)

            # Connect the signals to a handler function
            RPLIDARAngle_client.RADARPlot_message_received.connect(self.RPLIDAR_angles)
            RPLIDARDistance_client.RADARPlot_message_received.connect(self.RPLIDAR_distances)

            # Start threads
            RPLIDARAngle_client.start()
            RPLIDARDistance_client.start()

            # Store in list
            self.RPLIDAR_mqtt_clients.extend([RPLIDARAngle_client, RPLIDARDistance_client])
          
        self.live_plot_widget = Ultrasonic_Plot()
        Ultrasonic_layout = QVBoxLayout(self.Ultrasonic_Plot)  # Add the plot widget to the layout of `ultra_plot`
        Ultrasonic_layout.addWidget(self.live_plot_widget)
        	
   def connectSignalsSlots(self):
        t = 0;
        
   def Launch(self):
         self.main_window = MQTTClient4()
         self.main_window.show()
   
   def display_VelodyneDistance(self, index, distance):
       new_value = float(distance)
       item = self.VelodyneTable.topLevelItem(index)
       item.setText(1, f"{new_value:.2f}")
       item.setTextAlignment(1, Qt.AlignCenter)
       
   def display_VelodyneAngle(self, index, angle):
       new_value = float(angle)
       item = self.VelodyneTable.topLevelItem(index)
       item.setText(2, f"{new_value:.2f}")
       item.setTextAlignment(2, Qt.AlignCenter)
       
   def display_RPLIDARDistance(self, index, distance):
       d = float(distance)
       now = datetime.now()
       formatted = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Trim microseconds to milliseconds
       
   def display_RPLIDARAngle(self, index, angle):
        new_value = float(angle)
        
   def display_UltrasonicDistance(self, ultrasonic_distance):
   	new_value = float(ultrasonic_distance)
   	item = QTreeWidgetItem()
   	item.setText(0, str(f"{new_value:.2f}"))
   	item.setTextAlignment(0, Qt.AlignCenter)
   	self.UltrasonicTable.addTopLevelItem(item)
   	self.UltrasonicTable.scrollToBottom()
   	self.live_plot_widget.update_plot(new_value)
       
   def Velodyne_angles(self, topic, angle):
       """ Store the received angle and wait for its distance """
       index = topic.replace("angle", "")  # Extract index (1 to 7)
       if index not in self.Velodyne_data:
              self.Velodyne_data[index] = {"angle": None, "distance": None}
        
       self.Velodyne_data[index]["angle"] = angle

        # If both angle and distance are available, update radar
       if self.Velodyne_data[index]["distance"] is not None:
              self.Velodyne_radar_plot.update_Velodyne_points(angle, self.Velodyne_data[index]["distance"])

   def Velodyne_distances(self, topic, distance):
       """ Store the received distance and wait for its angle """
       index = topic.replace("distance", "")  # Extract index (1 to 7)
       if index not in self.Velodyne_data:
              self.Velodyne_data[index] = {"angle": None, "distance": None}
        
       self.Velodyne_data[index]["distance"] = distance

        # If both angle and distance are available, update radar
       if self.Velodyne_data[index]["angle"] is not None:
              self.Velodyne_radar_plot.update_Velodyne_points(self.Velodyne_data[index]["angle"], distance)
              
   def RPLIDAR_angles(self, topic, angle):
       """ Store the received angle and wait for its distance """
       index = topic.replace("ang", "")  # Extract index (1 to 7)
       if index not in self.RPLIDAR_data:
              self.RPLIDAR_data[index] = {"ang": None, "dist": None}
        
       self.RPLIDAR_data[index]["ang"] = angle

        # If both angle and distance are available, update radar
       if self.RPLIDAR_data[index]["dist"] is not None:
              self.data_queue.append((angle, self.RPLIDAR_data[index]["dist"]))
       
   def RPLIDAR_distances(self, topic, distance):
       """ Store the received distance and wait for its angle """
       index = topic.replace("dist", "")  # Extract index (1 to 7)
       if index not in self.RPLIDAR_data:
              self.RPLIDAR_data[index] = {"ang": None, "dist": None}
        
       self.RPLIDAR_data[index]["dist"] = distance

        # If both angle and distance are available, update radar
       if self.RPLIDAR_data[index]["ang"] is not None:
              self.data_queue.append((self.RPLIDAR_data[index]["ang"], distance))
              
   def process_data_queue(self):
        """ Process one data update at a time to prevent UI lag """
        if self.data_queue:
            angle, distance = self.data_queue.popleft()  # Get next update
            self.RPLIDAR_radar_plot.update_RPLIDAR_points(angle, distance)
        
class ControlPanel_Client(QDialog):
   # message_received = pyqtSignal(str)
   signal_update = pyqtSignal(str)
   def __init__(self):
        super().__init__()
        uic.loadUi("Control.ui", self)  # Load your .ui file
        
        self.start = 1; # On start
        self.stop = 0; # Not stopped
        self.auto = 1;
        self.manual = 0;
        self.forward = 1;
        self.backward = 0;
        self.left = 0;
        self.right = 0;
        self.save_mode = QSettings("MyApp", "ControlPanel_Client")
        last_mode = self.save_mode.value("control_status", 2)
        self.mode = last_mode;
        self.second_window = MQTTClient()
        self.second_window.update_mode("Autonomous Mode");
                
        self.client = mqtt.Client()
        self.client.connect("localhost", 1883, 60)  # Update this with your MQTT broker info
        self.client.loop_start()  # Start the loop to handle incoming and outgoing messages
        self.AutoLabel = QLabel("Autonomous Mode Enabled", self)
        self.AutoLabel.setStyleSheet("background-color: rgba(0, 0, 0, 150); color: white; padding: 10px; border-radius: 5px;")
        self.ManualLabel = QLabel("Manual Mode Enabled", self)
        self.ManualLabel.setStyleSheet("background-color: rgba(0, 0, 0, 150); color: white; padding: 10px; border-radius: 5px;")
        self.AutoLabel.hide()
        self.ManualLabel.hide()
        self.warning = QLabel("Cannot control in autonomous mode", self)
        self.warning.setStyleSheet("background-color: rgba(200, 0, 0, 150); color: white; padding: 10px; border-radius: 5px;")
        self.warning.hide()
        	
        self.EmStop.clicked.connect(self.publish_EmStop)
        self.Manual.clicked.connect(self.publish_ModeControlManual)
        self.Autonomous.clicked.connect(self.publish_ModeControlAuto)
        self.Start_Stop.clicked.connect(self.publish_StartStop)
        self.Forward.clicked.connect(self.publish_Forward)
        self.Backward.clicked.connect(self.publish_Backward)
        self.SetVelocity.clicked.connect(self.publish_Velocity)
        
        self.TurnLeft.pressed.connect(self.start_rotating_left)
        self.TurnLeft.released.connect(self.stop_rotating_left)
        
        self.TurnRight.pressed.connect(self.start_rotating_right)
        self.TurnRight.released.connect(self.stop_rotating_right)

        self.rotating = False  # Flag to control rotating
        
        safety_layout = QVBoxLayout()
        safety_layout.addWidget(self.EmStop, alignment=Qt.AlignCenter)

   def start_rotating_left(self):
        """ Starts the loop in a separate thread when button is pressed """
        if not self.rotating:
            self.rotating = True
            threading.Thread(target=self.publish_Left, daemon=True).start()
            
   def start_rotating_right(self):
        """ Starts the loop in a separate thread when button is pressed """
        if not self.rotating:
            self.rotating = True
            threading.Thread(target=self.publish_Right, daemon=True).start()

   def stop_rotating_left(self):
        """ Stops the loop when button is released """
        self.rotating = False
        self.left = 0
        self.client.publish("control/navigation/turn_left", self.left, qos=1)
        
   def stop_rotating_right(self):
        #Stops the loop when button is released
        self.rotating = False
        self.right = 0
        self.client.publish("control/navigation/turn_right", self.right, qos=1)
        
   def publish_EmStop(self):
        topic = "control/safety/emergency_shutdown"  # Specify the topic
        em_stop = 1  # Message to send
        self.client.publish(topic, em_stop, qos=2)  # Publish message
        
   def publish_StartStop(self):
        topic = "control/navigation/start_stop"  # Specify the topic
        
        if(self.mode == 1):
        	if(self.stop == 0):
        		self.stop = 1;
        		self.start = 0;
        		self.client.publish(topic, self.start, qos=1)
        	
        	elif(self.stop == 1):
        		self.stop = 0;
        		self.start = 1;
        		self.client.publish(topic, self.start, qos=1)
        		
        elif(self.mode == 2):
        	self.warning.setAlignment(Qt.AlignCenter)
        	self.warning.adjustSize()
        	self.ManualLabel.hide()
        	self.AutoLabel.hide()
        	
        	x = (self.width() - self.warning.width()) // 2
        	y = (self.height() - self.warning.height()) // 2
        	self.warning.move(x, y)

        # Initially hide label
        	self.warning.setVisible(False)
        	self.warning.setVisible(True)

        # Animation setup
	        self.animation = QPropertyAnimation(self.warning, b"windowOpacity")
        	self.animation.setDuration(3000)  # 2 seconds fade
	        self.animation.setStartValue(1.0)  # Fully visible
        	self.animation.setEndValue(0.0)  # Fully transparent
        	self.animation.start()

        # Hide the label after fade-out
        	self.animation.finished.connect(lambda: self.warning.setVisible(False))
        	
   def publish_Backward(self):
        topic = "control/navigation/move_backward"  # Specify the topic
        if(self.mode == 1):
        	self.backward = 1  # Message to send
        	self.client.publish(topic, self.backward, qos=1)  # Publish message
        	
        elif(self.mode == 2):
        	self.warning.setAlignment(Qt.AlignCenter)
        	self.warning.adjustSize()
        	self.ManualLabel.hide()
        	self.AutoLabel.hide()
        	
        	x = (self.width() - self.warning.width()) // 2
        	y = (self.height() - self.warning.height()) // 2
        	self.warning.move(x, y)

        # Initially hide label
        	self.warning.setVisible(False)
        	self.warning.setVisible(True)

        # Animation setup
	        self.animation = QPropertyAnimation(self.warning, b"windowOpacity")
        	self.animation.setDuration(3000)  # 2 seconds fade
	        self.animation.setStartValue(1.0)  # Fully visible
        	self.animation.setEndValue(0.0)  # Fully transparent
        	self.animation.start()

        # Hide the label after fade-out
        	self.animation.finished.connect(lambda: self.warning.setVisible(False))
        
   def publish_Forward(self):
        topic = "control/navigation/move_forward"  # Specify the topic
        if(self.mode == 1):
        	self.forward = 1  # Message to send
        	self.client.publish(topic, self.forward, qos=1)  # Publish message
        	
        elif(self.mode == 2):
        	self.warning.setAlignment(Qt.AlignCenter)
        	self.warning.adjustSize()
        	self.ManualLabel.hide()
        	self.AutoLabel.hide()
        	
        	x = (self.width() - self.warning.width()) // 2
        	y = (self.height() - self.warning.height()) // 2
        	self.warning.move(x, y)

        # Initially hide label
        	self.warning.setVisible(False)
        	self.warning.setVisible(True)

        # Animation setup
	        self.animation = QPropertyAnimation(self.warning, b"windowOpacity")
        	self.animation.setDuration(3000)  # 2 seconds fade
	        self.animation.setStartValue(1.0)  # Fully visible
        	self.animation.setEndValue(0.0)  # Fully transparent
        	self.animation.start()

        # Hide the label after fade-out
        	self.animation.finished.connect(lambda: self.warning.setVisible(False))
        
   def publish_Left(self):
   	topic = "control/navigation/turn_left" # Specify the topic
   	while self.rotating:
   		if(self.mode == 1):
        		self.left = 1
        		self.client.publish(topic, self.left, qos=1)
        	
   		elif(self.mode == 2):
        		self.warning.setAlignment(Qt.AlignCenter)
        		self.warning.adjustSize()
        		self.ManualLabel.hide()
        		self.AutoLabel.hide()
        	
        		x = (self.width() - self.warning.width()) // 2
        		y = (self.height() - self.warning.height()) // 2
        		self.warning.move(x, y)
	
        # Initially hide label
        		self.warning.setVisible(False)
        		self.warning.setVisible(True)

        # Animation setup
		        self.animation = QPropertyAnimation(self.warning, b"windowOpacity")
		        self.animation.setDuration(3000)  # 2 seconds fade
		        self.animation.setStartValue(1.0)  # Fully visible
		        self.animation.setEndValue(0.0)  # Fully transparent
		        self.animation.start()

        # Hide the label after fade-out
        		self.animation.finished.connect(lambda: self.warning.setVisible(False))
   		time.sleep(0.1)
        
   	
   def publish_Right(self):
   	topic = "control/navigation/turn_right" # Specify the topic
   	while self.rotating:
   		if(self.mode == 1):
        		self.right = 1
        		self.client.publish(topic, self.right, qos=1)
        	
   		elif(self.mode == 2):
        		self.warning.setAlignment(Qt.AlignCenter)
        		self.warning.adjustSize()
        		self.ManualLabel.hide()
        		self.AutoLabel.hide()
        	
        		x = (self.width() - self.warning.width()) // 2
        		y = (self.height() - self.warning.height()) // 2
        		self.warning.move(x, y)
	
        # Initially hide label
        		self.warning.setVisible(False)
        		self.warning.setVisible(True)

        # Animation setup
		        self.animation = QPropertyAnimation(self.warning, b"windowOpacity")
		        self.animation.setDuration(3000)  # 2 seconds fade
		        self.animation.setStartValue(1.0)  # Fully visible
		        self.animation.setEndValue(0.0)  # Fully transparent
		        self.animation.start()

        # Hide the label after fade-out
        		self.animation.finished.connect(lambda: self.warning.setVisible(False))
   		time.sleep(0.1)


   def publish_Velocity(self):
        topic = "control/navigation/vel_control"  # Specify the topic
        if(self.mode == 1):
	        velocity = 1  # Message to send
        	self.client.publish(topic, abs(self.EditSpeed.value()), qos=1)  # Publish message
        	
        elif(self.mode == 2):
        	self.warning.setAlignment(Qt.AlignCenter)
        	self.warning.adjustSize()
        	self.ManualLabel.hide()
        	self.AutoLabel.hide()
        	
        	x = (self.width() - self.warning.width()) // 2
        	y = (self.height() - self.warning.height()) // 2
        	self.warning.move(x, y)

        # Initially hide label
        	self.warning.setVisible(False)
        	self.warning.setVisible(True)

        # Animation setup
	        self.animation = QPropertyAnimation(self.warning, b"windowOpacity")
        	self.animation.setDuration(3000)  # 2 seconds fade
	        self.animation.setStartValue(1.0)  # Fully visible
        	self.animation.setEndValue(0.0)  # Fully transparent
        	self.animation.start()

        # Hide the label after fade-out
        	self.animation.finished.connect(lambda: self.warning.setVisible(False))
        	
   def send_update(self):
        """ Emit signal to update label in the second window """
        self.signal_update.emit("Label updated from Main Window!")
       	
   def publish_ModeControlManual(self):
        topic = "control/mode_control"  # Specify the topic
        
        self.ManualLabel.adjustSize()

        # Center the message
        x = (self.width() - self.ManualLabel.width()) // 2
        y = (self.height() - self.ManualLabel.height()) // 2
        self.ManualLabel.move(x, y)
        self.ManualLabel.show()
        self.AutoLabel.hide()
        self.warning.hide()

        # Animation setup
        self.animation = QPropertyAnimation(self.ManualLabel, b"windowOpacity")
        self.animation.setDuration(2000)  # 2 seconds fade
        self.animation.setStartValue(1.0)  # Fully visible
        self.animation.setEndValue(0.0)  # Fully transparent
        self.animation.start()

        # Hide the label after fade-out
        self.animation.finished.connect(lambda: self.ManualLabel.setVisible(False))
        
        self.manual = 1;
        self.auto = 0;
        self.mode = 1;
        self.client.publish(topic, self.mode, qos=1)
        self.second_window.update_mode("Manual Mode");
        self.save_mode.value("control_status", 1)
        #QMessageBox.about(self, "Try Again!!!", "Valid values are Empty")
        
   def publish_ModeControlAuto(self):
        topic = "control/mode_control"  # Specify the topic
        self.AutoLabel.adjustSize()

        # Center the message
        x = (self.width() - self.AutoLabel.width()) // 2
        y = (self.height() - self.AutoLabel.height()) // 2
        self.AutoLabel.move(x, y)
        self.AutoLabel.show()
        self.ManualLabel.hide()
        self.warning.hide()

        # Animation setup
        self.animation = QPropertyAnimation(self.AutoLabel, b"windowOpacity")
        self.animation.setDuration(2000)  # 2 seconds fade
        self.animation.setStartValue(1.0)  # Fully visible
        self.animation.setEndValue(0.0)  # Fully transparent
        self.animation.start()

        # Hide the label after fade-out
        self.animation.finished.connect(lambda: self.AutoLabel.setVisible(False))
        
        self.auto = 1;
        self.manual = 0;
        self.mode = 2;
        self.forward = 1;
        self.client.publish(topic, self.mode, qos=1)
        self.client.publish(topic, self.forward, qos=1)
        self.second_window.update_mode("Autonomous Mode");
        self.save_mode.value("control_status", 1)
    
class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi("MainUI.ui", self)  # Load .ui file
        self.setupUi(self)
        self.connectSignalsSlots()
        
    def connectSignalsSlots(self):
        self.VehicleDiagnostics.clicked.connect(self.launch_Vehicle_Diagnostics)
        self.Sensors.clicked.connect(self.launch_Sensors)
        self.Control.clicked.connect(self.launch_Control)

    def launch_Vehicle_Diagnostics(self):
         self.main_window = MQTTClient()
         self.main_window.show()
    
    def launch_Sensors(self):
         self.main_window = Sensor_Client()
         self.main_window.show()
         
    def launch_Control(self):
         self.main_window = ControlPanel_Client()
         self.main_window.show()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
