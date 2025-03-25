import subprocess
import re
import paho.mqtt.client as mqtt
from pyrplidar import PyRPlidar
from datetime import datetime
import time


# MQTT Configuration
BROKER = "192.168.249.10"
PORT = 1883  # Default MQTT port

client = mqtt.Client()
client.connect(BROKER, PORT, 60)

# Run the ultra_simple executable
process = subprocess.Popen(
    ["./output/Linux/Release/ultra_simple", "--channel", "--serial", "/dev/ttyUSB0", "115200"],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True
)

# Regex to match LIDAR output lines (theta, distance, quality)
lidar_data_pattern = re.compile(r"theta:\s*([\d\.]+)\s+Dist:\s*([\d\.]+)\s+Q:\s*(\d+)")

# print(f"{'Angle (°)':} {'Distance (m)':} {'Quality'}")
# print("-" * 35)

class LIDAR():
    
    def __init__(self):
        for i in range(1, 6):
            setattr(self, f"theta{i}", 0)
        
        for i in range(1, 6):
            setattr(self, f"distance{i}", 0)
        

    def func(self):
        try:
    
            while True:
    # Read and process the output line by line
                for line in process.stdout:
                    match = lidar_data_pattern.search(line)
                    if match:
                        theta = float(match.group(1))  # Angle
                        distance = float(match.group(2))  # Distance in mm
                        quality = int(match.group(3))  # Quality score
                
                        if (theta >= 0 and theta < 72):
                            self.theta1 = theta
                            self.distance1 = distance/10
                            client.publish("sensors/RPLIDAR/navigation/ang1", self.theta1, qos=1)
                            client.publish("sensors/RPLIDAR/navigation/dist1", self.distance1, qos=1)
                            #print(f"   Angle: {theta1:.2f}°  |  Distance: {distance1:.2f} mm")
                    
                        elif (theta >= 72 and theta < 144):
                            self.theta2 = theta
                            self.distance2 = distance/10
                            client.publish("sensors/RPLIDAR/navigation/ang2", self.theta2, qos=1)
                            client.publish("sensors/RPLIDAR/navigation/dist2", self.distance2, qos=1)
                            #print(f"   Angle: {theta2:.2f}°  |  Distance: {distance2:.2f} mm")
                    
                        elif (theta >= 144 and theta < 216):
                            self.theta3 = theta
                            self.distance3 = distance/10
                            client.publish("sensors/RPLIDAR/navigation/ang3", self.theta3, qos=1)
                            client.publish("sensors/RPLIDAR/navigation/dist3", self.distance3, qos=1)
                            
                        elif (theta >= 216 and theta < 288):
                            self.theta4 = theta
                            self.distance4 = distance/10
                            client.publish("sensors/RPLIDAR/navigation/ang4", self.theta4, qos=1)
                            client.publish("sensors/RPLIDAR/navigation/dist4", self.distance4, qos=1) 
                            #print(f"   Angle: {self.theta4:.2f}°  |  Distance: {self.distance4:.2f} mm")
                    
                        elif (theta >= 288 and theta < 360):
                            self.theta5 = theta
                            self.distance5 = distance/10
                            client.publish("sensors/RPLIDAR/navigation/ang5", self.theta5, qos=1)
                            client.publish("sensors/RPLIDAR/navigation/dist5", self.distance5, qos=1)

                        # now = datetime.now()
                        # formatted = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        # print("Angle {}		Message published at {}".format(theta, formatted))
                        # time.sleep(150)
                        #print(f"1: {self.theta1:.2f}°  |  {self.distance1:.2f} mm	1: {self.theta2:.2f}°  |  2: {self.distance2:.2f} mm	1: {self.theta3:.2f}°  |  2: {self.distance3:.2f} mm")

        except KeyboardInterrupt:
            print("\nStopping LiDAR...")
            lidar.stop()
            lidar.disconnect()

self = LIDAR()
self.func()