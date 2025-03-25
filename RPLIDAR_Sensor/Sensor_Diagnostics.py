import time
import subprocess
import re
import paho.mqtt.client as mqtt
from pyrplidar import PyRPlidar

# MQTT Configuration
BROKER = "192.168.249.10"
PORT = 1883  # Default MQTT port

client = mqtt.Client()
client.connect(BROKER, PORT, 60)

# Define Lidar connection port
LIDAR_PORT = "/dev/ttyUSB0" 

# Initialize Lidar
lidar = PyRPlidar()
lidar.connect(LIDAR_PORT, 115200)

try:
    time.sleep(2)  # Allow Lidar to initialize

    # Infinite loop to fetch health data
    while True:
        # Fetch Lidar health data
        health = str(lidar.get_health())  # Convert to string for regex parsing
        
        device_info = str(lidar.get_info())  # Convert to string for regex parsing
        
        sample_rate = str(lidar.get_samplerate())  # Convert to string for regex parsing

        # Regex pattern to extract standard and express sample rate
        samp_pattern = r"\{'t_standard':\s*(\d+),\s*'t_express':\s*(\d+)\}"
        samp_match = re.match(samp_pattern, sample_rate)
    
        # Regex pattern to extract values
        devinf_pattern = r"\{'model':\s*(\d+),\s*'firmware_minor':\s*(\d+),\s*'firmware_major':\s*(\d+),\s*'hardware':\s*(\d+),\s*'serialnumber':\s*'([\dA-F]+)'\}"
        devinf_match = re.match(devinf_pattern, device_info)
        
        # Regex pattern to extract status and error code
        pattern = r"\{'status':\s*(\d+),\s*'error_code':\s*(\d+)\}"
        match = re.match(pattern, health)

        if match:
            status = int(match.group(1))  # Extract status
            error_code = int(match.group(2))  # Extract error_code
            #(f"Status: {status}, Error Code: {error_code}")
            if(status == 0):
                health = "GOOD";
                status = "ON";
                client.publish("sensors/RPLIDAR/diagnostics/health_status", health, qos=1)
                client.publish("sensors/RPLIDAR/diagnostics/status", status, qos=1)
        
            else:
                health = "ERROR/WARNING";
                status = "OFF";
                client.publish("sensors/RPLIDAR/diagnostics/health_status", health, qos=1)
                client.publish("sensors/RPLIDAR/diagnostics/status", status, qos=1)
            

        if devinf_match:
            model = int(devinf_match.group(1))
            client.publish("sensors/RPLIDAR/diagnostics/model_num", model, qos=1)
            
        if samp_match:
            t_standard = int(samp_match.group(1))
            client.publish("sensors/RPLIDAR/diagnostics/samplerate_standard", t_standard, qos=1)
            t_express = int(samp_match.group(2))
            client.publish("sensors/RPLIDAR/diagnostics/samplerate_express", t_express, qos=1)

        time.sleep(1)  # Delay before next check

except KeyboardInterrupt:
    print("\nStopping Lidar...")
    lidar.stop()
    lidar.set_motor_pwm(0)  # Stop motor
    lidar.disconnect()
    print("Lidar stopped and disconnected.")