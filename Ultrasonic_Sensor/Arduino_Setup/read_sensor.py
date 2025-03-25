import serial
import time
import paho.mqtt.client as mqtt

arduino = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)  # Wait for the connection to establish

# MQTT Configuration
BROKER = "192.168.249.10" 
PORT = 1883  # Default MQTT port

client = mqtt.Client()
client.connect(BROKER, PORT, 60)

while True:
    if arduino.in_waiting > 0:
        distance = arduino.readline().decode('utf-8').strip()
        print(f"Distance: {distance} cm")
        client.publish("sensors/Ultrasonic/navigation/distance", distance, qos=1)
