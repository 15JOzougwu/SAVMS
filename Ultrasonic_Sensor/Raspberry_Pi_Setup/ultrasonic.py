import RPi.GPIO as GPIO
import time
import statistics
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt

# GPIO Pins
TRIG = 23
ECHO = 24
MAX_DISTANCE = 400  # Max valid reading in cm

# MQTT Configuration
BROKER = "192.168.249.10"
PORT = 1883  # Default MQTT port

client = mqtt.Client()
client.connect(BROKER, PORT, 60)

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def get_distance():
    """Measure distance using HC-SR04."""
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    timeout = start_time + 1  # Timeout after 1 second

    while GPIO.input(ECHO) == 0:
        start_time = time.time()
        if start_time > timeout:
            return None  # Timeout, return no result

    while GPIO.input(ECHO) == 1:
        end_time = time.time()
        if end_time > timeout:
            return None  # Timeout, return no result

    duration = end_time - start_time
    distance = round(duration * 17150, 2) # Calculate distance in centimetres

    if distance > MAX_DISTANCE:
        return None  # Ignore unrealistic readings
    
    return distance

def get_filtered_distance(samples=5):
    """Get a stable distance reading using median filtering."""
    distances = []
    for _ in range(samples):
        dist = get_distance()
        if dist is not None:
            distances.append(dist)
        time.sleep(0.05)  # Small delay between samples

    if distances:
        return round(statistics.median(distances), 2)  # Use median for stability
    return None

# 📈 Live Plot Setup
plt.ion()
fig, ax = plt.subplots()
x_data, y_data = [], []
line, = ax.plot(x_data, y_data, 'b-', label="Distance (cm)")

ax.set_xlabel("Time (s)")
ax.set_ylabel("Distance (cm)")
ax.set_title("Live Distance Measurements")
ax.legend()

try:
    start_time = time.time()
    
    while True:
        dist = get_filtered_distance()
        if dist is not None:
            elapsed_time = round(time.time() - start_time, 2)
            x_data.append(elapsed_time)
            y_data.append(dist)

            # Update Plot
            line.set_xdata(x_data)
            line.set_ydata(y_data)
            ax.relim()
            ax.autoscale_view()
            plt.draw()

            print(f"Distance: {dist} cm")
            client.publish("sensors/Ultrasonic/navigation/distance", dist, qos=1)
        else:
            print("Measurement failed.")

        time.sleep(0.2)  # Slightly slower to avoid interference

except KeyboardInterrupt:
    GPIO.cleanup()
    plt.ioff()
    plt.show()