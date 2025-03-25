#define TRIG_PIN 8
#define ECHO_PIN 9
#define MAX_DISTANCE 400  // Set the maximum valid distance in cm

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  long duration, distance;
  
  // Send a pulse to trigger the sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the pulse duration from the Echo pin
  float duration = pulseIn(ECHO_PIN, HIGH);
  
  // Convert the duration into distance (in cm)
  float distance = round(((duration / 2) / 29.1) * 100.0) / 100.0;

  // Only send valid distances
  if (distance > 0 && distance <= MAX_DISTANCE) {
    Serial.println(distance);  // Send only if within range
  }
  
  delay(200);  // Delay to avoid flooding the serial buffer
}
