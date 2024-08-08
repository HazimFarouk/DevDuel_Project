#include <NewPing.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Pin Definitions
const int trigPins[4] = {A1, 8, A3, 12};
const int echoPins[4] = {A2, 9, A4, 13};
const int lm35Pin = A0; 
const int micPin = A5;
const int gpsTxPin = 2;
const int gpsRxPin = 3;
const int btTxPin = 10;
const int btRxPin = 11;

const int motorLeftForward = 4;
const int motorLeftBackward = 5;
const int motorRightForward = 6;
const int motorRightBackward = 7;

// Constants
const int maxDistance = 10; // Maximum distance for obstacle detection (cm)
const double bodyHeatThreshold = 30.0; // Body heat detection threshold (°C)

// Create instances
NewPing sonar[4] = {
    NewPing(trigPins[0], echoPins[0], maxDistance), // Front
    NewPing(trigPins[1], echoPins[1], maxDistance), // Back
    NewPing(trigPins[2], echoPins[2], maxDistance), // Left
    NewPing(trigPins[3], echoPins[3], maxDistance)  // Right
};
SoftwareSerial gpsSerial(gpsTxPin, gpsRxPin);
SoftwareSerial btSerial(btTxPin, btRxPin);
TinyGPSPlus gps;

// A sample NMEA stream for simulation
const char *gpsStream =
"$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
"$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
"$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
"$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
"$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
"$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  btSerial.begin(9600);

  // Initialize sensor pins
  pinMode(lm35Pin, INPUT);
  pinMode(micPin, INPUT);

  // Initialize motor pins
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftBackward, OUTPUT);
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightBackward, OUTPUT);

  // Feed the simulated GPS data to the TinyGPS++ library
  for (const char *p = gpsStream; *p; p++) {
    gps.encode(*p);
  }

}

void loop() {
  // Read distances from ultrasonic sensors
  float distances[4];
  for (int i = 0; i < 4; i++) {
    distances[i] = sonar[i].ping_cm();
  }

  // Determine if any obstacles are detected and navigate accordingly
  if (distances[0] > 0 && distances[0] < maxDistance) { // If in front, move right
    turnRight();
    delay(500);
  } else if (distances[2] > 0 && distances[2] < maxDistance) { // If on the left, move right
    turnRight();
    delay(500);
  } else if (distances[3] > 0 && distances[3] < maxDistance) { // If on the right, move left
    turnLeft();
    delay(500);
  }
  moveForward();

  // Read body heat and sound detection
  double bodyHeat = readTemperature();
  bool heatDetected = (bodyHeat >= bodyHeatThreshold);

  bool SoundDetected = readSound();
  //delay(50); // Brief delay for stability

  // If either sound or heat is detected, send GPS location
  if (SoundDetected || heatDetected) {
    stopMotors();

    // Get and send GPS location
    String location = getGPSLocation();
    btSerial.print("Alert! Body detected. Location: ");
    btSerial.println(location);
    Serial.println(location);
    delay(50); // Pause briefly

    // Resume moving forward
    moveForward();
    delay(500);
  }
}

String getGPSLocation() {
  //if (gps.location.isUpdated()) {
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    return "Lat: " + String(latitude, 6) + ", Lon: " + String(longitude, 6);
  //}
  //return "No GPS data";
}

void moveForward() {
  digitalWrite(motorLeftForward, HIGH);
  digitalWrite(motorLeftBackward, LOW);
  digitalWrite(motorRightForward, HIGH);
  digitalWrite(motorRightBackward, LOW);
}

void stopMotors() {
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftBackward, LOW);
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightBackward, LOW);
}

void turnLeft() {
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftBackward, LOW);
  digitalWrite(motorRightForward, HIGH);
  digitalWrite(motorRightBackward, LOW);
}

void turnRight() {
  digitalWrite(motorLeftForward, HIGH);
  digitalWrite(motorLeftBackward, LOW);
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightBackward, LOW);
}

void moveBackward() {
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftBackward, HIGH);
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightBackward, HIGH);
}

double readTemperature() {
  int rawValue = analogRead(lm35Pin);
  // Convert the analog reading to voltage (assuming 5V reference)
  float voltage = (rawValue / 1024.0) * 5.0;
  // Convert voltage to temperature (LM35 outputs 10mV/°C)
  float temperature = voltage * 100.0;
  return temperature;
}

bool readSound() {
  return digitalRead(micPin);;
}
