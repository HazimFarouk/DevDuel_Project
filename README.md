# Earthquake Search and Rescue Vehicle

This project implements an autonomous obstacle detection and alert system using an Arduino. The system utilizes ultrasonic sensors for obstacle detection, an LM35 sensor for body heat detection, a microphone for sound detection, and a GPS module to send location alerts via Bluetooth.

## Features

- **Obstacle Detection:** Uses four ultrasonic sensors to detect obstacles in front, back, left, and right directions.
- **Body Heat Detection:** Uses an LM35 temperature sensor to detect body heat.
- **Sound Detection:** Uses a microphone to detect sound above a specified threshold.
- **GPS Location:** Sends GPS location data when body heat or sound is detected.
- **Motor Control:** Controls motors to navigate around obstacles.

## Components Used

- Arduino Board
- 4 Ultrasonic Sensors (HC-SR04)
- LM35 Temperature Sensor
- Microphone Sensor
- GPS Module
- Bluetooth Module
- Motor Driver
- Motors
- Miscellaneous: Resistors, Wires, Breadboard, etc.

## Pin Configuration

- **Ultrasonic Sensors:**
  - Front: `Trig -> A1`, `Echo -> A2`
  - Back: `Trig -> 8`, `Echo -> 9`
  - Left: `Trig -> A3`, `Echo -> A4`
  - Right: `Trig -> 12`, `Echo -> 13`
- **LM35 Sensor:** `Analog Pin -> A0`
- **Microphone Sensor:** `Analog Pin -> A5`
- **GPS Module:** `TX -> 2`, `RX -> 3`
- **Bluetooth Module:** `TX -> 10`, `RX -> 11`
- **Motor Control:**
  - Left Motor: `Forward -> 4`, `Backward -> 5`
  - Right Motor: `Forward -> 6`, `Backward -> 7`

## Constants

- **Maximum Obstacle Detection Distance:** 10 cm
- **Body Heat Detection Threshold:** 30°C
- **Sound Detection**

## Libraries Required

- `NewPing`
- `SoftwareSerial`
- `TinyGPS++`

## Code Explanation

### Setup

The `setup` function initializes serial communication for debugging, GPS, and Bluetooth. It also configures the sensor and motor pins and feeds a simulated GPS data stream for testing purposes.

### Loop

The `loop` function continuously:
1. Reads distances from ultrasonic sensors.
2. Navigates the robot based on detected obstacles.
3. Reads body heat and sound levels.
4. Sends GPS location via Bluetooth if body heat or sound is detected.
5. Resumes navigation after alerting.

### Functions

- **getGPSLocation:** Returns the current GPS location as a string.
- **moveForward:** Moves the robot forward.
- **stopMotors:** Stops all motor movements.
- **turnLeft:** Turns the robot left.
- **turnRight:** Turns the robot right.
- **moveBackward:** Moves the robot backward.
- **readTemperature:** Reads and converts the analog value from the LM35 sensor to temperature in Celsius.
- **readSound:** Reads the sound from the microphone sensor.

## Usage

1. **Hardware Setup:**
   - Connect all sensors and modules as per the pin configuration.
   - Ensure the power supply to the Arduino and motors is adequate.

2. **Software Setup:**
   - Install the required libraries (`NewPing`, `SoftwareSerial`, `TinyGPS++`) via the Arduino Library Manager.
   - Upload the provided code to the Arduino.

3. **Operation:**
   - Power the system.
   - The robot will start moving forward and navigate around obstacles.
   - If body heat or sound is detected, it will send an alert with GPS location via Bluetooth.

## Notes

- The GPS data stream is simulated for testing purposes. In a real-world application, the GPS module will provide live data.
- Adjust the `bodyHeatThreshold` constant as per the specific environment and sensor characteristics.

## Troubleshooting

- Ensure all connections are secure and correct.
- Verify the power supply to the sensors and motors.
- Use serial monitor for debugging information.