# Design and Implementation of an AI-Powered Multi-Sensor Hexapod Robot for Earthquake Search and Rescue

## Overview

This project presents the design and implementation of an AI-powered multi-sensor hexapod robot for earthquake search and rescue applications.

The robot is designed to operate in complex and hazardous environments where direct human access may be dangerous. By combining artificial intelligence, embedded systems, wireless communication and a six-legged locomotion platform, the system assists rescue teams in locating potential victims and monitoring disaster areas remotely.

The project was developed as a Graduation Thesis at Ho Chi Minh City University of Technology (HCMUT), Vietnam National University Ho Chi Minh City.

---

## Project Objectives
- Develop a hexapod robot capable of traversing rough terrain.
- Detect humans and animals using AI-based image recognition.
- Detect human and animal sounds using AI-based audio classification.
- Monitor post-earthquake vibrations and aftershocks.
- Support both manual and autonomous operation modes.
- Deploy rescue markers at suspected victim locations.
- Transmit real-time information to a remote control station.
- Provide live FPV video streaming and system monitoring dashboard.

---

## System Architecture

### Robot Platform
- Hexapod structure with 18 DOF.
- Tripod gait locomotion.
- Manual and autonomous navigation modes.
- Obstacle avoidance using distance sensors.

### AI Vision System
- Raspberry Pi 5.
- Raspberry Pi Camera V2.
- YOLOv8 object detection.
- Detection classes: Person, Dog and Cat.

### AI Audio System
- INMP441 I2S Microphone.
- CNN-based audio classification.
- Detection classes: Human voice, Dog bark and Environmental noise.

### Vibration Monitoring
- MPU6050 Accelerometer.
- PGA (Peak Ground Acceleration) calculation.
- MMI (Modified Mercalli Intensity) estimation.
- Aftershock monitoring.

### Wireless Communication
- LoRa SX1278.
- nRF24L01.
- Real-time sensor transmission.
- Remote command and control.

### FPV Monitoring
- FPV Camera.
- 5.8 GHz Video Transmission.
- Real-time video feedback to operator.

### Rescue Marker System
- Marker deployment mechanism.
- Wireless status reporting.
- Rescue state monitoring.

### Web Dashboard
- Real-time sensor monitoring.
- Robot status visualization.
- Marker tracking.
- Alert notifications.

---

## Technologies Used

### Main Hardware
- Raspberry Pi 5.
- Raspberry Pi 3
- Raspberry Pi Camera V2.
- INMP441 Microphone.
- MPU6050.
- LoRa SX1278.
- nRF24L01.
- PCA9685 Servo Driver.
- MG946R Servo Motors.
- FPV Camera System.

### Software
- Python.
- YOLOv8.
- TensorFlow / Keras.
- OpenCV.
- Flask.
- HTML / CSS / JavaScript.

### PCB Design
- Altium Designer.

### Mechanical Design
- CAD Modeling.
- 3D Printed Components.

---

## Demo Videos
YouTube Playlist: https://www.youtube.com/playlist?list=PLQ2nYnHPs12wPHYRxYziXRSKi9JFzzbWc

The playlist includes:
- Hexapod locomotion testing.
- Manual control mode.
- Autonomous navigation mode.
- Obstacle avoidance.
- Human detection.
- Audio detection.
- Marker deployment.
- Dashboard monitoring.
- System integration tests.

---

## Results
The developed system successfully demonstrates:
- Stable hexapod locomotion.
- Real-time AI-based human and animal detection.
- AI-based audio recognition.
- Obstacle avoidance capability.
- Aftershock monitoring.
- Wireless communication between robot and control station.
- Rescue marker deployment.
- FPV video transmission.
- Real-time dashboard monitoring.

The integration of AI, embedded systems, wireless communication and multi-sensor fusion improves situational awareness and supports search-and-rescue operations in hazardous environments.

---

## Authors
Ngô Diễm Quỳnh – 2212887

Nguyễn Trọng Tuấn – 2213794

Department of Electronics and Telecommunications Engineering

Ho Chi Minh City University of Technology (HCMUT)

Vietnam National University Ho Chi Minh City

Supervisor: TS. Nguyễn Lý Thiên Trường

---

## License

This repository is published for academic and portfolio purposes.
Please contact the authors before using the materials for commercial applications.
