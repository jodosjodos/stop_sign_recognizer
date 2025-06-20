# Stop Sign Detection and UART Integration System

## Project Overview

This project implements a real-time robotic system that uses computer vision to detect stop signs and communicates with a microcontroller to control robot movement. When a stop sign is detected, the system sends a stop signal to the Arduino for 3 seconds, then resumes normal operation.

## System Architecture

```
Camera → Python OpenCV → Stop Sign Detection → UART Communication → Arduino → Robot Control
```

## Features

- **Real-time stop sign detection** using both Haar cascade classifiers and color-based detection
- **UART communication** between Python and Arduino
- **Timed stop behavior** (3-second stop duration)
- **Safety timeout** mechanism
- **Visual feedback** with bounding boxes and status display
- **Simulation mode** for testing without actual stop signs

## Files Included

### Python Files

- `stop_sign_detector.py` - Main Python script for stop sign detection and UART communication

### Arduino Files

- `robot_control.ino` - Arduino code for receiving UART commands and controlling motors

## Hardware Requirements

### For Python Side

- Computer with camera (webcam or USB camera)
- USB-to-Serial adapter or Arduino with USB connection
- Python 3.7+ installed

### For Arduino Side

- Arduino Uno/Nano/Mega
- L298N Motor Driver or similar
- 2x DC Motors
- Power supply (6-12V for motors)
- Jumper wires
- Breadboard (optional)

## Software Requirements

### Python Packages

```
opencv-python==4.8.1.78
pyserial==3.5
numpy==1.24.3
```

### Arduino IDE

- Arduino IDE 1.8.19 or newer
- No additional libraries required (uses built-in functions)

## Installation and Setup

### 1. Python Environment Setup

```bash
# Create virtual environment (recommended)
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install required packages
pip install opencv-python pyserial numpy

# Alternative: Install from requirements.txt
pip install -r requirements.txt
```

### 2. Arduino Setup

1. Open Arduino IDE
2. Connect Arduino to computer via USB
3. Select correct board and port in Tools menu
4. Upload the `robot_control.ino` sketch

### 3. Hardware Connections

#### Motor Driver (L298N) Connections:

```
L298N → Arduino
IN1   → Pin 2
IN2   → Pin 3
IN3   → Pin 4
IN4   → Pin 5
ENA   → Pin 9 (PWM)
ENB   → Pin 10 (PWM)
VCC   → 5V
GND   → GND
```

#### Motors to L298N:

```
Left Motor  → OUT1, OUT2
Right Motor → OUT3, OUT4
```

#### Power Supply:

- Connect 6-12V external power to L298N motor power inputs
- Remove VCC jumper if using external power supply

### 4. Stop Sign Classifier (Optional Enhancement)

For better detection accuracy, download a pre-trained stop sign classifier:

1. Download from OpenCV's GitHub repository or train your own
2. Place the `.xml` file in the project directory
3. Update the file path in the Python script

## Usage Instructions

### 1. Basic Operation

1. **Connect Hardware**: Ensure Arduino and camera are connected
2. **Update Serial Port**: Modify the serial port in `stop_sign_detector.py`:
   ```python
   serial_port = 'COM3'  # Windows
   # or
   serial_port = '/dev/ttyUSB0'  # Linux
   ```
3. **Run the System**:
   ```bash
   python stop_sign_detector.py
   ```

### 2. Controls

- **'q'**: Quit the application
- **'s'**: Simulate stop sign detection (for testing)
- **ESC**: Exit video window

### 3. System Behavior

- **Green Status**: Robot moving (UART signal: 0)
- **Red Status**: Robot stopped (UART signal: 1)
- **Detection Duration**: 3 seconds stop period
- **Timeout Safety**: Robot stops if communication is lost

## Detection Methods

### 1. Haar Cascade Detection

- Uses machine learning-based object detection
- Requires trained classifier file
- More accurate but needs proper training data

### 2. Color-Based Detection

- Detects red octagonal shapes
- HSV color space filtering
- Contour analysis for shape verification
- Fallback method when classifier unavailable

## Troubleshooting

### Common Issues

1. **Camera Not Found**

   ```
   Error: Could not open camera
   ```

   - Check camera connection
   - Try different camera index (0, 1, 2...)
   - Verify camera permissions

2. **Serial Connection Failed**

   ```
   Error connecting to Arduino
   ```

   - Check COM port number
   - Verify Arduino is connected
   - Ensure correct baud rate (9600)
   - Check if port is already in use

3. **Poor Detection Accuracy**

   - Improve lighting conditions
   - Adjust color threshold values
   - Use better trained classifier
   - Ensure stop signs are clearly visible

4. **Robot Not Responding**
   - Check motor driver connections
   - Verify power supply
   - Test Arduino serial monitor
   - Check motor driver enable pins

### Debug Mode

Enable verbose output by adding debug prints:

```python
# Add to Python script for debugging
print(f"Serial data sent: {signal}")
print(f"Detection confidence: {detection_score}")
```

## Testing Procedure

### 1. System Testing Without Hardware

```bash
# Run with simulated serial connection
python stop_sign_detector.py
# Press 's' to simulate detection
```

### 2. UART Communication Test

```bash
# Arduino Serial Monitor should show:
# "STOP command received - Robot stopped"
# "MOVE command received - Robot moving"
```

### 3. Full System Integration Test

1. Place stop sign in camera view
2. Verify detection rectangle appears
3. Confirm robot stops for 3 seconds
4. Check robot resumes movement

## Performance Optimization

### For Better Real-time Performance:

- Reduce camera resolution
- Optimize detection parameters
- Use multi-threading for UART communication
- Implement frame skipping if needed

### Memory Optimization:

- Limit cascade detection area
- Reduce color mask resolution
- Clear OpenCV buffers regularly

## Future Enhancements

### Possible Improvements:

1. **Deep Learning Detection**: Use YOLO or SSD models
2. **Multiple Sign Types**: Detect traffic lights, yield signs
3. **GPS Integration**: Location-based behavior
4. **Speed Control**: Variable speed based on conditions
5. **Obstacle Avoidance**: Additional sensors integration
6. **Data Logging**: Record detection events
7. **Wireless Communication**: Replace UART with WiFi/Bluetooth

### Advanced Features:

- Machine learning model training
- Real-time parameter tuning
- Multi-camera support
- Remote monitoring dashboard

## Safety Considerations

1. **Timeout Protection**: System stops robot if communication fails
2. **Emergency Stop**: Manual override capability
3. **Power Management**: Proper power supply ratings
4. **Fail-Safe Operation**: Default to stopped state

## License and Credits

This project is created for educational purposes. Feel free to modify and distribute according to your institution's guidelines.

### Acknowledgments:

- OpenCV community for computer vision tools
- Arduino community for microcontroller resources
- Academic resources for robotics integration concepts

## Contact and Support

For technical issues or questions about this implementation, refer to:

- OpenCV documentation: https://docs.opencv.org/
- Arduino documentation: https://www.arduino.cc/reference/
- PySerial documentation: https://pyserial.readthedocs.io/

---

**Note**: This system is designed for educational and research purposes. For production use, additional safety measures and testing would be required.
