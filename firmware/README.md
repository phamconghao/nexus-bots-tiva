# nexus-bots-tiva
---
```
    This is a small implementation to port Nexus control program from AVR to Tiva C platform with the combination of ROSserial.
    Beside that, there are some simple demo of basic example.
```
## Platform
- Tiva C TM4C123
- Arduino framework with PlatformIO extension on VSCode
- ROS Noetic on Ubuntu 20.04 LTS

## Implemented Demo

### 1. LED Blink Demo
Blink the built-in LED on the Tiva C board with period 500ms.

** Implemented functions: 
  - void ledBlink_demo();
### 2. LED Blink with PWM Demo
Blink the built-in LED on the Tiva C board with period 500ms with fade-in and fade-out feature.

** Implemented functions: 
  - void ledBlinkPWM_demo();
### 3. External IO Interrupt Demo
Demonstration of External IO interrupt with Tiva C on-board button SW1.

** Implemented functions: 
  - void externalInterrupt_demo();
  - void extInterruptDemoHandler();
### 4. Serial Debug Printf Demo

### 5. PID Motor Control Demo

### 6. Timer Interrupt Demo

### 7. ROS communication Demo

After uploading firmware to Tiva, on ROS PC (with installed rosserial package), run bellow command:
```
    $ roscore
    $ rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
    $ rostopic echo tiva_chatter
    $ rostopic pub toggle_led std_msgs/Empty --once
```
