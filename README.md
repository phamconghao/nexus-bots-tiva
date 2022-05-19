# Nexus Robot Controller Project for TI Tiva TM4C123
## nexus-bots-tiva
---

## Prerequisites
### Hardware
- Tiva C Launchpad TM4C123
- Nexus robot controller Board with BoosterPack Compatible
- Motor driver: VNH2SP30
- IMU: BMX160
- LCD 20x4
- Nexus robot base

### Software
- Arduino framework with Titiva platform in PlatformIO
- Firmware flashing tool OpenOCD

---

## Firmware features
** To enable demo mode, un-comment DEMO macro in [firmware/include/demo.h](https://github.com/khuehm17/nexus-bots-tiva/blob/main/firmware/include/demo.h), or vice sera. **
### Simple Demo
1. LED Blink Demo: 
	- ledBlink_demo();
2. LED Blink with PWM Demo: 
	- ledBlinkPWM_demo();
3. External IO Interrupt Demo: 
	- externalInterrupt_demo();
4. Serial Debug Printf Demo: 
	- dbgPrintf_demo();
5. PID Motor Control Demo: 
	- pidMotorControl_demo();
6. Timer Interrupt Demo: 
	- tmrInterrupt_demo(void);
	- TimerInterruptDemo_Handler();
7. ROS communication Demo: 
	- ros_PubSub_demo();
8. IMU Demo: 
	- bmx160_demo();
9. LCD i2c Demo:
	- lcd16x2_I2CLCD_demo();

### Nexus robot autonomous control

---

## Documentation
- Hardware schematic: docs/schematic_v1.0.pdf
- Pin diagram: docs/pin_diagram.pdf
- Software diagram: docs/software_digram/html-docs/index.html
