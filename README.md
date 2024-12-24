# Distance Measurement and Obstacle Avoidance System Using STM32 Nucleo F401RE

## Description

This archive contains my final semester project code.

The distance measurement and obstacle avoidance system is designed based on the STM32 Nucleo F401RE, featuring the following main functions:

1. **Display Information on LCD**:
   - The LCD communicates via I2C to display distance and status.

2. **Distance Measurement and Motor Control**:
   - Ultrasonic sensors are used for distance measurement.
   - DC motors are controlled using timers.
   - Distance is verified through UART communication on Putty before displaying on the LCD.

3. **Button Functionality**:
   - Buttons are used to perform various functions through external interrupts.

4. **LED Functionality**:
   - The `getTick()` function is used to prevent system blocking.

5. **Motor Control**:
   - The L298N driver is used to control two motors, with PWM signals provided to the ENA and ENB pins. To simplify, you only need to supply signals to IN1 and IN3.

---

## System Diagram

<p align="center">  
  <img src="overview.png" alt="Pin Connection Diagram" width="600"/>  
</p> 

## Pin Configuration

**Sensor**
- TRIG      PA9
- ECHO      PA10

**LEDs**
- LED_Green PA6
- LED_RED   PA7

**BUTTON**
- PC13  

**L298N Driver**
- IN1_PIN PA0  
- IN2_PIN PA1  
- ENA     PA5
- IN3_PIN PA4  
- IN4_PIN PB0  
- ENB     PB7

```c
#define SLAVE_ADDRESS_LCD 0x27


# Results
<p align="center">  
  <img src="car_1.png" alt="Robot" width="600"/>  
</p> 

<p align="center">  
  <img src="lcd.jpg" alt="LCD Output" width="600"/>  
</p>
