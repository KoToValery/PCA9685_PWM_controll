# PCA9685 PWM Controller

<cite>
**Referenced Files in This Document**
- [run.py](file://run.py)
- [config.yaml](file://config.yaml)
</cite>

## Table of Contents
1. [Introduction](#introduction)
2. [Project Structure](#project-structure)
3. [Core Components](#core-components)
4. [Architecture Overview](#architecture-overview)
5. [Detailed Component Analysis](#detailed-component-analysis)
6. [Dependency Analysis](#dependency-analysis)
7. [Performance Considerations](#performance-considerations)
8. [Troubleshooting Guide](#troubleshooting-guide)
9. [Conclusion](#conclusion)

## Introduction
This document provides comprehensive technical documentation for a PCA9685 16-channel 12-bit PWM controller implementation. The PCA9685 is a sophisticated I2C-controlled PWM IC capable of driving up to 16 independent channels with 12-bit resolution (0-4095), enabling precise control of loads such as heaters, fans, steppers, and LEDs. This implementation integrates MQTT-based Home Assistant discovery, hardware feedback verification, and robust thread-safe I2C communication protocols.

Key technical specifications:
- 12-bit PWM resolution (0-4095)
- 16 independent channels (0-15)
- Configurable PWM frequencies via prescaler values
- Thread-safe I2C communication using SMBus2
- Hardware feedback verification through PCA9539 GPIO expander
- MQTT-based Home Assistant integration with automatic discovery

## Project Structure
The project follows a modular Python architecture centered around a single primary script that orchestrates multiple hardware components and services:

```mermaid
graph TB
subgraph "Main Application"
RunPy[run.py]
PCA9685Class[PCA9685 Class]
PCA9539Class[PCA9539 Class]
PCA9540BClass[PCA9540B Class]
BME280Class[BME280 Class]
end
subgraph "Hardware Interfaces"
SMBus[SMBus2 I2C Bus]
I2CBus[I2C Bus 1]
GPIOExpander[PCA9539 GPIO Expander]
PWMController[PCA9685 PWM Controller]
SensorMux[PCA9540B Multiplexer]
WeatherSensor[BME280 Sensors]
end
subgraph "System Services"
MQTTBroker[MQTT Broker]
HAIntegration[Home Assistant Discovery]
DiagnosticThread[Hardware Diagnostic]
FeedbackThread[Feedback Monitoring]
SensorThread[Weather Sensor Reading]
LEDIndicator[RGB LED Indicator]
SystemLED[System LED]
end
RunPy --> PCA9685Class
RunPy --> PCA9539Class
RunPy --> PCA9540BClass
RunPy --> BME280Class
PCA9685Class --> SMBus
PCA9539Class --> SMBus
PCA9540BClass --> SMBus
BME280Class --> SMBus
SMBus --> I2CBus
I2CBus --> GPIOExpander
I2CBus --> PWMController
I2CBus --> SensorMux
I2CBus --> WeatherSensor
RunPy --> MQTTBroker
RunPy --> HAIntegration
RunPy --> DiagnosticThread
RunPy --> FeedbackThread
RunPy --> SensorThread
RunPy --> LEDIndicator
RunPy --> SystemLED
```

**Diagram sources**
- [run.py:1-50](file://run.py#L1-L50)
- [run.py:61-160](file://run.py#L61-L160)

**Section sources**
- [run.py:1-50](file://run.py#L1-L50)
- [config.yaml:1-57](file://config.yaml#L1-L57)

## Core Components

### PCA9685 PWM Controller Class
The PCA9685 class encapsulates all PWM functionality with thread-safe I2C operations:

```mermaid
classDiagram
class PCA9685 {
+int address
+SMBus bus
+__init__(bus, address)
+close() void
+set_pwm_freq(freq_hz) void
+set_pwm(channel, on, off) void
+set_duty_12bit(channel, duty) void
-_write8(reg, val) void
-_read8(reg) int
}
class ChannelMapping {
+int CH_PWM1 = 0
+int CH_HEATER_1 = 1
+int CH_HEATER_2 = 2
+int CH_HEATER_3 = 3
+int CH_HEATER_4 = 4
+int CH_FAN_1_POWER = 5
+int CH_FAN_2_POWER = 6
+int CH_STEPPER_DIR = 7
+int CH_STEPPER_ENA = 8
+int CH_PU = 9
+int CH_PWM2 = 10
+int CH_RESERVE1 = 11
+int CH_LED_RED = 12
+int CH_LED_BLUE = 13
+int CH_LED_GREEN = 14
+int CH_SYS_LED = 15
}
PCA9685 --> ChannelMapping : "uses"
```

**Diagram sources**
- [run.py:61-109](file://run.py#L61-L109)
- [run.py:266-282](file://run.py#L266-L282)

### Hardware Component Classes
The implementation includes specialized classes for each hardware component:

```mermaid
classDiagram
class PCA9539 {
+int address
+SMBus bus
+__init__(bus, address)
+close() void
+read_inputs() int
-INPUT0 = 0x00
-INPUT1 = 0x01
-OUTPUT0 = 0x02
-OUTPUT1 = 0x03
-CONFIG0 = 0x06
-CONFIG1 = 0x07
}
class PCA9540B {
+int address
+SMBus bus
+__init__(bus, address)
+close() void
+select_channel(channel_code) void
+deselect_channels() void
-CH_NONE = 0x00
-CH0 = 0x04
-CH1 = 0x05
}
class BME280 {
+int address
+SMBus bus
+__init__(bus, address)
+close() void
+read_data() tuple
-cal dict
-is_bme bool
}
PCA9685 --> PCA9539 : "feedback monitoring"
PCA9539 --> PCA9540B : "mux support"
PCA9540B --> BME280 : "sensor selection"
```

**Diagram sources**
- [run.py:111-160](file://run.py#L111-L160)
- [run.py:162-264](file://run.py#L162-L264)

**Section sources**
- [run.py:61-160](file://run.py#L61-L160)
- [run.py:162-264](file://run.py#L162-L264)

## Architecture Overview

### System Architecture
The PCA9685 controller implements a multi-threaded architecture with dedicated workers for different system functions:

```mermaid
sequenceDiagram
participant Main as Main Application
participant PCA9685 as PCA9685 Controller
participant PCA9539 as GPIO Expander
participant MQTT as MQTT Broker
participant Threads as Worker Threads
Main->>PCA9685 : Initialize with I2C Bus
Main->>PCA9539 : Initialize GPIO Expander
Main->>Threads : Start System LED Thread
Main->>Threads : Start LED Indicator Thread
Main->>Threads : Start PCA9539 Worker Thread
Main->>Threads : Start BME280 Worker Thread
Main->>Threads : Start PU Worker Thread
loop Main Loop
Main->>MQTT : Publish Discovery Messages
Main->>Threads : Monitor System Status
end
Note over Main,Threads : Thread-Safe Operations with Locks
```

**Diagram sources**
- [run.py:571-604](file://run.py#L571-L604)
- [run.py:1128-1226](file://run.py#L1128-L1226)

### Channel Mapping System
The fixed channel mapping assigns specific functions to PCA9685 channels:

```mermaid
graph LR
subgraph "Channel Assignments"
PWM1[Channel 0<br/>PWM1]
Heater1[Channels 1-4<br/>Heaters]
Fan1[Channel 5<br/>Fan 1 Power]
Fan2[Channel 6<br/>Fan 2 Power]
StepperDir[Channel 7<br/>Stepper DIR]
StepperEna[Channel 8<br/>Stepper ENA]
PU[Channel 9<br/>PU Signal]
PWM2[Channel 10<br/>PWM2]
Reserve[Channel 11<br/>Reserve]
LEDRed[Channel 12<br/>LED Red]
LEDBlue[Channel 13<br/>LED Blue]
LEDGreen[Channel 14<br/>LED Green]
SysLED[Channel 15<br/>System LED]
end
PWM1 -.-> Heater1
Heater1 -.-> Fan1
Fan1 -.-> Fan2
Fan2 -.-> StepperDir
StepperDir -.-> StepperEna
StepperEna -.-> PU
PU -.-> PWM2
PWM2 -.-> Reserve
Reserve -.-> LEDRed
LEDRed -.-> LEDBlue
LEDBlue -.-> LEDGreen
LEDGreen -.-> SysLED
```

**Diagram sources**
- [run.py:266-282](file://run.py#L266-L282)
- [run.py:930-944](file://run.py#L930-L944)

**Section sources**
- [run.py:266-282](file://run.py#L266-L282)
- [run.py:930-944](file://run.py#L930-L944)

## Detailed Component Analysis

### PWM Frequency Management
The PCA9685 implements precise frequency control through prescaler calculations:

```mermaid
flowchart TD
Start([Set PWM Frequency]) --> CalcPrescale["Calculate Prescaler Value"]
CalcPrescale --> ClampRange["Clamp to Valid Range<br/>3 ≤ prescale ≤ 255"]
ClampRange --> EnterSleep["Enter Sleep Mode"]
EnterSleep --> WritePrescale["Write Prescaler Register"]
WritePrescale --> RestoreMode["Restore Previous Mode"]
RestoreMode --> RestartController["Restart Controller"]
RestartController --> End([Frequency Updated])
subgraph "Prescaler Calculation"
Formula["prescale = round(OSC_HZ/(4096 × freq) - 1)"]
OSC_HZ["OSC_HZ = 25,000,000 Hz"]
ValidRange["Valid Range: 3-255"]
end
CalcPrescale --> Formula
Formula --> ClampRange
ClampRange --> ValidRange
```

**Diagram sources**
- [run.py:79-93](file://run.py#L79-L93)

### Duty Cycle Calculation Methods
The implementation provides multiple approaches to calculate duty cycles:

```mermaid
flowchart TD
Start([Duty Cycle Request]) --> Method{"Select Method"}
Method --> |Direct 12-bit| DirectCalc["Direct 12-bit:<br/>duty = value<br/>range: 0-4095"]
Method --> |Percentage| PercentCalc["Percentage:<br/>duty = (percent/100) × 4095<br/>range: 0-100%"]
Method --> |Motor Compensation| MotorCalc["Motor Compensation:<br/>visual = value<br/>if visual == 0: pwm_percent = 100<br/>elif 0 < visual ≤ 10: pwm_percent = 90<br/>else: pwm_percent = 100 - visual<br/>duty = (pwm_percent/100) × 4095"]
DirectCalc --> ValidateRange["Validate Range<br/>max(0, min(4095, duty))"]
PercentCalc --> ValidateRange
MotorCalc --> ValidateRange
ValidateRange --> ApplyPWM["Apply PWM Setting"]
ApplyPWM --> End([Duty Cycle Applied])
subgraph "Channel Specific Mappings"
PWM1Mapping["PWM1: visual = value<br/>pwm_percent = 100 - value"]
PWM2Mapping["PWM2: visual = value<br/>pwm_percent = 100 - value"]
end
MotorCalc --> PWM1Mapping
MotorCalc --> PWM2Mapping
```

**Diagram sources**
- [run.py:106-109](file://run.py#L106-L109)
- [run.py:898-928](file://run.py#L898-L928)

### Thread-Safe I2C Communication Protocol
The implementation ensures thread-safe I2C operations through synchronized access:

```mermaid
sequenceDiagram
participant Thread1 as Thread 1
participant Thread2 as Thread 2
participant Lock as i2c_lock
participant SMBus as SMBus Instance
participant PCA9685 as PCA9685 Device
Thread1->>Lock : acquire()
Lock->>Thread1 : lock acquired
Thread1->>SMBus : write_byte_data(address, register, value)
SMBus->>PCA9685 : I2C Write Operation
Thread1->>Lock : release()
Lock->>Thread1 : lock released
Note over Thread1,Thread2 : Other threads blocked until lock released
Thread2->>Lock : acquire()
Lock->>Thread2 : lock acquired
Thread2->>SMBus : read_byte_data(address, register)
SMBus->>PCA9685 : I2C Read Operation
Thread2->>Lock : release()
Lock->>Thread2 : lock released
```

**Diagram sources**
- [run.py:39-46](file://run.py#L39-L46)
- [run.py:73-78](file://run.py#L73-L78)

**Section sources**
- [run.py:79-93](file://run.py#L79-L93)
- [run.py:898-928](file://run.py#L898-L928)
- [run.py:39-46](file://run.py#L39-L46)

### Hardware Feedback Verification System
The PCA9539 GPIO expander provides comprehensive feedback verification:

```mermaid
flowchart TD
Start([Feedback Verification]) --> ReadInputs["Read PCA9539 Inputs"]
ReadInputs --> CompareStates["Compare Expected vs Actual States"]
CompareStates --> Relays{"Relay Feedback?"}
Relays --> |Yes| RelayLogic["Relay Logic:<br/>Low (0) = ON<br/>High (1) = OFF"]
Relays --> |No| StepperLogic["Stepper Logic:<br/>High (1) = ON<br/>Low (0) = OFF"]
RelayLogic --> VerifyRelays["Verify Relay States"]
StepperLogic --> VerifyStepper["Verify Stepper Signals"]
VerifyRelays --> CheckProblems{"Problems Found?"}
VerifyStepper --> CheckProblems
CheckProblems --> |Yes| SetError["Set System Status: ERROR"]
CheckProblems --> |No| SetOK["Set System Status: OK"]
SetError --> PublishFeedback["Publish Feedback Topics"]
SetOK --> PublishFeedback
PublishFeedback --> End([Verification Complete])
```

**Diagram sources**
- [run.py:673-798](file://run.py#L673-L798)
- [run.py:950-992](file://run.py#L950-L992)

**Section sources**
- [run.py:673-798](file://run.py#L673-L798)
- [run.py:950-992](file://run.py#L950-L992)

### Stepper Motor Control Implementation
The stepper motor control system implements safe switching with pulse generation:

```mermaid
sequenceDiagram
participant User as User Command
participant Controller as Stepper Controller
participant PUWorker as PU Worker
participant PCA9685 as PCA9685 Controller
participant Feedback as Feedback System
User->>Controller : Change Direction (CW/CCW)
Controller->>Controller : Disable Pulse Generation
Controller->>Controller : Wait for Current Pulse to Finish
Controller->>Controller : Change DIR Signal
Controller->>Controller : Wait ≥ 50ms Setup Time
Controller->>Controller : Resume Pulse Generation (if enabled)
loop Pulse Generation
Controller->>PUWorker : Generate Pulses
PUWorker->>PCA9685 : Set CH9 High/Low
PCA9685->>Feedback : Monitor PU Feedback
Feedback->>PUWorker : Verify Pulse Detection
end
Note over Controller,Feedback : Safe Switching with Hardware Verification
```

**Diagram sources**
- [run.py:998-1036](file://run.py#L998-L1036)
- [run.py:1044-1105](file://run.py#L1044-L1105)

**Section sources**
- [run.py:998-1036](file://run.py#L998-L1036)
- [run.py:1044-1105](file://run.py#L1044-L1105)

## Dependency Analysis

### Hardware Component Dependencies
The system architecture establishes clear dependencies between hardware components:

```mermaid
graph TB
subgraph "Primary Dependencies"
PCA9685[PCA9685 PWM Controller]
SMBus2[SMBus2 Library]
I2CBus[I2C Bus 1]
end
subgraph "Secondary Dependencies"
PCA9539[PCA9539 GPIO Expander]
PCA9540B[PCA9540B Multiplexer]
BME280[BME280 Sensors]
end
subgraph "Software Dependencies"
MQTTClient[paho-mqtt Client]
JSON[JSON Processing]
Threading[Threading Module]
Logging[Logging Module]
end
PCA9685 --> SMBus2
SMBus2 --> I2CBus
PCA9685 --> PCA9539
PCA9539 --> PCA9540B
PCA9540B --> BME280
PCA9685 --> MQTTClient
PCA9685 --> JSON
PCA9685 --> Threading
PCA9685 --> Logging
```

**Diagram sources**
- [run.py:20-21](file://run.py#L20-L21)
- [run.py:111-160](file://run.py#L111-L160)

### Software Architecture Dependencies
The application follows a layered architecture with clear separation of concerns:

```mermaid
graph TB
subgraph "Application Layer"
MainApp[Main Application]
MQTTHandler[MQTT Message Handler]
HardwareManager[Hardware Manager]
end
subgraph "Service Layer"
PWMService[PWM Service]
FeedbackService[Feedback Service]
SensorService[Sensor Service]
LEDService[LED Service]
end
subgraph "Hardware Abstraction Layer"
PCA9685Class[PCA9685 Class]
PCA9539Class[PCA9539 Class]
PCA9540BClass[PCA9540B Class]
BME280Class[BME280 Class]
end
subgraph "System Layer"
SMBusInterface[SMBus Interface]
I2CInterface[I2C Interface]
ThreadSafety[Thread Safety]
end
MainApp --> MQTTHandler
MainApp --> HardwareManager
HardwareManager --> PWMService
HardwareManager --> FeedbackService
HardwareManager --> SensorService
HardwareManager --> LEDService
PWMService --> PCA9685Class
FeedbackService --> PCA9539Class
SensorService --> PCA9540BClass
LEDService --> PCA9539Class
PCA9685Class --> SMBusInterface
PCA9539Class --> SMBusInterface
PCA9540BClass --> SMBusInterface
BME280Class --> SMBusInterface
SMBusInterface --> I2CInterface
I2CInterface --> ThreadSafety
```

**Diagram sources**
- [run.py:571-604](file://run.py#L571-L604)
- [run.py:1228-1248](file://run.py#L1228-L1248)

**Section sources**
- [run.py:20-21](file://run.py#L20-L21)
- [run.py:571-604](file://run.py#L571-L604)

## Performance Considerations

### I2C Bus Performance
The implementation optimizes I2C communication through several mechanisms:

- **Shared Bus Access**: Single SMBus instance shared across all components
- **Thread Safety**: Global lock (`i2c_lock`) prevents concurrent I2C operations
- **Batch Operations**: I2C block writes for efficient register updates
- **Timing Delays**: Strategic delays for device stability during mode changes

### Memory Management
The application implements efficient memory usage patterns:

- **Object Lifecycle**: Proper cleanup of hardware objects during shutdown
- **Thread Management**: Daemon threads that terminate automatically
- **Resource Cleanup**: Context managers and explicit close() calls
- **Configuration Caching**: Static configuration loaded once at startup

### Real-Time Performance
Hardware feedback and control operations are designed for real-time responsiveness:

- **Feedback Sampling**: 1-second intervals for PCA9539 monitoring
- **Sensor Reading**: Configurable intervals for BME280 sensors
- **LED Indicators**: Non-blocking LED pattern updates
- **MQTT Publishing**: Asynchronous message handling

## Troubleshooting Guide

### Common PWM Issues

#### PWM Frequency Not Sticking
**Symptoms**: PWM frequency resets to default after reboots
**Causes**: 
- Prescaler value out of valid range (3-255)
- I2C communication failures during frequency setting
- Power cycling without proper shutdown sequence

**Solutions**:
1. Verify prescaler calculation: `prescale = round(25000000/(4096 × frequency) - 1)`
2. Check I2C bus connectivity and device addresses
3. Ensure proper shutdown sequence before power cycling

#### Duty Cycle Not Responding
**Symptoms**: PWM outputs not changing despite commands
**Causes**:
- Channel index out of range (0-15)
- Invalid duty cycle values (0-4095)
- Thread contention blocking I2C operations

**Solutions**:
1. Validate channel assignments in fixed mapping
2. Check duty cycle clamping: `max(0, min(4095, duty))`
3. Monitor I2C lock acquisition timing

#### Frequency Drift Issues
**Symptoms**: PWM frequency gradually changing over time
**Causes**:
- Temperature variations affecting crystal oscillator
- Power supply instability
- I2C timing issues causing partial register writes

**Solutions**:
1. Implement periodic frequency verification
2. Add temperature compensation if needed
3. Increase I2C timing delays for reliability

### Hardware Feedback Problems

#### Relay Feedback Mismatch
**Symptoms**: Relay state shows "ON" when expecting "OFF"
**Causes**:
- Incorrect wiring polarity (active-low vs active-high)
- Faulty relay contacts
- PCA9539 pin configuration issues

**Solutions**:
1. Verify relay wiring: Low (0) = ON, High (1) = OFF
2. Test individual relays with known good loads
3. Check PCA9539 configuration registers

#### Stepper Motor Control Issues
**Symptoms**: Stepper not responding to direction changes
**Causes**:
- Insufficient setup time (≥50ms) between direction changes
- Pulse generation conflicts
- Hardware driver protection circuits

**Solutions**:
1. Implement proper direction change sequence
2. Disable pulse generation during direction changes
3. Verify DM332T setup time requirements

### Channel Conflicts

#### Channel Interference
**Symptoms**: Unexpected behavior when multiple channels active
**Causes**:
- Shared power supplies causing voltage drops
- Ground loops between devices
- PWM frequency conflicts between channels

**Solutions**:
1. Use separate power supplies for sensitive channels
2. Implement proper grounding techniques
3. Configure different PWM frequencies for conflicting loads

#### I2C Bus Conflicts
**Symptoms**: Random I2C errors and device timeouts
**Causes**:
- Multiple applications accessing the same I2C bus
- Incorrect pull-up resistor values
- Device address conflicts

**Solutions**:
1. Verify unique I2C addresses for all devices
2. Check I2C pull-up resistor values (typically 4.7kΩ)
3. Close unused I2C connections

### System Shutdown and Restart Procedures

#### Safe Shutdown Sequence
1. Stop all worker threads gracefully
2. Set all PWM outputs to safe state (0 duty cycle)
3. Publish offline availability message
4. Disconnect from MQTT broker
5. Close hardware interfaces in reverse order

#### Restart Procedures
1. Reinitialize I2C bus and devices
2. Reset PCA9685 to default state
3. Reconfigure PWM frequencies
4. Re-establish MQTT connections
5. Resume normal operation

**Section sources**
- [run.py:1889-1931](file://run.py#L1889-L1931)
- [run.py:1898-1920](file://run.py#L1898-L1920)

## Conclusion

The PCA9685 16-channel 12-bit PWM controller implementation provides a robust, thread-safe solution for industrial automation and home control applications. Key strengths include:

**Technical Excellence**:
- Precise 12-bit PWM resolution with configurable frequencies
- Comprehensive hardware feedback verification
- Thread-safe I2C communication protocols
- MQTT-based Home Assistant integration

**Operational Reliability**:
- Safe shutdown procedures preventing hardware damage
- Real-time feedback monitoring for fault detection
- Graceful error handling and recovery mechanisms
- Configurable operational parameters

**Practical Applications**:
- Multi-zone heating control with precise temperature regulation
- Variable-speed fan and blower systems
- Stepper motor positioning with feedback verification
- RGB lighting control with status indication

The implementation demonstrates best practices in embedded system design, combining hardware abstraction with software engineering principles to create a maintainable and extensible solution. The modular architecture allows for easy modification and extension while maintaining system stability and reliability.