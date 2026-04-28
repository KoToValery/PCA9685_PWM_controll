# Thread Management and Concurrency

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

This document provides comprehensive documentation for the multi-threaded architecture of the PCA9685 PWM controller system. The application implements a sophisticated concurrent design pattern that coordinates multiple hardware access threads through a shared I2C bus while maintaining thread safety and graceful shutdown procedures.

The system manages four primary worker threads: `pca9539_worker` for hardware feedback monitoring, `bme_worker` for environmental sensor data collection, `sys_led_worker` for system status indication, and `led_indicator_worker` for diagnostic mode operation. Each thread operates independently while sharing a common I2C bus through carefully designed synchronization mechanisms.

## Project Structure

The project follows a modular architecture with clear separation between hardware abstraction, thread management, and MQTT communication:

```mermaid
graph TB
subgraph "Main Application"
RunPy[run.py]
Config[config.yaml]
end
subgraph "Hardware Abstraction Layer"
PCA9685[PCA9685 Driver]
PCA9539[PCA9539 GPIO Expander]
BME280[BME280 Sensor]
PCA9540[PCA9540 Multiplexer]
end
subgraph "Thread Management"
ThreadLocks[Thread Locks]
ThreadSync[Thread Synchronization]
Shutdown[Graceful Shutdown]
end
subgraph "Communication Layer"
MQTT[MQTT Client]
Topics[MQTT Topics]
end
RunPy --> HardwareAbstraction
HardwareAbstraction --> ThreadManagement
ThreadManagement --> CommunicationLayer
HardwareAbstraction --> PCA9685
HardwareAbstraction --> PCA9539
HardwareAbstraction --> BME280
HardwareAbstraction --> PCA9540
ThreadManagement --> ThreadLocks
ThreadManagement --> ThreadSync
ThreadManagement --> Shutdown
CommunicationLayer --> MQTT
CommunicationLayer --> Topics
```

**Diagram sources**
- [run.py:1-50](file://run.py#L1-L50)
- [config.yaml:1-57](file://config.yaml#L1-L57)

**Section sources**
- [run.py:1-80](file://run.py#L1-L80)
- [config.yaml:1-57](file://config.yaml#L1-L57)

## Core Components

The system implements a comprehensive thread management framework built around several key architectural patterns:

### Shared I2C Bus Architecture

The application establishes a single shared I2C bus connection that serves all hardware components:

```mermaid
sequenceDiagram
participant Main as Main Thread
participant Bus as Shared I2C Bus
participant PCA9685 as PCA9685 Driver
participant PCA9539 as PCA9539 Driver
participant BME280 as BME280 Driver
participant PCA9540 as PCA9540 Driver
Main->>Bus : Initialize SMBus(1)
Main->>PCA9685 : Create PCA9685(shared_bus, address)
Main->>PCA9539 : Create PCA9539(shared_bus, address)
Main->>BME280 : Create BME280(shared_bus, address)
Main->>PCA9540 : Create PCA9540(shared_bus, address)
Note over Main,Bus : All drivers share the same bus instance
```

**Diagram sources**
- [run.py:39-46](file://run.py#L39-L46)
- [run.py:111-160](file://run.py#L111-L160)

### Thread Lifecycle Management

Each worker thread follows a standardized lifecycle pattern with initialization, execution, and graceful shutdown:

```mermaid
stateDiagram-v2
[*] --> Initialized
Initialized --> Running : start()
Running --> Stopping : stop()
Stopping --> Stopped : join(timeout)
Stopped --> [*]
Running --> Error : exception
Error --> Stopped : cleanup
```

**Diagram sources**
- [run.py:800-820](file://run.py#L800-L820)
- [run.py:1107-1126](file://run.py#L1107-L1126)

### Lock-Based Synchronization Patterns

The system employs multiple synchronization mechanisms to coordinate access to shared resources:

| Lock Type | Purpose | Scope | Protection |
|-----------|---------|-------|------------|
| `i2c_lock` | I2C bus access | All hardware operations | Prevent concurrent I2C transactions |
| `status_lock` | System status updates | Status changes | Atomic status transitions |
| `any_problem_lock` | Real-time problem detection | Problem state updates | Consistent problem reporting |
| `pwm1_lock` | PWM1 channel control | PWM1 operations | Atomic duty cycle changes |
| `pwm2_lock` | PWM2 channel control | PWM2 operations | Atomic duty cycle changes |
| `pu_lock` | Pulse generation control | PU worker operations | Atomic frequency changes |

**Section sources**
- [run.py:39-46](file://run.py#L39-L46)
- [run.py:349-354](file://run.py#L349-L354)
- [run.py:632-650](file://run.py#L632-L650)

## Architecture Overview

The multi-threaded architecture implements a producer-consumer pattern with specialized workers for different hardware components:

```mermaid
graph TB
subgraph "System Threads"
SysLED[sys_led_worker<br/>System LED Blinking]
LedIndicator[led_indicator_worker<br/>Diagnostic Indicator]
PCA9539[pca9539_worker<br/>Hardware Feedback]
BME[bme_worker<br/>Sensor Data Collection]
PU[pu_worker<br/>Pulse Generation]
end
subgraph "Shared Resources"
I2CLock[I2C Bus Lock]
StatusLock[System Status Lock]
ProblemLock[Problem Detection Lock]
end
subgraph "Hardware Layer"
PCA9685[PCA9685 PWM Controller]
PCA9539HW[PCA9539 GPIO Expander]
BME280HW[BME280 Sensors]
PCA9540HW[PCA9540 Multiplexer]
end
subgraph "Communication"
MQTTClient[MQTT Client]
Topics[MQTT Topics]
end
SysLED --> I2CLock
LedIndicator --> ProblemLock
PCA9539 --> I2CLock
BME --> I2CLock
PU --> I2CLock
SysLED --> PCA9685
LedIndicator --> PCA9685
PCA9539 --> PCA9539HW
BME --> BME280HW
BME --> PCA9540HW
PU --> PCA9685
LedIndicator --> MQTTClient
PCA9539 --> MQTTClient
BME --> MQTTClient
MQTTClient --> Topics
```

**Diagram sources**
- [run.py:1128-1226](file://run.py#L1128-L1226)
- [run.py:673-798](file://run.py#L673-L798)
- [run.py:822-874](file://run.py#L822-L874)

## Detailed Component Analysis

### PCA9539 Worker - Hardware Feedback Monitoring

The `pca9539_worker` serves as the central monitoring thread for hardware feedback verification:

```mermaid
sequenceDiagram
participant Worker as pca9539_worker
participant PCA9539 as PCA9539 Driver
participant MQTT as MQTT Client
participant Status as Status Lock
loop Every 1 second
Worker->>PCA9539 : read_inputs()
PCA9539-->>Worker : 16-bit input state
alt Relay feedback (pins 0-5)
Worker->>Worker : Compare expected vs actual
Worker->>MQTT : Publish relay status
end
alt Stepper feedback (pins 8-10)
Worker->>Worker : Verify ENA/DIR/PULSE signals
Worker->>MQTT : Publish stepper status
end
alt TAXO feedback (pins 11-12)
Worker->>Worker : Monitor pulse detection
Worker->>MQTT : Publish TAXO status
end
Worker->>Status : Update any_problem_realtime
end
```

**Diagram sources**
- [run.py:673-798](file://run.py#L673-L798)

Key features of the PCA9539 worker:

- **Real-time feedback monitoring**: Continuously reads 16-pin input states every second
- **Multi-channel verification**: Validates relay states, stepper signals, and pulse detection
- **Problem detection**: Maintains real-time problem state for system-wide visibility
- **Historical analysis**: Uses sliding windows for pulse detection reliability

**Section sources**
- [run.py:673-798](file://run.py#L673-L798)
- [run.py:668-671](file://run.py#L668-L671)

### BME Worker - Environmental Sensor Data Collection

The `bme_worker` manages environmental sensor data collection through the PCA9540 multiplexer:

```mermaid
flowchart TD
Start([Worker Start]) --> SelectCH0["Select Channel 0"]
SelectCH0 --> ReadCH0_76["Read BME280 @ 0x76"]
ReadCH0_76 --> ReadCH0_77["Read BME280 @ 0x77"]
ReadCH0_77 --> DeselectCH0["Deselect Channel 0"]
DeselectCH0 --> SelectCH1["Select Channel 1"]
SelectCH1 --> ReadCH1_77["Read BME280 @ 0x77"]
ReadCH1_77 --> DeselectCH1["Deselect Channel 1"]
DeselectCH1 --> Sleep["Sleep BME_INTERVAL"]
Sleep --> SelectCH0
ReadCH0_76 --> PublishTemp["Publish Temperature"]
ReadCH0_76 --> PublishPress["Publish Pressure"]
ReadCH0_76 --> PublishHum["Publish Humidity (if BME)"]
ReadCH0_77 --> PublishTemp2["Publish Temperature"]
ReadCH0_77 --> PublishPress2["Publish Pressure"]
ReadCH0_77 --> PublishHum2["Publish Humidity (if BME)"]
ReadCH1_77 --> PublishTemp3["Publish Temperature"]
ReadCH1_77 --> PublishPress3["Publish Pressure"]
ReadCH1_77 --> PublishHum3["Publish Humidity (if BME)"]
```

**Diagram sources**
- [run.py:822-874](file://run.py#L822-L874)

**Section sources**
- [run.py:822-874](file://run.py#L822-L874)
- [run.py:606-625](file://run.py#L606-L625)

### System LED Worker - System Status Indication

The `sys_led_worker` provides continuous system status indication through the PCA9685 system LED:

```mermaid
stateDiagram-v2
[*] --> Blinking
Blinking --> On : Level = True
On --> Off : Level = False
Off --> On : Level = True
note right of Blinking
System LED (Channel 15)
Blinks continuously at 1Hz
end note
```

**Diagram sources**
- [run.py:1128-1144](file://run.py#L1128-L1144)

**Section sources**
- [run.py:1128-1144](file://run.py#L1128-L1144)

### LED Indicator Worker - Diagnostic Mode Operation

The `led_indicator_worker` implements intelligent diagnostic indication based on system health:

```mermaid
flowchart TD
Start([Worker Start]) --> CheckProblem["Check any_problem_realtime"]
CheckProblem --> HasProblem{"Has Problems?"}
HasProblem --> |Yes| BlinkRed["Blink Red LED for 5s"]
HasProblem --> |No| SolidGreen["Solid Green LED for 5s"]
BlinkRed --> WaitRest["Wait remaining interval"]
SolidGreen --> WaitRest
WaitRest --> CheckProblem
```

**Diagram sources**
- [run.py:1167-1205](file://run.py#L1167-L1205)

**Section sources**
- [run.py:1167-1205](file://run.py#L1167-L1205)

### Pulse Generation Worker - PU Signal Control

The `pu_worker` manages pulse generation for stepper motor control:

```mermaid
sequenceDiagram
participant Worker as pu_worker
participant Lock as pu_lock
participant PCA9685 as PCA9685 Driver
participant Feedback as Feedback Pin
loop While running
Worker->>Lock : Get enabled/frequency
alt Disabled or Zero Frequency
Worker->>PCA9685 : channel_off(CH_PU)
Worker->>Worker : Set pu_is_pulsing = False
else Enabled
Worker->>PCA9685 : channel_on(CH_PU)
Worker->>Feedback : Check pulse feedback
Worker->>PCA9685 : channel_off(CH_PU)
Worker->>Worker : Sleep for half period
end
end
```

**Diagram sources**
- [run.py:1044-1105](file://run.py#L1044-L1105)

**Section sources**
- [run.py:1044-1105](file://run.py#L1044-L1105)

## Dependency Analysis

The thread management system exhibits a well-structured dependency hierarchy:

```mermaid
graph TB
subgraph "Thread Dependencies"
PCA9539Worker[pca9539_worker]
BMEWorker[bme_worker]
SysLEDWorker[sys_led_worker]
LEDIndicatorWorker[led_indicator_worker]
PUWorker[pu_worker]
end
subgraph "Shared Dependencies"
I2CLock[i2c_lock]
StatusLock[status_lock]
ProblemLock[any_problem_lock]
MQTTPub[MQTT Publishing]
end
subgraph "Hardware Dependencies"
PCA9685[PCA9685]
PCA9539[PCA9539]
BME280[BME280]
PCA9540[PCA9540]
end
PCA9539Worker --> I2CLock
BMEWorker --> I2CLock
SysLEDWorker --> PCA9685
LEDIndicatorWorker --> ProblemLock
PUWorker --> I2CLock
PCA9539Worker --> PCA9539
BMEWorker --> BME280
BMEWorker --> PCA9540
LEDIndicatorWorker --> MQTTPub
PCA9539Worker --> MQTTPub
BMEWorker --> MQTTPub
```

**Diagram sources**
- [run.py:39-46](file://run.py#L39-L46)
- [run.py:349-354](file://run.py#L349-L354)
- [run.py:1167-1205](file://run.py#L1167-L1205)

**Section sources**
- [run.py:39-46](file://run.py#L39-L46)
- [run.py:349-354](file://run.py#L349-L354)

## Performance Considerations

The system implements several performance optimization strategies:

### Thread Pool Efficiency
- **Daemon threads**: All worker threads are daemon threads, ensuring automatic cleanup on main process exit
- **Minimal blocking**: Threads use non-blocking operations with strategic sleep intervals
- **Resource pooling**: Single shared I2C bus eliminates redundant bus connections

### Synchronization Optimization
- **Fine-grained locking**: Separate locks for different resource types minimize contention
- **Lock scope minimization**: Critical sections are kept as small as possible
- **Non-blocking operations**: Most hardware operations are performed under locks only when necessary

### Memory Management
- **Thread-local state**: Each worker maintains minimal local state to reduce memory footprint
- **Efficient data structures**: Simple lists used for pulse history tracking with automatic cleanup

## Troubleshooting Guide

### Common Thread Issues

**Thread Not Starting**
- Verify thread lock acquisition: Check if `with lock:` blocks are properly acquired
- Confirm thread state flags: Ensure `running` flags are set before thread creation
- Validate thread existence: Use `thread.is_alive()` checks before attempting operations

**Deadlock Situations**
- **I2C Deadlock**: Occurs when multiple threads attempt simultaneous I2C operations
- **Status Deadlock**: Can happen with nested lock acquisitions
- **Solution**: Always acquire locks in consistent order and keep critical sections minimal

**Hardware Access Conflicts**
- **Bus Contention**: Multiple threads accessing I2C simultaneously
- **Solution**: Use `i2c_lock` for all I2C operations and avoid nested hardware calls

### Debugging Procedures

**Thread Lifecycle Debugging**
1. Check thread state flags: `running` variables indicate thread status
2. Verify thread references: Ensure thread objects are properly maintained
3. Monitor thread logs: Each thread logs its lifecycle events

**Hardware Communication Debugging**
1. Validate I2C bus initialization: Confirm SMBus connection success
2. Check device addresses: Verify hardware address configuration
3. Monitor error logs: Review hardware access exceptions

**Section sources**
- [run.py:1889-1931](file://run.py#L1889-L1931)
- [run.py:813-820](file://run.py#L813-L820)
- [run.py:1118-1126](file://run.py#L1118-L1126)

### Practical Examples

**Thread Initialization Example**
```python
# Basic thread startup pattern
def worker_start():
    global thread, running
    with lock:
        if thread and thread.is_alive():
            return
        running = True
        thread = threading.Thread(target=worker_function, daemon=True)
        thread.start()
```

**Thread Synchronization Pattern**
```python
# Safe hardware access pattern
def safe_hardware_operation():
    with i2c_lock:
        # Perform hardware operation
        pass
```

**Graceful Shutdown Procedure**
```python
# Comprehensive shutdown sequence
def safe_shutdown():
    # Stop all workers
    bme_stop()
    pca9539_stop()
    sys_led_stop()
    led_indicator_stop()
    pu_stop()
    
    # Reset hardware states
    reset_all_hardware()
    
    # Cleanup resources
    cleanup_resources()
```

**Section sources**
- [run.py:800-820](file://run.py#L800-L820)
- [run.py:1889-1931](file://run.py#L1889-L1931)

## Conclusion

The PCA9685 PWM controller implements a robust multi-threaded architecture that effectively coordinates hardware access through a shared I2C bus while maintaining thread safety and system reliability. The design demonstrates excellent separation of concerns with dedicated workers for different hardware components, comprehensive synchronization mechanisms, and graceful shutdown procedures.

Key architectural strengths include:
- **Modular thread design**: Each worker has a specific, well-defined responsibility
- **Robust synchronization**: Carefully designed lock patterns prevent race conditions
- **Hardware abstraction**: Clean separation between hardware access and thread management
- **Error handling**: Comprehensive exception handling and graceful degradation
- **Performance optimization**: Efficient resource utilization with minimal contention

The system provides a solid foundation for embedded control applications requiring concurrent hardware access and real-time monitoring capabilities.