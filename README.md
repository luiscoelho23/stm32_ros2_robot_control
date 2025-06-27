# STM32 micro-ROS Robot Control System

A fully functional micro-ROS based robot control system for STM32F767 microcontroller with servo motor control capabilities. This project enables real-time servo control through ROS2 communication over Ethernet with joint state publishing and trajectory command handling.

## ✨ Features

- **🎯 Real-time Joint Control**: Control 3-DOF robot arm with precise servo positioning (0-180°)
- **📡 micro-ROS Integration**: Full ROS2 compatibility with joint state publishing and command handling
- **⚡ Dual Communication**: Support for both `JointJog` commands and `JointTrajectory` messages
- **🔧 Hardware Safety**: Built-in position clamping, memory protection, and error handling
- **🌐 Ethernet Communication**: Reliable UDP transport with 100Hz publishing rate
- **📊 Status Monitoring**: LED-based visual feedback for network and command status
- **🧵 Multi-threaded**: FreeRTOS task management for concurrent operations

## 🛠️ Hardware Setup

### Core Components
- **STM32F767ZI** (Nucleo-144 or equivalent)
- **PCA9685** 16-channel PWM servo driver  
- **3x Servo Motors** 
- **Ethernet Connection** for ROS2 communication

### Pin Configuration
```
I2C1 (PCA9685):
├── PB8: I2C1_SCL (Clock)
└── PB9: I2C1_SDA (Data)

Status LEDs:
├── PB0:  Green LED  (System Ready)
├── PB14: Red LED    (Error/Memory Fail)
└── PB7:  Blue LED   (Command Activity/Network Status)

Ethernet: Follow STM32F767ZI standard pins
```

### PCA9685 Wiring
```
STM32F767    PCA9685    Servos
PB8 -------> SCL        
PB9 -------> SDA        Servo 0: Channel 0
5V  -------> VCC        Servo 1: Channel 1  
GND -------> GND        Servo 2: Channel 2
             V+  -----> External 6V Power
```

## 📋 Software Requirements

- **STM32CubeIDE** or **STM32CubeMX** with GCC toolchain
- **Docker** (for micro-ROS library generation)
- **ROS2** (Jazzy/Humble) with micro-ROS agent
- **FreeRTOS** (included in STM32 HAL)
- **LwIP** stack for Ethernet communication

## 🚀 Quick Start

### Clone the Repository

```bash
git clone https://github.com/luiscoelho23/stm32_ros2_robot_control.git
cd stm32_ros2_robot_control
```

### 1. STM32CubeMX Configuration

1. **Create Project** in STM32CubeMX for STM32F767ZI
2. **Configure Ethernet and LwIP:**
   - Enable Ethernet in Connectivity tab
   - Enable LwIP with UDP support
   - **Network Configuration STM-PC** (use `192.168.10.x` subnet to avoid conflicts):
     - STM32 IP: `192.168.10.23`
     - Gateway: `192.168.10.1`
     - Netmask: `255.255.255.0`
     - Disable DHCP for static IP configuration
   - **Network Configuration STM-router router-PC (recommended)**
     - Enable DHCP in STM32CubeMX LwIP settings
     - Router assigns IP automatically (e.g., 192.168.1.100)
     - PC and STM32 both connect through same router
     - Update micro-ROS agent IP in code to match PC's router-assigned IP
3. **Configure I2C1** for PCA9685 communication
4. **Configure GPIO pins** for status LEDs and I2C:
   - PB0: Green LED (System ready)
   - PB14: Red LED (Error indicator)  
   - PB7: Blue LED (Command activity/Error indicator(overflows))
   - PB8: I2C1_SCL (PCA9685 clock)
   - PB9: I2C1_SDA (PCA9685 data)
5. **Enable FreeRTOS** with proper task configuration:
   - Ethernet task: 256 words
   - ROS task: **5840 words** (23KB - critical for micro-ROS)
   - Robot task: 256 words
   - **IMPORTANT: Enable heap allocation for threads**

### 2. Generate micro-ROS Library

Follow the instructions at: https://github.com/micro-ROS/micro_ros_stm32cubemx_utils

### 3. Build and Flash

1. Open project in STM32CubeIDE
2. Build → Flash to STM32

## 🎮 ROS2 Control Interface

### Published Topics

```bash
# Joint states (published at 100Hz)
/joint_states (sensor_msgs/JointState)
├── header.stamp: Current timestamp
├── name: ["joint_1", "joint_2", "joint_3"]
├── position: [servo0_angle, servo1_angle, servo2_angle]  # 0-180 degrees
├── velocity: [vel0, vel1, vel2]
└── effort: [eff0, eff1, eff2]
```

### Subscribed Topics

```bash
# Joint command interface
/joint_command (control_msgs/JointJog)
├── joint_names: ["joint_1", "joint_2", "joint_3"]
├── displacements: [angle0, angle1, angle2]  # 0-180 degrees
└── velocities: [vel0, vel1, vel2]

# Trajectory interface
/joint_trajectory (trajectory_msgs/JointTrajectory)
├── joint_names: ["joint_1", "joint_2", "joint_3"]
└── points[0].positions: [pos0, pos1, pos2]  # -π to π radians
```

## 🌐 Network Setup

### PC Ethernet Configuration
Configure your Ethernet adapter with static IP:
- **DHCP**: Disabled
- **IPv4 Address**: `192.168.10.1`
- **Subnet Mask**: `255.255.255.0`
- **Default Gateway**: (leave empty)


### STM32 Configuration (Pre-configured)
- **STM32 IP**: `192.168.10.23`
- **Gateway**: `192.168.10.1`
- **Micro-ROS Agent IP**: `192.168.10.1` (your PC)
- **Connection**: Direct Ethernet cable (no router needed)

## 🚦 LED Status Indicators

### System Status
- **Green LED (Solid)**: Network connected and ROS operational
- **Red LED (Fast blink)**: micro-ROS initialization failed
- **Blue LED (Fast blink)**: micro-ROS initialization failed (memory problmes)
- **Blue LED (Toggle)**: Joint command received and processed

### Error Patterns
- **Red LED (Medium blink)**: Publisher/Subscriber creation failed
- **Red LED (Slow blink)**: Executor initialization failed
- **Blue LED (Slow blink)**: Network waiting during startup

### WSL2 UDP Port Forwarding

If using ROS2 in WSL2, set up UDP port forwarding:

**Get WSL2 IP:**
```bash
# In WSL2 terminal
hostname -I
# Example: 172.25.240.45
```

**Forward UDP traffic (Windows):**
```bash
# Install socat if needed: choco install socat
socat UDP4-LISTEN:8888,fork UDP4-SENDTO:172.25.240.45:8888
```

**Automated script:**
```bash
#!/bin/bash
# Save as wsl_forward.sh
WSL_IP=$(wsl hostname -I | tr -d ' \n')
echo "Forwarding UDP 8888 to WSL IP: $WSL_IP"
socat -v UDP4-LISTEN:8888,fork UDP4-SENDTO:$WSL_IP:8888
```

### 1. Start micro-ROS Agent

**In WSL2/Linux:**
```bash
# Start the micro-ROS agent on your PC
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v 6
```

### 2. Monitor Joint States

```bash
# View real-time joint positions
ros2 topic echo /joint_states

# Check topic publishing rate
ros2 topic hz /joint_states
```

### 3. Control Servos

**Using Joint Commands:**
```bash
# Move all servos to center position (90 degrees)
ros2 topic pub /joint_command control_msgs/msg/JointJog "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names: ['joint_1', 'joint_2', 'joint_3']
displacements: [90.0, 90.0, 90.0]
velocities: []
duration: 0.0"

# Move individual servo
ros2 topic pub /joint_command control_msgs/msg/JointJog "
joint_names: ['joint_1']
displacements: [45.0]" --once
```

**Using Trajectory Commands:**
```bash
# Send trajectory point
ros2 topic pub /joint_trajectory trajectory_msgs/msg/JointTrajectory "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names: ['joint_1', 'joint_2', 'joint_3']
points:
- positions: [1.57, 0.0, -1.57]  # π/2, 0, -π/2 radians
  velocities: []
  accelerations: []
  effort: []
  time_from_start:
    sec: 1
    nanosec: 0"
```

### 4. Safety Features

- **Position Clamping**: Servo commands automatically clamped to 0-180° range
- **Input Validation**: NULL pointer checks and array bounds validation
- **Memory Protection**: Proper malloc() error handling prevents crashes
- **Network Recovery**: Automatic reconnection on network failures

## 🔧 Troubleshooting

### Fast Blue LED Blinking
**Cause**: micro-ROS initialization failure
**Solutions**:
1. Check network connectivity between PC and STM32
2. Verify micro-ROS agent is running on PC
3. Ensure heap size is sufficient (>32KB)
4. Confirm IP addresses match configuration

### Red LED Patterns
- **Fast blink**: Memory allocation failure - increase heap size
- **Medium blink**: ROS node creation failed - check agent connection
- **Slow blink**: Executor setup failed - verify message initialization

### No Joint States Published
1. Check network connection
2. Verify agent is listening on correct IP:port
3. Monitor agent logs for connection status
4. Ensure STM32 has valid IP configuration

## 🔧 Memory Requirements

### Critical Settings
- **Heap Size**: Minimum 32KB (required for micro-ROS dynamic allocation)
- **Stack Sizes**:
  - ROS Task: 5840 words (23KB)
  - Ethernet Task: 256 words (1KB)
  - Robot Task: 256 words (1KB)



## 📝 Technical Notes

- **Publishing Rate**: Joint states published at 100Hz (1ms executor + 9ms delay = 10ms total loop time)
- **Command Processing**: Real-time servo updates on message reception
- **Thread Safety**: FreeRTOS task synchronization for concurrent operations
- **Error Recovery**: Automatic retry on network or hardware failures
- **Position Feedback**: Real-time joint position reporting to ROS

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.