# STM32 ROS2 Robot Control

A micro-ROS based robot control system for STM32F767 microcontroller with servo motor control capabilities. This project enables real-time servo control through ROS2 communication over Ethernet.

## 🔧 Features

- **Real-time Servo Control**: Control up to 16 servo motors using PCA9685 PWM driver
- **ROS2 Integration**: Full micro-ROS integration for seamless ROS2 communication
- **Ethernet Communication**: UDP-based transport for reliable network communication
- **Multi-threaded Architecture**: FreeRTOS-based task management for concurrent operations
- **Visual Status Indicators**: LED-based network and command status feedback

## 🔩 Hardware Requirements

- **STM32F767ZI** (Nucleo-144 board or equivalent)
- **PCA9685** 16-channel PWM servo driver
- **Servo Motors** 
- **Ethernet Connection** for ROS2 communication
- **Status LEDs**: 3 LEDs for debugging (typically on-board LEDs)
  - PB0: Green LED (Network ready)
  - PB14: Red LED (Network error)
  - PB7: Blue LED (PHY scan/Network waiting)
  - PB8: I2C1_SCL (PCA9685 clock)
  - PB9: I2C1_SDA (PCA9685 data)

⚠️ **IMPORTANT**: Follow the UM1974 user manual for proper Ethernet pin configuration. Deviating from the recommended pin assignments may cause network communication failures.

## 🛠️ Software Requirements

- **STM32CubeIDE** or **STM32CubeMX** with Makefile
- **Docker** (for micro-ROS library generation)
- **ROS2** (Jazzy/Humble) with micro-ROS agent
- **FreeRTOS** (included in STM32 HAL)
- **LwIP** stack for Ethernet communication

## Clone the Repository

```bash
git clone https://github.com/luiscoelho23/stm32_ros2_robot_control.git
cd stm32_ros2_robot_control
```

## 📋 Setup Instructions

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
4. **Configure GPIO pins** for status LEDs, Ethernet connection, and I2C communication:
   - PB0: Green LED (Network ready)
   - PB14: Red LED (Network error)  
   - PB7: Blue LED (PHY scan/Network waiting)
   - PB8: I2C1_SCL (PCA9685 clock)
   - PB9: I2C1_SDA (PCA9685 data)
5. **Enable FreeRTOS** with sufficient stack sizes:
   - Ethernet task: 256 words minimum
   - ROS task: 3840 words (micro-ROS requirement)
   - Robot task: 128 words

### 2. Generate micro-ROS Library

Follow the instructions for your specific case:
https://github.com/micro-ROS/micro_ros_stm32cubemx_utils

### 3. Customize Code for Your Application

Use the example implementation provided in this project as a starting point.

### 4. Build the Project

**For STM32CubeIDE:**
- Build using the IDE

### 5. Flash the Firmware

Flash the generated binary to your STM32F767 board using your preferred method (ST-Link, OpenOCD, etc.).

## 🚀 Usage Guide

This setup has been tested on Windows WSL2 with direct Ethernet connection.

### Network Setup

#### PC Ethernet Configuration
Configure your Ethernet adapter with static IP:
- **IPv4 Address**: `192.168.10.1`
- **Subnet Mask**: `255.255.255.0`
- **Default Gateway**: (leave empty)
- **DHCP**: Disabled

#### STM32 Configuration (Pre-configured)
- **STM32 IP**: `192.168.10.23`
- **Gateway**: `192.168.10.1`
- **Micro-ROS Agent IP**: `192.168.10.1` (your PC)
- **Connection**: Direct Ethernet cable (no router needed)

### LED Status Indicators

#### Boot Sequence (PHY Detection)
1. **Blue LED ON**: PHY scanning starts
2. **Green LED flashes 5x**: PHY found successfully
3. **Red LED flashes 10x**: No PHY detected (check hardware)
4. **Blue LED OFF**: PHY scan complete

#### Operational Status
- **Red LED (Solid)**: Network error or no physical link
- **Green LED (Solid)**: Network connected and operational
- **Blue LED (Blinking)**: Waiting for network to initialize
- **Blue LED (Quick toggle)**: Successful servo command received

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
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v 6
```

### 2. Control Servos

**Basic Commands:**
```bash
# Center servo 0
ros2 topic pub /servo_cmd std_msgs/msg/String "data: 'servo0:90'"

# Move servo 1 to minimum position
ros2 topic pub /servo_cmd std_msgs/msg/String "data: 'servo1:0'"

# Move servo 2 to maximum position  
ros2 topic pub /servo_cmd std_msgs/msg/String "data: 'servo2:180'"
```

**Advanced Usage:**
```bash
# Control multiple servos in sequence
for i in {0..3}; do
  ros2 topic pub /servo_cmd std_msgs/msg/String "data: 'servo$i:90'" --once
  sleep 0.5
done

# Sweep servo 0 from 0 to 180 degrees
for angle in {0..180..10}; do
  ros2 topic pub /servo_cmd std_msgs/msg/String "data: 'servo0:$angle'" --once
  sleep 0.1
done
```

### 3. Monitor System Status

```bash
# Monitor heartbeat (published every second)
ros2 topic echo /counter

# List all available topics
ros2 topic list

# Check topic details
ros2 topic info /servo_cmd
ros2 topic hz /counter

# Monitor servo commands
ros2 topic echo /servo_cmd
```

### Command Format & Validation

**Format:** `servoX:Y`
- `X`: Servo channel (0-15)
- `Y`: Angle in degrees (0-180)

**Validation Rules:**
- Invalid servo numbers (>15) → ignored
- Invalid angles (<0 or >180) → ignored  
- Malformed commands → ignored
- Successful commands → blue LED toggles

## 🔧 Troubleshooting

### PHY Detection Issues

**Symptoms:** Red LED flashes 10x during boot
**Solutions:**
- Check Ethernet cable connection
- Verify cable type (Cat5e/Cat6 recommended)
- Try different Ethernet port on PC
- Check STM32F767 chip revision (see Hardware Issues below)

### Network Connection Issues

**Blue LED stuck blinking:**
- Verify PC Ethernet adapter configuration
- Check cable with `ipconfig /all` (should show connection)
- Ensure both devices on `192.168.10.x` subnet
- Try different Ethernet cable

**Red LED solid during operation:**
- Physical link not detected
- Check cable connectivity  
- Verify Ethernet port functionality
- Restart both devices

### Communication Issues

**micro-ROS agent connection fails:**
- Verify UDP port 8888 is open
- Check Windows Firewall settings
- Ensure WSL2 port forwarding is active
- Verify IP addresses match configuration

**Servo commands not working:**
- Check I2C connections to PCA9685
- Verify servo power supply (5V recommended)
- Monitor blue LED for command reception
- Validate command format

### Build Issues

**Compilation errors:**
- Ensure Docker is running for micro-ROS library generation
- Verify STM32CubeMX configuration matches code
- Check FreeRTOS stack sizes meet requirements
- Include all required library dependencies

## 📡 Network Architecture

```
┌─────────────────┐    Direct Ethernet    ┌──────────────────┐
│   PC/WSL2       │◄──────────────────────►│   STM32F767      │
│ 192.168.10.1    │                        │ 192.168.10.23    │
│ UDP Port 8888   │                        │ UDP Transport    │
└─────────────────┘                        └──────────────────┘
          │                                          │
    ┌─────────────┐                           ┌─────────────┐
    │ micro-ROS   │                           │ PCA9685 +   │
    │ Agent       │                           │ Servos      │
    └─────────────┘                           └─────────────┘
```

## 🎯 Project Structure

```
stm32_ros2_robot_control/
├── Core/
│   ├── Src/
│   │   ├── main.c              # Main application and ROS setup
│   │   ├── robot.c             # PCA9685 servo control
│   │   └── udp_transport.c     # micro-ROS UDP transport
│   └── Inc/
│       ├── main.h              # Main header definitions
│       └── robot.h             # Robot control interface
├── LWIP/App/
│   └── lwip.c                  # LwIP network configuration
├── micro_ros_stm32cubemx_utils/ # micro-ROS integration utilities
├── Drivers/                    # STM32 HAL drivers
├── Middlewares/               # FreeRTOS and LwIP middleware
└── README.md                  # This documentation
```

## 🔍 Advanced Configuration

### Custom Network Settings

To change network configuration, modify in STM32CubeMX:
1. LwIP settings for IP addresses
2. Update `main.c` line ~496 for micro-ROS agent IP
3. Rebuild and reflash firmware

### Adding More Servos

The PCA9685 supports 16 channels. To use additional channels:
1. Update servo initialization in `robotControl()` task
2. Send commands with servo numbers 0-15
3. Ensure adequate power supply for all servos

### Performance Tuning

- Increase FreeRTOS tick rate for faster response
- Adjust micro-ROS executor spin frequency
- Optimize I2C clock speed for PCA9685
- Fine-tune Ethernet buffer sizes in LwIP

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

For questions or issues, please open an issue on GitHub or contact the maintainer.