# STM32 ROS2 Robot Control

A micro-ROS based robot control system for STM32F767 microcontroller with servo motor control capabilities. This project enables real-time servo control through ROS2 communication over Ethernet.

## 🔧 Features

- **Real-time Servo Control**: Control up to 16 servo motors using PCA9685 PWM driver
- **ROS2 Integration**: Full micro-ROS integration for seamless ROS2 communication
- **Ethernet Communication**: UDP-based transport for reliable network communication
- **Multi-threaded Architecture**: FreeRTOS-based task management for concurrent operations
- **Visual Status Indicators**: LED-based network and command status feedback
- **STM32CubeMX Integration**: Easy configuration and code generation

## 🔩 Hardware Requirements

- **STM32F767ZI** (Nucleo-144 board or equivalent)
- **PCA9685** 16-channel PWM servo driver
- **Servo Motors** (up to 16 channels)
- **Ethernet Connection** for ROS2 communication
- **Status LEDs** for visual feedback

### Hardware Connections

| Component | STM32 Pin | Description |
|-----------|-----------|-------------|
| PCA9685 SDA | I2C1_SDA | I2C Data Line |
| PCA9685 SCL | I2C1_SCL | I2C Clock Line |
| Status LED (Green) | PB0 | Network Status |
| Status LED (Red) | PB14 | Error Indicator |
| Status LED (Blue) | PB7 | Command Received |

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

## 📋 Installation & Setup

### 1. STM32CubeMX Configuration

1. Open `EthernetCubeMX.ioc` in STM32CubeMX
2. Configure Ethernet and LwIP:
   - Enable Ethernet in Connectivity tab
   - Enable LwIP with UDP support
   - Set static IP configuration
3. Configure I2C1 for PCA9685 communication
4. Configure GPIO pins for status LEDs
5. Enable FreeRTOS with sufficient stack size (>10KB for micro-ROS task)

### 2. Generate micro-ROS Library

```bash
# Pull the micro-ROS library builder
docker pull microros/micro_ros_static_library_builder:jazzy

# Generate the static library
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:jazzy
```

### 3. Build the Project

**For STM32CubeIDE:**
- Import the project
- Build using the IDE

### 4. Flash the Firmware

Flash the generated binary to your STM32F767 board using your preferred method (ST-Link, OpenOCD, etc.).

## 🚀 Usage

### WSL2 UDP Port Forwarding Setup

If you're running ROS2 in WSL2, you need to set up UDP port forwarding since WSL2 doesn't automatically forward UDP ports.

**Get WSL2 IP Address:**
```bash
# In WSL2
hostname -I
# Example output: 172.25.240.45
```

**Forward UDP traffic:**
```bash
# Run this on Windows (replace with your actual WSL IP)
socat UDP4-LISTEN:8888,fork UDP4-SENDTO:172.25.240.45:8888
```

**Or create a simple script:**
```bash
#!/bin/bash
# Save as wsl_forward.sh
$WSL_IP= wsl hostname -I
echo "Forwarding UDP 8888 to WSL IP: $WSL_IP"
socat -v UDP4-LISTEN:8888,fork UDP4-SENDTO:$WSL_IP:8888
```

### 1. Start the micro-ROS Agent

**In WSL2:**
```bash

# Start the agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v 6
```

### 2. Control Servos

Send servo commands using ROS2 topics:

```bash
# Control servo 0 to 90 degrees
ros2 topic pub /servo_cmd std_msgs/msg/String "data: 'servo0:90'"

```

### Command Format

Servo commands follow the format: `servoX:Y`
- `X`: Servo channel (0-15)
- `Y`: Angle in degrees (0-180)

### Status Indicators

- **Green LED (Solid)**: Network connected and operational
- **Red LED (Solid)**: Network connection error
- **Blue LED (Toggle)**: Successful servo command received

## 📡 Network Configuration

Default network settings:
- **Protocol**: UDP
- **Port**: 8888
- **IP**: Configure in STM32CubeMX LwIP settings

Make sure your STM32 board and ROS2 system are on the same network.

## 🏗️ Project Structure

```
stm32_ros2_robot_control/
├── Core/
│   ├── Inc/
│   │   ├── main.h              # Main application header
│   │   ├── robot.h             # Robot control interface
│   │   └── ...
│   └── Src/
│       ├── main.c              # Main application with ROS2 integration
│       ├── robot.c             # PCA9685 servo control implementation
│       └── ...
├── micro_ros_stm32cubemx_utils/ # Micro-ROS integration utilities
├── Drivers/                     # STM32 HAL drivers
├── Middlewares/                 # FreeRTOS and other middleware
├── LWIP/                       # LwIP stack configuration
├── EthernetCubeMX.ioc          # STM32CubeMX project file
└── README.md                   # This file
```

## 🔧 Customization

### Adding More Servos

The system supports up to 16 servos (0-15) on the PCA9685. Simply send commands to different channel numbers.

### Modifying PWM Range

Edit the `angle_to_pwm()` function in `Core/Src/robot.c` to adjust servo pulse width ranges:

```c
static uint16_t angle_to_pwm(uint8_t angle) {
    // Current range: ~0.5ms to 2.5ms (102 to 512 PWM values)
    return (uint16_t)(102 + ((float)angle / 180.0f) * (512 - 102));
}
```

### Network Settings

Modify network configuration in `udp_transport.c` or through STM32CubeMX LwIP settings.

## 🐛 Troubleshooting

### Network Issues
- Verify IP configuration and network connectivity
- Ensure UDP port 8888 is available

### WSL2 Issues
- Make sure socat is running and forwarding to correct WSL IP
- Check Windows firewall allows UDP port 8888
- Verify WSL2 IP with `wsl hostname -I`

### Servo Control Issues
- Verify I2C connections to PCA9685
- Check servo power supply
- Validate command format (`servoX:Y`)

### Build Issues
- Ensure Docker is running for micro-ROS library generation
- Verify STM32CubeMX configuration matches hardware
- Check FreeRTOS stack size (minimum 10KB for micro-ROS)

## 📚 Dependencies

- **STM32 HAL Library**: Hardware abstraction layer
- **FreeRTOS**: Real-time operating system
- **LwIP**: Lightweight TCP/IP stack
- **micro-ROS**: ROS2 integration for microcontrollers
- **PCA9685 Driver**: Custom I2C servo controller driver

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-feature`)
3. Commit your changes (`git commit -am 'Add new feature'`)
4. Push to the branch (`git push origin feature/new-feature`)
5. Create a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **micro-ROS team** for the excellent microcontroller ROS2 integration
- **STMicroelectronics** for the comprehensive STM32 ecosystem
- **eProsima** for the Micro XRCE-DDS middleware

---

For questions or issues, please open an issue on GitHub or contact the maintainers.