# Contributing to STM32 ROS2 Robot Control

Thank you for your interest in contributing to the STM32 ROS2 Robot Control project! This document provides guidelines for contributing to this embedded robotics project.

## 🤝 How to Contribute

### Types of Contributions Welcome

- **Bug Reports**: Hardware compatibility issues, network communication problems, build errors
- **Feature Requests**: New servo control features, additional sensor support, performance improvements
- **Code Contributions**: Bug fixes, new features, optimization improvements
- **Documentation**: Setup guides, troubleshooting steps, hardware compatibility lists
- **Testing**: Verification on different STM32 boards, PHY chips, or network configurations

## 📋 Getting Started

### Prerequisites

Before contributing, ensure you have:

- **Hardware**: STM32F767ZI board (or compatible), PCA9685, servo motors
- **Software**: STM32CubeIDE/CubeMX, Docker, ROS2 (Jazzy/Humble)
- **Development Environment**: Git, C/C++ knowledge, embedded systems experience
- **Network Setup**: Ethernet connection capability for testing

### Development Setup

1. **Fork and Clone**
   ```bash
   git clone https://github.com/YOUR_USERNAME/stm32_ros2_robot_control.git
   cd stm32_ros2_robot_control
   ```

2. **Set up Development Environment**
   - Install STM32CubeIDE
   - Set up Docker for micro-ROS library generation
   - Configure ROS2 workspace
   - Set up hardware testing environment

3. **Verify Setup**
   - Build the project successfully
   - Flash and test basic functionality
   - Verify network communication

## 🐛 Reporting Issues

### Bug Reports

When reporting bugs, please include:

**Hardware Information:**
- STM32 board model and revision
- PHY chip type (LAN8742A, DP83848, etc.)
- PCA9685 board details
- Servo motor specifications

**Software Environment:**
- STM32CubeIDE/CubeMX version
- ROS2 distribution (Jazzy/Humble)
- Operating system (Windows/Linux/WSL2)
- Docker version

**Network Configuration:**
- Connection type (direct/router)
- IP addresses and subnet
- Cable type and length

**Problem Description:**
- LED status patterns observed
- Error messages or logs
- Steps to reproduce
- Expected vs actual behavior

### Issue Templates

Please use these templates when creating issues:

**Bug Report Template:**
```markdown
## Bug Description
Brief description of the issue

## Hardware Setup
- STM32 Board: 
- PHY Chip: 
- Other Hardware: 

## Software Environment
- STM32CubeIDE Version: 
- ROS2 Distribution: 
- OS: 

## Steps to Reproduce
1. 
2. 
3. 

## Expected Behavior

## Actual Behavior

## LED Status
- Red LED: 
- Green LED: 
- Blue LED: 

## Additional Information
```

## 💻 Code Contributions

### Coding Standards

**C/C++ Style Guidelines:**
- Use consistent indentation (4 spaces)
- Follow STM32 HAL naming conventions
- Add comments for complex logic
- Use meaningful variable names
- Keep functions focused and concise

**Example:**
```c
/**
 * @brief Configure servo angle with validation
 * @param servo_num Servo channel (0-15)
 * @param angle Target angle in degrees (0-180)
 * @return HAL_StatusTypeDef Operation status
 */
HAL_StatusTypeDef setServoAngle(uint8_t servo_num, uint8_t angle) {
    // Validate input parameters
    if (servo_num > 15 || angle > 180) {
        return HAL_ERROR;
    }
    
    // Calculate PWM duty cycle
    uint16_t duty_cycle = calculateDutyCycle(angle);
    
    // Set PWM via I2C
    return PCA9685_SetDutyCycle(servo_num, duty_cycle);
}
```

### Commit Guidelines

**Commit Message Format:**
```
type(scope): brief description

Detailed explanation if needed

- Additional bullet points
- Reference issues: Fixes #123
```

**Types:**
- `feat`: New features
- `fix`: Bug fixes
- `docs`: Documentation updates
- `test`: Testing improvements
- `refactor`: Code restructuring
- `perf`: Performance improvements
- `hw`: Hardware-related changes

**Examples:**
```
feat(servo): add multi-servo sweep command

fix(ethernet): resolve RMII clock configuration for revision A

docs(readme): update troubleshooting for DP83848 PHY

test(network): add automated connection verification
```

### Pull Request Process

1. **Create Feature Branch**
   ```bash
   git checkout -b feature/servo-speed-control
   ```

2. **Make Changes**
   - Follow coding standards
   - Add appropriate comments
   - Update documentation if needed

3. **Test Thoroughly**
   - Verify compilation
   - Test on hardware
   - Check network functionality
   - Verify servo control

4. **Update Documentation**
   - Update README if needed
   - Add troubleshooting notes
   - Document new features

5. **Submit Pull Request**
   - Clear title and description
   - Reference related issues
   - Include testing details

### Pull Request Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Documentation update
- [ ] Performance improvement
- [ ] Hardware compatibility

## Hardware Tested
- [ ] STM32F767ZI
- [ ] Other STM32 models: 
- [ ] PHY chips tested: 

## Testing Checklist
- [ ] Code compiles without errors
- [ ] Hardware boots successfully
- [ ] Network communication works
- [ ] Servo control functional
- [ ] LED indicators correct
- [ ] No regression in existing features

## Related Issues
Fixes #

## Additional Notes
```

## 🧪 Testing Guidelines

### Required Testing

**Before submitting contributions:**

1. **Compilation Testing**
   - Clean build succeeds
   - No compiler warnings
   - Binary size within limits

2. **Hardware Testing**
   - Board boots successfully
   - LED patterns correct
   - Network connectivity
   - Servo response

3. **Network Testing**
   - DHCP/Static IP assignment
   - micro-ROS agent connection
   - Command reception
   - Data transmission

4. **Regression Testing**
   - Existing features still work
   - No performance degradation
   - Compatibility maintained

### Testing on Different Hardware

If testing on different hardware configurations:

**Document:**
- Board model and revision
- PHY chip differences
- Any required modifications
- Performance observations

**Report:**
- Compatibility status
- Required code changes
- Performance metrics
- Known limitations

## 📚 Documentation Contributions

### Areas Needing Documentation

- **Hardware Compatibility**: Test results for different STM32 models
- **PHY Support**: Configuration guides for different Ethernet PHYs
- **Troubleshooting**: Common issues and solutions
- **Performance Tuning**: Optimization tips and benchmarks
- **Integration Guides**: Using with different ROS2 packages

### Documentation Standards

- Use clear, concise language
- Include code examples where relevant
- Add diagrams for complex setups
- Provide step-by-step instructions
- Include troubleshooting tips

## 🔧 Hardware Contributions

### Supported Hardware

**Currently Supported:**
- STM32F767ZI (primary target)
- LAN8742A PHY (default)
- PCA9685 PWM driver

**Under Development:**
- Other STM32F7 variants
- DP83848 PHY support
- Alternative PWM drivers

### Adding Hardware Support

When adding support for new hardware:

1. **Document Requirements**
   - Pin assignments
   - Clock configurations
   - Special considerations

2. **Provide Configuration**
   - STM32CubeMX settings
   - Code modifications needed
   - Build instructions

3. **Test Thoroughly**
   - All core functions
   - Network performance
   - Servo control accuracy

4. **Update Documentation**
   - Add to hardware compatibility list
   - Include setup instructions
   - Note any limitations

## 🚀 Release Process

### Version Numbering

Following Semantic Versioning (SemVer):
- **MAJOR**: Incompatible API changes
- **MINOR**: Backward-compatible functionality
- **PATCH**: Backward-compatible bug fixes

### Release Checklist

- [ ] All tests pass
- [ ] Documentation updated
- [ ] CHANGELOG.md updated
- [ ] Version numbers updated
- [ ] Hardware compatibility verified

## 📞 Getting Help

### Communication Channels

- **GitHub Issues**: Bug reports and feature requests
- **GitHub Discussions**: General questions and ideas
- **Pull Request Reviews**: Code-specific discussions

### Response Times

- **Issues**: Within 2-3 days
- **Pull Requests**: Within 1 week
- **Critical Bugs**: Within 24 hours

## 📄 License

By contributing to this project, you agree that your contributions will be licensed under the same license as the project (MIT License).

## 🙏 Recognition

Contributors will be acknowledged in:
- README.md contributors section
- Release notes
- Documentation credits

Thank you for helping improve STM32 ROS2 Robot Control! 🤖 