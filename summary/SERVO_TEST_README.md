# Servo Testing Suite

A comprehensive set of tools for testing servo channels using ROS2 services and MAVROS.

## 📁 Files Overview

| File | Purpose |
|------|---------|
| `run_servo_test.sh` | Main test runner with auto-detection |
| `test_servo_10.py` | Python test script using rosbridge |
| `test_servo_10_ros2.py` | Python test script using native ROS2 |
| `servo_test_config.sh` | Configuration file for test parameters |
| `servo_test_examples.sh` | Interactive examples and tutorials |
| `SERVO_TEST_README.md` | This documentation |

## 🚀 Quick Start

### Default Test (Servo 10, PWM 600→1000, 2s delay)
```bash
./run_servo_test.sh
```

### Custom Parameters (One-time)
```bash
SERVO_CHANNEL=9 FIRST_PWM=1200 SECOND_PWM=1800 ./run_servo_test.sh
```

### Custom Parameters (Persistent)
1. Edit `servo_test_config.sh`
2. Run `./run_servo_test.sh`

## ⚙️ Configuration Options

### Via Environment Variables
```bash
export SERVO_CHANNEL=10      # Servo channel (1-16)
export FIRST_PWM=600         # First PWM value (μs)
export SECOND_PWM=1000       # Second PWM value (μs)  
export DELAY_SECONDS=2       # Wait time between changes
export TIMEOUT_SECONDS=5     # Service call timeout
export ROSBRIDGE_PORT=9090   # Rosbridge port
```

### Via Configuration File
Edit `servo_test_config.sh`:
```bash
SERVO_CHANNEL=10
FIRST_PWM=600
SECOND_PWM=1000
DELAY_SECONDS=2
TIMEOUT_SECONDS=5
ROSBRIDGE_PORT=9090
```

## 🎯 Test Sequence

1. **Initialize**: Connect to MAVROS services
2. **Step 1**: Set servo to `FIRST_PWM` value
3. **Step 2**: Wait `DELAY_SECONDS` seconds
4. **Step 3**: Set servo to `SECOND_PWM` value  
5. **Complete**: Report results and disconnect

## 🔧 Technical Details

- **Command Used**: `MAV_CMD_DO_SET_SERVO` (183)
- **Parameters**: 
  - param1: Servo channel number
  - param2: PWM value in microseconds
- **Service**: `/mavros/cmd/command` (CommandLong)
- **Protocols**: rosbridge WebSocket or native ROS2

## 📋 Test Methods

### Auto-Detection (Recommended)
```bash
./run_servo_test.sh
```
Automatically selects the best available method.

### Force Specific Method
```bash
./run_servo_test.sh --bridge    # Use rosbridge
./run_servo_test.sh --ros2      # Use native ROS2
```

### Direct Python Execution
```bash
python3 test_servo_10.py        # Rosbridge version
python3 test_servo_10_ros2.py   # ROS2 version
```

## 🛡️ Safety Guidelines

### Before Testing
- [ ] Verify servo channel is properly configured
- [ ] Ensure no critical systems on test channel
- [ ] Remove propellers if testing motor outputs
- [ ] Test on ground only
- [ ] Check PWM values are within safe range

### Common PWM Ranges
- **Standard Servo**: 1000-2000μs (center ~1500μs)
- **ESC/Motor**: 1000-2000μs (1000=stop, 2000=full)
- **Camera Gimbal**: Varies by manufacturer
- **Landing Gear**: Varies by system

### Emergency Stop
- Press `Ctrl+C` to interrupt any test
- Use flight controller safety switches
- Have manual override ready

## 🔍 Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| No MAVROS connection | Check `ros2 topic list \| grep mavros` |
| Rosbridge not found | Start: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` |
| Service timeout | Increase `TIMEOUT_SECONDS` |
| Servo not moving | Check channel configuration and PWM range |

### Diagnostic Commands
```bash
# Check MAVROS topics
ros2 topic list | grep mavros

# Check servo output
ros2 topic echo /mavros/rc/out

# Check command service
ros2 service list | grep command

# Test rosbridge connection
netstat -tln | grep 9090
```

## 📊 Example Configurations

### ESC/Motor Test (Channel 1)
```bash
SERVO_CHANNEL=1
FIRST_PWM=1000    # Stop
SECOND_PWM=1200   # Low throttle  
DELAY_SECONDS=3
```

### Camera Gimbal (Channel 9)
```bash
SERVO_CHANNEL=9
FIRST_PWM=1200    # Up position
SECOND_PWM=1800   # Down position
DELAY_SECONDS=2
```

### Landing Gear (Channel 10)
```bash
SERVO_CHANNEL=10
FIRST_PWM=1100    # Retracted
SECOND_PWM=1900   # Extended
DELAY_SECONDS=5   # Allow time for movement
```

## 🎓 Learning Examples

Run interactive examples:
```bash
./servo_test_examples.sh
```

This provides guided examples for different servo types and use cases.

## 📝 Integration Notes

These scripts integrate with the existing NRP-Service infrastructure:
- Uses existing `MavrosBridge` class
- Compatible with current MAVROS setup  
- Follows project coding standards
- Includes comprehensive error handling

## 🔄 Extending the Tests

To add new test patterns:
1. Copy `test_servo_10.py` to new filename
2. Modify test sequence in `test_servo_channel()` function
3. Update configuration variables as needed
4. Add new script to `run_servo_test.sh` options

## 📞 Support

For issues or questions:
1. Check troubleshooting section above
2. Review MAVROS and ROS2 logs
3. Verify hardware connections
4. Test with known-good servo first