# Testing Without Hardware Guide

This guide explains how to test the complete teleop system without physical rover hardware using mock nodes and automated testing tools.

## 🎯 Overview

The testing framework provides:
- **Mock Firmware Nodes** - Simulate real hardware behavior
- **Configurable Scenarios** - Test different operating conditions
- **Automated Test Suite** - Comprehensive integration testing
- **Performance Validation** - Verify system performance
- **Data Accuracy Checks** - Ensure data integrity

## 🏗️ Testing Architecture

```
Mock Firmware Node → ROS Topics → ROS Manager → WebSocket → Frontend → UI
       ↓                ↓            ↓           ↓           ↓        ↓
   Simulated Data → Real Topics → REST API → Real-time → Live UI → User
```

## 🚀 Quick Start

### 1. Basic Mock Testing

Replace the real firmware node with a mock:

```bash
# Instead of: ros2 run control drive_firmware_node
# Use this:
ros2 launch sim mock_drive_test.launch.py scenario:=normal

# Then start your services as usual:
cd teleop/services/ros && python ros_manager.py
cd teleop/robot-controller-ui && npm run dev
```

### 2. Automated Testing

Run the complete test suite:

```bash
# Run all tests
./teleop/tests/scripts/run_integration_tests.sh

# Run specific scenarios
./teleop/tests/scripts/run_integration_tests.sh normal motor_fault

# Run with help
./teleop/tests/scripts/run_integration_tests.sh --help
```

## 🎭 Available Test Scenarios

### Normal Operation
```bash
ros2 launch sim mock_drive_test.launch.py scenario:=normal
```
- Healthy motors with good battery
- Voltage: 11.5-12.5V
- Current: 0-3A
- Temperature: 20-45°C
- All motors connected

### Low Battery
```bash
ros2 launch sim mock_drive_test.launch.py scenario:=low_battery
```
- Simulates low battery conditions
- Voltage: 9.0-10.5V
- Reduced performance
- Lower current limits

### Motor Fault
```bash
ros2 launch sim mock_drive_test.launch.py scenario:=motor_fault
```
- Right-back (RB) motor disconnected
- Tests fault detection and UI error handling
- Other motors operate normally

### Multiple Faults
```bash
ros2 launch sim mock_drive_test.launch.py scenario:=multiple_faults
```
- Multiple motors with issues
- Tests system resilience
- Complex fault scenarios

### Overheating
```bash
ros2 launch sim mock_drive_test.launch.py scenario:=overheating
```
- High temperature simulation
- Temperature: 60-85°C
- Tests thermal management alerts

### Extreme Conditions
```bash
ros2 launch sim mock_drive_test.launch.py scenario:=extreme_conditions
```
- Combines multiple stress factors
- Low battery + overheating + motor fault
- Ultimate stress test

### Intermittent Faults
```bash
ros2 launch sim mock_drive_test.launch.py scenario:=intermittent_faults enable_faults:=true
```
- Random fault injection
- Tests error recovery mechanisms
- Unpredictable failure patterns

## 🔧 Advanced Configuration

### Custom Scenarios

Create custom test scenarios by modifying `sim/mock_nodes/config/test_scenarios.yaml`:

```yaml
scenarios:
  my_custom_test:
    voltage_range: [10.0, 11.0]
    current_max: 2.5
    temp_range: [30.0, 70.0]
    speed_max: 80.0
    all_motors_connected: true
    description: "My custom test scenario"
```

### Launch Parameters

```bash
ros2 launch sim mock_drive_test.launch.py \
    scenario:=normal \
    diagnostics_rate:=20.0 \
    speeds_rate:=50.0 \
    enable_faults:=true \
    fault_probability:=0.02 \
    realistic_physics:=true
```

**Parameters:**
- `scenario`: Test scenario name
- `diagnostics_rate`: Motor diagnostics publishing rate (Hz)
- `speeds_rate`: Speed data publishing rate (Hz)
- `enable_faults`: Enable random fault injection
- `fault_probability`: Probability of random faults (0.0-1.0)
- `realistic_physics`: Enable realistic motor physics simulation

## 🧪 Manual Testing Procedures

### 1. Basic Connectivity Test

```bash
# Terminal 1: Start mock firmware
ros2 launch sim mock_drive_test.launch.py scenario:=normal

# Terminal 2: Verify topics
ros2 topic list | grep drive
ros2 topic echo /drive_motors_info --once
ros2 topic echo /drive_speeds_info --once

# Terminal 3: Test service
ros2 service call /drive_motors_status msg_srv_interface/srv/DriveMotorStatus
```

### 2. ROS Manager Integration Test

```bash
# Terminal 1: Mock firmware
ros2 launch sim mock_drive_test.launch.py scenario:=normal

# Terminal 2: ROS Manager
cd teleop/services/ros
python ros_manager.py

# Terminal 3: Test API
curl http://localhost:8082/api/health
curl http://localhost:8082/api/drive/summary
```

### 3. WebSocket Test

```bash
# Install wscat if needed
npm install -g wscat

# Test WebSocket connection
wscat -c ws://localhost:8082/ws

# You should see real-time data updates
```

### 4. Full UI Test

```bash
# Terminal 1: Mock firmware
ros2 launch sim mock_drive_test.launch.py scenario:=normal

# Terminal 2: ROS Manager
cd teleop/services/ros && python ros_manager.py

# Terminal 3: Frontend
cd teleop/robot-controller-ui && npm run dev

# Open browser: http://localhost:3000
# Check connection status and live data
```

## 🔍 Debugging and Troubleshooting

### Common Issues

**1. "Custom message interfaces not found"**
```bash
# Build message interfaces
cd ~/ros2_ws
colcon build --packages-select msg_srv_interface
source install/setup.bash
```

**2. "Mock node not publishing data"**
```bash
# Check if node is running
ros2 node list | grep mock

# Check topic publication
ros2 topic hz /drive_motors_info
ros2 topic hz /drive_speeds_info
```

**3. "ROS Manager can't connect to topics"**
```bash
# Verify ROS2 environment
ros2 topic list
ros2 service list

# Check ROS Manager logs
cd teleop/services/ros
python ros_manager.py  # Check console output
```

**4. "WebSocket connection failed"**
```bash
# Check if ROS Manager is running
curl http://localhost:8082/api/health

# Test WebSocket manually
wscat -c ws://localhost:8082/ws
```

### Debug Mode

Enable detailed logging:

```bash
# Mock firmware with debug
ros2 launch sim mock_drive_test.launch.py scenario:=normal --ros-args --log-level debug

# ROS Manager with debug
cd teleop/services/ros
PYTHONPATH=. python -c "
import logging
logging.basicConfig(level=logging.DEBUG)
from ros_manager import main
main()
"
```

## 📊 Performance Testing

### Data Rate Testing

```bash
# Test high-frequency publishing
ros2 launch sim mock_drive_test.launch.py \
    scenario:=normal \
    diagnostics_rate:=50.0 \
    speeds_rate:=100.0

# Measure actual rates
ros2 topic hz /drive_motors_info
ros2 topic hz /drive_speeds_info
```

### Load Testing

```bash
# Start multiple mock nodes (different namespaces)
ros2 launch sim mock_drive_test.launch.py scenario:=normal &
ros2 launch sim mock_drive_test.launch.py scenario:=motor_fault &

# Monitor system resources
htop
```

### Latency Testing

```bash
# Measure end-to-end latency
ros2 topic echo /drive_motors_info | while read line; do
    echo "$(date '+%H:%M:%S.%3N'): $line"
done
```

## 🎯 Test Scenarios by Use Case

### Development Testing
```bash
# Quick functionality check
ros2 launch sim mock_drive_test.launch.py scenario:=normal
```

### Integration Testing
```bash
# Full automated test suite
./teleop/tests/scripts/run_integration_tests.sh
```

### Stress Testing
```bash
# Multiple failure modes
./teleop/tests/scripts/run_integration_tests.sh extreme_conditions intermittent_faults
```

### UI Development
```bash
# Stable data for UI work
ros2 launch sim mock_drive_test.launch.py scenario:=normal realistic_physics:=false
```

### Error Handling Testing
```bash
# Random faults for robustness testing
ros2 launch sim mock_drive_test.launch.py scenario:=intermittent_faults enable_faults:=true fault_probability:=0.1
```

## 📈 Validation Checklist

### ✅ Basic Functionality
- [ ] Mock firmware publishes to correct topics
- [ ] ROS Manager receives and processes data
- [ ] REST API endpoints return valid data
- [ ] WebSocket streams real-time updates
- [ ] Frontend displays live data
- [ ] Connection status indicators work

### ✅ Data Accuracy
- [ ] Motor diagnostic values are realistic
- [ ] Speed data matches expected ranges
- [ ] Service calls return correct status
- [ ] WebSocket data matches REST API
- [ ] UI values match backend data

### ✅ Error Handling
- [ ] Disconnected motors show as offline
- [ ] Fault scenarios trigger appropriate alerts
- [ ] Connection recovery works after restart
- [ ] UI gracefully handles missing data
- [ ] Error messages are informative

### ✅ Performance
- [ ] Data publishing meets rate requirements
- [ ] WebSocket latency is acceptable
- [ ] UI updates smoothly without lag
- [ ] System handles high-frequency data
- [ ] Memory usage remains stable

## 🔄 Continuous Integration

### Automated Testing in CI/CD

```yaml
# Example GitHub Actions workflow
name: Teleop Integration Tests
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Setup ROS2
        uses: ros-tooling/setup-ros@v0.2
      - name: Build packages
        run: colcon build
      - name: Run tests
        run: ./teleop/tests/scripts/run_integration_tests.sh
```

### Test Reports

The automated test runner generates detailed reports:
- Test execution summary
- Performance metrics
- Error logs and diagnostics
- Environment information

Reports are saved in `teleop/tests/logs/` with timestamps.

## 🎉 Success Criteria

A successful test run should demonstrate:

1. **Data Continuity**: Seamless data flow from mock firmware to UI
2. **Real-time Performance**: UI updates within 100ms of data publication
3. **Fault Tolerance**: Graceful handling of disconnections and errors
4. **Recovery Capability**: Automatic reconnection when services restart
5. **Data Integrity**: Accurate representation of simulated hardware state

This testing framework ensures your teleop system works reliably even when the physical rover hardware isn't available!
