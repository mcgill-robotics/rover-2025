# Teleop System Testing Guide

This directory contains comprehensive testing tools for the teleop system, including both camera management and drive data simulation.

## **Interactive Testing with Frontend (Recommended)**

You can run the mock systems alongside your frontend to see real-time behavior and test the complete user experience.

### **Camera System Testing**

Test the on-demand camera management system:

1. **Start the camera backend:**
   ```bash
   cd teleop/services/camera
   python3 central_backend.py
   ```

2. **Start mock Jetson devices:**
   ```bash
   cd teleop/tests
   python3 mock_jetson_server.py
   ```

3. **Start your frontend:**
   ```bash
   cd teleop/robot-controller-ui
   npm run dev
   ```

4. **Navigate to `/drive`** and you'll see:
   - Available cameras from mock Jetson devices
   - Ability to start/stop cameras on demand
   - Real camera feeds (mock video patterns)
   - Device status and connection info

### **Drive System Testing**

Test the ROS-based drive data pipeline:

1. **Start the ROS mock data:**
   ```bash
   cd teleop/tests
   python3 mock_ros_drive_data.py --scenario normal
   ```

2. **Start ROS manager** (if available):
   ```bash
   cd teleop/services/ros
   python3 ros_manager.py
   ```

3. **Frontend will show:**
   - Real-time motor diagnostics (voltage, current, temperature)
   - Speed information updating continuously
   - Connection status for each motor
   - Realistic data that changes based on scenario

### **Combined System Testing**

Run both systems simultaneously:

```bash
# Terminal 1: Camera backend
cd teleop/services/camera && python3 central_backend.py

# Terminal 2: Mock Jetson devices
cd teleop/tests && python3 mock_jetson_server.py

# Terminal 3: Mock drive data
cd teleop/tests && python3 mock_ros_drive_data.py --scenario normal

# Terminal 4: Frontend
cd teleop/robot-controller-ui && npm run dev
```

Then navigate to `http://localhost:3000/drive` to see both systems working together.

## **Automated Testing (Edge Cases)**

For comprehensive automated testing that checks edge cases and system behavior:

### **Full Integration Test**

```bash
cd teleop/tests
python3 full_integration_test.py --test-types full
```

This automatically:
- Starts all required services
- Tests various failure scenarios
- Validates data accuracy between systems
- Checks system recovery capabilities
- Tests concurrent camera and drive operations
- Reports detailed results

### **Individual System Tests**

Test camera system only:
```bash
python3 full_integration_test.py --test-types camera
```

Test drive system only:
```bash
python3 full_integration_test.py --test-types drive
```

## **Test Scenarios**

### **Camera Test Scenarios**

#### **Multiple Devices:**
```bash
# Simulate 3 Jetson devices with 2 cameras each
python3 mock_jetson_server.py --num-devices 3 --cameras-per-device 2
```

#### **Connection Issues:**
```bash
# Simulate unstable connections
python3 mock_jetson_server.py --scenario unstable_connection

# Simulate device failures
python3 mock_jetson_server.py --scenario device_failure
```

#### **Different Camera Types:**
```bash
# Mix of different camera types and resolutions
python3 mock_jetson_server.py --scenario mixed_cameras
```

### **Drive Test Scenarios**

#### **Normal Operation:**
```bash
python3 mock_ros_drive_data.py --scenario normal
```

#### **Motor Fault:**
```bash
# Simulates RB motor disconnection
python3 mock_ros_drive_data.py --scenario motor_fault
```

#### **Low Battery:**
```bash
# Simulates reduced voltage and performance
python3 mock_ros_drive_data.py --scenario low_battery
```

#### **Overheating:**
```bash
# Simulates high temperatures and thermal throttling
python3 mock_ros_drive_data.py --scenario overheating
```

#### **Custom Parameters:**
```bash
# Custom rates and fault injection
python3 mock_ros_drive_data.py \
  --scenario normal \
  --diagnostics-rate 15.0 \
  --speeds-rate 30.0 \
  --enable-faults
```

## **Interactive Testing Workflow**

**Recommended workflow for development and testing:**

### **Setup (4 terminals):**

1. **Terminal 1 - Camera Backend:**
   ```bash
   cd teleop/services/camera
   python3 central_backend.py
   ```

2. **Terminal 2 - Mock Jetson Server:**
   ```bash
   cd teleop/tests
   python3 mock_jetson_server.py --num-devices 2
   ```

3. **Terminal 3 - Mock Drive Data:**
   ```bash
   cd teleop/tests
   python3 mock_ros_drive_data.py --scenario normal
   ```

4. **Terminal 4 - Frontend:**
   ```bash
   cd teleop/robot-controller-ui
   npm run dev
   ```

### **Testing Actions:**

1. **Navigate to** `http://localhost:3000/drive`
2. **Camera Testing:**
   - Click cameras on/off to see real-time start/stop
   - Watch video feeds appear/disappear
   - Check device status indicators
   - Test with multiple cameras simultaneously

3. **Drive Testing:**
   - Monitor real-time motor diagnostics
   - Watch speed gauges update
   - Check connection status indicators
   - Switch scenarios to test different conditions

4. **Scenario Testing:**
   - Restart mock services with different scenarios
   - Test system recovery by stopping/starting services
   - Verify UI handles connection failures gracefully

## **What You'll See in the UI**

### **Camera Section:**
- **Device Grid:** Shows all discovered Jetson devices
- **Camera Cards:** Individual cameras with start/stop controls
- **Live Feeds:** Real video streams with test patterns
- **Status Indicators:** Connection status, frame rates, device info
- **Dynamic Management:** Cameras start only when requested

### **Drive Section:**
- **Motor Diagnostics:** Real-time voltage, current, temperature
- **Speed Gauges:** Continuously updating motor speeds
- **Status Panel:** Connection status for each motor (RF, RB, LB, LF)
- **Scenario Behavior:** Data changes based on selected scenario

## **Test Files Overview**

### **Core Test Files:**

- **`mock_jetson_server.py`** - Simulates multiple Jetson devices with cameras
- **`mock_ros_drive_data.py`** - Launches ROS-based drive data simulation
- **`full_integration_test.py`** - Comprehensive automated testing

### **Enhanced System Files:**

- **`../services/camera/jetson_server.py`** - Service-based camera management
- **`../services/camera/central_backend.py`** - Request orchestration
- **`../sim/mock_nodes/mock_drive_firmware_node.py`** - Enhanced ROS mock node

## **Troubleshooting**

### **Common Issues:**

1. **ROS Environment:**
   ```bash
   # Make sure ROS2 is sourced
   source /opt/ros/humble/setup.bash  # or your ROS distro
   
   # Build custom messages if needed
   cd /path/to/workspace
   colcon build --packages-select msg_srv_interface
   source install/setup.bash
   ```

2. **Port Conflicts:**
   - Camera backend: `localhost:8081`
   - ROS manager: `localhost:8082`
   - Frontend: `localhost:3000`

3. **Service Dependencies:**
   ```bash
   # Install Python dependencies
   cd teleop/services
   pip install -r requirements.txt
   
   # Install Node.js dependencies
   cd teleop/robot-controller-ui
   npm install
   ```

### **Debug Mode:**

Enable verbose logging:
```bash
python3 mock_jetson_server.py --log-level DEBUG
python3 mock_ros_drive_data.py --log-level DEBUG
python3 full_integration_test.py --log-level DEBUG
```

## **Testing Checklist**

### **Camera System:**
- [ ] Multiple devices discovered
- [ ] Cameras start on demand
- [ ] Video feeds display correctly
- [ ] Cameras stop when not needed
- [ ] Device connection/disconnection handling
- [ ] Multiple simultaneous camera streams

### **Drive System:**
- [ ] Motor diagnostics updating
- [ ] Speed data flowing
- [ ] Connection status accurate
- [ ] Different scenarios working
- [ ] Data ranges realistic
- [ ] Fault conditions handled

### **Integration:**
- [ ] Both systems work simultaneously
- [ ] No interference between systems
- [ ] Frontend responsive to both data streams
- [ ] System recovery after failures
- [ ] Performance under load

## **Performance Testing**

Test system performance with high loads:

```bash
# High-frequency updates
python3 mock_ros_drive_data.py --diagnostics-rate 50.0 --speeds-rate 100.0

# Many cameras
python3 mock_jetson_server.py --num-devices 5 --cameras-per-device 4

# Stress test
python3 full_integration_test.py --test-types full --log-level DEBUG
```

This testing framework provides comprehensive coverage of the teleop system, allowing you to test both individual components and the complete integrated system with realistic scenarios and edge cases.
