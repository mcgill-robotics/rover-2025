# Integration Tests

This folder contains the comprehensive integration test suite for the Rover 2025 teleop system.

## Overview

The Integration Tests provide end-to-end testing of the complete teleop system, including the React UI, all backend services, and their interactions. This is the **single integration test** that verifies the entire system works together.

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Test Runner   │    │   Full System   │    │   Mock Hardware │
│   (Pytest)      │───►│   (All Services)│───►│   (Simulated)   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Test Results   │    │  UI + Services  │    │  Mock Rover     │
│  (Reports)      │    │  (Ports 3000+)  │    │  (ROS Topics)   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Key Components

### Test Files
- **`integration_test.py`** - Main integration test suite
- **`mock_jetson_server.py`** - Mock Jetson camera server
- **`full_integration_test.py`** - Complete system test

### Test Utilities
- **Selenium WebDriver** - UI automation and testing
- **Mock services** - Simulated backend services
- **Test fixtures** - System-wide test data

## Features

### System Integration
- ✅ **Full Stack Testing** - UI + Backend + Services
- ✅ **End-to-End Workflows** - Complete user journeys
- ✅ **Service Communication** - Inter-service messaging
- ✅ **Real-time Features** - WebSocket and SSE testing

### UI Testing
- ✅ **React Component Testing** - UI functionality
- ✅ **User Interaction Testing** - Click, type, navigation
- ✅ **Responsive Design Testing** - Different screen sizes
- ✅ **Cross-browser Testing** - Multiple browsers

### Service Testing
- ✅ **Service Startup** - All services initialization
- ✅ **Health Monitoring** - Service status verification
- ✅ **Data Flow Testing** - End-to-end data processing
- ✅ **Error Recovery** - System failure scenarios

## Integration with Teleop System

### React UI (`robot-controller-ui/`)
- **All pages** - Drive, Arm, Mapping, Status
- **All components** - Navigation, controls, displays
- **Real-time features** - WebSocket connections, SSE streams

### Backend Services (`services/`)
- **Service Manager** - Orchestration and health monitoring
- **ROS Manager** - ROS communication bridge
- **GPS Service** - GPS data and mapping
- **Camera Service** - Multi-camera streaming
- **TileServer** - Offline map serving

### Individual Tests (`services/tests/`)
- **Unit tests** - Individual service testing
- **Different scope** - Component vs. system testing

## Test Structure

### Main Integration Test (`integration_test.py`)
```python
class TestTeleopSystem:
    def test_system_startup(self):
        """Test complete system initialization."""
        
    def test_drive_control_workflow(self):
        """Test drive control from UI to hardware."""
        
    def test_arm_control_workflow(self):
        """Test arm control from UI to hardware."""
        
    def test_mapping_functionality(self):
        """Test GPS and mapping features."""
        
    def test_camera_streaming(self):
        """Test multi-camera video streaming."""
        
    def test_service_health_monitoring(self):
        """Test service health and status."""
```

### Mock Services
```python
class MockJetsonServer:
    """Simulates Jetson camera server."""
    
class MockROSSystem:
    """Simulates ROS hardware system."""
    
class MockGPSSystem:
    """Simulates GPS hardware."""
```

## Configuration

### Test Configuration
```yaml
# integration_test_config.yml
test_settings:
  ui_url: "http://localhost:3000"
  service_manager_url: "http://localhost:8083"
  ros_manager_url: "http://localhost:8082"
  gps_service_url: "http://localhost:5001"
  camera_service_url: "http://localhost:8001"
  tileserver_url: "http://localhost:8080"
  
browser_settings:
  headless: true
  window_size: "1920x1080"
  timeout: 30
  
mock_services:
  jetson_server: true
  ros_system: true
  gps_hardware: true
```

### Selenium Configuration
```python
# WebDriver setup
from selenium import webdriver
from selenium.webdriver.chrome.options import Options

chrome_options = Options()
chrome_options.add_argument("--headless")
chrome_options.add_argument("--no-sandbox")
chrome_options.add_argument("--disable-dev-shm-usage")

driver = webdriver.Chrome(options=chrome_options)
```

## Usage

### Running Integration Tests
```bash
# Run all integration tests
cd tests
python3 -m pytest integration_test.py -v

# Run specific test
python3 -m pytest integration_test.py::TestTeleopSystem::test_drive_control_workflow -v

# Run with UI visible (not headless)
python3 -m pytest integration_test.py --headed
```

### Test Modes
```bash
# Full system test
python3 -m pytest integration_test.py -m "full_system"

# UI-only test
python3 -m pytest integration_test.py -m "ui_only"

# Service-only test
python3 -m pytest integration_test.py -m "services_only"

# Quick test (basic functionality)
python3 -m pytest integration_test.py -m "quick"
```

### Debugging Tests
```bash
# Run with debug output
python3 -m pytest integration_test.py -v -s

# Run with browser visible
python3 -m pytest integration_test.py --headed --slow

# Run with screenshots on failure
python3 -m pytest integration_test.py --screenshot-on-failure
```

## File Structure

```
tests/
├── __init__.py                    # Python package marker
├── integration_test.py           # Main integration test
├── full_integration_test.py      # Complete system test
├── mock_jetson_server.py         # Mock Jetson server
├── conftest.py                   # Pytest configuration
├── fixtures/                     # Test fixtures
│   ├── system_setup.py          # System initialization
│   ├── mock_services.py         # Mock service setup
│   └── test_data.py             # Test data generation
├── utils/                        # Test utilities
│   ├── ui_helpers.py            # UI interaction helpers
│   ├── service_helpers.py       # Service interaction helpers
│   └── assertions.py            # Custom assertions
└── README.md                     # This documentation
```

## Dependencies

### Testing Dependencies
- **pytest** - Test framework
- **selenium** - Web browser automation
- **pytest-selenium** - Selenium integration
- **pytest-asyncio** - Async test support

### Browser Dependencies
- **Chrome/Chromium** - Web browser for testing
- **ChromeDriver** - Chrome automation driver
- **GeckoDriver** - Firefox automation driver (optional)

### Mock Dependencies
- **unittest.mock** - Mock objects
- **responses** - HTTP mocking
- **websockets** - WebSocket testing
- **aiohttp** - Async HTTP testing

## Test Scenarios

### Drive Control Workflow
1. **Start system** - Launch all services
2. **Open UI** - Navigate to drive page
3. **Connect to services** - Verify WebSocket connections
4. **Send drive command** - Use UI controls
5. **Verify ROS message** - Check ROS topic
6. **Check motor response** - Verify hardware response

### Arm Control Workflow
1. **Navigate to arm page** - Open arm interface
2. **Load arm model** - Initialize 3D visualization
3. **Send joint commands** - Use UI controls
4. **Verify IK calculations** - Check inverse kinematics
5. **Test trajectory planning** - Verify smooth movement

### Mapping Workflow
1. **Start mapping services** - GPS + TileServer
2. **Open mapping page** - Navigate to map interface
3. **Load offline tiles** - Verify map display
4. **Stream GPS data** - Check real-time updates
5. **Test waypoint creation** - Add and manage waypoints

### Camera Workflow
1. **Start camera services** - Camera + Jetson mock
2. **Open camera interface** - Navigate to camera page
3. **Connect to streams** - Establish WebSocket connections
4. **Display video feeds** - Verify multi-camera display
5. **Test ArUco detection** - Verify marker detection

## Mock Services

### Mock Jetson Server
```python
class MockJetsonServer:
    def __init__(self, device_id):
        self.device_id = device_id
        self.cameras = []
        self.streams = {}
    
    def start_camera_stream(self, camera_id, port):
        """Start mock camera stream."""
        
    def send_rtp_frame(self, camera_id, frame_data):
        """Send mock RTP video frame."""
```

### Mock ROS System
```python
class MockROSSystem:
    def __init__(self):
        self.topics = {}
        self.publishers = {}
        self.subscribers = {}
    
    def publish_gps_data(self, latitude, longitude):
        """Publish mock GPS data."""
        
    def publish_motor_status(self, motor_id, status):
        """Publish mock motor status."""
```

### Mock GPS Hardware
```python
class MockGPSHardware:
    def __init__(self):
        self.position = (45.5048, -73.5772)
        self.heading = 0.0
        self.accuracy = 5.0
    
    def get_gps_fix(self):
        """Get mock GPS fix."""
        
    def simulate_movement(self, distance, direction):
        """Simulate GPS movement."""
```

## Troubleshooting

### Common Test Issues
1. **Browser not found** - Install Chrome/Chromium and ChromeDriver
2. **Service startup failures** - Check port availability and dependencies
3. **UI element not found** - Check page loading and element selectors
4. **Timeout errors** - Increase timeout values or check system performance

### Debug Commands
```bash
# Check system status
curl http://localhost:8083/api/health
curl http://localhost:3000

# Check browser setup
which google-chrome
which chromedriver

# Run with verbose output
python3 -m pytest integration_test.py -v -s --headed

# Check test environment
python3 -c "import selenium; print(selenium.__version__)"
```

## Integration Notes

This integration test suite is designed to work with:
- **Complete teleop system** - All services and UI
- **Mock hardware** - Simulated rover components
- **Real browsers** - Actual web browser testing
- **Service tests** - Individual service testing in `services/tests/`

The tests provide comprehensive end-to-end verification of the entire teleop system functionality.

---

**Note**: This is the main integration testing component. For individual service testing, see the `services/tests/` folder documentation. 