# Services Tests

This folder contains the test suite for individual backend services in the Rover 2025 teleop system.

## Overview

The Services Tests provide comprehensive testing for each backend service component. These are **unit and service-specific tests** that verify individual service functionality, separate from the integration tests in the main `tests/` folder.

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Test Runner   │    │   Service       │    │   Mock Data     │
│   (Pytest)      │───►│   Under Test    │───►│   (Simulated)   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Test Results   │    │  Service API    │    │  Mock Services  │
│  (Reports)      │    │  (HTTP/WS)      │    │  (ROS/GPS)      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Key Components

### Test Files
- **`test_service_manager.py`** - Service manager orchestration tests
- **`test_gps_service.py`** - GPS service functionality tests
- **`test_camera_service.py`** - Camera service functionality tests

### Test Utilities
- **Mock services** - Simulated ROS, GPS, and camera data
- **Test fixtures** - Reusable test data and configurations
- **Assertion helpers** - Custom test assertions

## Features

### Service Testing
- ✅ **Unit Tests** - Individual service component testing
- ✅ **API Testing** - HTTP endpoint validation
- ✅ **WebSocket Testing** - Real-time communication testing
- ✅ **Configuration Testing** - YAML config validation

### Mock Services
- ✅ **ROS Mock** - Simulated ROS topics and messages
- ✅ **GPS Mock** - Simulated GPS data streams
- ✅ **Camera Mock** - Simulated video streams
- ✅ **Hardware Mock** - Simulated rover hardware

### Test Coverage
- ✅ **Service Startup** - Service initialization testing
- ✅ **Health Checks** - Service health monitoring
- ✅ **Data Processing** - Message handling and conversion
- ✅ **Error Handling** - Error scenarios and recovery

## Integration with Teleop System

### Service Manager (`services/`)
- **`service_manager.py`** - Main service under test
- **`service_config.yml`** - Configuration for testing
- **Service orchestration** - Multi-service testing

### Individual Services
- **`camera/`** - Camera service testing
- **`gps/`** - GPS service testing
- **`ros/`** - ROS service testing

### Integration Tests (`tests/`)
- **`integration_test.py`** - Full system integration testing
- **Different scope** - End-to-end vs. unit testing

## Test Structure

### Service Manager Tests (`test_service_manager.py`)
```python
class TestServiceManager:
    def test_service_initialization(self):
        """Test service manager startup."""
        
    def test_health_monitoring(self):
        """Test health check functionality."""
        
    def test_service_orchestration(self):
        """Test multi-service management."""
        
    def test_graceful_shutdown(self):
        """Test clean service shutdown."""
```

### GPS Service Tests (`test_gps_service.py`)
```python
class TestGPSService:
    def test_gps_data_processing(self):
        """Test GPS data handling."""
        
    def test_sse_streaming(self):
        """Test Server-Sent Events."""
        
    def test_api_endpoints(self):
        """Test REST API endpoints."""
        
    def test_configuration_loading(self):
        """Test YAML config loading."""
```

### Camera Service Tests (`test_camera_service.py`)
```python
class TestCameraService:
    def test_camera_discovery(self):
        """Test dynamic camera discovery."""
        
    def test_rtp_streaming(self):
        """Test RTP video streaming."""
        
    def test_aruco_detection(self):
        """Test ArUco marker detection."""
        
    def test_websocket_api(self):
        """Test WebSocket video streaming."""
```

## Configuration

### Test Configuration
```yaml
# test_config.yml
test_settings:
  timeout: 30
  retries: 3
  mock_services:
    ros: true
    gps: true
    camera: true
  
mock_data:
  gps_coordinates:
    latitude: 45.5048
    longitude: -73.5772
    altitude: 50.0
  
  camera_streams:
    - camera_id: "test-cam-01"
      port: 5000
      resolution: "640x480"
```

### Pytest Configuration (`pytest.ini`)
```ini
[tool:pytest]
testpaths = tests
python_files = test_*.py
python_classes = Test*
python_functions = test_*
addopts = -v --tb=short
timeout = 30
```

## Usage

### Running Tests
```bash
# Run all service tests
cd services/tests
python3 -m pytest

# Run specific service tests
python3 -m pytest test_service_manager.py -v
python3 -m pytest test_gps_service.py -v
python3 -m pytest test_camera_service.py -v

# Run with coverage
python3 -m pytest --cov=.. --cov-report=html
```

### Test Modes
```bash
# Unit tests only
python3 -m pytest -m "not integration"

# Integration tests only
python3 -m pytest -m integration

# Fast tests (no external dependencies)
python3 -m pytest -m "not slow"
```

### Debugging Tests
```bash
# Run with debug output
python3 -m pytest -v -s

# Run single test
python3 -m pytest test_service_manager.py::TestServiceManager::test_service_initialization -v

# Run with breakpoints
python3 -m pytest --pdb
```

## File Structure

```
services/tests/
├── __init__.py                    # Python package marker
├── test_service_manager.py       # Service manager tests
├── test_gps_service.py           # GPS service tests
├── test_camera_service.py        # Camera service tests
├── conftest.py                   # Pytest configuration
├── fixtures/                     # Test fixtures
│   ├── mock_ros.py              # ROS mock data
│   ├── mock_gps.py              # GPS mock data
│   └── mock_camera.py           # Camera mock data
├── utils/                        # Test utilities
│   ├── test_helpers.py          # Helper functions
│   └── assertions.py            # Custom assertions
└── README.md                     # This documentation
```

## Dependencies

### Testing Dependencies
- **pytest** - Test framework
- **pytest-asyncio** - Async test support
- **pytest-cov** - Coverage reporting
- **pytest-timeout** - Test timeout handling

### Mock Dependencies
- **unittest.mock** - Mock objects
- **responses** - HTTP mocking
- **websockets** - WebSocket testing
- **aiohttp** - Async HTTP testing

## Test Categories

### Unit Tests
- **Service initialization** - Startup and configuration
- **Data processing** - Message handling and conversion
- **API endpoints** - HTTP endpoint functionality
- **Configuration** - YAML config loading and validation

### Integration Tests
- **Service communication** - Inter-service messaging
- **WebSocket streaming** - Real-time data streaming
- **Database operations** - Data persistence
- **External APIs** - Third-party service integration

### Performance Tests
- **Load testing** - High-traffic scenarios
- **Memory usage** - Resource consumption
- **Response times** - API performance
- **Concurrent connections** - Multi-user scenarios

## Mock Services

### ROS Mock
```python
class MockROSService:
    def __init__(self):
        self.topics = {}
        self.publishers = {}
    
    def publish_gps_data(self, latitude, longitude):
        """Publish mock GPS data."""
        
    def publish_drive_command(self, linear_x, angular_z):
        """Publish mock drive command."""
```

### GPS Mock
```python
class MockGPSService:
    def __init__(self):
        self.current_position = (45.5048, -73.5772)
        self.heading = 0.0
    
    def get_gps_data(self):
        """Get mock GPS data."""
        
    def simulate_movement(self, distance, direction):
        """Simulate GPS movement."""
```

### Camera Mock
```python
class MockCameraService:
    def __init__(self):
        self.cameras = []
        self.streams = {}
    
    def add_camera(self, camera_id, port):
        """Add mock camera."""
        
    def start_stream(self, camera_id):
        """Start mock video stream."""
```

## Troubleshooting

### Common Test Issues
1. **Import errors** - Check Python path and dependencies
2. **Timeout errors** - Increase timeout values
3. **Mock failures** - Verify mock service setup
4. **Port conflicts** - Use different ports for tests

### Debug Commands
```bash
# Check test environment
python3 -c "import pytest; print(pytest.__version__)"

# List available tests
python3 -m pytest --collect-only

# Run with verbose output
python3 -m pytest -v -s --tb=long

# Check test coverage
python3 -m pytest --cov=.. --cov-report=term-missing
```

## Integration Notes

This test suite is designed to work with:
- **Individual services** in the `services/` folder
- **Service manager** for orchestration testing
- **Mock services** for isolated testing
- **Integration tests** in the main `tests/` folder

The tests provide comprehensive coverage of backend service functionality while maintaining isolation from external dependencies.

---

**Note**: This is the service-specific testing component. For full system integration tests, see the main `tests/` folder documentation. 