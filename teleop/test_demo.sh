#!/bin/bash

# Quick Demo Script for Testing Without Hardware
# This script demonstrates the complete testing system

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

print_status $GREEN "Teleop Testing Demo - Hardware-Free Testing"
print_status $BLUE "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    print_status $RED "❌ ROS2 not found. Please source your ROS2 environment:"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   source ~/ros2_ws/install/setup.bash"
    exit 1
fi

print_status $BLUE "📋 Available Testing Options:"
echo ""
echo "1. Mock Firmware Demo"
echo "   - Start mock firmware node with different scenarios"
echo "   - See realistic motor data without hardware"
echo ""
echo "2. Integration Test Demo"
echo "   - Test complete ROS → UI pipeline"
echo "   - Automated validation of all components"
echo ""
echo "3. Full System Demo"
echo "   - Start mock firmware + ROS Manager + UI"
echo "   - Complete working system demonstration"
echo ""
echo "4. Performance Test Demo"
echo "   - High-frequency data testing"
echo "   - System performance validation"
echo ""

read -p "Choose an option (1-4): " choice

case $choice in
    1)
        print_status $BLUE "Starting Mock Firmware Demo..."
        echo ""
        print_status $YELLOW "Available scenarios:"
        echo "  - normal: Healthy operation"
        echo "  - motor_fault: RB motor disconnected"
        echo "  - low_battery: Low voltage simulation"
        echo "  - overheating: High temperature simulation"
        echo ""
        
        read -p "Enter scenario (default: normal): " scenario
        scenario=${scenario:-normal}
        
        print_status $BLUE "Starting mock firmware with scenario: $scenario"
        print_status $YELLOW "Press Ctrl+C to stop"
        echo ""
        
        # Start mock firmware
        ros2 launch sim mock_drive_test.launch.py scenario:=$scenario
        ;;
        
    2)
        print_status $BLUE "Starting Integration Test Demo..."
        echo ""
        print_status $YELLOW "This will run automated tests of the complete system"
        print_status $YELLOW "Tests include: API endpoints, WebSocket, data accuracy"
        echo ""
        
        read -p "Press Enter to start tests..."
        
        # Run integration tests
        ./teleop/tests/scripts/run_integration_tests.sh normal motor_fault
        ;;
        
    3)
        print_status $BLUE "Starting Full System Demo..."
        echo ""
        print_status $YELLOW "This will start:"
        echo "  1. Mock firmware node"
        echo "  2. ROS Manager (API + WebSocket)"
        echo "  3. Frontend development server"
        echo ""
        print_status $YELLOW "After startup, open: http://localhost:3000"
        echo ""
        
        read -p "Press Enter to start full system..."
        
        # Use the existing startup script
        ./start_teleop.sh &
        STARTUP_PID=$!
        
        # Wait a moment then start mock firmware instead of real firmware
        sleep 5
        print_status $BLUE "Starting mock firmware..."
        ros2 launch sim mock_drive_test.launch.py scenario:=normal &
        MOCK_PID=$!
        
        print_status $GREEN "✅ System started!"
        print_status $BLUE "Web UI: http://localhost:3000"
        print_status $BLUE "API: http://localhost:8082/api/health"
        print_status $YELLOW "Press Ctrl+C to stop all services"
        
        # Wait for user interrupt
        trap "kill $STARTUP_PID $MOCK_PID 2>/dev/null; exit" INT
        wait
        ;;
        
    4)
        print_status $BLUE "Starting Performance Test Demo..."
        echo ""
        print_status $YELLOW "Testing high-frequency data publishing..."
        
        # Start high-rate mock firmware
        print_status $BLUE "Starting mock firmware with high data rates..."
        ros2 launch sim mock_drive_test.launch.py \
            scenario:=normal \
            diagnostics_rate:=50.0 \
            speeds_rate:=100.0 &
        MOCK_PID=$!
        
        sleep 3
        
        print_status $BLUE "Measuring topic publication rates..."
        echo ""
        
        # Test diagnostics rate
        print_status $YELLOW "Testing /drive_motors_info rate (target: 50 Hz)..."
        timeout 10 ros2 topic hz /drive_motors_info || true
        
        echo ""
        
        # Test speeds rate  
        print_status $YELLOW "Testing /drive_speeds_info rate (target: 100 Hz)..."
        timeout 10 ros2 topic hz /drive_speeds_info || true
        
        # Cleanup
        kill $MOCK_PID 2>/dev/null || true
        
        print_status $GREEN "✅ Performance test complete!"
        ;;
        
    *)
        print_status $RED "Invalid option. Please choose 1-4."
        exit 1
        ;;
esac

print_status $BLUE "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
print_status $GREEN "Demo complete!"
echo ""
print_status $BLUE "For more information, see:"
echo "  - teleop/TESTING_WITHOUT_HARDWARE.md - Complete testing guide"
echo "  - teleop/robot-controller-ui/README.md - UI documentation"
echo "  - sim/mock_nodes/config/test_scenarios.yaml - Test scenarios"
echo ""
print_status $BLUE "vQuick commands:"
echo "  - Mock firmware: ros2 launch sim mock_drive_test.launch.py scenario:=normal"
echo "  - Full tests: .//tests/scripts/run_integration_tests.sh"
echo "  - System startup: ./start_teleop.sh"
