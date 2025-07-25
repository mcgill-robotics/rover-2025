#!/bin/bash

# Integration Test Runner for Teleop System
# Runs comprehensive tests of the ROS to UI pipeline without hardware

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
TELEOP_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
TEST_TIMEOUT=300  # 5 minutes
LOG_DIR="$TELEOP_DIR/tests/logs"

# Test scenarios to run
DEFAULT_SCENARIOS=("normal" "motor_fault" "low_battery" "overheating")

# Function to print colored output
print_status() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

# Function to check prerequisites
check_prerequisites() {
    print_status $BLUE "🔍 Checking prerequisites..."
    
    # Check ROS2
    if ! command -v ros2 &> /dev/null; then
        print_status $RED "❌ ROS2 not found. Please source your ROS2 environment."
        exit 1
    fi
    
    # Check Python dependencies
    python3 -c "import aiohttp, websockets" 2>/dev/null || {
        print_status $YELLOW "⚠️  Installing Python test dependencies..."
        pip3 install aiohttp websockets
    }
    
    # Check if custom message interfaces are built
    if ! ros2 interface list | grep -q "msg_srv_interface"; then
        print_status $YELLOW "⚠️  Custom message interfaces not found. Building..."
        cd "$TELEOP_DIR/.."
        colcon build --packages-select msg_srv_interface
        source install/setup.bash
    fi
    
    print_status $GREEN "✅ Prerequisites check passed"
}

# Function to create log directory
setup_logging() {
    mkdir -p "$LOG_DIR"
    local timestamp=$(date +"%Y%m%d_%H%M%S")
    export TEST_LOG_FILE="$LOG_DIR/integration_test_$timestamp.log"
    print_status $BLUE "📝 Logging to: $TEST_LOG_FILE"
}

# Function to run a specific test scenario
run_scenario_test() {
    local scenario=$1
    local test_name="test_scenario_$scenario"
    
    print_status $BLUE "🧪 Testing scenario: $scenario"
    
    # Create a temporary test script for this scenario
    local temp_test="$LOG_DIR/temp_test_$scenario.py"
    
    cat > "$temp_test" << EOF
#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'integration'))

from test_ros_to_ui_pipeline import ROSToUIPipelineTest, TestRunner
import unittest

class ScenarioTest(ROSToUIPipelineTest):
    def test_scenario_$scenario(self):
        self.assertTrue(self._start_ros_manager(), "Failed to start ROS Manager")
        self.assertTrue(self._start_mock_firmware("$scenario"), "Failed to start mock firmware with scenario $scenario")
        
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            import time
            time.sleep(5)  # Wait for data stabilization
            
            api_results = loop.run_until_complete(self._test_rest_api())
            
            # Basic validation
            self.assertIn("/api/health", api_results)
            health_data = api_results["/api/health"]
            self.assertNotIn("error", health_data, f"Health check failed for scenario $scenario")
            
            # Test WebSocket
            websocket_data = loop.run_until_complete(self._test_websocket_connection())
            self.assertTrue(len(websocket_data) > 0, f"No WebSocket data for scenario $scenario")
            
        finally:
            loop.close()
            self._stop_processes()

if __name__ == "__main__":
    suite = unittest.TestSuite()
    suite.addTest(ScenarioTest('test_scenario_$scenario'))
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
EOF
    
    # Run the test
    if timeout $TEST_TIMEOUT python3 "$temp_test" >> "$TEST_LOG_FILE" 2>&1; then
        print_status $GREEN "✅ Scenario $scenario: PASSED"
        rm -f "$temp_test"
        return 0
    else
        print_status $RED "❌ Scenario $scenario: FAILED"
        print_status $YELLOW "   Check log file for details: $TEST_LOG_FILE"
        rm -f "$temp_test"
        return 1
    fi
}

# Function to run basic functionality tests
run_basic_tests() {
    print_status $BLUE "🔧 Running basic functionality tests..."
    
    local test_script="$TELEOP_DIR/tests/integration/test_ros_to_ui_pipeline.py"
    
    if [ ! -f "$test_script" ]; then
        print_status $RED "❌ Test script not found: $test_script"
        return 1
    fi
    
    if timeout $TEST_TIMEOUT python3 "$test_script" >> "$TEST_LOG_FILE" 2>&1; then
        print_status $GREEN "✅ Basic functionality tests: PASSED"
        return 0
    else
        print_status $RED "❌ Basic functionality tests: FAILED"
        print_status $YELLOW "   Check log file for details: $TEST_LOG_FILE"
        return 1
    fi
}

# Function to run performance tests
run_performance_tests() {
    print_status $BLUE "⚡ Running performance tests..."
    
    # Test high-frequency data publishing
    print_status $BLUE "   Testing high-frequency publishing..."
    
    # Start mock firmware with high rates
    ros2 launch sim mock_drive_test.launch.py \
        scenario:=normal \
        diagnostics_rate:=50.0 \
        speeds_rate:=100.0 &
    local mock_pid=$!
    
    sleep 3
    
    # Test topic rates
    local diagnostics_rate=$(timeout 10 ros2 topic hz /drive_motors_info 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
    local speeds_rate=$(timeout 10 ros2 topic hz /drive_speeds_info 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "0")
    
    kill $mock_pid 2>/dev/null || true
    wait $mock_pid 2>/dev/null || true
    
    # Validate rates (allow some tolerance)
    if (( $(echo "$diagnostics_rate > 40" | bc -l) )); then
        print_status $GREEN "   ✅ Diagnostics rate: ${diagnostics_rate} Hz"
    else
        print_status $RED "   ❌ Diagnostics rate too low: ${diagnostics_rate} Hz"
        return 1
    fi
    
    if (( $(echo "$speeds_rate > 80" | bc -l) )); then
        print_status $GREEN "   ✅ Speeds rate: ${speeds_rate} Hz"
    else
        print_status $RED "   ❌ Speeds rate too low: ${speeds_rate} Hz"
        return 1
    fi
    
    print_status $GREEN "✅ Performance tests: PASSED"
    return 0
}

# Function to generate test report
generate_report() {
    local total_tests=$1
    local passed_tests=$2
    local failed_tests=$3
    
    local report_file="$LOG_DIR/test_report_$(date +"%Y%m%d_%H%M%S").txt"
    
    cat > "$report_file" << EOF
===========================================
TELEOP INTEGRATION TEST REPORT
===========================================
Date: $(date)
Test Duration: $((SECONDS / 60)) minutes $((SECONDS % 60)) seconds

SUMMARY:
- Total Tests: $total_tests
- Passed: $passed_tests
- Failed: $failed_tests
- Success Rate: $(( passed_tests * 100 / total_tests ))%

ENVIRONMENT:
- ROS2 Version: $(ros2 --version 2>/dev/null || echo "Unknown")
- Python Version: $(python3 --version)
- OS: $(uname -a)

LOG FILE: $TEST_LOG_FILE

===========================================
EOF
    
    print_status $BLUE "📊 Test report generated: $report_file"
    cat "$report_file"
}

# Function to cleanup
cleanup() {
    print_status $YELLOW "🧹 Cleaning up..."
    
    # Kill any remaining processes
    pkill -f "mock_drive_firmware_node" 2>/dev/null || true
    pkill -f "ros_manager.py" 2>/dev/null || true
    
    # Wait a moment for cleanup
    sleep 2
    
    print_status $GREEN "✅ Cleanup complete"
}

# Main execution
main() {
    local scenarios=("${@:-${DEFAULT_SCENARIOS[@]}}")
    local total_tests=0
    local passed_tests=0
    local failed_tests=0
    
    print_status $GREEN "🚀 Starting Teleop Integration Tests"
    print_status $BLUE "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    # Setup
    check_prerequisites
    setup_logging
    
    # Set up cleanup trap
    trap cleanup EXIT
    
    # Run basic functionality tests
    print_status $BLUE "\n📋 Phase 1: Basic Functionality Tests"
    if run_basic_tests; then
        ((passed_tests++))
    else
        ((failed_tests++))
    fi
    ((total_tests++))
    
    # Run scenario tests
    print_status $BLUE "\n🎭 Phase 2: Scenario Tests"
    for scenario in "${scenarios[@]}"; do
        if run_scenario_test "$scenario"; then
            ((passed_tests++))
        else
            ((failed_tests++))
        fi
        ((total_tests++))
    done
    
    # Run performance tests
    print_status $BLUE "\n⚡ Phase 3: Performance Tests"
    if run_performance_tests; then
        ((passed_tests++))
    else
        ((failed_tests++))
    fi
    ((total_tests++))
    
    # Generate report
    print_status $BLUE "\n📊 Generating Test Report"
    generate_report $total_tests $passed_tests $failed_tests
    
    # Final status
    print_status $BLUE "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    if [ $failed_tests -eq 0 ]; then
        print_status $GREEN "🎉 All tests passed! ($passed_tests/$total_tests)"
        exit 0
    else
        print_status $RED "❌ Some tests failed ($failed_tests/$total_tests)"
        print_status $YELLOW "Check the log file for details: $TEST_LOG_FILE"
        exit 1
    fi
}

# Help function
show_help() {
    cat << EOF
Teleop Integration Test Runner

USAGE:
    $0 [scenarios...]

SCENARIOS:
    normal          - Normal operation test
    motor_fault     - Motor fault simulation
    low_battery     - Low battery simulation
    overheating     - Overheating simulation
    multiple_faults - Multiple motor faults
    extreme_conditions - Extreme operating conditions

EXAMPLES:
    $0                          # Run default scenarios
    $0 normal motor_fault       # Run specific scenarios
    $0 all                      # Run all available scenarios

OPTIONS:
    -h, --help     Show this help message

ENVIRONMENT VARIABLES:
    TEST_TIMEOUT   Test timeout in seconds (default: 300)
EOF
}

# Parse command line arguments
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    show_help
    exit 0
fi

if [[ "$1" == "all" ]]; then
    main normal motor_fault low_battery overheating multiple_faults extreme_conditions
else
    main "$@"
fi
