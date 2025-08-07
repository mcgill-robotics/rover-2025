#!/bin/bash

# Test Runner for Rover 2025 Services
# Runs all tests for the service manager, GPS service, and other components

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🧪 Running Tests for Rover 2025 Services${NC}"
echo "=============================================="

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to run a test file
run_test_file() {
    local test_file=$1
    local test_name=$2
    
    echo -e "${YELLOW}Running $test_name...${NC}"
    
    if python3 "$test_file" -v; then
        echo -e "${GREEN}✅ $test_name passed${NC}"
        return 0
    else
        echo -e "${RED}❌ $test_name failed${NC}"
        return 1
    fi
}

# Check prerequisites
echo -e "${YELLOW}Checking prerequisites...${NC}"

if ! command_exists python3; then
    echo -e "${RED}❌ Python 3 is not installed. Please install Python 3 first.${NC}"
    exit 1
fi

if ! command_exists pip3; then
    echo -e "${RED}❌ pip3 is not installed. Please install pip3 first.${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Prerequisites check passed${NC}"

# Check if pytest is installed
if ! python3 -c "import pytest" 2>/dev/null; then
    echo -e "${YELLOW}Installing pytest...${NC}"
    pip3 install pytest pytest-asyncio
    echo -e "${GREEN}✅ pytest installed${NC}"
fi

# Check if aiohttp is installed (needed for some tests)
if ! python3 -c "import aiohttp" 2>/dev/null; then
    echo -e "${YELLOW}Installing aiohttp...${NC}"
    pip3 install aiohttp
    echo -e "${GREEN}✅ aiohttp installed${NC}"
fi

# Create test results directory
mkdir -p test_results

# Initialize test counters
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

echo -e "${YELLOW}Starting test execution...${NC}"
echo ""

# Run Service Manager tests
if [ -f "tests/test_service_manager.py" ]; then
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    if run_test_file "tests/test_service_manager.py" "Service Manager Tests"; then
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        FAILED_TESTS=$((FAILED_TESTS + 1))
    fi
    echo ""
else
    echo -e "${YELLOW}⚠️  Service Manager tests not found${NC}"
fi

# Run GPS Service tests
if [ -f "tests/test_gps_service.py" ]; then
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    if run_test_file "tests/test_gps_service.py" "GPS Service Tests"; then
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        FAILED_TESTS=$((FAILED_TESTS + 1))
    fi
    echo ""
else
    echo -e "${YELLOW}⚠️  GPS Service tests not found${NC}"
fi

# Run ROS Manager tests (if they exist)
if [ -f "ros/test_ros_manager.py" ]; then
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    if run_test_file "ros/test_ros_manager.py" "ROS Manager Tests"; then
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        FAILED_TESTS=$((FAILED_TESTS + 1))
    fi
    echo ""
fi

# Run integration tests (if they exist)
if [ -f "tests/test_integration.py" ]; then
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    if run_test_file "tests/test_integration.py" "Integration Tests"; then
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        FAILED_TESTS=$((FAILED_TESTS + 1))
    fi
    echo ""
fi

# Run all tests with pytest (alternative method)
echo -e "${YELLOW}Running all tests with pytest...${NC}"
if pytest tests/ -v --tb=short --junitxml=test_results/test_results.xml; then
    echo -e "${GREEN}✅ All pytest tests passed${NC}"
else
    echo -e "${RED}❌ Some pytest tests failed${NC}"
    FAILED_TESTS=$((FAILED_TESTS + 1))
fi

echo ""
echo -e "${BLUE}📊 Test Summary${NC}"
echo "=================="
echo -e "Total test suites: $TOTAL_TESTS"
echo -e "Passed: ${GREEN}$PASSED_TESTS${NC}"
echo -e "Failed: ${RED}$FAILED_TESTS${NC}"

if [ $FAILED_TESTS -eq 0 ]; then
    echo -e "${GREEN}🎉 All tests passed!${NC}"
    exit 0
else
    echo -e "${RED}❌ Some tests failed. Check the output above for details.${NC}"
    exit 1
fi 