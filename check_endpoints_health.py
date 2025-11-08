#!/usr/bin/env python3
"""
Backend Endpoint Health Check Script

Checks all REST API endpoints and SocketIO events to verify they're working properly.
Tests both GET and POST endpoints with appropriate test data.
"""

import requests
import json
import sys
import time
from typing import Dict, List, Tuple
import socketio

# Configuration
BASE_URL = "http://localhost:5001"
TIMEOUT = 5  # seconds

# Color codes for output
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    END = '\033[0m'

def print_header(text: str):
    """Print formatted header."""
    print(f"\n{Colors.BOLD}{Colors.CYAN}{'='*70}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.CYAN}{text.center(70)}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.CYAN}{'='*70}{Colors.END}\n")

def print_success(text: str):
    """Print success message."""
    print(f"{Colors.GREEN}✓{Colors.END} {text}")

def print_error(text: str):
    """Print error message."""
    print(f"{Colors.RED}✗{Colors.END} {text}")

def print_warning(text: str):
    """Print warning message."""
    print(f"{Colors.YELLOW}⚠{Colors.END} {text}")

def print_info(text: str):
    """Print info message."""
    print(f"{Colors.BLUE}ℹ{Colors.END} {text}")

def check_server_running() -> bool:
    """Check if the server is running."""
    try:
        response = requests.get(f"{BASE_URL}/api/health", timeout=TIMEOUT)
        return response.status_code == 200
    except requests.exceptions.RequestException:
        return False

def test_get_endpoint(endpoint: str, expected_status: int = 200) -> Tuple[bool, str, Dict]:
    """Test a GET endpoint."""
    try:
        response = requests.get(f"{BASE_URL}{endpoint}", timeout=TIMEOUT)
        success = response.status_code == expected_status
        
        try:
            data = response.json()
        except:
            data = {"text": response.text[:100]}
        
        return success, f"Status: {response.status_code}", data
    except requests.exceptions.Timeout:
        return False, "Request timeout", {}
    except requests.exceptions.RequestException as e:
        return False, f"Request failed: {str(e)}", {}

def test_post_endpoint(endpoint: str, payload: Dict = None, expected_status: int = 200) -> Tuple[bool, str, Dict]:
    """Test a POST endpoint."""
    try:
        headers = {'Content-Type': 'application/json'}
        response = requests.post(
            f"{BASE_URL}{endpoint}", 
            json=payload or {},
            headers=headers,
            timeout=TIMEOUT
        )
        success = response.status_code == expected_status
        
        try:
            data = response.json()
        except:
            data = {"text": response.text[:100]}
        
        return success, f"Status: {response.status_code}", data
    except requests.exceptions.Timeout:
        return False, "Request timeout", {}
    except requests.exceptions.RequestException as e:
        return False, f"Request failed: {str(e)}", {}

def test_rest_endpoints():
    """Test all REST API endpoints."""
    print_header("REST API ENDPOINTS")
    
    endpoints = {
        "Core Endpoints": [
            ("GET", "/", 200, None, "Home page"),
            ("GET", "/api/health", 200, None, "Health check"),
        ],
        "Activity & Monitoring": [
            ("GET", "/monitor", 200, None, "Monitor page"),
            ("GET", "/api/activity", 200, None, "Get activity logs"),
            ("GET", "/api/activity/types", 200, None, "Get activity types"),
        ],
        "Vehicle Control": [
            ("POST", "/api/arm", 200, {"arm": False}, "Arm/disarm vehicle"),
            ("POST", "/api/set_mode", 200, {"mode": "STABILIZE"}, "Set flight mode"),
        ],
        "Mission Management": [
            ("GET", "/api/mission/download", 200, None, "Download mission"),
            ("POST", "/api/mission/upload", 200, {"waypoints": []}, "Upload mission"),
            ("POST", "/api/mission/clear", 200, None, "Clear mission"),
            ("POST", "/api/mission/set_current", 200, {"seq": 0}, "Set current waypoint"),
            ("POST", "/api/mission/pause", 200, None, "Pause mission"),
            ("POST", "/api/mission/resume", 200, None, "Resume mission"),
        ],
        "RTK/GPS": [
            ("GET", "/api/rtk/status", 200, None, "Get RTK status"),
            ("POST", "/api/rtk/inject", 200, {"data": "test"}, "Inject RTK data"),
            ("POST", "/api/rtk/stop", 200, None, "Stop RTK injection"),
        ],
        "Servo Control": [
            ("POST", "/api/servo/control", 200, {"channel": 10, "pwm": 1500}, "Control servo"),
            ("GET", "/servo/status", 200, None, "Get servo status"),
            ("GET", "/servo/run", 200, None, "Run servo sequence"),
            ("GET", "/servo/stop", 200, None, "Stop servo sequence"),
            ("POST", "/servo/emergency_stop", 200, None, "Emergency stop servo"),
            ("GET", "/servo/edit", 200, None, "Edit servo config page"),
            ("POST", "/servo/edit", 200, {}, "Save servo config"),
            ("GET", "/servo/log", 200, None, "Get servo logs"),
        ],
        "Node Management": [
            ("GET", "/api/nodes", 200, None, "Get all nodes"),
        ],
    }
    
    total = 0
    passed = 0
    failed = 0
    
    for category, tests in endpoints.items():
        print(f"\n{Colors.BOLD}{category}:{Colors.END}")
        for test in tests:
            method, endpoint, expected_status, payload, description = test
            total += 1
            
            if method == "GET":
                success, message, data = test_get_endpoint(endpoint, expected_status)
            else:  # POST
                success, message, data = test_post_endpoint(endpoint, payload, expected_status)
            
            if success:
                passed += 1
                print_success(f"{method:4} {endpoint:35} - {description}")
                if data and isinstance(data, dict):
                    # Show some response data for key endpoints
                    if endpoint in ["/api/health", "/api/rtk/status", "/servo/status"]:
                        print(f"       Response: {json.dumps(data, indent=2)[:200]}")
            else:
                failed += 1
                print_error(f"{method:4} {endpoint:35} - {description}")
                print(f"       {message}")
    
    return total, passed, failed

def test_socketio_events():
    """Test SocketIO events (connection only, not full functionality)."""
    print_header("SOCKETIO EVENTS")
    
    events = [
        ("connect", "Client connection"),
        ("disconnect", "Client disconnection"),
        ("ping", "Ping/pong"),
        ("request_mission_logs", "Request mission logs"),
        ("mission_upload", "Upload mission via SocketIO"),
        ("connect_caster", "Connect to RTK caster"),
        ("disconnect_caster", "Disconnect from RTK caster"),
        ("request_rover_reconnect", "Request rover reconnect"),
        ("send_command", "Send MAVLink command"),
        ("inject_mavros_telemetry", "Inject telemetry data"),
    ]
    
    print(f"{Colors.BOLD}SocketIO Events Available:{Colors.END}")
    for event_name, description in events:
        print_info(f"{event_name:30} - {description}")
    
    # Try to connect to SocketIO
    print(f"\n{Colors.BOLD}Testing SocketIO Connection:{Colors.END}")
    try:
        sio = socketio.Client()
        
        @sio.event
        def connect():
            print_success("SocketIO connection established")
        
        @sio.event
        def disconnect():
            print_info("SocketIO disconnected")
        
        sio.connect(BASE_URL, wait_timeout=TIMEOUT)
        time.sleep(1)
        
        # Test ping
        try:
            sio.emit('ping')
            print_success("Ping event sent successfully")
        except Exception as e:
            print_error(f"Ping event failed: {e}")
        
        sio.disconnect()
        return True
    except Exception as e:
        print_error(f"SocketIO connection failed: {e}")
        return False

def check_background_services():
    """Check status of background services."""
    print_header("BACKGROUND SERVICES")
    
    services = [
        ("MAVROS Bridge", "ROS2 connection to flight controller"),
        ("Telemetry Loop", "Continuous telemetry updates"),
        ("Connection Monitor", "Health monitoring"),
        ("RTK Injection", "RTK correction data"),
        ("Network Monitor", "WiFi/LoRa status"),
    ]
    
    # Try to get health status
    try:
        response = requests.get(f"{BASE_URL}/api/health", timeout=TIMEOUT)
        if response.status_code == 200:
            health_data = response.json()
            print_success(f"Server Status: {health_data.get('status', 'unknown')}")
            print_info(f"Timestamp: {health_data.get('timestamp', 'N/A')}")
            
            if 'mavros_connected' in health_data:
                if health_data['mavros_connected']:
                    print_success("MAVROS: Connected")
                else:
                    print_warning("MAVROS: Disconnected")
            
            if 'ros_connection' in health_data:
                if health_data['ros_connection']:
                    print_success("ROS Connection: Active")
                else:
                    print_warning("ROS Connection: Inactive")
    except Exception as e:
        print_error(f"Could not retrieve health status: {e}")
    
    print(f"\n{Colors.BOLD}Expected Background Services:{Colors.END}")
    for service, description in services:
        print_info(f"{service:20} - {description}")

def print_summary(total: int, passed: int, failed: int, socketio_ok: bool):
    """Print test summary."""
    print_header("TEST SUMMARY")
    
    print(f"Total REST Endpoints Tested: {Colors.BOLD}{total}{Colors.END}")
    print(f"Passed: {Colors.GREEN}{passed}{Colors.END}")
    print(f"Failed: {Colors.RED}{failed}{Colors.END}")
    
    success_rate = (passed / total * 100) if total > 0 else 0
    print(f"Success Rate: {Colors.BOLD}{success_rate:.1f}%{Colors.END}")
    
    print(f"\nSocketIO Status: ", end="")
    if socketio_ok:
        print(f"{Colors.GREEN}✓ Working{Colors.END}")
    else:
        print(f"{Colors.YELLOW}⚠ Not tested or failed{Colors.END}")
    
    print(f"\n{Colors.BOLD}Overall Status:{Colors.END} ", end="")
    if failed == 0 and socketio_ok:
        print(f"{Colors.GREEN}✓ All systems operational{Colors.END}")
        return 0
    elif failed < total * 0.2:  # Less than 20% failure
        print(f"{Colors.YELLOW}⚠ Mostly operational (some issues){Colors.END}")
        return 1
    else:
        print(f"{Colors.RED}✗ Multiple issues detected{Colors.END}")
        return 2

def main():
    """Main function."""
    print(f"{Colors.BOLD}{Colors.BLUE}")
    print("╔═══════════════════════════════════════════════════════════════════╗")
    print("║         NRP Backend Endpoint Health Check                        ║")
    print("╚═══════════════════════════════════════════════════════════════════╝")
    print(Colors.END)
    
    print_info(f"Testing server at: {BASE_URL}")
    print_info(f"Timeout: {TIMEOUT} seconds")
    
    # Check if server is running
    print_header("SERVER STATUS")
    if check_server_running():
        print_success("Backend server is running")
    else:
        print_error("Backend server is not responding!")
        print_warning("Make sure the server is running:")
        print(f"  {Colors.BOLD}sudo systemctl status nrp-service{Colors.END}")
        print(f"  or")
        print(f"  {Colors.BOLD}cd Backend && python3 server.py{Colors.END}")
        return 1
    
    # Run tests
    total, passed, failed = test_rest_endpoints()
    socketio_ok = test_socketio_events()
    check_background_services()
    
    # Print summary
    return print_summary(total, passed, failed, socketio_ok)

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print(f"\n\n{Colors.YELLOW}⚠ Test interrupted by user{Colors.END}")
        sys.exit(130)
    except Exception as e:
        print(f"\n{Colors.RED}✗ Fatal error: {e}{Colors.END}")
        sys.exit(1)
