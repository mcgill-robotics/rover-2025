#!/usr/bin/env python3
"""
Vision System Status Checker
Quick utility to check the status of the vision system.
"""

import json
import socket
import time
import argparse
from typing import Dict, Any

def check_backend_status(host: str, port: int) -> Dict[str, Any]:
    """Check backend camera service status."""
    try:
        # Try to connect to backend API
        import requests
        response = requests.get(f"http://{host}:8001/api/health", timeout=5)
        if response.status_code == 200:
            return {
                'status': 'online',
                'data': response.json()
            }
        else:
            return {
                'status': 'error',
                'error': f"HTTP {response.status_code}"
            }
    except Exception as e:
        return {
            'status': 'offline',
            'error': str(e)
        }

def check_network_connectivity(host: str, port: int) -> Dict[str, Any]:
    """Check network connectivity to backend."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        result = sock.connect_ex((host, port))
        sock.close()
        
        if result == 0:
            return {'status': 'connected'}
        else:
            return {'status': 'failed', 'error': f'Connection failed (code: {result})'}
    except Exception as e:
        return {'status': 'error', 'error': str(e)}

def check_camera_devices() -> Dict[str, Any]:
    """Check available camera devices."""
    try:
        import subprocess
        result = subprocess.run(['v4l2-ctl', '--list-devices'], 
                              capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            devices = []
            lines = result.stdout.strip().splitlines()
            current_device = None
            
            for line in lines:
                if not line.startswith('\t') and line.strip():
                    current_device = line.strip()
                elif line.startswith('\t') and current_device and '/dev/video' in line:
                    devices.append({
                        'name': current_device,
                        'path': line.strip()
                    })
            
            return {
                'status': 'success',
                'devices': devices,
                'count': len(devices)
            }
        else:
            return {
                'status': 'error',
                'error': result.stderr
            }
    except Exception as e:
        return {
            'status': 'error',
            'error': str(e)
        }

def main():
    parser = argparse.ArgumentParser(description='Check vision system status')
    parser.add_argument('--backend-host', default='192.168.1.100', 
                       help='Backend server host')
    parser.add_argument('--backend-port', type=int, default=9999,
                       help='Backend server port')
    parser.add_argument('--format', choices=['text', 'json'], default='text',
                       help='Output format')
    
    args = parser.parse_args()
    
    print("🔍 Vision System Status Check")
    print("=" * 40)
    
    # Check camera devices
    print("\n📷 Camera Devices:")
    camera_status = check_camera_devices()
    if camera_status['status'] == 'success':
        print(f"  ✅ Found {camera_status['count']} camera devices")
        for device in camera_status['devices']:
            print(f"     - {device['name']} ({device['path']})")
    else:
        print(f"  ❌ Camera check failed: {camera_status['error']}")
    
    # Check network connectivity
    print(f"\n🌐 Network Connectivity ({args.backend_host}:{args.backend_port}):")
    network_status = check_network_connectivity(args.backend_host, args.backend_port)
    if network_status['status'] == 'connected':
        print("  ✅ Network connection successful")
    else:
        print(f"  ❌ Network connection failed: {network_status.get('error', 'Unknown error')}")
    
    # Check backend service
    print(f"\n🖥️  Backend Service ({args.backend_host}:8001):")
    backend_status = check_backend_status(args.backend_host, 8001)
    if backend_status['status'] == 'online':
        print("  ✅ Backend service is online")
        if 'data' in backend_status:
            data = backend_status['data']
            print(f"     - Status: {data.get('status', 'unknown')}")
            print(f"     - Uptime: {data.get('uptime', 0):.1f}s")
    else:
        print(f"  ❌ Backend service is {backend_status['status']}: {backend_status.get('error', 'Unknown error')}")
    
    # Summary
    print("\n📊 Summary:")
    all_good = (
        camera_status['status'] == 'success' and
        network_status['status'] == 'connected' and
        backend_status['status'] == 'online'
    )
    
    if all_good:
        print("  ✅ Vision system is healthy and ready")
    else:
        print("  ⚠️  Vision system has issues - check the details above")
    
    if args.format == 'json':
        result = {
            'timestamp': time.time(),
            'camera_devices': camera_status,
            'network_connectivity': network_status,
            'backend_service': backend_status,
            'overall_status': 'healthy' if all_good else 'issues'
        }
        print(json.dumps(result, indent=2))

if __name__ == "__main__":
    main()
