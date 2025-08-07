#!/usr/bin/env python3

from flask import Flask, jsonify, request
from flask_cors import CORS
import threading
import time
import json
from typing import Dict, Any

# Import the GPS service
from gps_service import GPSService
import rclpy

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Global GPS service instance
gps_service = None
gps_thread = None

def run_gps_service():
    """Run the GPS service in a separate thread"""
    global gps_service
    
    # Initialize ROS
    rclpy.init()
    
    # Create GPS service
    gps_service = GPSService()
    
    # Spin the node
    rclpy.spin(gps_service)

@app.route('/api/gps/status', methods=['GET'])
def get_gps_status():
    """Get GPS status information"""
    if gps_service is None:
        return jsonify({
            'error': 'GPS service not initialized',
            'has_fix': False,
            'fix_quality': 0,
            'satellites': 0,
            'accuracy': 0,
            'last_update': 0
        }), 503
    
    try:
        status = gps_service.get_gps_status()
        return jsonify(status)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/gps/data', methods=['GET'])
def get_gps_data():
    """Get current GPS data"""
    if gps_service is None:
        return jsonify({
            'error': 'GPS service not initialized',
            'latitude': 0.0,
            'longitude': 0.0,
            'heading': 0.0,
            'accuracy': 0.0,
            'timestamp': 0
        }), 503
    
    try:
        data = gps_service.get_gps_data()
        return jsonify(data)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/gps/stream', methods=['GET'])
def gps_stream():
    """Stream GPS data (Server-Sent Events)"""
    def generate():
        while True:
            if gps_service is None:
                yield f"data: {json.dumps({'error': 'GPS service not initialized'})}\n\n"
            else:
                try:
                    data = gps_service.get_gps_data()
                    yield f"data: {json.dumps(data)}\n\n"
                except Exception as e:
                    yield f"data: {json.dumps({'error': str(e)})}\n\n"
            
            time.sleep(1)  # Update every second
    
    return app.response_class(
        generate(),
        mimetype='text/plain'
    )

@app.route('/api/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({
        'status': 'healthy',
        'gps_service_running': gps_service is not None,
        'timestamp': int(time.time() * 1000)
    })

@app.route('/api/config', methods=['GET'])
def get_config():
    """Get GPS configuration"""
    return jsonify({
        'gps_topic': '/gps/fix',
        'imu_topic': '/imu/data',
        'velocity_topic': '/cmd_vel',
        'update_rate': 1.0,  # Hz
        'default_accuracy': 50.0  # meters
    })

def start_gps_service():
    """Start the GPS service in a background thread"""
    global gps_thread
    
    if gps_thread is None or not gps_thread.is_alive():
        gps_thread = threading.Thread(target=run_gps_service, daemon=True)
        gps_thread.start()
        print("GPS service started in background thread")

if __name__ == '__main__':
    # Start GPS service
    start_gps_service()
    
    # Start Flask app
    print("Starting GPS API server on http://localhost:5001")
    app.run(host='0.0.0.0', port=5001, debug=False) 