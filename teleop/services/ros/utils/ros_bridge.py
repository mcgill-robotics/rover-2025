"""
ROS Bridge utilities for connecting ROS2 nodes to web services.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import json
import asyncio
from typing import Dict, Any, Callable, Optional
import logging

logger = logging.getLogger(__name__)


class ROSBridge:
    """
    A bridge between ROS2 and web services that manages ROS node lifecycle
    and provides async interfaces for web applications.
    """
    
    def __init__(self):
        self.executor = None
        self.executor_thread = None
        self.nodes = {}
        self.is_running = False
        
    def initialize(self):
        """Initialize ROS2 context and executor."""
        if not rclpy.ok():
            rclpy.init()
        
        self.executor = MultiThreadedExecutor()
        self.is_running = True
        
        # Start executor in separate thread
        self.executor_thread = threading.Thread(target=self._run_executor, daemon=True)
        self.executor_thread.start()
        
        logger.info("ROS Bridge initialized")
    
    def _run_executor(self):
        """Run the ROS executor in a separate thread."""
        try:
            self.executor.spin()
        except Exception as e:
            logger.error(f"Executor error: {e}")
    
    def add_node(self, name: str, node: Node):
        """Add a ROS node to the bridge."""
        if not self.is_running:
            raise RuntimeError("ROS Bridge not initialized")
            
        self.nodes[name] = node
        self.executor.add_node(node)
        logger.info(f"Added node: {name}")
    
    def remove_node(self, name: str):
        """Remove a ROS node from the bridge."""
        if name in self.nodes:
            node = self.nodes[name]
            self.executor.remove_node(node)
            node.destroy_node()
            del self.nodes[name]
            logger.info(f"Removed node: {name}")
    
    def get_node(self, name: str) -> Optional[Node]:
        """Get a ROS node by name."""
        return self.nodes.get(name)
    
    def shutdown(self):
        """Shutdown the ROS bridge and cleanup resources."""
        self.is_running = False
        
        # Remove all nodes
        for name in list(self.nodes.keys()):
            self.remove_node(name)
        
        # Shutdown executor
        if self.executor:
            self.executor.shutdown()
        
        # Shutdown ROS
        if rclpy.ok():
            rclpy.shutdown()
        
        logger.info("ROS Bridge shutdown")


class DataCollector:
    """
    Utility class for collecting and caching ROS data for web services.
    """
    
    def __init__(self):
        self.data_cache = {}
        self.callbacks = {}
        self.lock = threading.Lock()
    
    def update_data(self, key: str, data: Any):
        """Update cached data and trigger callbacks."""
        with self.lock:
            self.data_cache[key] = {
                'data': data,
                'timestamp': asyncio.get_event_loop().time()
            }
        
        # Trigger callbacks
        if key in self.callbacks:
            for callback in self.callbacks[key]:
                try:
                    callback(data)
                except Exception as e:
                    logger.error(f"Callback error for {key}: {e}")
    
    def get_data(self, key: str) -> Optional[Dict[str, Any]]:
        """Get cached data by key."""
        with self.lock:
            return self.data_cache.get(key)
    
    def get_all_data(self) -> Dict[str, Any]:
        """Get all cached data."""
        with self.lock:
            return dict(self.data_cache)
    
    def register_callback(self, key: str, callback: Callable):
        """Register a callback for data updates."""
        if key not in self.callbacks:
            self.callbacks[key] = []
        self.callbacks[key].append(callback)
    
    def clear_data(self, key: str = None):
        """Clear cached data."""
        with self.lock:
            if key:
                self.data_cache.pop(key, None)
            else:
                self.data_cache.clear()


def serialize_ros_message(msg) -> Dict[str, Any]:
    """
    Serialize a ROS message to a dictionary for JSON transmission.
    """
    result = {}
    
    # Get all slots (fields) of the message
    if hasattr(msg, '__slots__'):
        for slot in msg.__slots__:
            if hasattr(msg, slot):
                value = getattr(msg, slot)
                
                # Handle different data types
                if hasattr(value, '__slots__'):
                    # Nested message
                    result[slot] = serialize_ros_message(value)
                elif isinstance(value, (list, tuple)):
                    # Array/list
                    result[slot] = [serialize_ros_message(item) if hasattr(item, '__slots__') else item for item in value]
                else:
                    # Primitive type
                    result[slot] = value
    
    return result


def create_motor_diagnostic_dict(msg) -> Dict[str, Any]:
    """
    Convert DriveMotorDiagnostic message to a structured dictionary.
    """
    return {
        'motors': {
            'RF': {
                'voltage': float(msg.rf_voltage),
                'current': float(msg.rf_current),
                'state': int(msg.rf_state),
                'temperature': float(msg.rf_temperature)
            },
            'RB': {
                'voltage': float(msg.rb_voltage),
                'current': float(msg.rb_current),
                'state': int(msg.rb_state),
                'temperature': float(msg.rb_temperature)
            },
            'LB': {
                'voltage': float(msg.lb_voltage),
                'current': float(msg.lb_current),
                'state': int(msg.lb_state),
                'temperature': float(msg.lb_temperature)
            },
            'LF': {
                'voltage': float(msg.lf_voltage),
                'current': float(msg.lf_current),
                'state': int(msg.lf_state),
                'temperature': float(msg.lf_temperature)
            }
        },
        'timestamp': asyncio.get_event_loop().time()
    }


def create_motor_status_dict(response) -> Dict[str, Any]:
    """
    Convert DriveMotorStatus service response to a dictionary.
    """
    return {
        'status': {
            'RF': bool(response.rf_ok),
            'RB': bool(response.rb_ok),
            'LB': bool(response.lb_ok),
            'LF': bool(response.lf_ok)
        },
        'timestamp': asyncio.get_event_loop().time()
    }
