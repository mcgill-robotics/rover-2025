"""
ROS Manager - Main orchestrator for ROS services and web API integration.
"""

import asyncio
import json
import logging
import signal
import sys
import os
from typing import Dict, Any, Optional
from aiohttp import web, WSMsgType
import aiohttp_cors
import threading
import time

# Add local modules to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'utils'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'drive', 'subscriber'))

from ros_bridge import ROSBridge, DataCollector
from drive_data_subscriber import create_drive_subscriber

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class ROSManager:
    """
    Main manager class that orchestrates ROS nodes and provides web API endpoints.
    """
    
    def __init__(self, port: int = 8082):
        self.port = port
        self.ros_bridge = ROSBridge()
        self.data_collector = DataCollector()
        self.app = None
        self.websocket_connections = set()
        self.is_running = False
        
        # ROS nodes
        self.drive_subscriber = None
        
    async def initialize(self):
        """Initialize the ROS manager and all components."""
        try:
            # Initialize ROS bridge
            self.ros_bridge.initialize()
            
            # Create and add ROS nodes
            self.drive_subscriber = create_drive_subscriber(self.data_collector)
            self.ros_bridge.add_node('drive_subscriber', self.drive_subscriber)
            
            # Setup web application
            self.setup_web_app()
            
            # Register data callbacks for WebSocket broadcasting
            self.data_collector.register_callback('drive_diagnostics', self.broadcast_data)
            self.data_collector.register_callback('drive_speeds', self.broadcast_data)
            self.data_collector.register_callback('drive_status', self.broadcast_data)
            
            self.is_running = True
            logger.info("ROS Manager initialized successfully")
            
        except Exception as e:
            logger.error(f"Failed to initialize ROS Manager: {e}")
            raise
    
    def setup_web_app(self):
        """Setup the web application with routes and middleware."""
        self.app = web.Application()
        
        # Setup CORS
        cors = aiohttp_cors.setup(self.app, defaults={
            "*": aiohttp_cors.ResourceOptions(
                allow_credentials=True,
                expose_headers="*",
                allow_headers="*",
                allow_methods="*"
            )
        })
        
        # Add routes
        self.app.router.add_get('/api/drive/diagnostics', self.get_drive_diagnostics)
        self.app.router.add_get('/api/drive/speeds', self.get_drive_speeds)
        self.app.router.add_get('/api/drive/status', self.get_drive_status)
        self.app.router.add_get('/api/drive/summary', self.get_drive_summary)
        self.app.router.add_get('/api/health', self.health_check)
        self.app.router.add_get('/ws', self.websocket_handler)
        
        # Add CORS to all routes
        for route in list(self.app.router.routes()):
            cors.add(route)
    
    async def get_drive_diagnostics(self, request):
        """Get current drive motor diagnostics."""
        try:
            data = self.data_collector.get_data('drive_diagnostics')
            if data:
                return web.json_response({
                    'success': True,
                    'data': data['data'],
                    'timestamp': data['timestamp']
                })
            else:
                return web.json_response({
                    'success': False,
                    'error': 'No diagnostic data available'
                }, status=404)
        except Exception as e:
            logger.error(f"Error getting drive diagnostics: {e}")
            return web.json_response({
                'success': False,
                'error': str(e)
            }, status=500)
    
    async def get_drive_speeds(self, request):
        """Get current drive motor speeds."""
        try:
            data = self.data_collector.get_data('drive_speeds')
            if data:
                return web.json_response({
                    'success': True,
                    'data': data['data'],
                    'timestamp': data['timestamp']
                })
            else:
                return web.json_response({
                    'success': False,
                    'error': 'No speed data available'
                }, status=404)
        except Exception as e:
            logger.error(f"Error getting drive speeds: {e}")
            return web.json_response({
                'success': False,
                'error': str(e)
            }, status=500)
    
    async def get_drive_status(self, request):
        """Get current drive motor status."""
        try:
            data = self.data_collector.get_data('drive_status')
            if data:
                return web.json_response({
                    'success': True,
                    'data': data['data'],
                    'timestamp': data['timestamp']
                })
            else:
                return web.json_response({
                    'success': False,
                    'error': 'No status data available'
                }, status=404)
        except Exception as e:
            logger.error(f"Error getting drive status: {e}")
            return web.json_response({
                'success': False,
                'error': str(e)
            }, status=500)
    
    async def get_drive_summary(self, request):
        """Get a summary of all drive data and connection status."""
        try:
            if self.drive_subscriber:
                summary = self.drive_subscriber.get_summary_data()
                return web.json_response({
                    'success': True,
                    'data': summary
                })
            else:
                return web.json_response({
                    'success': False,
                    'error': 'Drive subscriber not available'
                }, status=503)
        except Exception as e:
            logger.error(f"Error getting drive summary: {e}")
            return web.json_response({
                'success': False,
                'error': str(e)
            }, status=500)
    
    async def health_check(self, request):
        """Health check endpoint."""
        try:
            health_data = {
                'status': 'healthy' if self.is_running else 'unhealthy',
                'ros_bridge_running': self.ros_bridge.is_running,
                'active_nodes': list(self.ros_bridge.nodes.keys()),
                'websocket_connections': len(self.websocket_connections),
                'timestamp': time.time()
            }
            
            return web.json_response({
                'success': True,
                'data': health_data
            })
        except Exception as e:
            logger.error(f"Error in health check: {e}")
            return web.json_response({
                'success': False,
                'error': str(e)
            }, status=500)
    
    async def websocket_handler(self, request):
        """WebSocket handler for real-time data streaming."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self.websocket_connections.add(ws)
        logger.info(f"WebSocket connection established. Total connections: {len(self.websocket_connections)}")
        
        try:
            # Send initial data
            await self.send_initial_data(ws)
            
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        await self.handle_websocket_message(ws, data)
                    except json.JSONDecodeError:
                        await ws.send_str(json.dumps({
                            'error': 'Invalid JSON format'
                        }))
                elif msg.type == WSMsgType.ERROR:
                    logger.error(f'WebSocket error: {ws.exception()}')
                    break
        except Exception as e:
            logger.error(f"WebSocket error: {e}")
        finally:
            self.websocket_connections.discard(ws)
            logger.info(f"WebSocket connection closed. Total connections: {len(self.websocket_connections)}")
        
        return ws
    
    async def send_initial_data(self, ws):
        """Send initial data to newly connected WebSocket."""
        try:
            if self.drive_subscriber:
                summary = self.drive_subscriber.get_summary_data()
                await ws.send_str(json.dumps({
                    'type': 'initial_data',
                    'data': summary
                }))
        except Exception as e:
            logger.error(f"Error sending initial data: {e}")
    
    async def handle_websocket_message(self, ws, data):
        """Handle incoming WebSocket messages."""
        try:
            message_type = data.get('type')
            
            if message_type == 'ping':
                await ws.send_str(json.dumps({
                    'type': 'pong',
                    'timestamp': time.time()
                }))
            elif message_type == 'subscribe':
                # Handle subscription requests
                topics = data.get('topics', [])
                await ws.send_str(json.dumps({
                    'type': 'subscription_ack',
                    'topics': topics
                }))
            else:
                await ws.send_str(json.dumps({
                    'error': f'Unknown message type: {message_type}'
                }))
                
        except Exception as e:
            logger.error(f"Error handling WebSocket message: {e}")
            await ws.send_str(json.dumps({
                'error': 'Internal server error'
            }))
    
    def broadcast_data(self, data):
        """Broadcast data to all connected WebSocket clients."""
        if not self.websocket_connections:
            return
        
        message = json.dumps({
            'type': 'data_update',
            'data': data,
            'timestamp': time.time()
        })
        
        # Remove closed connections and send to active ones
        closed_connections = set()
        for ws in self.websocket_connections:
            try:
                if ws.closed:
                    closed_connections.add(ws)
                else:
                    asyncio.create_task(ws.send_str(message))
            except Exception as e:
                logger.error(f"Error broadcasting to WebSocket: {e}")
                closed_connections.add(ws)
        
        # Clean up closed connections
        self.websocket_connections -= closed_connections
    
    async def run(self):
        """Run the web server."""
        try:
            runner = web.AppRunner(self.app)
            await runner.setup()
            
            site = web.TCPSite(runner, '0.0.0.0', self.port)
            await site.start()
            
            logger.info(f"ROS Manager web server started on port {self.port}")
            
            # Keep the server running
            while self.is_running:
                await asyncio.sleep(1)
                
        except Exception as e:
            logger.error(f"Error running web server: {e}")
            raise
    
    def shutdown(self):
        """Shutdown the ROS manager and cleanup resources."""
        logger.info("Shutting down ROS Manager...")
        
        self.is_running = False
        
        # Close WebSocket connections
        for ws in self.websocket_connections:
            asyncio.create_task(ws.close())
        self.websocket_connections.clear()
        
        # Shutdown ROS bridge
        self.ros_bridge.shutdown()
        
        logger.info("ROS Manager shutdown complete")


async def main():
    """Main entry point."""
    # Setup signal handlers
    manager = ROSManager()
    
    def signal_handler(signum, frame):
        logger.info(f"Received signal {signum}")
        manager.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Initialize and run
        await manager.initialize()
        await manager.run()
        
    except KeyboardInterrupt:
        logger.info("Received keyboard interrupt")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
    finally:
        manager.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
