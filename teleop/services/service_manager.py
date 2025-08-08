#!/usr/bin/env python3

"""
Unified Service Manager for Rover 2025
Manages all ROS services, GPS service, and web APIs in a single process.
"""

import asyncio
import logging
import signal
import sys
import os
import threading
import time
from typing import Dict, Any, Optional
from dataclasses import dataclass
from contextlib import asynccontextmanager

import yaml

# Add local modules to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'ros'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'gps'))

from ros.ros_manager import ROSManager
from gps.gps_api import app as gps_app, start_gps_service

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class ServiceConfig:
    """Configuration for each service."""
    name: str
    port: int
    enabled: bool = True
    health_check_url: Optional[str] = None


class ServiceManager:
    """
    Unified manager for all rover services.
    """
    
    def __init__(self, config: Dict[str, Any] = None):
        self.config = config or self._get_default_config()
        self.services = {}
        self.is_running = False
        self.shutdown_event = threading.Event()
        
        # Service instances
        self.ros_manager = None
        self.gps_service = None
        self.tileserver_process = None
        self.camera_service = None
        
        # Health monitoring
        self.health_status = {}
        
    def _get_default_config(self) -> Dict[str, Any]:
        """Get default service configuration."""
        return {
            'services': {
                'ros_manager': {
                    'port': 8082,
                    'enabled': True,
                    'health_check_url': '/api/health'
                },
                'gps_service': {
                    'port': 5001,
                    'enabled': True,
                    'health_check_url': '/api/health'
                }
            },
            'monitoring': {
                'health_check_interval': 30,  # seconds
                'restart_failed_services': True,
                'max_restart_attempts': 3
            },
            'logging': {
                'level': 'INFO',
                'file': 'service_manager.log'
            }
        }
    
    async def initialize(self):
        """Initialize all services."""
        logger.info("Initializing Service Manager...")
        
        try:
            # Initialize ROS Manager
            if self.config['services']['ros_manager']['enabled']:
                logger.info("Initializing ROS Manager...")
                self.ros_manager = ROSManager(
                    port=self.config['services']['ros_manager']['port']
                )
                await self.ros_manager.initialize()
                self.services['ros_manager'] = self.ros_manager
                logger.info("ROS Manager initialized successfully")
            
            # Initialize GPS Service
            if self.config['services']['gps_service']['enabled']:
                logger.info("Initializing GPS Service...")
                # Start GPS service in background thread
                self.gps_service = threading.Thread(
                    target=self._run_gps_service,
                    daemon=True
                )
                self.gps_service.start()
                self.services['gps_service'] = self.gps_service
                logger.info("GPS Service started successfully")
            
            # Initialize TileServer
            if self.config['services'].get('tileserver', {}).get('enabled', False):
                logger.info("Initializing TileServer...")
                await self._start_tileserver()
            
            # Initialize Camera Service
            if self.config['services'].get('camera_service', {}).get('enabled', False):
                logger.info("Initializing Camera Service...")
                await self._start_camera_service()
            
            # Start health monitoring
            asyncio.create_task(self._health_monitor())
            
            self.is_running = True
            logger.info("Service Manager initialized successfully")
            
        except Exception as e:
            logger.error(f"Failed to initialize Service Manager: {e}")
            raise
    
    def _run_gps_service(self):
        """Run GPS service in a separate thread."""
        try:
            # Start GPS service
            start_gps_service()
            
            # Run Flask app
            gps_app.run(
                host='0.0.0.0',
                port=self.config['services']['gps_service']['port'],
                debug=False,
                use_reloader=False
            )
        except Exception as e:
            logger.error(f"GPS Service error: {e}")
    
    async def _start_tileserver(self):
        """Start TileServer using docker-compose."""
        try:
            import subprocess
            
            tileserver_config = self.config['services']['tileserver']
            docker_compose_file = tileserver_config.get('docker_compose_file', 'gps/docker-compose.tileserver.yml')
            
            # Start TileServer using docker-compose
            self.tileserver_process = subprocess.Popen(
                ['docker-compose', '-f', docker_compose_file, 'up', '-d'],
                cwd=os.path.dirname(__file__),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # Wait a moment for startup
            await asyncio.sleep(5)
            
            self.services['tileserver'] = self.tileserver_process
            logger.info("TileServer started successfully")
            
        except Exception as e:
            logger.error(f"Failed to start TileServer: {e}")
    
    async def _stop_tileserver(self):
        """Stop TileServer."""
        try:
            if self.tileserver_process:
                import subprocess
                
                tileserver_config = self.config['services']['tileserver']
                docker_compose_file = tileserver_config.get('docker_compose_file', 'gps/docker-compose.tileserver.yml')
                
                # Stop TileServer
                subprocess.run(
                    ['docker-compose', '-f', docker_compose_file, 'down'],
                    cwd=os.path.dirname(__file__),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                
                logger.info("TileServer stopped successfully")
                
        except Exception as e:
            logger.error(f"Failed to stop TileServer: {e}")
    
    async def _start_camera_service(self):
        """Start the camera service."""
        try:
            from camera.camera_service import MultiCameraBackend
            
            # Get camera service config from service_config.yml
            camera_config = self.config['services']['camera_service']['config']
            http_port = self.config['services']['camera_service']['port']
            
            # Create and start the camera backend
            self.camera_service = MultiCameraBackend(http_port=http_port, enable_aruco=True)
            
            # Start the camera service in a separate thread
            def run_camera_service():
                self.camera_service.run()
            
            camera_thread = threading.Thread(target=run_camera_service, daemon=True)
            camera_thread.start()
            
            logger.info("Camera Service started successfully")
            
        except Exception as e:
            logger.error(f"Failed to start Camera Service: {e}")
            raise
    
    async def _stop_camera_service(self):
        """Stop the camera service."""
        if self.camera_service:
            try:
                self.camera_service.running = False
                self.camera_service = None
                logger.info("Camera Service stopped successfully")
            except Exception as e:
                logger.error(f"Error stopping Camera Service: {e}")
                raise
    
    async def _health_monitor(self):
        """Monitor health of all services."""
        while self.is_running and not self.shutdown_event.is_set():
            try:
                await self._check_service_health()
                await asyncio.sleep(self.config['monitoring']['health_check_interval'])
            except Exception as e:
                logger.error(f"Health monitoring error: {e}")
    
    async def _check_service_health(self):
        """Check health of all services."""
        import aiohttp
        
        for service_name, service_config in self.config['services'].items():
            if not service_config['enabled']:
                continue
                
            health_url = service_config['health_check_url']
            if not health_url:
                continue
            
            try:
                async with aiohttp.ClientSession() as session:
                    url = f"http://localhost:{service_config['port']}{health_url}"
                    async with session.get(url, timeout=5) as response:
                        if response.status == 200:
                            self.health_status[service_name] = {
                                'status': 'healthy',
                                'last_check': time.time(),
                                'response_time': response.headers.get('X-Response-Time', 'N/A')
                            }
                        else:
                            self.health_status[service_name] = {
                                'status': 'unhealthy',
                                'last_check': time.time(),
                                'error': f"HTTP {response.status}"
                            }
            except Exception as e:
                self.health_status[service_name] = {
                    'status': 'error',
                    'last_check': time.time(),
                    'error': str(e)
                }
                logger.warning(f"Health check failed for {service_name}: {e}")
    
    async def get_service_status(self) -> Dict[str, Any]:
        """Get status of all services."""
        return {
            'manager_status': 'running' if self.is_running else 'stopped',
            'services': self.health_status,
            'uptime': time.time() - getattr(self, '_start_time', time.time()),
            'config': self.config
        }
    
    async def restart_service(self, service_name: str) -> bool:
        """Restart a specific service."""
        if service_name not in self.services:
            logger.error(f"Service {service_name} not found")
            return False
        
        try:
            logger.info(f"Restarting service: {service_name}")
            
            if service_name == 'ros_manager':
                # Restart ROS Manager
                await self.ros_manager.shutdown()
                self.ros_manager = ROSManager(
                    port=self.config['services']['ros_manager']['port']
                )
                await self.ros_manager.initialize()
                self.services['ros_manager'] = self.ros_manager
                
            elif service_name == 'gps_service':
                # Restart GPS Service
                # Note: GPS service runs in a thread, so we need to handle this differently
                logger.warning("GPS service restart not implemented (runs in thread)")
                return False
                
            elif service_name == 'tileserver':
                # Restart TileServer
                await self._stop_tileserver()
                await asyncio.sleep(2)  # Wait for cleanup
                await self._start_tileserver()
                return True
                
            elif service_name == 'camera_service':
                # Restart Camera Service
                await self._stop_camera_service()
                await asyncio.sleep(2)  # Wait for cleanup
                await self._start_camera_service()
                return True
            
            logger.info(f"Service {service_name} restarted successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to restart service {service_name}: {e}")
            return False
    
    async def shutdown(self):
        """Shutdown all services gracefully."""
        logger.info("Shutting down Service Manager...")
        
        self.is_running = False
        self.shutdown_event.set()
        
        # Shutdown ROS Manager
        if self.ros_manager:
            try:
                await self.ros_manager.shutdown()
                logger.info("ROS Manager shut down successfully")
            except Exception as e:
                logger.error(f"Error shutting down ROS Manager: {e}")
        
        # Shutdown TileServer
        await self._stop_tileserver()
        
        # Shutdown Camera Service
        if self.camera_service:
            try:
                await self._stop_camera_service()
                logger.info("Camera Service shut down successfully")
            except Exception as e:
                logger.error(f"Error shutting down Camera Service: {e}")
        
        # Note: GPS service runs in a thread and will be terminated when main process exits
        logger.info("Service Manager shut down successfully")


class ServiceManagerAPI:
    """Web API for managing services."""
    
    def __init__(self, service_manager: ServiceManager, port: int = 8083):
        self.service_manager = service_manager
        self.port = port
        self.app = None
    
    def setup_app(self):
        """Setup the web application."""
        from aiohttp import web
        import aiohttp_cors
        
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
        self.app.router.add_get('/api/status', self.get_status)
        self.app.router.add_post('/api/restart/{service}', self.restart_service)
        self.app.router.add_get('/api/config', self.get_config)
        self.app.router.add_get('/api/health', self.health_check)
        
        # Apply CORS to all routes
        for route in list(self.app.router.routes()):
            cors.add(route)
    
    async def get_status(self, request):
        """Get status of all services."""
        status = await self.service_manager.get_service_status()
        return web.json_response(status)
    
    async def restart_service(self, request):
        """Restart a specific service."""
        service_name = request.match_info['service']
        success = await self.service_manager.restart_service(service_name)
        
        return web.json_response({
            'success': success,
            'service': service_name,
            'message': f"Service {service_name} {'restarted' if success else 'failed to restart'}"
        })
    
    async def get_config(self, request):
        """Get current configuration."""
        return web.json_response(self.service_manager.config)
    
    async def health_check(self, request):
        """Health check endpoint."""
        return web.json_response({
            'status': 'healthy',
            'timestamp': time.time(),
            'manager_running': self.service_manager.is_running
        })
    
    async def run(self):
        """Run the API server."""
        self.setup_app()
        runner = web.AppRunner(self.app)
        await runner.setup()
        site = web.TCPSite(runner, '0.0.0.0', self.port)
        await site.start()
        
        logger.info(f"Service Manager API running on http://localhost:{self.port}")
        
        try:
            await asyncio.Future()  # Run forever
        finally:
            await runner.cleanup()


async def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Rover Service Manager')
    parser.add_argument('--port', type=int, default=8083, help='API port')
    parser.add_argument('--config', type=str, help='Configuration file path')
    args = parser.parse_args()
    
    # Load configuration
    config = {}
    if args.config:
        config_file = args.config
    else:
        config_file = os.path.join(os.path.dirname(__file__), 'service_config.yml')
    
    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            logger.info(f"Loaded configuration from {config_file}")
        except Exception as e:
            logger.error(f"Failed to load config file {config_file}: {e}")
            return
    else:
        logger.warning(f"Config file {config_file} not found, using default config")
    
    # Create service manager
    service_manager = ServiceManager(config)
    service_manager._start_time = time.time()
    
    # Create API server
    api_server = ServiceManagerAPI(service_manager, port=args.port)
    
    # Setup signal handlers
    def signal_handler(signum, frame):
        logger.info(f"Received signal {signum}, shutting down...")
        asyncio.create_task(service_manager.shutdown())
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Initialize services
        await service_manager.initialize()
        
        # Start API server
        await api_server.run()
        
    except Exception as e:
        logger.error(f"Service Manager error: {e}")
        await service_manager.shutdown()
        sys.exit(1)


if __name__ == '__main__':
    asyncio.run(main()) 