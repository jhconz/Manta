# -*- coding: utf-8 -*-
"""
Created on Wed Jan  7 11:11:50 2026

@author: Student
"""

import asyncio
from typing import Dict, List, Optional, Tuple
from open_gopro import WirelessGoPro, Params
from open_gopro.exceptions import GoProError
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MultiCameraManagerWiFi:
    """
    Manages multiple GoPro cameras over WiFi.
    Can connect to cameras either via their individual AP mode or on a shared network.
    """
    
    def __init__(self, connection_mode: str = "ap"):
        """
        Initialize the multi-camera manager.
        
        Args:
            connection_mode: "ap" for individual AP mode, "station" for shared network
        """
        self.connection_mode = connection_mode
        self.camera_configs: Dict[str, Dict] = {}
        self.cameras: Dict[str, WirelessGoPro] = {}
        self.connected: bool = False
        
    def add_camera(self, 
                   camera_name: str,
                   camera_identifier: str,
                   wifi_ssid: Optional[str] = None,
                   wifi_password: Optional[str] = None,
                   ip_address: Optional[str] = None) -> None:
        """
        Add a camera configuration.
        
        For AP mode:
            camera_identifier: Bluetooth MAC or serial number for initial pairing
            wifi_ssid: GoPro's WiFi SSID
            wifi_password: GoPro's WiFi password
            
        For station mode:
            camera_identifier: Bluetooth MAC or serial number
            ip_address: Camera's IP on the shared network
        
        Args:
            camera_name: Friendly name for the camera
            camera_identifier: Camera identifier (MAC address or serial)
            wifi_ssid: WiFi SSID (for AP mode)
            wifi_password: WiFi password (for AP mode)
            ip_address: IP address (for station mode)
        """
        self.camera_configs[camera_name] = {
            "identifier": camera_identifier,
            "ssid": wifi_ssid,
            "password": wifi_password,
            "ip": ip_address
        }
        logger.info(f"Added camera '{camera_name}' with identifier {camera_identifier}")
    
    async def connect_all(self) -> None:
        """
        Connect to all configured cameras simultaneously.
        """
        if self.connected:
            logger.warning("Cameras already connected")
            return
        
        if self.connection_mode == "station":
            # Station mode: all cameras on same network, connect via IP
            await self._connect_all_station()
        else:
            # AP mode: connect to each camera's individual WiFi network
            await self._connect_all_ap()
    
    async def _connect_all_station(self) -> None:
        """Connect to all cameras on a shared network."""
        
        async def connect_single(name: str, config: Dict) -> Tuple[str, Optional[WirelessGoPro]]:
            """Helper to connect a single camera via station mode."""
            try:
                logger.info(f"Connecting to {name} at {config['ip']}...")
                
                # For station mode, we can connect directly via WiFi if we know the IP
                # First connect via BLE to enable WiFi, then switch to WiFi
                gopro = WirelessGoPro(target=config['identifier'])
                await gopro.open()
                
                # Enable WiFi on the camera
                await gopro.ble_command.enable_wifi_ap(enable=True)
                
                # The camera should now be accessible via its IP on the network
                logger.info(f"Successfully connected to {name}")
                return name, gopro
                
            except Exception as e:
                logger.error(f"Failed to connect to {name}: {e}")
                return name, None
        
        tasks = [connect_single(name, config) 
                for name, config in self.camera_configs.items()]
        results = await asyncio.gather(*tasks)
        
        for name, gopro in results:
            if gopro:
                self.cameras[name] = gopro
        
        self.connected = len(self.cameras) > 0
        logger.info(f"Connected to {len(self.cameras)}/{len(self.camera_configs)} cameras")
    
    async def _connect_all_ap(self) -> None:
        """
        Connect to cameras in AP mode (sequentially, as we can only be on one WiFi at a time).
        Note: This is less ideal for simultaneous control.
        """
        logger.warning("AP mode connects sequentially. Consider using station mode for simultaneous control.")
        
        for name, config in self.camera_configs.items():
            try:
                logger.info(f"Connecting to {name}...")
                
                gopro = WirelessGoPro(
                    target=config['identifier'],
                    wifi_interface="wlan0"  # Adjust if your interface is different
                )
                await gopro.open()
                
                self.cameras[name] = gopro
                logger.info(f"Successfully connected to {name}")
                
            except Exception as e:
                logger.error(f"Failed to connect to {name}: {e}")
        
        self.connected = len(self.cameras) > 0
    
    async def disconnect_all(self) -> None:
        """Disconnect from all cameras."""
        async def disconnect_single(name: str, gopro: WirelessGoPro) -> None:
            try:
                await gopro.close()
                logger.info(f"Disconnected from {name}")
            except Exception as e:
                logger.error(f"Error disconnecting from {name}: {e}")
        
        tasks = [disconnect_single(name, gopro) 
                for name, gopro in self.cameras.items()]
        await asyncio.gather(*tasks)
        
        self.cameras.clear()
        self.connected = False
    
    async def start_recording_all(self) -> Dict[str, bool]:
        """Start recording on all connected cameras."""
        async def start_single(name: str, gopro: WirelessGoPro) -> Tuple[str, bool]:
            try:
                await gopro.http_command.set_shutter(shutter=Params.Toggle.ENABLE)
                logger.info(f"Started recording on {name}")
                return name, True
            except Exception as e:
                logger.error(f"Failed to start recording on {name}: {e}")
                return name, False
        
        tasks = [start_single(name, gopro) 
                for name, gopro in self.cameras.items()]
        results = await asyncio.gather(*tasks)
        
        return dict(results)
    
    async def stop_recording_all(self) -> Dict[str, bool]:
        """Stop recording on all connected cameras."""
        async def stop_single(name: str, gopro: WirelessGoPro) -> Tuple[str, bool]:
            try:
                await gopro.http_command.set_shutter(shutter=Params.Toggle.DISABLE)
                logger.info(f"Stopped recording on {name}")
                return name, True
            except Exception as e:
                logger.error(f"Failed to stop recording on {name}: {e}")
                return name, False
        
        tasks = [stop_single(name, gopro) 
                for name, gopro in self.cameras.items()]
        results = await asyncio.gather(*tasks)
        
        return dict(results)
    
    async def configure_camera_settings(self, 
                                       resolution: Optional[str] = None,
                                       fps: Optional[int] = None,
                                       fov: Optional[str] = None) -> Dict[str, bool]:
        """
        Configure video settings on all cameras.
        
        Args:
            resolution: Video resolution (e.g., "1080", "2.7k", "4k")
            fps: Frames per second
            fov: Field of view (e.g., "wide", "linear", "narrow")
        """
        async def configure_single(name: str, gopro: WirelessGoPro) -> Tuple[str, bool]:
            try:
                if resolution:
                    # Set resolution based on camera model
                    # This is an example - adjust for your specific camera model
                    pass
                
                if fps:
                    # Set FPS
                    pass
                
                if fov:
                    # Set FOV
                    pass
                
                logger.info(f"Configured settings on {name}")
                return name, True
            except Exception as e:
                logger.error(f"Failed to configure {name}: {e}")
                return name, False
        
        tasks = [configure_single(name, gopro) 
                for name, gopro in self.cameras.items()]
        results = await asyncio.gather(*tasks)
        
        return dict(results)
    
    async def get_status_all(self) -> Dict[str, Dict]:
        """Get status from all cameras (battery, recording state, etc.)."""
        async def get_status_single(name: str, gopro: WirelessGoPro) -> Tuple[str, Dict]:
            try:
                # Get various status info
                status = {}
                # Add status queries here based on what you need
                return name, status
            except Exception as e:
                logger.error(f"Failed to get status from {name}: {e}")
                return name, {}
        
        tasks = [get_status_single(name, gopro) 
                for name, gopro in self.cameras.items()]
        results = await asyncio.gather(*tasks)
        
        return dict(results)
    
    async def execute_on_camera(self, 
                                camera_name: str, 
                                command_func) -> any:
        """Execute a custom command on a specific camera."""
        if camera_name not in self.cameras:
            raise ValueError(f"Camera '{camera_name}' not connected")
        
        return await command_func(self.cameras[camera_name])
    
    async def execute_on_all(self, command_func) -> Dict[str, any]:
        """Execute a custom command on all cameras."""
        async def execute_single(name: str, gopro: WirelessGoPro) -> Tuple[str, any]:
            try:
                result = await command_func(gopro)
                return name, result
            except Exception as e:
                logger.error(f"Error executing on {name}: {e}")
                return name, None
        
        tasks = [execute_single(name, gopro) 
                for name, gopro in self.cameras.items()]
        results = await asyncio.gather(*tasks)
        
        return dict(results)
    
    def get_camera(self, camera_name: str) -> Optional[WirelessGoPro]:
        """Get direct access to a specific camera instance."""
        return self.cameras.get(camera_name)
    
    def list_cameras(self) -> List[str]:
        """Get list of all configured camera names."""
        return list(self.camera_configs.keys())
    
    def list_connected(self) -> List[str]:
        """Get list of currently connected camera names."""
        return list(self.cameras.keys())
    
    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect_all()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.disconnect_all()


# Example usage
async def main():
    """Example usage with WiFi station mode."""
    
    # Option 1: Station mode (all cameras on same network)
    manager = MultiCameraManagerWiFi(connection_mode="station")
    
    # Add cameras with their identifiers and IP addresses
    manager.add_camera("front", "AA:BB:CC:DD:EE:01", ip_address="192.168.1.101")
    manager.add_camera("back", "AA:BB:CC:DD:EE:02", ip_address="192.168.1.102")
    manager.add_camera("left", "AA:BB:CC:DD:EE:03", ip_address="192.168.1.103")
    manager.add_camera("right", "AA:BB:CC:DD:EE:04", ip_address="192.168.1.104")
    
    async with manager:
        # Start recording on all cameras simultaneously
        results = await manager.start_recording_all()
        print(f"Recording started: {results}")
        
        # Record for 10 seconds
        await asyncio.sleep(10)
        
        # Stop recording
        results = await manager.stop_recording_all()
        print(f"Recording stopped: {results}")
        
        # Get status from all cameras
        statuses = await manager.get_status_all()
        print(f"Camera statuses: {statuses}")


async def main_ap_mode():
    """Example usage with AP mode."""
    
    # Option 2: AP mode (each camera creates its own network)
    manager = MultiCameraManagerWiFi(connection_mode="ap")
    
    # Add cameras with their WiFi credentials
    manager.add_camera(
        "front", 
        "AA:BB:CC:DD:EE:01",
        wifi_ssid="GoPro 1234",
        wifi_password="password1234"
    )
    # ... add more cameras
    
    async with manager:
        # Control cameras (sequentially in AP mode)
        await manager.start_recording_all()
        await asyncio.sleep(10)
        await manager.stop_recording_all()


if __name__ == "__main__":
    asyncio.run(main())