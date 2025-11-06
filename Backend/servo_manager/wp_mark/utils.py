"""
WP_MARK Utility Functions Module
GPS calculations, logging setup, and configuration management
"""

import math
import json
import logging
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional


# Constants
EARTH_RADIUS_METERS = 6371000  # Earth radius in meters


def calculate_gps_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate distance between two GPS coordinates using Haversine formula
    
    Args:
        lat1: Current latitude (degrees)
        lon1: Current longitude (degrees)
        lat2: Target latitude (degrees)
        lon2: Target longitude (degrees)
    
    Returns:
        Distance in meters
    """
    # Convert to radians
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    # Haversine formula
    a = (math.sin(delta_phi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) *
         math.sin(delta_lambda / 2) ** 2)
    
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    distance = EARTH_RADIUS_METERS * c
    return distance


def setup_logging(log_dir: str = "/home/flash/NRP_ROS/Backend/logs") -> logging.Logger:
    """
    Configure logging for WP_MARK mission
    
    Args:
        log_dir: Directory to store log files
        
    Returns:
        Configured logger instance
    """
    # Create log directory if it doesn't exist
    Path(log_dir).mkdir(parents=True, exist_ok=True)
    
    # Generate timestamped log filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_dir, f"wp_mark_{timestamp}.log")
    
    # Configure logging
    logger = logging.getLogger('WPMarkMission')
    logger.setLevel(logging.INFO)
    
    # Remove existing handlers to avoid duplicates
    logger.handlers.clear()
    
    # File handler
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.INFO)
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    
    # Formatter
    formatter = logging.Formatter(
        '%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    logger.info(f"Logging initialized - Log file: {log_file}")
    
    return logger


class ConfigManager:
    """Manages WP_MARK configuration storage and retrieval"""
    
    def __init__(self, config_dir: str = "/home/flash/NRP_ROS/Backend/config"):
        """
        Initialize configuration manager
        
        Args:
            config_dir: Directory to store configuration files
        """
        self.config_dir = Path(config_dir)
        self.config_dir.mkdir(parents=True, exist_ok=True)
        
        self.config_file = self.config_dir / "wp_mark_config.json"
        self.state_file = self.config_dir / "wp_mark_state.json"
        self.log_file = self.config_dir.parent / "logs" / "mission_log.json"
        
        # Ensure log directory exists
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
    
    def save_config(self, config: Dict[str, Any]) -> bool:
        """
        Save configuration to JSON file
        
        Args:
            config: Configuration dictionary
            
        Returns:
            True if successful
        """
        try:
            full_config = {
                "wp_mark": {
                    "enabled": True,
                    "version": "1.0.0",
                    "parameters": config,
                    "settings": {
                        "servo_channel": 10,
                        "waypoint_arrival_threshold": 2.0,
                        "waypoint_timeout": 300.0,
                        "gps_fix_required": 3,
                        "log_enabled": True
                    },
                    "last_updated": datetime.now().isoformat()
                }
            }
            
            with open(self.config_file, 'w') as f:
                json.dump(full_config, f, indent=2)
            
            return True
            
        except Exception as e:
            logging.error(f"Failed to save config: {str(e)}")
            return False
    
    def load_config(self) -> Optional[Dict[str, Any]]:
        """
        Load configuration from JSON file
        
        Returns:
            Configuration dictionary or None if not found
        """
        try:
            if not self.config_file.exists():
                return None
            
            with open(self.config_file, 'r') as f:
                full_config = json.load(f)
            
            return full_config.get('wp_mark', {}).get('parameters')
            
        except Exception as e:
            logging.error(f"Failed to load config: {str(e)}")
            return None
    
    def save_mission_state(self, state: Dict[str, Any]) -> bool:
        """
        Save current mission state for recovery
        
        Args:
            state: Mission state dictionary
            
        Returns:
            True if successful
        """
        try:
            state['timestamp'] = datetime.now().isoformat()
            
            with open(self.state_file, 'w') as f:
                json.dump(state, f, indent=2)
            
            return True
            
        except Exception as e:
            logging.error(f"Failed to save mission state: {str(e)}")
            return False
    
    def load_mission_state(self) -> Optional[Dict[str, Any]]:
        """
        Load saved mission state
        
        Returns:
            Mission state dictionary or None if not found
        """
        try:
            if not self.state_file.exists():
                return None
            
            with open(self.state_file, 'r') as f:
                return json.load(f)
            
        except Exception as e:
            logging.error(f"Failed to load mission state: {str(e)}")
            return None
    
    def clear_mission_state(self) -> bool:
        """
        Clear saved mission state
        
        Returns:
            True if successful
        """
        try:
            if self.state_file.exists():
                self.state_file.unlink()
            return True
            
        except Exception as e:
            logging.error(f"Failed to clear mission state: {str(e)}")
            return False
    
    def log_mission_event(self, event_type: str, details: Dict[str, Any],
                         current_gps: Optional[Dict[str, float]] = None,
                         waypoint_index: Optional[int] = None) -> bool:
        """
        Log mission event to JSON log file
        
        Args:
            event_type: Type of event (e.g., 'WAYPOINT_REACHED', 'SERVO_ON')
            details: Event details dictionary
            current_gps: Current GPS coordinates (lat, lon, alt)
            waypoint_index: Current waypoint index
            
        Returns:
            True if successful
        """
        try:
            log_entry = {
                'timestamp': datetime.now().isoformat(),
                'event': event_type,
                'waypoint': waypoint_index,
                'gps': current_gps if current_gps else None,
                'details': details
            }
            
            # Append to log file
            with open(self.log_file, 'a') as f:
                f.write(json.dumps(log_entry) + '\n')
            
            return True
            
        except Exception as e:
            logging.error(f"Failed to log mission event: {str(e)}")
            return False
    
    def get_settings(self) -> Dict[str, Any]:
        """
        Get mission settings from config file
        
        Returns:
            Settings dictionary with defaults
        """
        try:
            if self.config_file.exists():
                with open(self.config_file, 'r') as f:
                    full_config = json.load(f)
                    return full_config.get('wp_mark', {}).get('settings', {})
        except Exception:
            pass
        
        # Return defaults if config doesn't exist or can't be read
        return {
            "servo_channel": 10,
            "waypoint_arrival_threshold": 2.0,
            "waypoint_timeout": 300.0,
            "gps_fix_required": 3,
            "log_enabled": True
        }


def format_gps_coordinates(lat: float, lon: float, alt: Optional[float] = None) -> str:
    """
    Format GPS coordinates for display
    
    Args:
        lat: Latitude in degrees
        lon: Longitude in degrees
        alt: Altitude in meters (optional)
        
    Returns:
        Formatted coordinate string
    """
    lat_dir = 'N' if lat >= 0 else 'S'
    lon_dir = 'E' if lon >= 0 else 'W'
    
    coord_str = f"{abs(lat):.6f}°{lat_dir}, {abs(lon):.6f}°{lon_dir}"
    
    if alt is not None:
        coord_str += f", Alt: {alt:.1f}m"
    
    return coord_str


def estimate_mission_duration(num_waypoints: int, config: Dict[str, Any],
                              avg_travel_time: float = 60.0) -> float:
    """
    Estimate total mission duration in minutes
    
    Args:
        num_waypoints: Number of waypoints in mission
        config: Mission configuration with delay parameters
        avg_travel_time: Average travel time between waypoints in seconds
        
    Returns:
        Estimated duration in minutes
    """
    # Time per waypoint (delays only)
    time_per_wp = (
        config.get('delay_before_start', 0) +
        config.get('delay_before_stop', 0) +
        config.get('delay_after_stop', 0)
    )
    
    # Total mission time
    total_seconds = num_waypoints * (time_per_wp + avg_travel_time)
    
    return total_seconds / 60.0  # Convert to minutes
