"""
WP_MARK Mission Module
Industrial-grade waypoint-based servo control system for precision agricultural spraying
Enhanced with robust state management, retry mechanisms, and monitoring
"""

__version__ = "1.1.0"
__author__ = "NRP Team"

from .validators import validate_parameters, WPMarkConfig
from .utils import calculate_gps_distance, setup_logging, ConfigManager
from .state_machine import StateMachine, MissionPhase, StateTransitionError, StateValidationError
from .retry_mechanism import get_retry_mechanism, RetryError, RetryConfig
from .monitoring import performance_monitor, metrics_collector, health_monitor
from .mission_controller import WPMarkMissionNode, get_mission_node, initialize_mission_node, shutdown_mission_node
from .mission_controller import WPMarkMissionNode, MissionPhase

__all__ = [
    'validate_parameters',
    'WPMarkConfig',
    'calculate_gps_distance',
    'setup_logging',
    'ConfigManager',
    'WPMarkMissionNode',
    'MissionPhase'
]
