"""
WP_MARK Flask API Routes
REST API endpoints for mission control
"""

from flask import Blueprint, request, jsonify
from typing import Dict, Any
import logging

from .validators import validate_parameters, WPMarkConfig
from .mission_controller import get_mission_node, initialize_mission_node, shutdown_mission_node
from .utils import estimate_mission_duration

# Create Blueprint
wp_mark_bp = Blueprint('wp_mark', __name__, url_prefix='/wp_mark')

# Setup logging
logger = logging.getLogger('WPMarkAPI')


@wp_mark_bp.route('/start', methods=['POST'])
def start_mission() -> Dict[str, Any]:
    """
    Start WP_MARK mission with configuration
    
    Request Body:
        {
            "delay_before_start": float,
            "pwm_start": int,
            "delay_before_stop": float,
            "pwm_stop": int,
            "delay_after_stop": float,
            "servo_channel": int (optional, defaults to 10)
        }
    
    Returns:
        JSON response with mission info or error
    """
    try:
        # Get request data
        data = request.get_json()
        
        if not data:
            return jsonify({
                'success': False,
                'error': 'No data provided in request body'
            }), 400
        
        logger.info(f"Received start mission request: {data}")
        
        # Validate parameters
        is_valid, error_msg, config = validate_parameters(data)
        
        if not is_valid:
            logger.warning(f"Invalid parameters: {error_msg}")
            return jsonify({
                'success': False,
                'error': f'Invalid parameter: {error_msg}'
            }), 400
        
        # Check if mission already running
        node = get_mission_node()
        if node and node.mission_active:
            logger.warning("Attempted to start mission while already running")
            return jsonify({
                'success': False,
                'error': 'WP_MARK mission already running. Stop current mission first.'
            }), 409
        
        # Initialize mission node with new config
        logger.info("Initializing mission node with configuration")
        node = initialize_mission_node(config)
        
        # Start mission
        logger.info("Starting mission execution")
        result = node.start_mission()
        
        if not result['success']:
            return jsonify(result), 400
        
        # Add estimated duration
        if 'mission_info' in result and node.waypoints:
            result['mission_info']['estimated_duration_minutes'] = round(
                estimate_mission_duration(
                    len(node.waypoints),
                    config.to_dict()
                ), 1
            )
        
        logger.info("Mission started successfully")
        return jsonify(result), 200
        
    except Exception as e:
        logger.error(f"Error starting mission: {str(e)}", exc_info=True)
        return jsonify({
            'success': False,
            'error': f'Internal error: {str(e)}'
        }), 500


@wp_mark_bp.route('/stop', methods=['POST'])
def stop_mission() -> Dict[str, Any]:
    """
    Stop currently running WP_MARK mission
    
    Returns:
        JSON response with mission statistics or error
    """
    try:
        logger.info("Received stop mission request")
        
        # Get mission node
        node = get_mission_node()
        
        if not node:
            logger.warning("No mission node initialized")
            return jsonify({
                'success': False,
                'error': 'No mission currently running'
            }), 400
        
        # Stop mission
        result = node.stop_mission()
        
        if not result['success']:
            return jsonify(result), 400
        
        logger.info("Mission stopped successfully")
        return jsonify(result), 200
        
    except Exception as e:
        logger.error(f"Error stopping mission: {str(e)}", exc_info=True)
        return jsonify({
            'success': False,
            'error': f'Internal error: {str(e)}'
        }), 500


@wp_mark_bp.route('/status', methods=['GET'])
def get_status() -> Dict[str, Any]:
    """
    Get current WP_MARK mission status
    
    Returns:
        JSON response with current mission status
    """
    try:
        # Get mission node
        node = get_mission_node()
        
        if not node:
            # Return idle status if no node
            return jsonify({
                'running': False,
                'current_waypoint': 0,
                'total_waypoints': 0,
                'current_phase': 'idle',
                'config': {},
                'uptime_seconds': 0,
                'last_action': 'No mission running'
            }), 200
        
        # Get status from node
        status = node.get_status()
        
        return jsonify(status), 200
        
    except Exception as e:
        logger.error(f"Error getting status: {str(e)}", exc_info=True)
        return jsonify({
            'success': False,
            'error': f'Internal error: {str(e)}'
        }), 500


@wp_mark_bp.route('/config', methods=['GET'])
def get_config() -> Dict[str, Any]:
    """
    Get current mission configuration
    
    Returns:
        JSON response with saved configuration
    """
    try:
        from .utils import ConfigManager
        
        config_manager = ConfigManager()
        config = config_manager.load_config()
        
        if not config:
            return jsonify({
                'success': False,
                'error': 'No configuration found'
            }), 404
        
        return jsonify({
            'success': True,
            'config': config
        }), 200
        
    except Exception as e:
        logger.error(f"Error getting config: {str(e)}", exc_info=True)
        return jsonify({
            'success': False,
            'error': f'Internal error: {str(e)}'
        }), 500


@wp_mark_bp.route('/health', methods=['GET'])
def health_check() -> Dict[str, Any]:
    """
    Health check endpoint
    
    Returns:
        JSON response with system health
    """
    try:
        import rclpy
        
        node = get_mission_node()
        
        health = {
            'success': True,
            'ros2_initialized': rclpy.ok() if rclpy else False,
            'mission_node_active': node is not None,
            'mission_running': node.mission_active if node else False,
            'timestamp': __import__('time').strftime('%Y-%m-%dT%H:%M:%SZ', __import__('time').gmtime())
        }
        
        return jsonify(health), 200
        
    except Exception as e:
        logger.error(f"Health check error: {str(e)}", exc_info=True)
        return jsonify({
            'success': False,
            'error': f'Health check failed: {str(e)}'
        }), 500


# Error handlers
@wp_mark_bp.errorhandler(404)
def not_found(error):
    """Handle 404 errors"""
    return jsonify({
        'success': False,
        'error': 'Endpoint not found'
    }), 404


@wp_mark_bp.errorhandler(405)
def method_not_allowed(error):
    """Handle 405 errors"""
    return jsonify({
        'success': False,
        'error': 'Method not allowed'
    }), 405


@wp_mark_bp.errorhandler(500)
def internal_error(error):
    """Handle 500 errors"""
    logger.error(f"Internal server error: {str(error)}", exc_info=True)
    return jsonify({
        'success': False,
        'error': 'Internal server error'
    }), 500


# Cleanup on shutdown
def cleanup_wp_mark():
    """Cleanup function to be called on application shutdown"""
    logger.info("Cleaning up WP_MARK mission node...")
    try:
        shutdown_mission_node()
        logger.info("WP_MARK cleanup complete")
    except Exception as e:
        logger.error(f"Error during cleanup: {str(e)}")


# Export blueprint and cleanup function
__all__ = ['wp_mark_bp', 'cleanup_wp_mark']
