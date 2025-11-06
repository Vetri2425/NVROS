#!/usr/bin/env python3
"""
WP_MARK Backend Installation Verification Script
Tests all components without requiring ROS2/MAVROS
"""

import sys
import os

# Add Backend to path
sys.path.insert(0, '/home/flash/NRP_ROS/Backend')

def test_imports():
    """Test that all modules can be imported"""
    print("Testing module imports...")
    
    try:
        from servo_manager.wp_mark import validators
        print("  ✅ validators module imported")
    except Exception as e:
        print(f"  ❌ validators import failed: {e}")
        return False
    
    try:
        from servo_manager.wp_mark import utils
        print("  ✅ utils module imported")
    except Exception as e:
        print(f"  ❌ utils import failed: {e}")
        return False
    
    try:
        from servo_manager.wp_mark import api_routes
        print("  ✅ api_routes module imported")
    except Exception as e:
        print(f"  ❌ api_routes import failed: {e}")
        return False
    
    # mission_controller requires ROS2, skip for now
    print("  ⚠️  mission_controller requires ROS2 (skipped)")
    
    return True


def test_validators():
    """Test parameter validation"""
    print("\nTesting parameter validation...")
    
    from servo_manager.wp_mark.validators import validate_parameters
    
    # Valid config
    valid_config = {
        'delay_before_start': 2.0,
        'pwm_start': 1500,
        'delay_before_stop': 5.0,
        'pwm_stop': 1000,
        'delay_after_stop': 1.0
    }
    
    is_valid, error, config = validate_parameters(valid_config)
    if is_valid:
        print("  ✅ Valid config accepted")
    else:
        print(f"  ❌ Valid config rejected: {error}")
        return False
    
    # Invalid delay
    invalid_config = valid_config.copy()
    invalid_config['delay_before_start'] = 100  # Too high
    is_valid, error, config = validate_parameters(invalid_config)
    if not is_valid:
        print("  ✅ Invalid delay rejected correctly")
    else:
        print("  ❌ Invalid delay accepted incorrectly")
        return False
    
    # Invalid PWM
    invalid_config = valid_config.copy()
    invalid_config['pwm_start'] = 3000  # Too high
    is_valid, error, config = validate_parameters(invalid_config)
    if not is_valid:
        print("  ✅ Invalid PWM rejected correctly")
    else:
        print("  ❌ Invalid PWM accepted incorrectly")
        return False
    
    # Same PWM values
    invalid_config = valid_config.copy()
    invalid_config['pwm_stop'] = 1500  # Same as pwm_start
    is_valid, error, config = validate_parameters(invalid_config)
    if not is_valid:
        print("  ✅ Same PWM values rejected correctly")
    else:
        print("  ❌ Same PWM values accepted incorrectly")
        return False
    
    return True


def test_gps_distance():
    """Test GPS distance calculation"""
    print("\nTesting GPS distance calculation...")
    
    from servo_manager.wp_mark.utils import calculate_gps_distance
    
    # Test 1: Same location (should be 0)
    dist = calculate_gps_distance(37.7749, -122.4194, 37.7749, -122.4194)
    if abs(dist) < 0.01:
        print(f"  ✅ Same location: {dist:.2f}m")
    else:
        print(f"  ❌ Same location should be 0m, got {dist:.2f}m")
        return False
    
    # Test 2: Known distance (approximately 111km per degree at equator)
    # 0.001 degrees latitude ≈ 111 meters
    dist = calculate_gps_distance(0.0, 0.0, 0.001, 0.0)
    if 100 < dist < 120:
        print(f"  ✅ 0.001° latitude: {dist:.2f}m (expected ~111m)")
    else:
        print(f"  ❌ 0.001° latitude: {dist:.2f}m (expected ~111m)")
        return False
    
    # Test 3: Waypoint threshold (2 meters)
    dist = calculate_gps_distance(37.7749, -122.4194, 37.77492, -122.41942)
    print(f"  ✅ Small distance: {dist:.2f}m")
    
    return True


def test_config_manager():
    """Test configuration manager"""
    print("\nTesting configuration manager...")
    
    from servo_manager.wp_mark.utils import ConfigManager
    import tempfile
    import shutil
    
    # Use temporary directory
    temp_dir = tempfile.mkdtemp()
    
    try:
        config_mgr = ConfigManager(temp_dir)
        
        # Test save config
        test_config = {
            'delay_before_start': 2.0,
            'pwm_start': 1500,
            'delay_before_stop': 5.0,
            'pwm_stop': 1000,
            'delay_after_stop': 1.0
        }
        
        if config_mgr.save_config(test_config):
            print("  ✅ Config saved successfully")
        else:
            print("  ❌ Config save failed")
            return False
        
        # Test load config
        loaded_config = config_mgr.load_config()
        if loaded_config == test_config:
            print("  ✅ Config loaded correctly")
        else:
            print(f"  ❌ Config mismatch: {loaded_config}")
            return False
        
        # Test mission state
        test_state = {
            'current_waypoint': 2,
            'phase': 'navigating'
        }
        
        if config_mgr.save_mission_state(test_state):
            print("  ✅ Mission state saved")
        else:
            print("  ❌ Mission state save failed")
            return False
        
        loaded_state = config_mgr.load_mission_state()
        if loaded_state and loaded_state['current_waypoint'] == 2:
            print("  ✅ Mission state loaded correctly")
        else:
            print("  ❌ Mission state load failed")
            return False
        
        # Test event logging
        if config_mgr.log_mission_event(
            'TEST_EVENT',
            {'test': 'data'},
            {'lat': 37.7749, 'lon': -122.4194, 'alt': 10.0},
            0
        ):
            print("  ✅ Event logged successfully")
        else:
            print("  ❌ Event logging failed")
            return False
        
    finally:
        # Cleanup
        shutil.rmtree(temp_dir)
    
    return True


def test_file_structure():
    """Test that all required files exist"""
    print("\nTesting file structure...")
    
    base_path = '/home/flash/NRP_ROS/Backend/servo_manager/wp_mark'
    
    required_files = [
        '__init__.py',
        'validators.py',
        'utils.py',
        'mission_controller.py',
        'api_routes.py'
    ]
    
    all_exist = True
    for filename in required_files:
        filepath = os.path.join(base_path, filename)
        if os.path.exists(filepath):
            print(f"  ✅ {filename} exists")
        else:
            print(f"  ❌ {filename} missing")
            all_exist = False
    
    # Check config directory
    config_path = '/home/flash/NRP_ROS/Backend/config'
    if os.path.exists(config_path):
        print(f"  ✅ config/ directory exists")
        
        config_file = os.path.join(config_path, 'wp_mark_config.json')
        if os.path.exists(config_file):
            print(f"  ✅ wp_mark_config.json exists")
        else:
            print(f"  ❌ wp_mark_config.json missing")
            all_exist = False
    else:
        print(f"  ❌ config/ directory missing")
        all_exist = False
    
    # Check logs directory
    logs_path = '/home/flash/NRP_ROS/Backend/logs'
    if os.path.exists(logs_path):
        print(f"  ✅ logs/ directory exists")
    else:
        print(f"  ❌ logs/ directory missing")
        all_exist = False
    
    return all_exist


def main():
    """Run all tests"""
    print("="*60)
    print("WP_MARK Backend Installation Verification")
    print("="*60)
    
    tests = [
        ("File Structure", test_file_structure),
        ("Module Imports", test_imports),
        ("Parameter Validation", test_validators),
        ("GPS Distance Calculation", test_gps_distance),
        ("Configuration Manager", test_config_manager),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"\n❌ {test_name} crashed: {e}")
            results.append((test_name, False))
    
    print("\n" + "="*60)
    print("Test Results Summary")
    print("="*60)
    
    all_passed = True
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status:10} {test_name}")
        if not result:
            all_passed = False
    
    print("="*60)
    
    if all_passed:
        print("\n🎉 All tests passed! WP_MARK backend is ready.")
        print("\nNext steps:")
        print("  1. Start the backend: cd Backend && python3 server.py")
        print("  2. Test health: curl http://localhost:5001/wp_mark/health")
        print("  3. Integrate with frontend")
        return 0
    else:
        print("\n⚠️  Some tests failed. Please review errors above.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
