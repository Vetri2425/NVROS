#!/usr/bin/env python3
"""Script to update server.py with GPS/RTK telemetry support"""
import sys

def update_server_py(filepath):
    with open(filepath, 'r') as f:
        content = f.read()
    
    # 1. Update CurrentState dataclass to add satellites_visible
    old_currentstate = """    imu_status: str = 'UNALIGNED'
    activeWaypointIndex: Optional[int] = None"""
    
    new_currentstate = """    imu_status: str = 'UNALIGNED'
    satellites_visible: int = 0
    activeWaypointIndex: Optional[int] = None"""
    
    if old_currentstate in content:
        content = content.replace(old_currentstate, new_currentstate)
        print("✓ Updated CurrentState dataclass - added satellites_visible")
    else:
        print("⚠ CurrentState pattern not found - may already be updated")
    
    # 2. Update _merge_ros2_telemetry function
    old_merge_start = """def _merge_ros2_telemetry(payload: dict) -> None:
    \"\"\"Merge telemetry published via the ROS2 bridge into the shared rover state.\"\"\"
    if not payload or not isinstance(payload, dict):
        return
    global is_vehicle_connected
    changed = False
    now = time.time()
    try:
        state = payload.get('state')
        position = payload.get('position')"""
    
    new_merge_start = """def _merge_ros2_telemetry(payload: dict) -> None:
    \"\"\"Merge telemetry published via the ROS2 bridge into the shared rover state.\"\"\"
    if not payload or not isinstance(payload, dict):
        return
    global is_vehicle_connected
    changed = False
    now = time.time()
    try:
        state = payload.get('state')
        global_data = payload.get('global')
        position = payload.get('position')  # Legacy support
        rtk = payload.get('rtk')"""
    
    if old_merge_start in content:
        content = content.replace(old_merge_start, new_merge_start)
        print("✓ Updated _merge_ros2_telemetry - added global_data and rtk handling")
    else:
        print("⚠ _merge_ros2_telemetry start pattern not found")
    
    # 3. Update position handling in _merge_ros2_telemetry
    old_position_handling = """            if isinstance(position, dict):
                lat = position.get('latitude')
                lon = position.get('longitude')
                if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
                    current_state.position = Position(lat=float(lat), lng=float(lon))
                    changed = True
                    current_state.last_update = now"""
    
    new_position_handling = """            # Process global position data (NEW FORMAT)
            if isinstance(global_data, dict):
                lat = global_data.get('latitude')
                lon = global_data.get('longitude')
                alt = global_data.get('altitude')
                vel = global_data.get('vel')
                satellites = global_data.get('satellites_visible')
                
                if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
                    current_state.position = Position(lat=float(lat), lng=float(lon))
                    changed = True
                    current_state.last_update = now
                
                # Update satellites count
                if isinstance(satellites, (int, float)):
                    if int(satellites) != current_state.satellites_visible:
                        current_state.satellites_visible = int(satellites)
                        changed = True

            # Legacy position format support
            elif isinstance(position, dict):
                lat = position.get('latitude')
                lon = position.get('longitude')
                if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
                    current_state.position = Position(lat=float(lat), lng=float(lon))
                    changed = True
                    current_state.last_update = now

            # Process RTK status data (NEW)
            if isinstance(rtk, dict):
                fix_type = rtk.get('fix_type')
                if isinstance(fix_type, (int, float)):
                    fix_type_int = int(fix_type)
                    # Map fix type to RTK status string
                    rtk_status_map = {
                        0: 'No GPS',
                        1: 'GPS Fix',
                        2: 'DGPS',
                        3: 'RTK Float',
                        4: 'RTK Fixed'
                    }
                    new_rtk_status = rtk_status_map.get(fix_type_int, 'No GPS')
                    if new_rtk_status != current_state.rtk_status:
                        current_state.rtk_status = new_rtk_status
                        changed = True"""
    
    if old_position_handling in content:
        content = content.replace(old_position_handling, new_position_handling)
        print("✓ Updated position handling - added global/RTK processing")
    else:
        print("⚠ Position handling pattern not found - checking if already updated")
    
    # Write updated content
    with open(filepath, 'w') as f:
        f.write(content)
    
    print("\n✅ server.py updated successfully!")
    print("Backup saved as server.py.backup")

if __name__ == '__main__':
    if len(sys.argv) > 1:
        update_server_py(sys.argv[1])
    else:
        print("Usage: python update_server.py <path_to_server.py>")
