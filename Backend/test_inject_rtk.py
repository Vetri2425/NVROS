import json
import time

# Import the running server module
import server

# Construct a synthetic rtk_baseline payload similar to MAVROS RTKBaseline broadcast
baseline_msg = {
    'type': 'rtk_baseline',
    'rtk_baseline': {
        'baseline_a_mm': 100,    # 0.1 m
        'baseline_b_mm': 0,
        'baseline_c_mm': 0,
        'baseline_distance': 0.1,
        'accuracy': 0.01,
        'iar_num_hypotheses': 1,
        'tow': 123456,
        'rtk_receiver_id': 42,
        'rtk_health': 0,
        'rtk_rate': 1,
        'nsats': 12
    }
}

print('[TEST] Injecting synthetic RTK baseline message into server handler')
server._handle_mavros_telemetry(baseline_msg)

# Allow a moment for handler to process
time.sleep(0.1)

# Retrieve rover data and pretty-print
data = server.get_rover_data()
print('\n[TEST] get_rover_data() output after injection:')
print(json.dumps(data, indent=2))
