import socketio, json, time

s = socketio.Client(logger=False, reconnection=False)

@s.on('connect')
def on_connect():
    print('connected to backend')

@s.on('inject_ack')
def on_ack(data):
    print('inject_ack:', data)

@s.on('rover_data')
def on_rover_data(data):
    print('rover_data received:')
    print(json.dumps(data, indent=2))
    # Disconnect after first rover_data
    try:
        s.disconnect()
    except Exception:
        pass

s.connect('http://localhost:5001')

# Construct synthetic RTK baseline payload
payload = {
    'type': 'rtk_baseline',
    'rtk_baseline': {
        'baseline_a_mm': 100,
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

# Emit test injection
s.emit('inject_mavros_telemetry', payload)

# Wait for rover_data or ack
try:
    s.wait(5)
except Exception:
    pass

try:
    s.disconnect()
except Exception:
    pass
