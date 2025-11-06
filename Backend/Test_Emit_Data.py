import socketio, json, sys, time, threading

done = threading.Event()
# Track how many rover_data messages we've received
_received_count = 0
_count_lock = threading.Lock()
s = socketio.Client(logger=False, reconnection=False)

@s.on('connect')
def on_connect():
    print('connected')

@s.on('rover_data')
def on_rover_data(data):
    global _received_count
    # Increment safely as callbacks may arrive from a different thread
    with _count_lock:
        _received_count += 1
        current = _received_count
    print(f'rover_data #{current}:', json.dumps(data, indent=2))
    # Wait for 5 messages, then signal completion
    if current >= 5:
        done.set()

@s.on('connect_error')
def on_connect_error(data):
    # Helpful diagnostics when namespace connection fails
    print('connect_error:', data)

# Don't force transports; allow engine to negotiate (polling then upgrade)
# For servers behind eventlet/gevent, forcing 'websocket' can fail depending on env.
s.connect('http://localhost:5001')

# Wait up to 60s for 5 rover_data messages, then disconnect
if not done.wait(60):
    print('timeout waiting for 5 rover_data messages')
try:
    s.disconnect()
except Exception:
    pass
