#!/usr/bin/env python3
"""Quick test to check if RTK injection handler is being called."""

import socketio
import time

sio = socketio.Client()

@sio.on('connect')
def on_connect():
    print("✅ Connected")
    time.sleep(1)
    
    # Simple injection
    msg = {
        'type': 'rtk_baseline',
        'rtk_baseline': {
            'baseline_a_mm': 12500,
            'baseline_b_mm': -8300,
            'baseline_c_mm': 4200,
            'iar_num_hypotheses': 1,
            'accuracy': 0.014,
            'base_linked': True
        }
    }
    
    print("📤 Sending injection...")
    sio.emit('inject_mavros_telemetry', msg)

@sio.on('inject_ack')
def on_ack(data):
    print(f"📥 Ack received: {data}")
    time.sleep(2)
    sio.disconnect()

@sio.on('rover_data')
def on_rover(data):
    print(f"📥 rover_data: rtk_baseline={data.get('rtk_baseline')}, rtk_baseline_ts={data.get('rtk_baseline_ts')}")

try:
    sio.connect('http://localhost:5001')
    sio.wait()
except Exception as e:
    print(f"❌ Error: {e}")
