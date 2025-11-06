#!/usr/bin/env python3
"""
fix_servo_cors.py
Adds enhanced CORS handling to server.py to fix "Failed to fetch" errors
from external browsers.
"""

import sys
import os
from datetime import datetime

def fix_cors_handling(filepath: str):
    """Add enhanced CORS and OPTIONS handling for servo endpoints."""
    
    if not os.path.isfile(filepath):
        print(f"❌ Error: {filepath} not found")
        return False
    
    # Create backup
    backup_path = f"{filepath}.backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    with open(filepath, 'r') as f:
        original_content = f.read()
    
    with open(backup_path, 'w') as f:
        f.write(original_content)
    print(f"✅ Backup created: {backup_path}")
    
    # Check if already patched
    if '@app.before_request' in original_content and 'OPTIONS' in original_content:
        print("⚠️  CORS OPTIONS handler already exists, skipping...")
        return True
    
    # Find the location after CORS initialization
    cors_marker = "CORS(app, resources={r\"/*\": {\"origins\": \"*\"}})"
    
    if cors_marker not in original_content:
        print("❌ Could not find CORS initialization marker")
        return False
    
    # Add OPTIONS handler and enhanced CORS
    cors_fix = '''CORS(app, resources={r"/*": {"origins": "*"}})

# Enhanced CORS handling for external browser access
@app.before_request
def handle_preflight():
    """Handle CORS preflight OPTIONS requests"""
    if request.method == "OPTIONS":
        response = app.make_default_options_response()
        headers = response.headers
        headers['Access-Control-Allow-Origin'] = '*'
        headers['Access-Control-Allow-Methods'] = 'GET, POST, PUT, DELETE, OPTIONS'
        headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization, X-Requested-With'
        headers['Access-Control-Max-Age'] = '3600'
        return response

@app.after_request
def add_cors_headers(response):
    """Add CORS headers to all responses"""
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, PUT, DELETE, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization, X-Requested-With'
    return response
'''
    
    updated_content = original_content.replace(cors_marker, cors_fix)
    
    # Write updated file
    with open(filepath, 'w') as f:
        f.write(updated_content)
    
    print("✅ Added enhanced CORS handling")
    print("✅ Added OPTIONS preflight handler")
    print("✅ server.py updated successfully!")
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 fix_servo_cors.py <path_to_server.py>")
        sys.exit(1)
    
    server_path = sys.argv[1]
    success = fix_cors_handling(server_path)
    sys.exit(0 if success else 1)
