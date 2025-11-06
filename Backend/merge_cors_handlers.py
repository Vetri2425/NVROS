#!/usr/bin/env python3
"""
merge_cors_handlers.py
Merges the duplicate @app.after_request handlers into one unified handler.
"""

import sys
import os
import re
from datetime import datetime

def merge_after_request_handlers(filepath: str):
    """Merge duplicate @app.after_request handlers."""
    
    if not os.path.isfile(filepath):
        print(f"❌ Error: {filepath} not found")
        return False
    
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Create backup
    backup_path = f"{filepath}.backup_merge_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    with open(backup_path, 'w') as f:
        f.write(content)
    print(f"✅ Backup created: {backup_path}")
    
    # Find the first @app.after_request (CORS handler at line ~54)
    cors_handler_pattern = r'@app\.after_request\s+def add_cors_headers\(response\):.*?return response'
    
    # Find the second @app.after_request (_track_http_response at line ~586)
    track_handler_pattern = r'@app\.after_request\s+def _track_http_response\(response\):.*?return response'
    
    # Extract both handlers
    cors_match = re.search(cors_handler_pattern, content, re.DOTALL)
    track_match = re.search(track_handler_pattern, content, re.DOTALL)
    
    if not cors_match or not track_match:
        print("⚠️  Could not find both handlers, checking if already merged...")
        # Check if already has combined handler
        if 'def unified_response_handler(response):' in content:
            print("✅ Handlers already merged")
            return True
        return False
    
    # Remove the first CORS handler (we'll merge it into the second)
    content_without_first = content.replace(cors_match.group(0), '')
    
    # Create unified handler that combines both
    unified_handler = '''@app.after_request
def unified_response_handler(response):
    """Unified handler: Add CORS headers + track HTTP responses"""
    # Add CORS headers
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, PUT, DELETE, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization, X-Requested-With'
    
    # Track HTTP responses for observability
    try:
        if request.path.startswith('/socket.io'):
            return response
        method = request.method
        path = request.path
        status = response.status_code
        _response_log.append({
            'ts': time.time(),
            'method': method,
            'path': path,
            'status': status
        })
        # Keep last 500
        if len(_response_log) > 500:
            _response_log.pop(0)
    except Exception:
        pass
    
    return response'''
    
    # Replace the second handler with unified one
    final_content = content_without_first.replace(track_match.group(0), unified_handler)
    
    # Write updated file
    with open(filepath, 'w') as f:
        f.write(final_content)
    
    print("✅ Merged duplicate @app.after_request handlers")
    print("✅ Created unified_response_handler with CORS + tracking")
    print("✅ server.py updated successfully!")
    return True

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 merge_cors_handlers.py <path_to_server.py>")
        sys.exit(1)
    
    server_path = sys.argv[1]
    success = merge_after_request_handlers(server_path)
    sys.exit(0 if success else 1)
