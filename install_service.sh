#!/bin/bash

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}Setting up NRP systemd service...${NC}"

# Copy service file to systemd directory
echo "Copying service file..."
sudo cp nrp.service /etc/systemd/system/

# Reload systemd daemon
echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

# Enable the service
echo "Enabling NRP service..."
sudo systemctl enable nrp.service

# Start the service
echo "Starting NRP service..."
sudo systemctl start nrp.service

echo -e "${GREEN}Service installation complete!${NC}"
echo ""
echo "You can now use the following commands to manage the service:"
echo "- sudo systemctl start nrp    # Start the service"
echo "- sudo systemctl stop nrp     # Stop the service"
echo "- sudo systemctl restart nrp  # Restart the service"
echo "- sudo systemctl status nrp   # Check service status"
echo "- journalctl -u nrp -f       # View service logs"