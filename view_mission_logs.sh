#!/bin/bash
# View Mission Controller Logs in journalctl
# This script shows all mission-related logs with timestamps

echo "=================================================="
echo "Mission Controller Logs Viewer"
echo "=================================================="
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "Available commands:"
echo "  1. View recent mission logs (last 5 minutes)"
echo "  2. View all mission logs from today"
echo "  3. Follow mission logs in real-time"
echo "  4. View logs with specific pattern"
echo ""

# Function to display mission logs
show_recent_logs() {
    echo ""
    echo -e "${GREEN}Recent Mission Controller Logs (last 5 minutes):${NC}"
    echo "=================================================="
    sudo journalctl -u nrp-service.service --since "5 minutes ago" --no-pager | \
        grep -E "MISSION_CONTROLLER" | \
        grep --color=always -E "MISSION LOADED|MISSION STARTED|EXECUTING WAYPOINT|PIXHAWK ARMED|AUTO mode|HOLD mode|Waypoint.*reached|MISSION COMPLETED|$"
    echo "=================================================="
}

show_today_logs() {
    echo ""
    echo -e "${GREEN}Mission Controller Logs (today):${NC}"
    echo "=================================================="
    sudo journalctl -u nrp-service.service --since today --no-pager | \
        grep -E "MISSION_CONTROLLER" | \
        grep --color=always -E "MISSION LOADED|MISSION STARTED|EXECUTING WAYPOINT|PIXHAWK ARMED|AUTO mode|HOLD mode|Waypoint.*reached|MISSION COMPLETED|$"
    echo "=================================================="
}

follow_logs() {
    echo ""
    echo -e "${GREEN}Following Mission Controller Logs (Ctrl+C to stop):${NC}"
    echo "=================================================="
    sudo journalctl -u nrp-service.service -f --no-pager | \
        grep --line-buffered -E "MISSION_CONTROLLER" | \
        grep --line-buffered --color=always -E "MISSION LOADED|MISSION STARTED|EXECUTING WAYPOINT|PIXHAWK ARMED|AUTO mode|HOLD mode|Waypoint.*reached|MISSION COMPLETED|$"
}

search_logs() {
    echo ""
    read -p "Enter search pattern: " pattern
    echo ""
    echo -e "${GREEN}Mission Controller Logs matching '$pattern':${NC}"
    echo "=================================================="
    sudo journalctl -u nrp-service.service --no-pager | \
        grep -E "MISSION_CONTROLLER" | \
        grep --color=always -E "$pattern|$"
    echo "=================================================="
}

# Main menu
case "${1:-}" in
    1|recent)
        show_recent_logs
        ;;
    2|today)
        show_today_logs
        ;;
    3|follow)
        follow_logs
        ;;
    4|search)
        search_logs
        ;;
    *)
        # Interactive mode
        read -p "Enter choice (1-4): " choice
        case $choice in
            1) show_recent_logs ;;
            2) show_today_logs ;;
            3) follow_logs ;;
            4) search_logs ;;
            *) echo "Invalid choice" ;;
        esac
        ;;
esac

echo ""
echo -e "${BLUE}Tip: You can also use these direct commands:${NC}"
echo "  sudo journalctl -u nrp-service.service -f | grep MISSION_CONTROLLER"
echo "  sudo journalctl -u nrp-service.service --since '10 minutes ago' | grep MISSION_CONTROLLER"
echo ""
