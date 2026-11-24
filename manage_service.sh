#!/usr/bin/env bash
# Deploy and manage Conveyor Dashboard V2 startup service
# Automatically detects Linux (systemd) vs macOS (launchd)

ACTION="${1:-status}"
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$DIR/../.." && pwd)"

if [[ "$OSTYPE" == "darwin"* ]]; then
    PLIST_NAME="com.conveyor.dashboard-v2.plist"
    DEST_DIR="$HOME/Library/LaunchAgents"
    PLIST_PATH="$DEST_DIR/$PLIST_NAME"

    case "$ACTION" in
        install|enable)
            mkdir -p "$DEST_DIR"
            cp "$DIR/$PLIST_NAME" "$PLIST_PATH"
            launchctl unload "$PLIST_PATH" 2>/dev/null || true
            launchctl load "$PLIST_PATH"
            echo "Installed and started macOS LaunchAgent: $PLIST_PATH"
            ;;
        start)
            launchctl load "$PLIST_PATH" 2>/dev/null || true
            launchctl start com.conveyor.dashboard-v2
            echo "Started service com.conveyor.dashboard-v2"
            ;;
        stop)
            launchctl stop com.conveyor.dashboard-v2 2>/dev/null || true
            echo "Stopped service com.conveyor.dashboard-v2"
            ;;
        uninstall|disable)
            launchctl unload "$PLIST_PATH" 2>/dev/null || true
            rm -f "$PLIST_PATH"
            echo "Uninstalled LaunchAgent: $PLIST_PATH"
            ;;
        status)
            launchctl list | grep "com.conveyor.dashboard-v2" || echo "Service not running."
            ;;
        *)
            echo "Usage: $0 {install|start|stop|status|uninstall}"
            ;;
    esac
else
    # Linux (Raspberry Pi / Debian / Ubuntu)
    SERVICE_FILE="conveyor-dashboard-v2.service"
    case "$ACTION" in
        install|enable)
            sudo cp "$DIR/$SERVICE_FILE" /etc/systemd/system/
            sudo systemctl daemon-reload
            sudo systemctl enable "$SERVICE_FILE"
            sudo systemctl restart "$SERVICE_FILE"
            echo "Installed and enabled systemd service: $SERVICE_FILE"
            ;;
        start)
            sudo systemctl start "$SERVICE_FILE"
            ;;
        stop)
            sudo systemctl stop "$SERVICE_FILE"
            ;;
        status)
            systemctl status "$SERVICE_FILE" --no-pager
            ;;
        uninstall|disable)
            sudo systemctl stop "$SERVICE_FILE"
            sudo systemctl disable "$SERVICE_FILE"
            sudo rm -f "/etc/systemd/system/$SERVICE_FILE"
            sudo systemctl daemon-reload
            echo "Uninstalled systemd service: $SERVICE_FILE"
            ;;
        *)
            echo "Usage: $0 {install|start|stop|status|uninstall}"
            ;;
    esac
fi
