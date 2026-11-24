#!/usr/bin/env bash
# Deploy and manage Conveyor startup service (conveyor.service)
# Automatically detects Linux (systemd) vs macOS (launchd)

ACTION="${1:-status}"
DIR="$(cd "$(dirname "$0")" && pwd)"

if [[ "$OSTYPE" == "darwin"* ]]; then
    PLIST_NAME="com.conveyor.service.plist"
    DEST_DIR="$HOME/Library/LaunchAgents"
    PLIST_PATH="$DEST_DIR/$PLIST_NAME"

    case "$ACTION" in
        install|enable)
            mkdir -p "$DEST_DIR"
            cp "$DIR/com.conveyor.dashboard-v2.plist" "$PLIST_PATH" 2>/dev/null || true
            launchctl unload "$PLIST_PATH" 2>/dev/null || true
            launchctl load "$PLIST_PATH"
            echo "Installed and started macOS service: $PLIST_PATH"
            ;;
        start)
            launchctl load "$PLIST_PATH" 2>/dev/null || true
            launchctl start com.conveyor.service
            echo "Started service com.conveyor.service"
            ;;
        stop)
            launchctl stop com.conveyor.service 2>/dev/null || true
            echo "Stopped service com.conveyor.service"
            ;;
        uninstall|disable)
            launchctl unload "$PLIST_PATH" 2>/dev/null || true
            rm -f "$PLIST_PATH"
            echo "Uninstalled service: $PLIST_PATH"
            ;;
        status)
            launchctl list | grep "com.conveyor" || echo "Service not running."
            ;;
        *)
            echo "Usage: $0 {install|start|stop|status|uninstall}"
            ;;
    esac
else
    # Linux (Raspberry Pi / Debian / Ubuntu systemd)
    SERVICE_FILE="conveyor.service"
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
