#!/bin/bash
# One-time setup, run ON THE PI (host) as the desktop user, NOT with sudo:
#     ./setup_host_display.sh
#
# Installs a desktop autostart entry that runs `xhost +local:docker` every time
# the desktop session starts, so the container's GUI can open on :0 after a reboot
# without anyone having to SSH in and run it by hand.
#
# Undo with:  rm ~/.config/autostart/frankenmolder-xhost.desktop
set -e

if [ "$(id -u)" -eq 0 ]; then
    echo "Run this as your normal desktop user, not root/sudo." >&2
    exit 1
fi

if ! command -v xhost >/dev/null 2>&1; then
    echo "xhost not found; installing x11-xserver-utils..."
    sudo apt-get install -y x11-xserver-utils
fi

AUTOSTART_DIR="$HOME/.config/autostart"
ENTRY="$AUTOSTART_DIR/frankenmolder-xhost.desktop"
mkdir -p "$AUTOSTART_DIR"

cat > "$ENTRY" <<'EOF'
[Desktop Entry]
Type=Application
Name=Frankenmolder X access for Docker
Comment=Allow the frankenmolder_ros container to draw the GUI on this display
Exec=sh -c 'export DISPLAY="${DISPLAY:-:0}"; for i in $(seq 1 30); do xhost +local:docker && exit 0; sleep 1; done'
X-GNOME-Autostart-enabled=true
NoDisplay=true
EOF

echo "Installed $ENTRY"

# Apply it right now too, if a display is up, so no reboot is needed to test.
if DISPLAY="${DISPLAY:-:0}" xhost +local:docker >/dev/null 2>&1; then
    echo "Applied xhost +local:docker to the current session."
else
    echo "No display reachable right now; it will apply at next desktop login."
fi
