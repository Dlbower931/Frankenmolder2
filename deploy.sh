#!/bin/bash
# One command, run FROM YOUR MAC, to push the code to the Pi and restart it:
#     ./deploy.sh            # copy code, ensure X access, rebuild + restart, follow logs
#     ./deploy.sh --no-logs  # same, but don't follow logs at the end
#
# Override the target with env vars, e.g.  PI_HOST=other-pi ./deploy.sh
set -euo pipefail

PI_USER="${PI_USER:-dlbower}"
PI_HOST="${PI_HOST:-fm-pi01}"
PI_DIR="${PI_DIR:-~/apps/Frankenmolder2}"
TARGET="$PI_USER@$PI_HOST"

cd "$(dirname "$0")"

echo "==> Copying code to $TARGET:$PI_DIR"
# No --delete: never removes anything on the Pi (logs, local tweaks).
rsync -az \
    --exclude '.git/' \
    --exclude '.DS_Store' \
    --exclude 'pi_ros_logs/' \
    --exclude '.pio/' \
    --exclude '__pycache__/' \
    --exclude '*.pdf' \
    ./ "$TARGET:$PI_DIR/"

echo "==> Ensuring desktop grants X access to Docker (idempotent)"
ssh -t "$TARGET" "cd $PI_DIR && chmod +x setup_host_display.sh start_node.sh && ./setup_host_display.sh"

echo "==> Rebuilding and restarting container"
ssh -t "$TARGET" "cd $PI_DIR && docker compose up -d --build --remove-orphans"

if [ "${1:-}" != "--no-logs" ]; then
    echo "==> Following logs (Ctrl-C to stop watching; container keeps running)"
    ssh -t "$TARGET" "docker logs -f --tail 50 frankenmolder_ros"
fi
