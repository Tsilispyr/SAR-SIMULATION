#!/usr/bin/env bash
# SAR batch test launcher - works on Ubuntu bare-metal and WSL2.
# Runs inside tmux so the session survives terminal close / screen lock.
#
# Usage (run from anywhere inside the repo):
#   ./evaluation/run_tests.sh                 # V1 standard, 30 runs per strategy
#   ./evaluation/run_tests.sh --v2            # V2 stress profile
#   ./evaluation/run_tests.sh --runs 50
#   ./evaluation/run_tests.sh --skip none
#   ./evaluation/run_tests.sh --only hybrid
#
# Watch progress:        tmux attach -t sar_batch
# Detach (leave running): Ctrl+B then D

set -euo pipefail

SESSION="sar_batch"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/logs"
LOG_FILE="$LOG_DIR/batch_$(date +%Y%m%d_%H%M%S).log"

mkdir -p "$LOG_DIR"

# --- Dependency checks ---
if ! command -v python3 &>/dev/null; then
    echo "ERROR: python3 not found. sudo apt install python3"; exit 1
fi
if ! command -v tmux &>/dev/null; then
    echo "ERROR: tmux not found. sudo apt install tmux"; exit 1
fi

# --- WSL2 detection ---
if grep -qi microsoft /proc/version 2>/dev/null; then
    echo "[i] WSL2 detected - systemd-inhibit skipped (no logind access)"
    echo "    Prevent Windows sleep by running once in PowerShell (as admin):"
    echo "      powercfg /change standby-timeout-ac 0"
    echo ""
fi

# --- Kill stale session ---
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Existing tmux session '$SESSION' found."
    read -rp "Kill it and start fresh? [y/N] " ans
    if [[ "${ans,,}" == "y" ]]; then
        tmux kill-session -t "$SESSION"
    else
        echo "Attaching to existing session."
        exec tmux attach -t "$SESSION"
    fi
fi

# --- Pick runner: --v2 flag selects stress profile ---
RUNNER="$SCRIPT_DIR/run_all_strategies.py"
PASS_ARGS=()
for arg in "$@"; do
    if [[ "$arg" == "--v2" ]]; then
        RUNNER="$SCRIPT_DIR/run_all_strategies_v2.py"
    else
        PASS_ARGS+=("$arg")
    fi
done

# --- Write an inner script so tmux doesn't have to deal with nested quotes ---
INNER="$LOG_DIR/inner_run.sh"
cat > "$INNER" << HEREDOC
#!/usr/bin/env bash
cd "$SCRIPT_DIR"
echo "Log: $LOG_FILE"
echo ""
PYTHONUNBUFFERED=1 python3 "$RUNNER" ${PASS_ARGS[@]} 2>&1 | tee "$LOG_FILE"
echo ""
echo "=== DONE === \$(date)"
HEREDOC
chmod +x "$INNER"

# --- Launch ---
echo ""
echo "Starting in tmux session: $SESSION"
echo "Log: $LOG_FILE"
echo ""
echo "  Watch:  tmux attach -t $SESSION"
echo "  Detach: Ctrl+B then D"
echo ""

tmux new-session -d -s "$SESSION" -x 220 -y 50
tmux send-keys -t "$SESSION" "bash $INNER" Enter
exec tmux attach -t "$SESSION"
