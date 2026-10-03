#!/usr/bin/env bash
# Disable Ubuntu sleep/suspend for the current session.
# Run once before starting the batch tests; revert with revert_sleep_ubuntu.sh.
#
# Works on Ubuntu 20.04+ with GNOME and systemd.
# Does NOT require sudo (all changes are per-user, except the systemd mask).

echo "=== Disabling sleep / suspend for this session ==="

# 1. Blank screen and auto-suspend via GNOME settings (per-user, no sudo)
gsettings set org.gnome.desktop.session      idle-delay 0              2>/dev/null || true
gsettings set org.gnome.settings-daemon.plugins.power \
    sleep-inactive-ac-type    'nothing'                                  2>/dev/null || true
gsettings set org.gnome.settings-daemon.plugins.power \
    sleep-inactive-battery-type 'nothing'                                2>/dev/null || true
gsettings set org.gnome.settings-daemon.plugins.power \
    idle-dim false                                                        2>/dev/null || true

# 2. Mask systemd sleep targets (requires sudo — skip if unavailable)
if sudo -n systemctl mask \
        sleep.target suspend.target hibernate.target hybrid-sleep.target \
        2>/dev/null; then
    echo "  [✓] systemd sleep targets masked (sudo OK)"
    echo "      Revert: sudo systemctl unmask sleep.target suspend.target hibernate.target hybrid-sleep.target"
else
    echo "  [~] systemd targets not masked (no passwordless sudo — OK, inhibit lock will handle it)"
fi

echo ""
echo "Done. Sleep is disabled for this session."
echo "Run 'evaluation/run_tests.sh' now."
echo ""
echo "To restore default sleep settings later:"
echo "  gsettings reset org.gnome.desktop.session idle-delay"
echo "  gsettings reset org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type"
