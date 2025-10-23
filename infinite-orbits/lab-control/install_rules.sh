#!/bin/bash
# ===========================================================
# install_rules.sh — Install persistent USB rules for lab setup
# ===========================================================

RULES_FILE="system/99-lab-boards.rules"
DEST_FILE="/etc/udev/rules.d/99-lab-boards.rules"

echo "Installing udev rules from $RULES_FILE to $DEST_FILE..."

if [ "$EUID" -ne 0 ]; then
    echo "Please run this script with sudo:"
    echo "sudo bash install_rules.sh"
    exit 1
fi

if [ ! -f "$RULES_FILE" ]; then
    echo "❌ Error: $RULES_FILE not found."
    exit 1
fi

cp "$RULES_FILE" "$DEST_FILE"
udevadm control --reload-rules
udevadm trigger

echo "✅ Udev rules installed successfully."
echo "Reconnect USB devices to apply the new names."
