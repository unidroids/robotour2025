#!/bin/bash
set -e

RULE_FILE="/etc/udev/rules.d/99-howerboard.rules"
SERIAL="066EFF524956846687145438"
SYMLINK_NAME="howerboard"

echo "🛠️ Vytvářím udev pravidlo pro /dev/$SYMLINK_NAME..."

echo "SUBSYSTEM==\"tty\", ENV{ID_SERIAL_SHORT}==\"$SERIAL\", SYMLINK+=\"$SYMLINK_NAME\"" | \
sudo tee "$RULE_FILE" > /dev/null

echo "🔄 Načítám pravidla a spouštím trigger..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# Zkusíme vyčkat, zda se zařízení objeví
echo "⏳ Čekám na /dev/$SYMLINK_NAME..."
for i in {1..5}; do
    if [ -e /dev/$SYMLINK_NAME ]; then
        echo "✅ Symbolický odkaz /dev/$SYMLINK_NAME byl úspěšně vytvořen."
        ls -l /dev/$SYMLINK_NAME
        exit 0
    fi
    sleep 1
done

echo "❌ Nepodařilo se vytvořit /dev/$SYMLINK_NAME. Zkus zařízení odpojit a připojit znovu." >&2
exit 1
