#!/usr/bin/env python3
import dbus
import dbus.mainloop.glib
from gi.repository import GLib

TARGET_ADDR = "03:B6:41:D3:A7:1B"  # MAC gamepadu

ADAPTER = "hci0"
ADAPTER_PATH = f"/org/bluez/{ADAPTER}"
DEVICE_PATH = f"{ADAPTER_PATH}/dev_{TARGET_ADDR.replace(':', '_')}"

AUTOCONNECT_ENABLED = True
CONNECT_RATE_LIMIT_SEC = 2  # minimální rozestup pokusů o Connect

bus = None
_device_iface = None
_props_iface = None
last_connect_attempt_us = 0


def ensure_ifaces():
    """Zajistí Device1 + Properties iface pro náš gamepad."""
    global _device_iface, _props_iface

    if _device_iface is not None and _props_iface is not None:
        return

    dev_obj = bus.get_object("org.bluez", DEVICE_PATH)
    _device_iface = dbus.Interface(dev_obj, "org.bluez.Device1")
    _props_iface = dbus.Interface(dev_obj, "org.freedesktop.DBus.Properties")
    print(f"[autoconnect] ifaces ready for {DEVICE_PATH}")


def connect_if_needed(reason: str):
    """Pokus o Connect(), ale jen když dává smysl."""
    global last_connect_attempt_us

    if not AUTOCONNECT_ENABLED:
        print(f"[autoconnect] {reason}: autoconnect disabled, skipping")
        return

    try:
        ensure_ifaces()
    except dbus.exceptions.DBusException as e:
        print(f"[autoconnect] {reason}: device/props not ready: {e}")
        return

    try:
        connected = bool(
            _props_iface.Get("org.bluez.Device1", "Connected")
        )
    except dbus.exceptions.DBusException as e:
        print(f"[autoconnect] {reason}: Get(Connected) failed: {e}")
        return

    if connected:
        # už připojeno – není co dělat
        print(f"Opravdu? Jak poznat, že je opravdu připojen? ServicesResolved?")
        return

    now_us = GLib.get_monotonic_time()
    if now_us - last_connect_attempt_us < CONNECT_RATE_LIMIT_SEC * 1_000_000:
        # příliš brzy po posledním pokusu
        print("příliš brzy po posledním pokusu")
        return

    last_connect_attempt_us = now_us

    print(f"[autoconnect] {reason}: trying Connect()")
    try:
        _device_iface.Connect()
        print("[autoconnect] Connect() called (no exception)")
    except dbus.exceptions.DBusException as e:
        msg = str(e)
        # typické chyby, které nechceme hrotit
        if "In Progress" in msg or "br-connection-create-socket" in msg:
            print(f"[autoconnect] Connect() transient error: {msg}")
        else:
            print(f"[autoconnect] Connect() failed: {msg}")


def interfaces_added(path, interfaces):
    """Nově přidané zařízení – zajímá nás náš gamepad."""
    dev_iface = interfaces.get("org.bluez.Device1")
    if dev_iface is None:
        return

    addr = dev_iface.get("Address")
    if addr != TARGET_ADDR:
        return

    print(f"[autoconnect] InterfacesAdded: target device at {path}")
    connect_if_needed("InterfacesAdded")


def properties_changed(interface, changed, invalidated, path=None):
    """Změna vlastností – React na náš DEVICE_PATH."""
    if interface != "org.bluez.Device1":
        return
    if path != DEVICE_PATH:
        return

    # Log jen pro náš device
    if "Connected" in changed:
        val = bool(changed["Connected"])
        print(f"[autoconnect] Connected -> {val}")
        # když spadne spojení, zkusíme reconnect
        if not val:
            connect_if_needed("Connected=False")

    if "RSSI" in changed:
        rssi = int(changed["RSSI"])
        print(f"[autoconnect] RSSI -> {rssi}")
        # RSSI znamená: device vysílá → zkus connect
        connect_if_needed("RSSI update")


def main():
    global bus

    print("[autoconnect] === start ===")
    print(f"[autoconnect] TARGET_ADDR={TARGET_ADDR}")
    print(f"[autoconnect] DEVICE_PATH={DEVICE_PATH}")
    print(f"[autoconnect] AUTOCONNECT_ENABLED={AUTOCONNECT_ENABLED}")

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    adapter_obj = bus.get_object("org.bluez", ADAPTER_PATH)
    adapter = dbus.Interface(adapter_obj, "org.bluez.Adapter1")

    # scan – potřebujeme RSSI / discovery eventy
    try:
        adapter.StartDiscovery()
        print("[autoconnect] StartDiscovery() ok")
    except dbus.exceptions.DBusException as e:
        print(f"[autoconnect] StartDiscovery() failed: {e}")

    # signály
    bus.add_signal_receiver(
        interfaces_added,
        dbus_interface="org.freedesktop.DBus.ObjectManager",
        signal_name="InterfacesAdded",
    )

    bus.add_signal_receiver(
        properties_changed,
        dbus_interface="org.freedesktop.DBus.Properties",
        signal_name="PropertiesChanged",
        path_keyword="path",
    )

    # jednorázově – kdyby byl gamepad už zapnutý
    connect_if_needed("initial")

    loop = GLib.MainLoop()
    loop.run()


if __name__ == "__main__":
    main()
