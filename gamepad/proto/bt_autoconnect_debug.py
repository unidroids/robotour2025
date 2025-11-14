#!/usr/bin/env python3
import dbus
import dbus.mainloop.glib
from gi.repository import GLib

TARGET_ADDR = "03:B6:41:D3:A7:1B"  # MAC gamepadu

ADAPTER = "hci0"
ADAPTER_PATH = f"/org/bluez/{ADAPTER}"
DEVICE_PATH = f"{ADAPTER_PATH}/dev_{TARGET_ADDR.replace(':', '_')}"

# když nastavíš na False, skript bude jen logovat, bez Connect()
AUTOCONNECT_ENABLED = False

bus = None
_device_iface = None


def get_device_iface():
    """Lazy získání org.bluez.Device1 pro náš gamepad."""
    global _device_iface
    print(f"[autoconnect] get_device_iface(): start, current cache={_device_iface is not None}")
    if _device_iface is not None:
        print("[autoconnect] get_device_iface(): using cached interface")
        return _device_iface

    print(f"[autoconnect] get_device_iface(): getting object at {DEVICE_PATH}")
    dev_obj = bus.get_object("org.bluez", DEVICE_PATH)
    _device_iface = dbus.Interface(dev_obj, "org.bluez.Device1")
    print("[autoconnect] get_device_iface(): interface created and cached")
    return _device_iface


def get_props_iface():
    """Properties iface pro náš gamepad."""
    print(f"[autoconnect] get_props_iface(): getting Properties iface for {DEVICE_PATH}")
    dev_obj = bus.get_object("org.bluez", DEVICE_PATH)
    props = dbus.Interface(dev_obj, "org.freedesktop.DBus.Properties")
    print("[autoconnect] get_props_iface(): got Properties iface")
    return props


def connect_if_needed(reason: str):
    """Pokus o Connect(), ale jen pokud ještě připojeno není."""
    print(f"[autoconnect] connect_if_needed(): reason='{reason}'")

    try:
        dev = get_device_iface()
        props = get_props_iface()
    except dbus.exceptions.DBusException as e:
        print(f"[autoconnect] connect_if_needed(): device/props object missing: {e}")
        return

    try:
        print("[autoconnect] connect_if_needed(): querying Connected property...")
        connected = bool(props.Get("org.bluez.Device1", "Connected"))
        print(f"[autoconnect] connect_if_needed(): Connected={connected}")
    except dbus.exceptions.DBusException as e:
        print(f"[autoconnect] connect_if_needed(): Get(Connected) failed: {e}")
        return

    if connected:
        print(f"[autoconnect] connect_if_needed(): already connected (reason={reason})")
        return

    print(f"[autoconnect] connect_if_needed(): not connected, AUTOCONNECT_ENABLED={AUTOCONNECT_ENABLED}")
    if not AUTOCONNECT_ENABLED:
        print("[autoconnect] connect_if_needed(): autoconnect disabled, not calling Connect()")
        return

    print("[autoconnect] connect_if_needed(): calling Connect() now")
    try:
        dev.Connect()
        print("[autoconnect] connect_if_needed(): Connect() returned without exception")
    except dbus.exceptions.DBusException as e:
        print(f"[autoconnect] connect_if_needed(): Connect() failed: {e}")


def interfaces_added(path, interfaces):
    """Nově přidaný objekt v BlueZ (např. poprvé nalezené zařízení)."""
    print(f"[autoconnect] InterfacesAdded: path={path}")
    print(f"[autoconnect] InterfacesAdded: raw interfaces keys={list(interfaces.keys())}")

    dev_iface = interfaces.get("org.bluez.Device1")
    if dev_iface is None:
        print("[autoconnect] InterfacesAdded: no Device1 interface, ignoring")
        return

    addr = dev_iface.get("Address")
    print(f"[autoconnect] InterfacesAdded: Device1 Address={addr}")

    if addr != TARGET_ADDR:
        print("[autoconnect] InterfacesAdded: different device, ignoring")
        return

    print("[autoconnect] InterfacesAdded: this is our target device, calling connect_if_needed()")
    connect_if_needed("InterfacesAdded")


def properties_changed(interface, changed, invalidated, path=None):
    """Změna vlastností – logujeme vše, co přijde pro náš device."""
    print(f"[autoconnect] PropertiesChanged: interface={interface}, path={path}")
    print(f"[autoconnect] PropertiesChanged: changed keys={list(changed.keys())}, invalidated={list(invalidated)}")

    if interface != "org.bluez.Device1":
        print("[autoconnect] PropertiesChanged: not Device1, ignoring")
        return

    if path != DEVICE_PATH:
        print(f"[autoconnect] PropertiesChanged: not our DEVICE_PATH ({DEVICE_PATH}), ignoring")
        return

    # detailní log hodnot:
    for key, value in changed.items():
        print(f"[autoconnect] PropertiesChanged: {key} -> {value}")

    if "Connected" in changed:
        print(f"[autoconnect] >>> Connected changed to {bool(changed['Connected'])}")

    if "RSSI" in changed:
        # původní trigger – teď jen log + volání connect_if_needed
        print(f"[autoconnect] >>> RSSI changed to {changed['RSSI']}, will call connect_if_needed()")
        connect_if_needed("RSSI update")


def heartbeat():
    """Jednoduchý heartbeat, aby bylo vidět, že GLib loop stále běží."""
    print("[autoconnect] heartbeat: main loop alive")
    return True  # True → GLib timeout se znovu naplánuje


def main():
    global bus

    print("[autoconnect] === script start ===")
    print(f"[autoconnect] TARGET_ADDR={TARGET_ADDR}")
    print(f"[autoconnect] DEVICE_PATH={DEVICE_PATH}")
    print(f"[autoconnect] AUTOCONNECT_ENABLED={AUTOCONNECT_ENABLED}")

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    print("[autoconnect] main(): connected to system bus")

    adapter_obj = bus.get_object("org.bluez", ADAPTER_PATH)
    adapter = dbus.Interface(adapter_obj, "org.bluez.Adapter1")
    print(f"[autoconnect] main(): got adapter at {ADAPTER_PATH}")

    # spustit discovery
    try:
        adapter.StartDiscovery()
        print("[autoconnect] main(): StartDiscovery() ok")
    except dbus.exceptions.DBusException as e:
        print(f"[autoconnect] main(): StartDiscovery() failed: {e}")

    # signál: nové objekty
    bus.add_signal_receiver(
        interfaces_added,
        dbus_interface="org.freedesktop.DBus.ObjectManager",
        signal_name="InterfacesAdded",
    )
    print("[autoconnect] main(): InterfacesAdded receiver registered")

    # signál: změny vlastností
    bus.add_signal_receiver(
        properties_changed,
        dbus_interface="org.freedesktop.DBus.Properties",
        signal_name="PropertiesChanged",
        path_keyword="path",
    )
    print("[autoconnect] main(): PropertiesChanged receiver registered")

    # heartbeat každých 5 s
    GLib.timeout_add_seconds(5, heartbeat)
    print("[autoconnect] main(): heartbeat timer registered")

    # počáteční pokus o connect – kdyby už byl gamepad zapnutý
    #connect_if_needed("initial")

    print("[autoconnect] main(): entering GLib.MainLoop()")
    loop = GLib.MainLoop()
    loop.run()


if __name__ == "__main__":
    main()
