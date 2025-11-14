#!/usr/bin/env python3
import dbus
import dbus.mainloop.glib
from gi.repository import GLib
import time
#             "03:B6:41:D1:A7:1B"
TARGET_ADDR = "03:B6:41:D1:A7:1B"  # MAC tvého gamepadu (velká písmena)

bus = None


def find_device(address):
    """
    Najde DBus objekt zařízení podle MAC adresy.
    Vrací (path, properties_dict) nebo (None, None).
    """
    mngr = dbus.Interface(bus.get_object("org.bluez", "/"),
                          "org.freedesktop.DBus.ObjectManager")
    objects = mngr.GetManagedObjects()

    for path, ifaces in objects.items():
        dev = ifaces.get("org.bluez.Device1")
        if not dev:
            continue
        if dev.get("Address") == address:
            return path, dev
    return None, None


def ensure_services_resolved(path):
    """
    Zajistí, aby pro dané zařízení byl ServicesResolved == True.
    Pokud není, zavolá Connect().
    """
    dev_obj = bus.get_object("org.bluez", path)
    props = dbus.Interface(dev_obj, "org.freedesktop.DBus.Properties")
    dev_if = dbus.Interface(dev_obj, "org.bluez.Device1")

    try:
        connected = props.Get("org.bluez.Device1", "Connected")
        services_resolved = props.Get("org.bluez.Device1", "ServicesResolved")
    except dbus.exceptions.DBusException as e:
        print(f"[WARN] Nemůžu načíst vlastnosti zařízení: {e}")
        return

    print(f"[INFO] Connected={connected}, ServicesResolved={services_resolved}")

    if connected and not services_resolved:
        print("[INFO] Connected == True, ServicesResolved == False -> volám Connect()")
        try:
            dev_if.Connect()
        except dbus.exceptions.DBusException as e:
            print(f"[ERROR] Connect() selhal: {e}")


def on_properties_changed(interface, changed, invalidated, path):
    """
    Handler DBus signálu PropertiesChanged.
    Reaguje jen na Device1 a jen na náš gamepad.
    """
    if interface != "org.bluez.Device1":
        return

    # Zjistit adresu zařízení
    try:
        dev_obj = bus.get_object("org.bluez", path)
        props = dbus.Interface(dev_obj, "org.freedesktop.DBus.Properties")
        address = props.Get("org.bluez.Device1", "Address")
    except dbus.exceptions.DBusException as e:
        print(f"[WARN] Nemůžu načíst adresu pro {path}: {e}")
        return

    if address != TARGET_ADDR:
        return

    # Zajímá nás změna Connected nebo ServicesResolved
    connected_changed = "Connected" in changed
    services_changed = "ServicesResolved" in changed

    if not connected_changed and not services_changed:
        return

    print(f"[EVENT] {address} changed: {dict(changed)}")

    # Pár stovek ms pauza, aby se BlueZ stihl „rozkývat“
    time.sleep(0.3)
    ensure_services_resolved(path)


def initial_check():
    """
    Po startu démonu zkontroluje stav zařízení a případně ho „do-connectuje“.
    """
    path, dev = find_device(TARGET_ADDR)
    if not path:
        print(f"[WARN] Zařízení {TARGET_ADDR} není v BlueZ známé (zatím?).")
        return

    print(f"[INFO] Našel jsem zařízení na path {path}")
    ensure_services_resolved(path)


def main():
    global bus

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    # Připojení na PropertiesChanged signály
    bus.add_signal_receiver(
        on_properties_changed,
        signal_name="PropertiesChanged",
        dbus_interface="org.freedesktop.DBus.Properties",
        path_keyword="path",
    )

    # Po spuštění hned zkontrolovat aktuální stav
    initial_check()

    print("[INFO] gamepad_autoconnect běží, čekám na DBus signály...")
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        print("\n[INFO] Ukončeno pomocí Ctrl+C")
    finally:
        # kdyby náhodou někdo volal quit odjinud, nicemu to nevadí
        try:
            loop.quit()
        except Exception:
            pass

if __name__ == "__main__":
    main()
