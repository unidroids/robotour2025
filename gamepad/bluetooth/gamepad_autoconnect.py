#!/usr/bin/env python3
import dbus
import dbus.mainloop.glib
from gi.repository import GLib

# Priorita: NAME, adresa je jen "pojistka"
TARGET_NAME = "GameSir-Nova Lite"
TARGET_ADDR = None  # můžeš vyplnit "03:B6:41:D1:A7:1B", ale není nutné

bus = None
target_path = None  # D-Bus path našeho zařízení


def find_device_by_name_or_addr():
    """
    Najde DBus objekt zařízení podle Name/Alias a/nebo MAC adresy.
    Vrací (path, properties_dict) nebo (None, None).
    """
    mngr = dbus.Interface(bus.get_object("org.bluez", "/"),
                          "org.freedesktop.DBus.ObjectManager")
    objects = mngr.GetManagedObjects()

    candidate = None

    for path, ifaces in objects.items():
        dev = ifaces.get("org.bluez.Device1")
        if not dev:
            continue

        name = dev.get("Name", "")
        alias = dev.get("Alias", "")
        addr = dev.get("Address", "")

        # 1) Filtr na jméno
        if TARGET_NAME and TARGET_NAME not in (name, alias):
            continue

        # 2) Případný filtr na adresu (pokud je zadaná)
        if TARGET_ADDR and addr != TARGET_ADDR:
            continue

        # první, co splní podmínky, bereme
        candidate = (path, dev)
        break

    return candidate if candidate else (None, None)


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
        address = props.Get("org.bluez.Device1", "Address")
        name = props.Get("org.bluez.Device1", "Name")
    except dbus.exceptions.DBusException as e:
        print(f"[WARN] Nemůžu načíst vlastnosti zařízení: {e}")
        return

    print(f"[INFO] Device {name} ({address}) Connected={connected}, "
          f"ServicesResolved={services_resolved}")

    if connected and not services_resolved:
        print("[INFO] Connected == True, ServicesResolved == False -> volám Connect()")
        try:
            dev_if.Connect()
        except dbus.exceptions.DBusException as e:
            print(f"[ERROR] Connect() selhal: {e}")


def on_properties_changed(interface, changed, invalidated, path):
    """
    Handler DBus signálu PropertiesChanged.
    Reaguje jen na Device1 a jen na náš GameSir.
    """
    global target_path

    if interface != "org.bluez.Device1":
        return

    if path != target_path:
        # signál z jiného zařízení, ignorujeme
        return

    connected_changed = "Connected" in changed
    services_changed = "ServicesResolved" in changed

    if not connected_changed and not services_changed:
        return

    print(f"[EVENT] {path} changed: {dict(changed)}")

    # Lehké zpoždění, ale neblokujeme mainloop (žádný time.sleep)
    GLib.timeout_add(300, lambda: (ensure_services_resolved(path) or False))


def initial_check():
    """
    Po startu démonu zkontroluje stav zařízení a případně ho „do-connectuje“.
    """
    global target_path

    path, dev = find_device_by_name_or_addr()
    if not path:
        print(f"[WARN] Zařízení '{TARGET_NAME}' zatím v BlueZ není "
              f"(zkontroluj, že je spárované a zapnuté).")
        return

    target_path = path

    addr = dev.get("Address", "??")
    name = dev.get("Name", "??")
    print(f"[INFO] Našel jsem zařízení '{name}' na path {path}, address {addr}")

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
        try:
            loop.quit()
        except Exception:
            pass


if __name__ == "__main__":
    main()
