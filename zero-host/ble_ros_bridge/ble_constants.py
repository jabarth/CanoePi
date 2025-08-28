#
# BLE Constants for the CanoeDash Project
#

# --- Custom Service and Characteristic UUIDs ---
# These UUIDs are custom for this project.
# Using a base UUID and incrementing the first part is a common practice.
BASE_UUID = "-0000-1000-8000-00805f9b34fb"
def custom_uuid(short_uuid):
    return "0000" + short_uuid + BASE_UUID

CANOEDASH_SERVICE_UUID = custom_uuid("181c")

# Characteristic UUIDs
SENSORS_CHAR_UUID = custom_uuid("2a6e") # Based on standard "Sensor Location"
LIGHTS_CHAR_UUID = custom_uuid("2b01")  # Custom
BILGE_CHAR_UUID = custom_uuid("2b02")   # Custom


# --- BlueZ D-Bus Constants ---
BLUEZ_SERVICE_NAME = 'org.bluez'
DBUS_OM_IFACE = 'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE = 'org.freedesktop.DBus.Properties'

GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
GATT_SERVICE_IFACE = 'org.bluez.GattService1'
GATT_CHRC_IFACE = 'org.bluez.GattCharacteristic1'
GATT_DESC_IFACE = 'org.bluez.GattDescriptor1'
