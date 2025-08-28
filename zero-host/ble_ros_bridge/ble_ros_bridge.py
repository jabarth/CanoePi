import rclpy
from rclpy.node import Node
import threading
import json
import dbus
import dbus.mainloop.glib
from gi.repository import GLib

from std_msgs.msg import Float32, Int32, Bool, Int32MultiArray

from .ble_constants import *
from .ble_gatt_server import Application, Service, Characteristic

class SensorsCharacteristic(Characteristic):
    def __init__(self, bus, index, service):
        super().__init__(bus, index, SENSORS_CHAR_UUID, ['notify'], service)
        self.notifying = False
        self.value = []

    def update_value(self, sensor_data):
        json_payload = json.dumps(sensor_data)
        self.value = [dbus.Byte(c.encode()) for c in json_payload]
        self.PropertiesChanged(GATT_CHRC_IFACE, {'Value': self.value}, [])

    def StartNotify(self):
        if self.notifying:
            return
        self.notifying = True

    def StopNotify(self):
        if not self.notifying:
            return
        self.notifying = False

class LightsCharacteristic(Characteristic):
    def __init__(self, bus, index, service, ros_node):
        super().__init__(bus, index, LIGHTS_CHAR_UUID, ['write'], service)
        self.ros_node = ros_node

    def WriteValue(self, value, options):
        # Expecting a 6-byte array for the 6 light circuits
        if len(value) != 6:
            self.ros_node.get_logger().error("Invalid light setting received.")
            return

        msg = Int32MultiArray()
        msg.data = [int(v) for v in value]
        self.ros_node.light_settings_pub.publish(msg)

class BilgeCharacteristic(Characteristic):
    def __init__(self, bus, index, service, ros_node):
        super().__init__(bus, index, BILGE_CHAR_UUID, ['write'], service)
        self.ros_node = ros_node

    def WriteValue(self, value, options):
        # Expecting a 1-byte value (0 or 1)
        if len(value) != 1:
            self.ros_node.get_logger().error("Invalid bilge control received.")
            return

        msg = Bool()
        msg.data = bool(value[0])
        self.ros_node.bilge_control_pub.publish(msg)

class CanoeDashService(Service):
    def __init__(self, bus, index, ros_node):
        super().__init__(bus, index, CANOEDASH_SERVICE_UUID, True)
        self.ros_node = ros_node

        self.sensors_char = SensorsCharacteristic(bus, 0, self)
        self.lights_char = LightsCharacteristic(bus, 1, self, ros_node)
        self.bilge_char = BilgeCharacteristic(bus, 2, self, ros_node)

        self.add_characteristic(self.sensors_char)
        self.add_characteristic(self.lights_char)
        self.add_characteristic(self.bilge_char)

class BleRosBridgeNode(Node):
    def __init__(self, mainloop):
        super().__init__('ble_ros_bridge_node')
        self.mainloop = mainloop
        self.sensor_data = {}

        # Publishers
        self.bilge_control_pub = self.create_publisher(Bool, 'bilge_control', 10)
        self.light_settings_pub = self.create_publisher(Int32MultiArray, 'light_settings', 10)

        # Subscribers
        self.create_subscription(Float32, 'oil_level', self.oil_level_callback, 10)
        self.create_subscription(Int32, 'rpm', self.rpm_callback, 10)
        # ... other subscribers

        self.get_logger().info('BLE-ROS Bridge node started.')

    def oil_level_callback(self, msg):
        self.sensor_data['oil'] = msg.data
        self.update_ble_sensors()

    def rpm_callback(self, msg):
        self.sensor_data['rpm'] = msg.data
        self.update_ble_sensors()

    def update_ble_sensors(self):
        # This is where we need to find the characteristic and update it
        # This requires a reference to the service/characteristic object
        # which is tricky. A better design might be to pass a callback
        # to the ROS node.
        pass

def main(args=None):
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    mainloop = GLib.MainLoop()

    # Initialize ROS
    rclpy.init(args=args)
    ros_node = BleRosBridgeNode(mainloop)

    # Set up BLE GATT Application
    app = Application(bus)
    service = CanoeDashService(bus, 0, ros_node)
    app.add_service(service)

    # This is how the ROS node can update the BLE characteristic
    ros_node.update_ble_sensors = lambda: service.sensors_char.update_value(ros_node.sensor_data)

    # Register the GATT application
    adapter = find_adapter(bus)
    if not adapter:
        ros_node.get_logger().fatal('BLE adapter not found')
        return

    gatt_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter), GATT_MANAGER_IFACE)
    gatt_manager.RegisterApplication(app.get_path(), {},
                                     reply_handler=register_app_cb,
                                     error_handler=register_app_error_cb)

    # Spin ROS node in a background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,))
    ros_thread.daemon = True
    ros_thread.start()

    try:
        ros_node.get_logger().info('Running BLE GATT server...')
        mainloop.run()
    except KeyboardInterrupt:
        pass
    finally:
        gatt_manager.UnregisterApplication(app.get_path())
        ros_node.get_logger().info('BLE GATT server stopped.')
        ros_node.destroy_node()
        rclpy.shutdown()

def find_adapter(bus):
    remote_om = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, '/'), DBUS_OM_IFACE)
    objects = remote_om.GetManagedObjects()
    for o, props in objects.items():
        if GATT_MANAGER_IFACE in props.keys():
            return o
    return None

def register_app_cb():
    print('GATT application registered')

def register_app_error_cb(error):
    print('Failed to register application: ' + str(error))
    mainloop.quit()

if __name__ == '__main__':
    main()
