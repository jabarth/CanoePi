import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:geolocator/geolocator.dart';
import 'package:provider/provider.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:percent_indicator/percent_indicator.dart';

// --- MODELS ---
// In a real app, this would be in models/sensor_data.dart
class SensorData {
  final double oilLevel;
  final int rpm;
  final bool bilgeStatus;
  final List<int> lightLevels;
  final double voltage;
  final double speed;

  SensorData({
    this.oilLevel = 0.0,
    this.rpm = 0,
    this.bilgeStatus = false,
    this.lightLevels = const [0, 0, 0, 0, 0, 0],
    this.voltage = 0.0,
    this.speed = 0.0,
  });

  factory SensorData.fromJson(Map<String, dynamic> json) {
    return SensorData(
      oilLevel: (json['oil'] ?? 0.0).toDouble(),
      rpm: json['rpm'] ?? 0,
      bilgeStatus: json['bilge_status'] ?? false,
      lightLevels: List<int>.from(json['lights'] ?? []),
      voltage: (json['voltage'] ?? 0.0).toDouble(),
    );
  }
}

// --- STATE MANAGEMENT ---
// In a real app, this would be in services/ble_service.dart
class AppState extends ChangeNotifier {
  FlutterBluePlus flutterBlue = FlutterBluePlus.instance;
  BluetoothDevice? canoeDashPi;
  BluetoothCharacteristic? sensorCharacteristic;
  BluetoothCharacteristic? lightCharacteristic;
  BluetoothCharacteristic? bilgeCharacteristic;

  SensorData sensorData = SensorData();

  void scanForDevices() {
    flutterBlue.startScan(timeout: Duration(seconds: 4));
    flutterBlue.scanResults.listen((results) {
      for (ScanResult r in results) {
        if (r.device.name == 'CanoeDashPi') {
          canoeDashPi = r.device;
          connectToDevice();
          flutterBlue.stopScan();
          break;
        }
      }
    });
  }

  void connectToDevice() async {
    if (canoeDashPi == null) return;
    await canoeDashPi!.connect();
    discoverServices();
    notifyListeners();
  }

  void discoverServices() async {
    List<BluetoothService> services = await canoeDashPi!.discoverServices();
    for (var service in services) {
      // Find characteristics
    }
  }

  void updateSensorData(String jsonData) {
    sensorData = SensorData.fromJson(jsonDecode(jsonData));
    notifyListeners();
  }
}

void main() {
  runApp(
    ChangeNotifierProvider(
      create: (context) => AppState(),
      child: const CanoeDashApp(),
    ),
  );
}

class CanoeDashApp extends StatelessWidget {
  const CanoeDashApp({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'CanoeDash',
      theme: ThemeData.dark(),
      home: const DashboardScreen(),
    );
  }
}

// --- SCREENS ---
class DashboardScreen extends StatelessWidget {
  const DashboardScreen({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    var appState = context.watch<AppState>();

    return Scaffold(
      appBar: AppBar(
        title: const Text('CanoeDash'),
        actions: [
          IconButton(
            icon: const Icon(Icons.bluetooth_searching),
            onPressed: () => appState.scanForDevices(),
          )
        ],
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Text('RPM: ${appState.sensorData.rpm}'),
            Text('Voltage: ${appState.sensorData.voltage.toStringAsFixed(1)}V'),
            // Add other widgets here
            RpmGauge(rpm: appState.sensorData.rpm),
          ],
        ),
      ),
    );
  }
}

// --- WIDGETS ---
class RpmGauge extends StatelessWidget {
  final int rpm;
  const RpmGauge({Key? key, required this.rpm}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return CircularPercentIndicator(
      radius: 120.0,
      lineWidth: 13.0,
      percent: (rpm / 6000).clamp(0.0, 1.0),
      center: Text("${rpm} RPM"),
      progressColor: Colors.green,
    );
  }
}
