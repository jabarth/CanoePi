# System Setup Guide

This guide provides detailed instructions for setting up the CanoeDash development environment and running the project.

## Prerequisites

*   **Podman**: The container engine used for this project. See the [official installation guide](https://podman.io/getting-started/installation).
*   **podman-compose**: A tool to run multi-container applications with Podman. See the [installation instructions](https://github.com/containers/podman-compose).
*   **Git**: For cloning the repository.

## 1. Clone the Repository

```bash
git clone https://github.com/your-username/canoedash.git
cd canoedash
```

## 2. Build the Container Images

All components of the CanoeDash system are built within containers. You can build all images at once using the provided script:

```bash
./tools/build-all.sh
```

Alternatively, you can use `podman-compose` to build the images as defined in the `compose.yaml` file:

```bash
podman-compose build
```

## 3. Build the Pico Firmware

The firmware for the Raspberry Pi Pico is built using a dedicated builder service.

1.  **Run the builder container:**
    ```bash
    podman-compose run --rm pico-builder
    ```
    This command starts the `pico-builder` service, which compiles the firmware. The `--rm` flag removes the container after the build is complete.

2.  **Locate the firmware:**
    The compiled firmware, `pico_firmware.uf2`, will be located in the `pico-firmware/build/` directory.

3.  **Flash the Pico:**
    *   Hold down the `BOOTSEL` button on your Pico while connecting it to your computer via USB.
    *   It will mount as a USB mass storage device (like a flash drive).
    *   Copy the `pico_firmware.uf2` file to the Pico. It will automatically reboot and run the new firmware.

## 4. Run the Zero Host System

The Zero Host system consists of the Micro-ROS agent and the BLE-ROS bridge.

1.  **Connect the Pico to the Pi Zero:**
    Connect the Pico to the Raspberry Pi Zero 2 W via a USB-to-serial connection or directly via the UART pins (GP0/GP1 on the Pico to the Pi's UART pins). Ensure the device path (e.g., `/dev/ttyAMA0`) matches the one in the `compose.yaml` file.

2.  **Start the host services:**
    ```bash
    podman-compose up zero-host
    ```
    This will start the `zero-host` service in the foreground, and you will see the logs from both the Micro-ROS agent and the BLE bridge node.

## 5. Build and Install the Android App

1.  **Run the builder container:**
    ```bash
    podman-compose run --rm app-builder
    ```

2.  **Locate the APK:**
    The Android application package (`.apk`) will be located in `android-app/build/app/outputs/flutter-apk/app-release.apk`.

3.  **Install the APK:**
    *   Copy the `.apk` file to your Android device.
    *   You may need to enable "Install from unknown sources" in your device's security settings.
    *   Open the file on your device to install the app.

## 6. Using the App

1.  Open the CanoeDash app on your phone.
2.  Tap the Bluetooth icon to scan for and connect to the "CanoeDashPi" device.
3.  Once connected, the dashboard should start displaying live data from the sensors.
