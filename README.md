# CanoeDash: A Containerized Marine Dashboard

This repository contains the complete source code and configuration for CanoeDash, a modular and portable marine dashboard system. The project is designed to be built and run entirely within containers using Podman, ensuring a reproducible and clean development environment.

## Overview

The system consists of three main components:

1.  **`pico-firmware`**: A Raspberry Pi Pico 2 based firmware written in C using FreeRTOS and Micro-ROS. It reads sensor data (oil, voltage, RPM, etc.) and controls actuators (lights, bilge pump).
2.  **`zero-host`**: A containerized service for the Raspberry Pi Zero 2 W that runs both a Micro-ROS agent and a ROS 2 application. The agent communicates with the Pico over a serial connection, while the ROS 2 application provides a BLE GATT server to interface with the mobile app.
3.  **`android-app`**: A configurable dashboard application built with Flutter. It connects to the Pi Zero host via BLE to display real-time data and send commands.

All components are designed to be built and managed through the provided `compose.yaml` file.

## Getting Started

**Prerequisites:**
*   [Podman](https://podman.io/)
*   [podman-compose](https://github.com/containers/podman-compose)

To build all container images, run:

```bash
./tools/build-all.sh
```

To launch the services (e.g., the `zero-host`):

```bash
podman-compose up zero-host
```

See the documentation in the `docs/` directory for more details on hardware setup, calibration, and execution.
