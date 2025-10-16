# SIMPLE_UNITREE_D1_FIRMWARE

A short software package for controlling the Unitree-D1 arm

**Source Documentation**: [D1 Mechanical Arm Services Interface](https://support.unitree.com/home/en/developer/D1Arm_services)

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [System Service Setup](#system-service-setup)
- [Videos](#videos)

## Prerequisites

The following libraries must be **compiled and installed from source** before building this project:

- [`cyclonedds`](https://github.com/eclipse-cyclonedds/cyclonedds) (releases/0.10.x branch)
- [`cyclonedds-cxx`](https://github.com/eclipse-cyclonedds/cyclonedds-cxx) (releases/0.10.x branch)

The following libraries are **pre-installed on the Unitree D1 robot**. Only install if you've removed them:

- [`CSerialPort`](https://github.com/itas109/CSerialPort)
- [`FashionStar`](https://fashionrobo.com/downloadcenter/)
- [`unitree_sdk2`](https://github.com/unitreerobotics/unitree_sdk2)

Please follow each library’s instructions for building and installing on your system.

## Installation

### 1. Clone the repository:
```bash
git clone https://github.com/MOBILAB-UDESC/simple_unitree_D1_firmware.git
cd simple_unitree_D1_firmware
```

### 2. Build the project:
```bash
mkdir build
cd build
cmake ..
make
```

## Usage

### 1. Stop Default Unitree Services
Before running the custom firmware, stop the default Unitree services:
```bash
sudo systemctl stop marm_controller.service
sudo systemctl stop marm_control.service
sudo systemctl stop marm_communication.service
sudo systemctl stop marm_subscripber.service
```

To permanently disable them:
```bash
sudo systemctl disable marm_controller.service
sudo systemctl disable marm_control.service
sudo systemctl disable marm_communication.service
sudo systemctl disable marm_subscripber.service
```

### 2. Run the Main Service
```bash
./arm_service
```

> **⚠️ Important**: When operating multiple robots on the same network, ensure each robot uses different DDS topic names. Configure topic names in arm_service.cpp before building.

### 3. Master-Slave Configuration (Mimic Mode)

To set up arm mimicking with two D1 robots:

#### On Master Robot:
```bash
./arm_service
```

#### On Slave Robot (stop arm_service if running):
```bash
./arm_slave
```

## System Service Setup

For automatic startup on boot, configure the firmware as a systemd service.

### Step 1: Create launch script (e.g. /home/ubuntu/d1_service.sh)
```bash
#!/bin/bash
/home/ubuntu/simple_unitree_D1_firmware/build/arm_service
```

Make it executable:
```bash
chmod +x /home/ubuntu/d1_service.sh
```

### Step 2: Create a .service file in /etc/systemd/system/ directory (e.g. d1_service.service)
```service
[Unit]
Description=Service for starting the communication with the D1 arm
After=network.after

[Service]
Type=simple
User=ubuntu
ExecStart=/home/ubuntu/d1_service.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### Step 3: Start and activate the service
```bash
sudo systemctl start d1_service.service
sudo systemctl enable d1_service.service
```

## Videos
![D1-Mimic](https://media2.giphy.com/media/v1.Y2lkPTc5MGI3NjExd2k0NWhnNnlqbWhoemp1M3QxaXR5NHRsZ213Zmo3c2o5bW9lb2lvdiZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/niBrMiIwfDMe2jw5Vr/giphy.gif)