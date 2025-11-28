# Overview

<div align="center">

[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/index.html)
[![Middleware](https://img.shields.io/badge/Middleware-CycloneDDS-blue?style=flat)](https://cyclonedds.io/)
![Updated At](https://img.shields.io/badge/Updated_At-November-64748B?style=flat-square)
![Version](https://img.shields.io/badge/Version-1.0.0-2563EB?style=flat-square)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-059669?style=flat-square)](https://opensource.org/licenses/BSD-3-Clause)

**Native ROS 2 integration for PNDbotics robots. Leveraging CycloneDDS for direct `msg` communication and control, eliminating the need for intermediate SDK forwarding.**

</div>

# 📋 Table of Contents

- [System Requirements](#-system-requirements)
- [Installation](#-installation)
  - [Install ROS 2 Humble](#install-ros-2-humble)
  - [Install PNDbotics Packages](#install-pndbotics-packages)
- [Contributing](#-contributing)
- [License](#-license)
- [Contact](#-contact)
- [Version Log](#-version-log)

# 💻 System Requirements

Tested environment configuration:

| System | ROS 2 Version |
| :--- | :--- |
| **Ubuntu 22.04** | **Humble (Recommended)** |

# 🛠️ Installation

## Install ROS 2 Humble

This documentation uses **ROS2 Humble** as an example. Replace `humble` with another ROS2 version if needed.

### 1. Setup Sources

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Install ros-apt-source:

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### 2. Install ROS2 Packages

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

## Install PNDbotics Packages

### 1. Clone pnd_ros2

```bash
git clone https://github.com/pndbotics/pnd_ros2
```

Description:
- `cyclonedds_ws`: workspace for pndbotics ROS2 msg definitions.
- `example`: ROS2 example programs for pndbotics robots.

### 2. Install Dependencies

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
sudo apt install libyaml-cpp-dev
```

### 3. Build Cyclone DDS

pndbotics robots use **cyclonedds 0.10.2**, so ROS2’s middleware must be switched accordingly.
Refer to
Before building, ensure **ROS2 environment variables are NOT auto-sourced**.  
If `~/.bashrc` contains:

```bash
source /opt/ros/humble/setup.bash
```

Remove or comment it out:

```bash
sudo gedit ~/.bashrc
# comment out ROS2 auto-source
# source /opt/ros/humble/setup.bash
```

Build cmd:

```bash
cd ~/pnd_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ..
colcon build --packages-select cyclonedds
```

## 4. Build ROS2 Packages

```bash
source /opt/ros/humble/setup.bash
colcon build
```

# 🤝 Contributing

Contributions are welcome.

Feel free to open issues or pull requests.

# 📄 License

[BSD-3 Clause © PNDbotics](./LICENSE)

# 📞 Contact

- Email: info@pndbotics.com
- Wiki: https://wiki.pndbotics.com  
- SDK: https://github.com/pndbotics/pnd_sdk_python  
- Issues: https://github.com/pndbotics/pnd_mujoco/issues

# 📜 Version Log

| Version | Date       | Updates                                                                              |
| ------- | ---------- | ------------------------------------------------------------------------------------ |
| v1.0.0  | 2025-11-10 | Initial release |

---

<div align="center">

[![Website](https://img.shields.io/badge/Website-PNDbotics-black?)](https://www.pndbotics.com)
[![Twitter](https://img.shields.io/badge/Twitter-@PNDbotics-1DA1F2?logo=twitter&logoColor=white)](https://x.com/PNDbotics)
[![YouTube](https://img.shields.io/badge/YouTube-ff0000?style=flat&logo=youtube&logoColor=white)](https://www.youtube.com/@PNDbotics)
[![Bilibili](https://img.shields.io/badge/-bilibili-ff69b4?style=flat&labelColor=ff69b4&logo=bilibili&logoColor=white)](https://space.bilibili.com/303744535)

**⭐ Star us on GitHub — it helps!**

</div>