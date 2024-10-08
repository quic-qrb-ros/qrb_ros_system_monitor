## Overview

qrb_ros_system_monitor is a ROS package to access and publish system informations.

Package features:

| Interface           | Component                                  |
| ------------------- | ------------------------------------------ |
| /cpu                | qrb_ros_system_monitor::CpuMonitor         |
| /memory             | qrb_ros_system_monitor::MemoryMonitor      |
| /disk               | qrb_ros_system_monitor::DiskMonitor        |
| /swap               | qrb_ros_system_monitor::SwapMonitor        |
| /temperature        | qrb_ros_system_monitor::TemperatureMonitor |
| /battery            | qrb_ros_system_monitor::BatteryMonitor     |
| /system_info_server | qrb_ros_system_monitor::SystemInfoServer   |

## Quickstart

### Code Sync and Build

Currently, we only support build with QCLINUX SDK.

1. Setup QCLINUX SDK environments follow this document: [Set up the cross-compile environment](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate)

2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

     ```bash
     mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
     ```

3. Clone this project under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`

     ```bash
     git clone https://github.com/quic-qrb-ros/qrb_ros_system_monitor.git
     ```

4. Build projects

     ```bash
     export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
     export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages
     
     colcon build --merge-install --cmake-args \
       -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
       -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
       -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
       -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
       -DBUILD_TESTING=OFF
     ```

5. Install system monitor to device

    ```bash
    cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws/install
    tar czvf qrb_ros_system_monitor.tar.gz lib share
    scp qrb_ros_system_monitor.tar.gz root@[ip-addr]:/opt/
    ssh ssh root@[ip-addr]
    (ssh) tar -zxf /opt/qrb_ros_system_monitor.tar.gz -C /opt/qcom/qirp-sdk/usr/
    ```
### Run System Monitor

1. Source ROS environment

    ```bash
    ssh root@[ip-addr]
    (ssh) export HOME=/opt
    (ssh) source /opt/qcom/qirp-sdk/qirp-setup.sh
    (ssh) export ROS_DOMAIN_ID=xx
    (ssh) source /usr/bin/ros_setup.bash
    ```
2. Run ROS nodes

    ```bash
    (ssh) ros2 run qrb_ros_system_monitor qrb_ros_system_monitor
    ```

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on Qualcomm RB3 gen2.

| Hardware                                                     | Software          |
| ------------------------------------------------------------ | ----------------- |
| [Qualcomm RB3 gen2](https://www.qualcomm.com/developer/hardware/rb3-gen-2-development-kit) | LE.QCROBOTICS.1.0 |

## Contributions

Thanks for your interest in contributing to qrb_ros_system_monitor! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_ros_system_monitor is licensed under the BSD 3-clause "New" or "Revised" License.

Check out the [LICENSE](LICENSE) for more details.
