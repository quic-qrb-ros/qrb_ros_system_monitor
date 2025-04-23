# QRB ROS System Monitor

## Overview

`qrb_ros_system_monitor` is a ROS package designed to access and publish system information.

## Quickstart

For the Qualcomm QCLinux platform, we provide two ways to build this package.

<details>
<summary>On-Device Compilation with Docker</summary>

1. Set up the QCLinux Docker environment following the [QRB ROS Docker Setup](https://github.com/qualcomm-qrb-ros/qrb_ros_docker?tab=readme-ov-file#quickstart).

2. Clone and build the source code:

    ```bash
    cd ~/qrb_ros_ws/src/qrb_ros_docker/scripts && \
    bash docker_run.sh

    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_system_monitor.git
    colcon build
    ```

3. Run ROS node

   ```bash
   source install/setup.bash
   ros2 run qrb_ros_system_monitor qrb_ros_system_monitor
   ```

</details>

<details><summary>Cross Compilation with QIRP SDK</summary>

1. Set up the QIRP SDK environment: Refer to [QRB ROS Documents: Getting Started](https://qualcomm-qrb-ros.github.io/main/getting_started/environment_setup.html).

2. Create a workspace and clone the source code:

    ```bash
    mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
    cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws

    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_system_monitor.git
    ```

3. Build the source code with QIRP SDK:

    ```bash
    colcon build --merge-install --cmake-args \
      -DPYTHON_EXECUTABLE=${OECORE_NATIVE_SYSROOT}/usr/bin/python3 \
      -DPython3_NumPy_INCLUDE_DIR=${OECORE_NATIVE_SYSROOT}/usr/lib/python3.12/site-packages/numpy/core/include \
      -DPYTHON_SOABI=cpython-312-aarch64-linux-gnu \
      -DCMAKE_MAKE_PROGRAM=/usr/bin/make \
      -DCMAKE_LIBRARY_PATH=${OECORE_TARGET_SYSROOT}/usr/lib \
      -DBUILD_TESTING=OFF
    ```

4. Install ROS package to device

   ```bash
   cd install
   tar czvf qrb_ros_system_monitor.tar.gz include lib share
   scp qrb_ros_system_monitor.tar.gz root@[ip-addr]:~
   ssh root@[ip-addr]
   (ssh) mount -o remount,rw /usr
   (ssh) tar --no-same-owner -zxf ~/qrb_ros_system_monitor.tar.gz -C /usr/
   ```

6. Login to device and run

   ```bash
   ssh root@[ip-addr]
   (ssh) source /usr/bin/ros_setup.bash
   (ssh) ros2 run qrb_ros_system_monitor qrb_ros_system_monitor
   ```

</details>


You can get more details from [here](https://qualcomm-qrb-ros.github.io/main/index.html).

## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)

## Authors

* **Peng Wang** - *Maintainer* - [@penww](https://github.com/penww)

See also the list of [contributors](https://github.com/qualcomm-qrb-ros/qrb_ros_system_monitor/graphs/contributors) who participated in this project.


## License

Project is licensed under the [BSD-3-Clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.
