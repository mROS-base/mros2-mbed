[![build_app](https://github.com/mROS-base/mros2-mbed/actions/workflows/build_app.yaml/badge.svg)](https://github.com/mROS-base/mros2-mbed/actions/workflows/build_app.yaml)
[![build_board](https://github.com/mROS-base/mros2-mbed/actions/workflows/build_board.yaml/badge.svg)](https://github.com/mROS-base/mros2-mbed/actions/workflows/build_board.yaml)

# mros2-mbed

mROS 2 (`mros2` as casually codename) realizes a agent-less and lightweight runtime environment compatible with ROS 2 for embedded devices.
mROS 2 mainly offers pub/sub APIs compatible with [rclcpp](https://docs.ros.org/en/rolling/p/rclcpp/index.html) for embedded devices.

mROS 2 consists of communication library for pub/sub APIs, RTPS protocol, UDP/IP stack, and real-time kernel.
This repository provides the reference implementation of mROS 2 that can be operated on the Mbed enabled board.
Please also check [mros2 repository](https://github.com/mROS-base/mros2) for more details and another implementations.

## Supported environment

- Mbed device
  - Board: Mbed enabled boards having an Ethernet port  
    - For now, these boards below are confirmed to run the example on them.
      - [STM32 NUCLEO-F767ZI](https://www.st.com/en/evaluation-tools/nucleo-f767zi.html)
      - [STM32 NUCLEO-H743ZI2](https://www.st.com/en/evaluation-tools/nucleo-h743zi.html)
    - These boards below are also confirmed but not always supported in the latest version (due to our development resources,,,).
      - [STM32 NUCLEO-F429ZI](https://www.st.com/en/evaluation-tools/nucleo-f429zi.html)
      - [STM32 NUCLEO-F746ZG](https://www.st.com/en/evaluation-tools/nucleo-f746zg.html)
      - [STM32 F746NG-Discovery](https://www.st.com/ja/evaluation-tools/32f746gdiscovery.html)
      - [STM32 F769NI-Discovery](https://www.st.com/ja/evaluation-tools/32f769idiscovery.html)
      - [Seeed Arch Max V1.1](https://wiki.seeedstudio.com/Arch_Max_v1.1/)
      - [RENESAS GR-MANGO](https://www.renesas.com/products/gadget-renesas/boards/gr-mango)
  - Kernel: [Mbed OS 6](https://github.com/ARMmbed/mbed-os)
  - check the Mbed website for [the boards list](https://os.mbed.com/platforms/?q=&Mbed+OS+6=Bare+metal&Mbed+OS+6=RTOS&Communication=Ethernet) where mros2 may work. Please let us know if you find a new board that can work as mros2 enabled device.
- Host environment
  - [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04 LTS
  - [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/index.html) on Ubuntu 20.04 LTS
- Network setting
  - When running by default, make sure both the device and the host are connected to the wired network with the following setting, since they are statically configured to the board.
    - IP address: 192.168.11.x (.2 will be assigned to the board)
    - Netmask: 255.255.255.0
    - Gateway: 192.168.11.1
      - You can configure these setting to yours by editing `platform/mros2-platform.h`.
      - If you want to use DHCP connection, please comment out the `#define MROS2_IP_ADDRESS_STATIC` line in `platform/mros2-platform.h`.
  - The firewall on the host (Ubuntu) needs to be disabled for ROS 2 (DDS) communication (e.g. `$ sudo ufw disable`).
  - If the host is connected to the Internet with other network adapters, communication with mros2 may not work properly. In that case, please turn off them.

## Getting Started

1. Prepare these items below.
- Host PC having an Ethernet port whose network is set to the above and a docker environment.
- Mbed board having an Ethernet port (listed above).
2. Build Mbed executable binary using [the Docker environment for Mbed CLI2](https://github.com/ARMmbed/mbed-os/pkgs/container/mbed-os-env).  
(You can also use the native environment where MBed CLI2 could work well. Please add `native` to 4th arg. (see [the example instruction to prepare native env](https://github.com/mROS-base/mros2-mbed/commit/90225c77e07e5cedc8473285b457827cb047e481)))
```
git clone https://github.com/mROS-base/mros2-mbed
cd mros2-mbed
#(Please replace the [TARGET] with the ones as below.)
# +-------------------+----------------+
# | Your target board | [TARGET]       |
# +-------------------+----------------+
# | NUCLEO-F767ZI     | NUCLEO_F767ZI  |
# | NUCLEO-H743ZI2    | NUCLEO_H743ZI2 |
# | NUCLEO-F429ZI     | NUCLEO_F429ZI  |
# | NUCLEO-F746ZG     | NUCLEO_F746ZG  |
# | F746NG-Discovery  | DISCO_F746NG   |
# | F769NI-Discovery  | DISCO_F769NI   |
# | Arch Max v1.1     | ARCH_MAX       |
# | GR-MANGO          | GR_MANGO       |
# +-------------------+----------------+
./build.bash all [TARGET] echoback_string
```
After that, you will find an executable binary is created in the path below.
```
cmake_build/[TARGET]/develop/GCC_ARM/mros2-mbed.bin
```
3. Connect the PC and Mbed Board with USB and LAN cables.
4. Open Serial Console of the Mbed board. (115200bps)
5. Copy the executable binary above to the Mbed Board.
   (you may find it in the Nautilus file manager as NODE_F429ZI, F767ZI or DAPLINK.)
```
mros2-mbed start!
app name: echoback_string
Successfully connected to network
  IP Address: 192.168.11.2
[MROS2LIB] set IP address for RTPS communication
[MROS2LIB] mros2_init task start
mROS 2 initialization is completed

[MROS2LIB] create_node
[MROS2LIB] start creating participant
[MROS2LIB] successfully created participant
[MROS2LIB] create_publisher complete.
[MROS2LIB] create_subscription complete.
[MROS2LIB] Initilizing Domain complete
ready to pub/sub message

publishing msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 0'
publishing msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 1'
publishing msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 2'
publishing msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 3'
...(SNIPPED)...
```
6. One of the easiest way to operate the host is using Docker. On the host terminal, type the command below.
```
docker run --rm -it --net=host ros:humble /bin/bash \
  -c "source /opt/ros/humble/setup.bash &&
  cd &&
  git clone https://github.com/mROS-base/mros2-host-examples &&
  cd mros2-host-examples &&
  colcon build --packages-select mros2_echoreply_string &&
  source install/setup.bash &&
  ros2 run mros2_echoreply_string echoreply_node"
```
Then, we can confirm the communication between the host and Mbed board via ROS 2 topic.
```
Cloning into 'mros2-host-examples'...
remote: Enumerating objects: 831, done.
remote: Counting objects: 100% (85/85), done.
remote: Compressing objects: 100% (68/68), done.
remote: Total 831 (delta 46), reused 26 (delta 15), pack-reused 746
Receiving objects: 100% (831/831), 96.01 KiB | 7.38 MiB/s, done.
Resolving deltas: 100% (448/448), done.
Starting >>> mros2_echoreply_string
Finished <<< mros2_echoreply_string [9.02s]                     

Summary: 1 package finished [9.17s]
[INFO] [1666012200.122092282] [mros2_echoreply_node]: 
Subscribed msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 7'
[INFO] [1666012200.122210443] [mros2_echoreply_node]: 
Publishing msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 7'
[INFO] [1666012201.127168943] [mros2_echoreply_node]: 
Subscribed msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 8'
[INFO] [1666012201.127216518] [mros2_echoreply_node]: 
Publishing msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 8'
[INFO] [1666012202.132162620] [mros2_echoreply_node]: 
Subscribed msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 9'
[INFO] [1666012202.132208473] [mros2_echoreply_node]: 
Publishing msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 9'
[INFO] [1666012203.137265544] [mros2_echoreply_node]: 
...(SNIPPED)...
```
serial console on the board
```
...(SNIPPED)...
publishing msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 5'                   
publishing msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 6'                   
[MROS2LIB] subscriber matched with remote publisher                             
[MROS2LIB] publisher matched with remote subscriber                             
publishing msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 7'                   
subscribed msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 7'                   
publishing msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 8'                   
subscribed msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 8'                   
publishing msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 9'                   
subscribed msg: 'Hello from mros2-mbed onto NUCLEO_F767ZI: 9'                   
...(SNIPPED)...
```

## Examples

This repository contains some example applications in [workspace/](workspace/) to communicate with ROS 2 nodes on the host.
You can switch the example by specifying the third argument of `build.bash`.
Of course you can also create a new program file and specify it as your own application.

Please also check [mROS-base/mros2-host-examples](https://github.com/mROS-base/mros2-host-examples) repository for more detail about the host examples.

### echoback_string (default)

- Description:
  - The mROS 2 node on the embedded board publishes `string` (`std_msgs::msg::String`) message to `/to_linux` topic.
  - (The node on the host will echoreply this message as it is.)
  - The mROS 2 node subscribes the replied message from `/to_stm` topic.
- Host operation:
  - `$ ros2 run mros2_echoreply_string echoreply_node`

### echoreply_string

- Description:
  - The mROS 2 node on the embedded board subscribes `string` (`std_msgs::msg::String`) message from `/to_stm` topic.
  - And then publishes this `string` message as it is to `/to_linux` as the reply.
- Host operation:
  - at first terminal: `$ ros2 run mros2_echoback_string sub_node`
  - and then, at second terminal: `$ ros2 run mros2_echoback_string pub_node`
  - or, at one terminal:
    - `$ ros2 launch mros2_echoback_string pubsub.launch.py`

### pub_float32

- Description:
  - The mROS 2 node on the embedded board publishes `float32` (`std_msgs::msg::Float32`) message to `/to_linux` topic.
    - Note that this application just print whether the value of message is less than 0.0, between 0.0 and 1.0, or greater than 1.0.
    - If you want to print float value in serial console, you need to add `"target.printf_lib": "std"` into mbed_app.json (see [detail](https://forums.mbed.com/t/float-printf-doesnt-work-in-desktop-version/9164)). Note that linking std lib will increase the size of Flash memory.
- Host operation:
  - `$ ros2 run mros2_sub_float32 sub_node`
  - or, `$ ros2 launch mros2_sub_float32 sub.launch.py`

### sub_uint16

- Description:
  - The mROS 2 node on the embedded board subscribes `uint16` (`std_msgs::msg::UInt16`) message from `/to_stm` topic.
- Host operation:
  - `$ ros2 run mros2_pub_uint16 pub_node`
  - or, `$ ros2 launch mros2_pub_uint16 pub.launch.py`

### pub_twist

- Description:
  - The mROS 2 node on the embedded board publishes `Twist` (`geometry_msgs::msg::Twist`) message to `/cmd_vel` topic.
  - This application requires to generated header files for `Twist` and `Vector3`. See detail in [<repo_root>/README.md#generating-header-files-for-custom-msgtypes](./README.md#generating-header-files-for-custom-msgtypes).
- Host operation:
  - `$ ros2 run mros2_sub_twist sub_node`
  - or, `$ ros2 launch mros2_sub_twist sub.launch.py`

### sub_pose

- Description:
  - The mROS 2 node on the embedded board subscibes `Pose` (`geometry_msgs::msg::Pose`) message to `/cmd_vel` topic.
  - This application requires to generated header files for `Pose`, `Point` and `Quartenion`. See detail in [<repo_root>/README.md#generating-header-files-for-custom-msgtypes](./README.md#generating-header-files-for-custom-msgtypes).
- Host operation:
  - `$ ros2 run mros2_pub_pose pub_node`
  - or, `$ ros2 launch mros2_pub_pose pub.launch.py`

### mturtle_teleop

- Description:
  - This is a sample application along with [mturtlesim](https://github.com/mROS-base/ros_tutorials/tree/mros2/humble-devel/turtlesim) (mros2 dedicated version of turtlesim).
  - The mROS 2 node on the embedded board publishes `Twist` (`geometry_msgs::msg::Twist`) message to `/turtle1/cmd_vel` topic, according to the input from keyboard via serial console.
- Please see [mturtle_teleop/README.md](workspace/mturtle_teleop/README.md) for more detail including host operation.

### mturtle_teleop_joy

- Description:
  - This is a sample application along with [mturtlesim](https://github.com/mROS-base/ros_tutorials/tree/mros2/humble-devel/turtlesim) (mros2 dedicated version of turtlesim).
  - The mROS 2 node on the embedded board publishes `Twist` (`geometry_msgs::msg::Twist`) message to `/turtle1/cmd_vel` topic, according to the input from Joystick module.
- Please see [mturtle_teleop_joy/README.md](workspace/mturtle_teleop_joy/README.md) for more detail including host operation.

### pub_long_string_sub_crc

- Description:
  - This sample application is an example to demonstrate the fragmented message feature added by the Pull-Request below.
    https://github.com/mROS-base/mros2-mbed/issues/32
  - The mROS 2 node on the embedded board publishes too long `string` (`std_msgs::msg::String`) message prepared in `long_text.txt` to the `/to_linux` topic.
  - (The node on the host will calculate the CRC value of subscribed long string, and publish the value as the reply.)
  - The mROS 2 node subscribes `uint32` (`std_msgs::msg::UInt32`) message as the calculated CRC value from `/to_stm` topic.
- Host operation:
  - `$ ros2 run mros2_sub_long_string_pub_crc sub_long_string_pub_crc_node`

### pub_camera_image

- Description:
  - This sample application is an example to demonstrate the fragmented message feature added by the Pull-Request below.
    https://github.com/mROS-base/mros2-mbed/issues/32
  - The mROS 2 node on the embedded board publishes an `Image` (`sensor_msgs::msg::Image`) message to the `/to_linux` topic.
  - This sample uses the DCMI I/F of STM32 MCU as a camera I/F. The camera module to be used is the prevalence one like below.
    https://papers.ssrn.com/sol3/papers.cfm?abstract_id=4166233
    - If you don't have the camera module but want to try publishing image, you can use `pub_image` sample instead (that publises a dummy image defined in `mros_image.h`).
  - For some NUCLEO boards (e.g., NUCELO-F429ZI), you might have to solder two pin headers on both PA8 and PB7 to connect them to XCLK and VSYNC respectively. The whole of the pin connections between the MCU and the camera module should be below.
    ```
    MCU         Camera module
    3.3V    ---      3V3
    GND     ---      GND
    PB9     ---      SDA
    PB8     ---      SCL
    PB7     ---      VSYNC
    PA4     ---      HSYNC(or HREF)
    PA6     ---      PIXCLK(or PCLK)
    PA8     ---      XCLK
    PE6     ---      D7
    PE5     ---      D6
    PB6     ---      D5
    PE4     ---      D4
    PC9     ---      D3
    PC8     ---      D2
    PC7     ---      D1
    PC6     ---      D0
    PA0     --       Reset
    GND     ---      PWNDN
    ```
    refs: https://www.stmcu.jp/design/document/users_manual/52234/
- Host operation:
  - `$ rqt `
  - And then, subscribe to the topic `/to_linux` and visualize it by `Image View` on the GUI like below.  
    [Movie Sample](https://github-production-user-asset-6210df.s3.amazonaws.com/90823686/243369057-839cf812-eb1d-45bf-820e-e0166253899c.webm)

## Note: File(s) for the application

The main source of the application is `app.cpp`.
You can change and/or add filename by editing `${APP_SRCS}` in `CMakeLists.txt`.

If you have several directories that contain application code files,
you also need to edit `CMakeLists.txt` (see details in comment).

## Generating header files for custom MsgTypes

If you want to use your own customized message type followed by the ROS 2 manner, please refer to [mros2#generating-header-files-for-custom-msgtypes](https://github.com/mROS-base/mros2#generating-header-files-for-custom-msgtypes) section.
(this section was moved because it is common feature for mROS 2).

## Tips 1: Configure the network

`platform/rtps/config.h` is the configuration file for embeddedRTPS.
We may be able to realize the RTPS communication to the appropriate configuration by editting this file.  
For lwIP (UDP/IP) configuration, you may need to edit `mbed_app.json` according to your application requirement.
Note that we have added some cofigurations for mros2 (UDP and RTPS) communication.

We should seek the appropreate configurations or how to fit them to the demand of applications.
Please let us know about them if you have any opinions or awesome knowledges! 

## Tips 2: Development with the latest mros2 repo (a.k.a memo for me)

When using the docker environment for build, it will try to clone `mros2/` dir/repo automatically during the cmake process.
To prevent this, we can simply clone code from GitHub as the development mode and/or add `#` to the beginning of `mros2.lib`.

## Tips 3: Getting started in 5 minutes with the online compiler

> Caution: 
> Although you can easily try out the basic features of mros2 online, this environment has not been fully maintained.
> Note that new features such as uniquely defined message types are not available.

We also provide an online development environment for mros2-mbed. 
By using the following Codes on Keil Studio Cloud (a.k.a Mbed Online Complier), you can try out the power of mros2 just in 5 minute. (we wish :D

- example application:
  - [mbed-os-example-mros2 (echoreply_string)](https://os.mbed.com/users/smoritaemb/code/mbed-os-example-mros2/)
  - [mbed-os-example-mros2-pub-twist (pub-twist)](https://os.mbed.com/users/smoritaemb/code/example-mbed-mros2-pub-twist/)
  - [mbed-os-example-mros2-sub-pose (sub-pose)](https://os.mbed.com/users/smoritaemb/code/example-mbed-mros2-sub-pose/)
- [mbed-mros2 (core library for mros2-mbed)](https://os.mbed.com/users/smoritaemb/code/mbed-mros2/)

Please feel free to let us know in [Issues on this repository](https://github.com/mROS-base/mros2-mbed/issues) if you have any troubles or causes.

## Submodules and Licenses

The source code of this repository itself is published under [Apache License 2.0](https://github.com/mROS-base/mros2/blob/main/LICENSE).  
Please note that this repository requires the following stacks as the submodules, and also check their Licenses.

- [mros2](https://github.com/mROS-base/mros2): the pub/sub APIs compatible with ROS 2 Rclcpp
  - [embeddedRTPS](https://github.com/mROS-base/embeddedRTPS): RTPS communication layer (including lwIP and Micro-CDR)
- [Mbed OS 6](https://github.com/ARMmbed/mbed-os): an open source embedded operating system designed specifically for the "things" in the Internet of Things
