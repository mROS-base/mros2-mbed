# mturtle_teleop_joy

This is a sample application along with [mturtlesim](https://github.com/mROS-base/ros_tutorials/tree/mros2/humble-devel/turtlesim) (mros2 dedicated version of turtlesim).

The mROS 2 node on the embedded board publishes `Twist` (`geometry_msgs::msg::Twist`) message to `/turtle1/cmd_vel` topic, according to the input from Joystick module.
You can also enter the console mode to hit (w|x|a|d|s) key.

## Hardware modules

This application requires the Joystick module which is analog input. Here is the list we used for now.

- [STM32 NUCLEO-F767ZI](https://www.st.com/en/evaluation-tools/nucleo-f767zi.html)
- [Seeed Base Shiled V2](https://wiki.seeedstudio.com/Base_Shield_V2/): connect to Arduino compatible header on the board
- [Grove - Thumb Joystick](https://wiki.seeedstudio.com/Grove-Thumb_Joystick/): connect to A0 conector on Base Shield

We think this application will work well on other boards that supports the Mbed environment and has an Ethernet port.
Also, if it is an analog input, I think it will work with other Joystick modules and other connection methods.

## Generate header files for Twist

See detail in [<repo_root>/README.md#generating-header-files-for-custom-msgtypes](../../README.md#generating-header-files-for-custom-msgtypes).

## Build and Run for embedded devices

Make sure to set `app=mturtle_teleop_joy` as `make` option.

```
### build the application
$ pwd
<snip.>/mros2-mbed
$ ./build.bash all NUCLEO_F767ZI mturtle_teleop_joy
<snip.>
Total Static RAM memory (data + bss): 73880(+73880) bytes
Total Flash memory (text + data): 355652(+355652) bytes

### connect the board and then copy the binary manually.
$ cp cmake_build/NUCLEO_F767ZI/develop/GCC_ARM/mros2-mbed.bin /media/${USER}/NODE_F767ZI/
```

## Host operation for native ROS 2

Since the current version of does not support the QoS control, the original version of `turtlesim` could not work well with this application. So please use the following repository (that is customized for mros2) to clone repository and build package.

```
$ cd <your_ros2_ws>/src
# When you use Foxy as the host, please change the branch to `mros2/foxy-devel`
$ git clone -b mros2/humble-devel https://github.com/mROS-base/ros_tutorials
$ cd ..
$ colcon build --packages-select mturtlesim
$ source install/local_setup.bash
```

## Expected output

### demo movie (on Twitter)

https://twitter.com/takasehideki/status/1505066116921524228

### serial console for the baord with mros2

```
$ picocom -b 115200 /dev/ttyACM0
<snip.>
mbed mros2 start!
app name: mturtle_teleop_joy
[MROS2LIB] mros2_init task start
mROS 2 initialization is completed

[MROS2LIB] create_node
[MROS2LIB] start creating participant
[MROS2LIB] successfully created participant
[MROS2LIB] create_publisher complete.
[MROS2LIB] Initilizing Domain complete
ready to pub/sub message

publish Twist msg to mturtlesim according to the input from Joystick module
to the enter console mode, hit (w|x|a|d|s) key
[keymap in cosole mode]
  w/x: go forward/back
  a/d: turn left/right
  s: stop
  q: quit console mode and return to Joystick mode
```

### terminal console on the host

```
$ ros2 run mturtlesim turtlesim_node
<snip.>
[INFO] [1647589190.956007464] [mturtlesim]: Starting mturtlesim with node name /mturtlesim
[INFO] [1647589190.957948401] [mturtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
[INFO] [1647589195.831260526] [mturtlesim]: subscribed Twist msg from mros2
[INFO] [1647589196.422450459] [mturtlesim]: subscribed Twist msg from mros2
[INFO] [1647589196.934933835] [mturtlesim]: subscribed Twist msg from mros2
<cont.>
```
