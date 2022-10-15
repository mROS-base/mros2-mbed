# mturtle_teleop

This is a sample application along with [mturtlesim](https://github.com/mROS-base/ros_tutorials/tree/mros2/humble-devel/turtlesim) (mros2 dedicated version of turtlesim).

The mROS 2 node on the embedded board publishes `Twist` (`geometry_msgs::msg::Twist`) message to `/turtle1/cmd_vel` topic, according to the input from keyboard via serial console.
The feature is almost the same as [ros2/teleop_twist_keyboard](https://github.com/ros2/teleop_twist_keyboard).

## Generate header files for Twist

See detail in [<repo_root>/README.md#generating-header-files-for-custom-msgtypes](../../README.md#generating-header-files-for-custom-msgtypes).

## Build and Run for embedded devices

Make sure to set `app=mturtle_teleop` as `make` option.

```
### build the application
$ pwd
<snip.>/mros2-mbed
$ ./build.bash all NUCLEO_F767ZI mturtle_teleop
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

keymap to move arround:
------------------
   u    i    o
   j    k    l
   m    ,    .
------------------
q/z : increase/decrease max speeds by 10 percent
w/x : increase/decrease only linear speed by 10 percent
e/c : increase/decrease only angular speed by 10 percent
currently: speed 0.5000 / turn 1.0000
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
