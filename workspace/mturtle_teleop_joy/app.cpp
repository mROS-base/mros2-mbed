/* mros2 example
 * Copyright (c) 2022 mROS-base
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mros2.h"
#include "mros2-platform.h"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"

AnalogIn inputA0(A0);
AnalogIn inputA1(A1);

#define COEFF_LIN 10.0
#define COEFF_ANG 10.0
#define CONSOLE_LIN 0.5
#define CONSOLE_ANG 1.0

int main()
{
  /* connect to the network */
  if (mros2_platform::network_connect())
  {
    MROS2_ERROR("failed to connect and setup network! aborting,,,");
    return -1;
  }
  else
  {
    MROS2_INFO("successfully connect and setup network\r\n---");
  }

  MROS2_INFO("%s start!", MROS2_PLATFORM_NAME);
  MROS2_INFO("app name: mturtle_teleop_joy");

  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed");

  mros2::Node node = mros2::Node::create_node("mturtle_teleop_joy");
  mros2::Publisher pub = node.create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
  osDelay(100);
  MROS2_INFO("ready to pub/sub message\r\n---");

  geometry_msgs::msg::Vector3 linear;
  geometry_msgs::msg::Vector3 angular;
  geometry_msgs::msg::Twist twist;
  linear.y = 0;
  linear.z = 0;
  angular.x = 0;
  angular.y = 0;

  MROS2_INFO("publish Twist msg to mturtlesim according to the input from Joystick module");
  MROS2_INFO("to the enter console mode, hit (w|x|a|d|s) key");
  MROS2_INFO("[keymap in cosole mode]");
  MROS2_INFO("  w/x: go forward/back");
  MROS2_INFO("  a/d: turn left/right");
  MROS2_INFO("  s: stop");
  MROS2_INFO("  q: quit console mode and return to Joystick mode");
  float initialA = inputA0.read();
  pollfh fds[1];
  fds[0].fh = mbed::mbed_file_handle(STDIN_FILENO);
  fds[0].events = POLLIN;
  bool console_mode = false;

  while (1)
  {
    if (console_mode || poll(fds, 1, 0))
    {
      console_mode = true;
      if (poll(fds, 1, 0))
      {
        char c;
        mbed::mbed_file_handle(STDIN_FILENO)->read(&c, 1);
        switch (c)
        {
        case 'w':
          linear.x = CONSOLE_LIN;
          angular.z = 0.0;
          break;
        case 'x':
          linear.x = -CONSOLE_LIN;
          angular.z = 0.0;
          break;
        case 'a':
          angular.z = CONSOLE_ANG;
          break;
        case 'd':
          angular.z = -CONSOLE_ANG;
          break;
        case 's':
          linear.x = 0.0;
          angular.z = 0.0;
          break;
        case 'q':
          linear.x = 0.0;
          angular.z = 0.0;
          console_mode = false;
          break;
        }
      }
    }
    else
    {
      linear.x = COEFF_LIN * (inputA0.read() - initialA);
      angular.z = COEFF_ANG * (inputA1.read() - initialA);
    }
    twist.linear = linear;
    twist.angular = angular;
    pub.publish(twist);
    osDelay(100);
  }

  mros2::spin();
  return 0;
}
