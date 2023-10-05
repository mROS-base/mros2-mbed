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

geometry_msgs::msg::Twist set_twist_val(
    double linear_x, double linear_y, double linear_z,
    double angular_x, double angular_y, double angular_z)
{
  geometry_msgs::msg::Vector3 linear;
  geometry_msgs::msg::Vector3 angular;
  geometry_msgs::msg::Twist twist;

  linear.x = linear_x;
  linear.y = linear_y;
  linear.z = linear_z;
  angular.x = angular_x;
  angular.y = angular_y;
  angular.z = angular_z;
  twist.linear = linear;
  twist.angular = angular;

  return twist;
}

std::string double_to_string(double value)
{
  int intpart, fracpart;
  char str[12];
  intpart = (int)value;
  fracpart = (int)((value - intpart) * 10000);
  sprintf(str, "%d.%04d", intpart, fracpart);
  return str;
}

int main()
{
  MROS2_INFO("%s start!", MROS2_PLATFORM_NAME);
  MROS2_INFO("app name: mturtle_teleop");

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

  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed");

  mros2::Node node = mros2::Node::create_node("mturtle_teleop");
  mros2::Publisher pub = node.create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
  osDelay(100);
  MROS2_INFO("ready to pub/sub message\r\n---");

  geometry_msgs::msg::Vector3 linear;
  geometry_msgs::msg::Vector3 angular;
  geometry_msgs::msg::Twist twist;

  pollfh fds[1];
  fds[0].fh = mbed::mbed_file_handle(STDIN_FILENO);
  fds[0].events = POLLIN;

  auto publish_count = 10;
  double speed = 0.5;
  double turn = 1.0;
  while (1)
  {
    if (publish_count < 10)
    {
      publish_count++;
    }
    else
    {
      publish_count = 0;
      MROS2_INFO("keymap to move around:");
      MROS2_INFO("------------------");
      MROS2_INFO("   u    i    o");
      MROS2_INFO("   j    k    l");
      MROS2_INFO("   m    ,    .");
      MROS2_INFO("------------------");
      MROS2_INFO("q/z : increase/decrease max speeds by 10 percent");
      MROS2_INFO("w/x : increase/decrease only linear speed by 10 percent");
      MROS2_INFO("e/c : increase/decrease only angular speed by 10 percent");
      MROS2_INFO("currently: speed %s / turn %s\r\n",
                 double_to_string(speed).c_str(), double_to_string(turn).c_str());
    }

    char c;
    mbed::mbed_file_handle(STDIN_FILENO)->read(&c, 1);
    switch (c)
    {
    /* increase/decrease speeds */
    case 'q':
      speed = speed * 1.1;
      turn = turn * 1.1;
      MROS2_INFO("currently: speed %s / turn %s",
                 double_to_string(speed).c_str(), double_to_string(turn).c_str());
      break;
    case 'z':
      speed = speed * 0.9;
      turn = turn * 0.9;
      MROS2_INFO("currently: speed %s / turn %s",
                 double_to_string(speed).c_str(), double_to_string(turn).c_str());
      break;
    case 'w':
      speed = speed * 1.1;
      MROS2_INFO("currently: speed %s / turn %s",
                 double_to_string(speed).c_str(), double_to_string(turn).c_str());
      break;
    case 'x':
      speed = speed * 0.9;
      MROS2_INFO("currently: speed %s / turn %s",
                 double_to_string(speed).c_str(), double_to_string(turn).c_str());
      break;
    case 'e':
      turn = turn * 1.1;
      MROS2_INFO("currently: speed %s / turn %s",
                 double_to_string(speed).c_str(), double_to_string(turn).c_str());
      break;
    case 'c':
      turn = turn * 0.9;
      MROS2_INFO("currently: speed %s / turn %s",
                 double_to_string(speed).c_str(), double_to_string(turn).c_str());
      break;
    /* set direction */
    case 'u':
      twist = set_twist_val(speed, 0.0, 0.0, 0.0, 0.0, turn);
      MROS2_INFO("publishing Twist msg by 'u' command");
      pub.publish(twist);
      break;
    case 'i':
      twist = set_twist_val(speed, 0.0, 0.0, 0.0, 0.0, 0.0);
      MROS2_INFO("publishing Twist msg by 'i' command");
      pub.publish(twist);
      break;
    case 'o':
      twist = set_twist_val(speed, 0.0, 0.0, 0.0, 0.0, -turn);
      MROS2_INFO("publishing Twist msg by 'o' command");
      pub.publish(twist);
      break;
    case 'j':
      twist = set_twist_val(0.0, 0.0, 0.0, 0.0, 0.0, turn);
      MROS2_INFO("publishing Twist msg by 'j' command");
      pub.publish(twist);
      break;
    case 'k':
      twist = set_twist_val(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      MROS2_INFO("publishing Twist msg by 'k' command");
      pub.publish(twist);
      break;
    case 'l':
      twist = set_twist_val(0.0, 0.0, 0.0, 0.0, 0.0, -turn);
      MROS2_INFO("publishing Twist msg by 'l' command");
      pub.publish(twist);
      break;
    case 'm':
      twist = set_twist_val(-speed, 0.0, 0.0, 0.0, 0.0, -turn);
      MROS2_INFO("publishing Twist msg by 'm' command");
      pub.publish(twist);
      break;
    case ',':
      twist = set_twist_val(-speed, 0.0, 0.0, 0.0, 0.0, 0.0);
      MROS2_INFO("publishing Twist msg by ',' command");
      pub.publish(twist);
      break;
    case '.':
      twist = set_twist_val(-speed, 0.0, 0.0, 0.0, 0.0, turn);
      MROS2_INFO("publishing Twist msg by '.' command");
      pub.publish(twist);
      break;
    default:
      MROS2_INFO("ERROR: wrong command");
      publish_count = 10;
      break;
    }
  }

  mros2::spin();
  return 0;
}
