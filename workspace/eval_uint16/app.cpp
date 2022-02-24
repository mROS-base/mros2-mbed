/* mros2 example
 * Copyright (c) 2021 smorita_emb
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

#include "mbed.h"
#include "mros2.h"
#include "std_msgs/msg/u_int16.hpp"
#include "EthernetInterface.h"

#define IP_ADDRESS ("192.168.11.2") /* IP address */
#define SUBNET_MASK ("255.255.255.0") /* Subnet mask */
#define DEFAULT_GATEWAY ("192.168.11.1") /* Default gateway */


mros2::Subscriber sub;
mros2::Publisher pub;

void userCallback(std_msgs::msg::UInt16 *msg)
{
  /*
  printf("subscribed msg: '%d'\r\n", msg->data);
  printf("publishing msg: '%d'\r\n", msg->data);
  */
  pub.publish(*msg);
}

int main() {
  EthernetInterface network;
  network.set_dhcp(false);
  network.set_network(IP_ADDRESS, SUBNET_MASK, DEFAULT_GATEWAY);
  nsapi_size_or_error_t result = network.connect();

  //printf("mbed mros2 start!\r\n");
  mros2::init(0, NULL);
  //MROS2_DEBUG("mROS 2 initialization is completed\r\n");

  mros2::Node node = mros2::Node::create_node("mros2_node");
  pub = node.create_publisher<std_msgs::msg::UInt16>("to_linux", 10);
  sub = node.create_subscription<std_msgs::msg::UInt16>("to_stm", 10, userCallback);

  //MROS2_INFO("ready to pub/sub message\r\n");

  mros2::spin();
  return 0;
}