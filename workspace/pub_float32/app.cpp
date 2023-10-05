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
#include "std_msgs/msg/float32.hpp"

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
  MROS2_INFO("app name: pub_float32");

  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed");

  mros2::Node node = mros2::Node::create_node("mros2_node");
  mros2::Publisher pub = node.create_publisher<std_msgs::msg::Float32>("to_linux", 10);
  osDelay(100);
  MROS2_INFO("ready to pub/sub message\r\n---");

  std_msgs::msg::Float32 msg;
  auto publish_count = -0.5;
  while (1)
  {
    msg.data = publish_count;
    MROS2_INFO("publishing float msg!!");
    pub.publish(msg);

    if (0.0 >= msg.data)
      MROS2_INFO("msg <= 0.0");
    else if (0.0 < msg.data && msg.data < 0.5)
      MROS2_INFO("0.0 < msg < 0.5");
    else if (0.5 < msg.data && msg.data < 1.0)
      MROS2_INFO("0.5 < msg < 1.0");
    else
      MROS2_INFO("msg >= 1.0");
    /*
     * you need to add `"target.printf_lib": "std"` into mbed_app.json
     * to print float (%f) data
    printf("publishing msg: '%f'\r\n", msg.data);
     */

    publish_count = publish_count + 0.1;
    osDelay(1000);
  }

  mros2::spin();
  return 0;
}