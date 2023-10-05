/* mros2 example
 * Copyright (c) 2022 smorita_emb
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
#include "sensor_msgs/msg/image.hpp"

#include "mros_image.h"

int main(int argc, char *argv[])
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
  MROS2_INFO("app name: pub_image");

  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed");

  mros2::Node node = mros2::Node::create_node("mros2_node");
  mros2::Publisher pub = node.create_publisher<sensor_msgs::msg::Image>("to_linux", 10);

  osDelay(100);
  MROS2_INFO("ready to pub image\r\n---");

  auto msg = sensor_msgs::msg::Image();

  while (1)
  {
    msg.sec = time(NULL);
    msg.nanosec = 0;
    msg.frame_id = "frame";
    msg.height = MROS_IMAGE_HEIGHT;
    msg.width = MROS_IMAGE_WIDTH;
    msg.encoding = "rgb8";
    msg.is_bigendian = 0;
    msg.step = MROS_IMAGE_WIDTH * 3;
    size_t image_size = sizeof(mros_image);
    msg.data.resize(image_size);
    std::memcpy(reinterpret_cast<char *>(&msg.data[0]),
                mros_image, image_size);

    MROS2_INFO("publishing image");
    pub.publish(msg);

    osDelay(1000);
  }

  mros2::spin();
  return 0;
}
