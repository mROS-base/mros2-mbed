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
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

// imported from
// https://github.com/aeldidi/crc32/blob/master/src/crc32.c
uint32_t
crc32_for_byte(uint32_t byte)
{
  const uint32_t polynomial = 0xEDB88320L;
  uint32_t result = byte;
  size_t i = 0;

  for (; i < 8; i++)
  {
    result = (result >> 1) ^ (result & 1) * polynomial;
  }
  return result;
}

uint32_t
crc32(const void *input, size_t size)
{
  const uint8_t *current = static_cast<const uint8_t *>(input);
  uint32_t result = 0xFFFFFFFF;
  size_t i = 0;

  for (; i < size; i++)
  {
    result ^= current[i];
    result = crc32_for_byte(result);
  }

  return ~result;
}

const char long_text[] =
#include "long_text.txt"
    ;
const size_t text_size = sizeof(long_text) / 4;

void userCallback(std_msgs::msg::UInt32 *msg)
{
  if (msg->data == crc32(long_text, text_size))
  {
    MROS2_INFO("CRC is OK: 0x%0lx", msg->data);
  }
  else
  {
    MROS2_INFO("CRC is NG: 0x%0lx", msg->data);
  }
}

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
  MROS2_INFO("app name: pub_long_string_sub_crc");

  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed");

  mros2::Node node = mros2::Node::create_node("mros2_node");
  mros2::Publisher pub = node.create_publisher<std_msgs::msg::String>("to_linux", 1);
  mros2::Subscriber sub = node.create_subscription<std_msgs::msg::UInt32>("to_stm", 10, userCallback);

  osDelay(100);
  MROS2_INFO("ready to pub/sub message\r\n---");

  auto msg = std_msgs::msg::String();

  while (1)
  {
    msg.data.resize(text_size);
    memcpy(reinterpret_cast<char *>(&msg.data[0]),
           long_text, text_size);

    MROS2_INFO("publishing message whose CRC(len=%d) is 0x%0lx",
               msg.data.size(), crc32(long_text, sizeof(long_text) / 4));
    pub.publish(msg);

    osDelay(1000);
  }

  mros2::spin();
  return 0;
}
