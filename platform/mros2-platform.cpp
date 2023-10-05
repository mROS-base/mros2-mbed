/* target dependent procedure for mros2-mbed
 * Copyright (c) 2023 mROS-base
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
#include "mbed.h"


namespace mros2_platform
{

/*
 *  Setup network I/F
 */
nsapi_error_t network_connect(void)
{
  EthernetInterface network;
  nsapi_size_or_error_t result = 0;

#ifdef MROS2_IP_ADDRESS_STATIC
  network.set_dhcp(false);
  network.set_network(MROS2_IP_ADDRESS, MROS2_SUBNET_MASK, MROS2_DEFAULT_GATEWAY);
#else  /* MROS2_IP_ADDRESS_STATIC */
  network.set_dhcp(true);
#endif /* MROS2_IP_ADDRESS_STATIC */
  result = network.connect();

  if (result)
  {
    MROS2_ERROR("Network connection failed: %d", result);
    return result;
  }
  else
  {
    MROS2_DEBUG("Successfully connected to network");
  }

  SocketAddress socketAddress;
  network.get_ip_address(&socketAddress);
  const char* ip_address = socketAddress.get_ip_address();
  MROS2_DEBUG("  IP Address: %s", ip_address);

  /* convert IP address to be used in rtps/config.h */
  std::array<uint8_t, 4> ipaddr;
  sscanf(ip_address, "%hhd.%hhd.%hhd.%hhd", &ipaddr[0], &ipaddr[1], &ipaddr[2], &ipaddr[3]);

  mros2::setIPAddrRTPS(ipaddr);

  return result;
}

}  /* namespace mros2_platform */
