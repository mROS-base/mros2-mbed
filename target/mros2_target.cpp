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

#include "mros2_target.h"
#include "mbed.h"


namespace mros2_target
{

/*
 *  Setup network I/F
 */
nsapi_error_t network_connect(void)
{
  EthernetInterface network;
  nsapi_size_or_error_t result;

#ifdef MROS2_IP_ADDRESS_STATIC
  network.set_dhcp(false);
  network.set_network(IP_ADDRESS, SUBNET_MASK, DEFAULT_GATEWAY);
#else  /* MROS2_IP_ADDRESS_STATIC */
  network.set_dhcp(true);
#endif /* MROS2_IP_ADDRESS_STATIC */
  result = network.connect();

  if (result) {
    printf("Network connection failed: %d\r\n", result);
    return result;
  } else {
    printf("Successfully connected to network\r\n");
  }

  SocketAddress socketAddress;
  network.get_ip_address(&socketAddress);
  printf("  IP Address: %s\r\n", socketAddress.get_ip_address());
  network.get_netmask(&socketAddress);
  printf("  NETMASK   : %s\r\n", socketAddress.get_ip_address());
  network.get_gateway(&socketAddress);
  printf("  GATEWAY   : %s\r\n", socketAddress.get_ip_address());

  return result;
}

}  /* namespace mros2_target */
