#ifndef MROS2_PLATFORM_H
#define MROS2_PLATFORM_H

#define MROS2_PLATFORM_NAME "mros2-mbed"

#include "mbed.h"
#include "EthernetInterface.h"

/* Comment out the below line enables DHCP connection */
#define MROS2_IP_ADDRESS_STATIC

/* Set network config when using static IP (default usage) */
#ifdef MROS2_IP_ADDRESS_STATIC
#define MROS2_IP_ADDRESS ("192.168.11.2") /* IP address */
#define MROS2_SUBNET_MASK ("255.255.255.0") /* Subnet mask */
#define MROS2_DEFAULT_GATEWAY ("192.168.11.1") /* Default gateway */
#endif /* MROS2_IP_ADDRESS_STATIC */

/* convert TARGET_NAME to put into message */
#define quote(x) std::string(q(x))
#define q(x) #x


namespace mros2_platform
{

nsapi_error_t network_connect(void);

}  /* namespace mros2_platform */

#endif /* MROS2_PLATFORM_H */
