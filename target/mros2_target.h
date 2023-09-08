#ifndef MROS2_TARGET_H
#define MROS2_TARGET_H

#include "mbed.h"
#include "EthernetInterface.h"

#define IP_ADDRESS ("192.168.11.2") /* IP address */
#define SUBNET_MASK ("255.255.255.0") /* Subnet mask */
#define DEFAULT_GATEWAY ("192.168.11.1") /* Default gateway */

/* convert TARGET_NAME to put into message */
#define quote(x) std::string(q(x))
#define q(x) #x

namespace mros2_target
{

nsapi_error_t network_connect(void);

}  /* namespace mros2_target */

#endif /* MROS2_TARGET_H */