
#include "std_msgs/msg/u_int16.hpp"


template mros2::Publisher mros2::Node::create_publisher<std_msgs::msg::UInt16>(std::string topic_name, int qos);
template void mros2::Publisher::publish(std_msgs::msg::UInt16 &msg);



template mros2::Subscriber mros2::Node::create_subscription(std::string topic_name, int qos, void (*fp)(std_msgs::msg::UInt16*));
template void mros2::Subscriber::callback_handler<std_msgs::msg::UInt16>(void *callee, const rtps::ReaderCacheChange &cacheChange);
