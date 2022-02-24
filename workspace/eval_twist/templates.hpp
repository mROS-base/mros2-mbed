
#include "geometry_msgs/msg/twist.hpp"


template mros2::Publisher mros2::Node::create_publisher<geometry_msgs::msg::Twist>(std::string topic_name, int qos);
template void mros2::Publisher::publish(geometry_msgs::msg::Twist &msg);



template mros2::Subscriber mros2::Node::create_subscription(std::string topic_name, int qos, void (*fp)(geometry_msgs::msg::Twist*));
template void mros2::Subscriber::callback_handler<geometry_msgs::msg::Twist>(void *callee, const rtps::ReaderCacheChange &cacheChange);
