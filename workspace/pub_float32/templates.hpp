
#include "std_msgs/msg/float32.hpp"


template mros2::Publisher mros2::Node::create_publisher<std_msgs::msg::Float32>(std::string topic_name, int qos);
template void mros2::Publisher::publish(std_msgs::msg::Float32 &msg);


