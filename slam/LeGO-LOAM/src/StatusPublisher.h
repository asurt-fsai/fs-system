#ifndef STATUS_PUBLISHER_H
#define STATUS_PUBLISHER_H

#include <string>
#include <vector>
#include <mutex>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <asurt_msgs/msg/node_status.hpp>

class StatusPublisher
{
public:
    static std::vector<std::string> topicNamesCreated;

    StatusPublisher(const std::string &topicName, rclcpp::Node::SharedPtr nodeObject);

    asurt_msgs::msg::NodeStatus baseMessage();

    void starting();
    void ready();
    void running();
    void error(const std::string &errMsg);

private:
    rclcpp::Node::SharedPtr nodeObject;
    rclcpp::Publisher<asurt_msgs::msg::NodeStatus>::SharedPtr publisher;
    std::mutex lock;
};

#endif // STATUS_PUBLISHER_H

