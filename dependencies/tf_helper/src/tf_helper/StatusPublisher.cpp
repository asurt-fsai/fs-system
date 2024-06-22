/**
 * StatusPublisher class to provide an easy way to publish a heartbeat
 */
#include <string>
#include <vector>
#include <mutex>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <asurt_msgs/msg/node_status.hpp>

class StatusPublisher
{
public:
    /**
     * Class used to publish a heartbeat.
     * Each node should have one and only one StatusPublisher
     * The following are the statuses that can be published:
     *     - Starting
     *     - Ready
     *     - Running
     *     - Error
     * 
     * Parameters
     * ----------
     * topicName: std::string
     *     Name of the topic to publish the heartbeat on
     *     IMPORTANT: must be unique, otherwise an exception will be thrown
     */

    static std::vector<std::string> topicNamesCreated;

    StatusPublisher(const std::string &topicName, rclcpp::Node::SharedPtr nodeObject)
    {
        if (std::find(topicNamesCreated.begin(), topicNamesCreated.end(), topicName) != topicNamesCreated.end())
        {
            throw std::invalid_argument(
                "StatusPublisher: Topic name already exists, "
                "can't publish a heartbeat on the same topic twice"
            );
        }

        auto latchingQOS = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_transient_local)).keep_last(1);
        this->nodeObject = nodeObject;
        this->publisher = this->nodeObject->create_publisher<asurt_msgs::msg::NodeStatus>(topicName, latchingQOS);
        topicNamesCreated.push_back(topicName);
    }

    asurt_msgs::msg::NodeStatus baseMessage()
    {
        /**
         * Creates a base NodeStatus message with a timestamp
         * 
         * Returns
         * -------
         * asurt_msgs::msg::NodeStatus
         *     Base NodeStatus message
         */
        asurt_msgs::msg::NodeStatus msg;
        msg.header.stamp = this->nodeObject->get_clock()->now();
        return msg;
    }

    void starting()
    {
        /**
         * Publishes a NodeStatus message with the state "Starting"
         */
        std::lock_guard<std::mutex> guard(lock);
        auto msg = this->baseMessage();
        msg.status = 0;
        this->publisher->publish(msg);
    }

    void ready()
    {
        /**
         * Publishes a NodeStatus message with the state "Ready"
         */
        std::lock_guard<std::mutex> guard(lock);
        auto msg = this->baseMessage();
        msg.status = 1;
        this->publisher->publish(msg);
    }

    void running()
    {
        /**
         * Publishes a NodeStatus message with the state "Running"
         */
        std::lock_guard<std::mutex> guard(lock);
        auto msg = this->baseMessage();
        msg.status = 2;
        this->publisher->publish(msg);
    }

    void error(const std::string &errMsg)
    {
        /**
         * Publishes a NodeStatus message with the state "Error"
         * 
         * Parameters
         * ----------
         * errMsg: std::string
         *     Error message to include with the NodeStatus message
         */
        std::lock_guard<std::mutex> guard(lock);
        auto msg = this->baseMessage();
        msg.status = 3;
        msg.message = errMsg;
        this->publisher->publish(msg);
    }

private:
    rclcpp::Node::SharedPtr nodeObject;
    rclcpp::Publisher<asurt_msgs::msg::NodeStatus>::SharedPtr publisher;
    std::mutex lock;
};

std::vector<std::string> StatusPublisher::topicNamesCreated = {};
