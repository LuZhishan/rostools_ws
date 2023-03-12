#include "iostream"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// #include <unistd.h> 

class Sub_Pub : public rclcpp::Node
{
public:
    Sub_Pub():Node("sub_pub")
    {
        this->declare_parameter<std::string>("sub_topic_name", "/rslidar_points");
        this->declare_parameter<std::string>("pub_topic_name", "/rslidar");
        this->get_parameter("sub_topic_name", sub_topic_name);
        this->get_parameter("pub_topic_name", pub_topic_name);

        this->declare_parameter<bool>("use_topic_time", true);
        this->get_parameter("use_topic_time", use_topic_time);

        this->declare_parameter<std::string>("sub_qos_param", "reliable");
        this->declare_parameter<std::string>("pub_qos_param", "best_effort");
        this->get_parameter("sub_qos_param", sub_qos_param);
        this->get_parameter("pub_qos_param", pub_qos_param);
        

        if(sub_qos_param == "best_effort")
        {
            sub_qos.best_effort();
        }
        else if(sub_qos_param == "reliable")
        {
            sub_qos.reliable();
        }
        if(pub_qos_param == "best_effort")
        {
            pub_qos.best_effort();
        }
        else if(sub_qos_param == "reliable")
        {
            pub_qos.reliable();
        }

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        sub_topic_name, 
        sub_qos,
        std::bind(&Sub_Pub::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_topic_name, pub_qos);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2 & msg)
    {
        RCLCPP_INFO(this->get_logger(), "Doing !");
        sensor_msgs::msg::PointCloud2 new_msg = msg;
        if(!use_topic_time)
        {
            new_msg.header.stamp = get_clock()->now();
        }
        new_msg.header.frame_id = msg.header.frame_id;
        publisher_->publish(new_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    std::string sub_topic_name;
    std::string pub_topic_name;

    bool use_topic_time;
    
    std::string sub_qos_param, pub_qos_param;
    rclcpp::QoS sub_qos = rclcpp::QoS(10).best_effort();
    rclcpp::QoS pub_qos = rclcpp::QoS(10).reliable();
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sub_Pub>());
    rclcpp::shutdown();
    return 0;
}