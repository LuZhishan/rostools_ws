#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;
using namespace std::chrono_literals;


class TF_Publisher : public rclcpp::Node
{
public:
    TF_Publisher::TF_Publisher() : Node("TF_publisher")
    {
        this->declare_parameter<std::string>("father_frame_id", "base_link");
        this->declare_parameter<std::string>("child_frame_id", "velodyne_top");
        this->get_parameter("father_frame_id", father_frame_id_);
        this->get_parameter("child_frame_id", child_frame_id_);

        this->declare_parameter<bool>("use_topic_time", false);
        this->get_parameter("use_topic_time", use_topic_time_);
        this->declare_parameter<std::string>("topic_name", "/sensing/lidar/top/rectified/pointcloud");
        this->get_parameter("topic_name", topic_name);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        if(use_topic_time_)
        {
            subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic_name, rclcpp::SensorDataQoS().keep_last(1), std::bind(&TF_Publisher::topic_callback, this, _1));
        }
        else
        {
            timer_ = this->create_wall_timer(100ms, std::bind(&TF_Publisher::on_timer, this));
        }
    }

private:

    void topic_callback()
    {
        ;
    }
    void on_timer()
    {
        ;
    }







    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;



    std::string father_frame_id_;
    std::string child_frame_id_;
    bool use_topic_time_;
    std::string topic_name;


};

