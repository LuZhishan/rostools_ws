#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


class Publisher : public rclcpp::Node
{
public:
    Publisher() : Node("pcd_publisher")
    {
        this->declare_parameter<std::string>("pcd_filename", "123.pcd");
        this->get_parameter("pcd_filename", pcd_filename_);
        this->declare_parameter<std::string>("topic_name", "/sensing/lidar/top/rectified/pointcloud");
        this->get_parameter("topic_name", topic_name_);
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->get_parameter("frame_id", frame_id_);

        this->declare_parameter<std::string>("qos_param", "best_effort");
        this->get_parameter("qos_param", qos_param);

        if (qos_param == "best_effort")
        {
            my_qos.best_effort();
        }
        else if (qos_param == "reliable")
        {
            my_qos.reliable();
        }

        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_filename_, cloud) == -1)
        {
            std::cout << "NO such pcd file" << std::endl;
            rclcpp::shutdown();
        }
        pcl::toROSMsg(cloud, output);
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, my_qos);
    }

private:
    void timer_callback()
    {
        output.header.frame_id = frame_id_;
        output.header.stamp = rclcpp::Clock().now();
        RCLCPP_INFO(this->get_logger(), "Publishing PCD");
        publisher_->publish(output);
    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    std::string pcd_filename_;
    std::string topic_name_, frame_id_;
    sensor_msgs::msg::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    std::string qos_param;
    rclcpp::QoS my_qos = rclcpp::QoS(10).best_effort();
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}
