#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


class PointCloudToLaserScanNode : public rclcpp::Node
{
public:
    PointCloudToLaserScanNode() : Node("pointcloud_to_laserscan_node")
    {
        // Parameters
        this->declare_parameter("min_height", -0.5);
        this->declare_parameter("max_height", 0.5);
        this->declare_parameter("angle_min", -M_PI);
        this->declare_parameter("angle_max", M_PI);
        this->declare_parameter("angle_increment", 0.0174533); // 1 degree
        this->declare_parameter("scan_time", 0.1);
        this->declare_parameter("range_min", 0.1);
        this->declare_parameter("range_max", 50.0);
        this->declare_parameter("use_inf", true);
        this->declare_parameter("inf_epsilon", 1.0);
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("input_topic", "/unilidar/cloud");
        this->declare_parameter("output_topic", "/scan");

        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();
        angle_min_ = this->get_parameter("angle_min").as_double();
        angle_max_ = this->get_parameter("angle_max").as_double();
        angle_increment_ = this->get_parameter("angle_increment").as_double();
        scan_time_ = this->get_parameter("scan_time").as_double();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();
        use_inf_ = this->get_parameter("use_inf").as_bool();
        inf_epsilon_ = this->get_parameter("inf_epsilon").as_double();
        target_frame_ = this->get_parameter("target_frame").as_string();
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();

        // Calculate number of points in scan
        num_points_ = static_cast<int>((angle_max_ - angle_min_) / angle_increment_) + 1;

        // Publishers and subscribers
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, std::bind(&PointCloudToLaserScanNode::cloudCallback, this, std::placeholders::_1));
        
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);



        RCLCPP_INFO(this->get_logger(), "PointCloud to LaserScan converter initialized");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        // Convert ROS message to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // Note: Frame transformation is disabled for now to simplify the setup
        // The point cloud will be used as-is from the LiDAR frame

        // Filter points by height
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_height_, max_height_);
        pass.filter(*filtered_cloud);

        // Create laser scan message
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = cloud_msg->header.stamp;
        scan_msg.header.frame_id = target_frame_;
        scan_msg.angle_min = angle_min_;
        scan_msg.angle_max = angle_max_;
        scan_msg.angle_increment = angle_increment_;
        scan_msg.time_increment = scan_time_ / num_points_;
        scan_msg.scan_time = scan_time_;
        scan_msg.range_min = range_min_;
        scan_msg.range_max = range_max_;
        scan_msg.ranges.resize(num_points_);

        // Initialize ranges with infinity
        for (int i = 0; i < num_points_; ++i) {
            scan_msg.ranges[i] = use_inf_ ? std::numeric_limits<float>::infinity() : range_max_;
        }

        // Convert 3D points to 2D scan
        for (const auto& point : filtered_cloud->points) {
            // Calculate distance and angle
            double range = std::sqrt(point.x * point.x + point.y * point.y);
            double angle = std::atan2(point.y, point.x);

            // Check if point is within range limits
            if (range < range_min_ || range > range_max_) {
                continue;
            }

            // Find the closest scan angle
            int index = static_cast<int>((angle - angle_min_) / angle_increment_ + 0.5);
            if (index >= 0 && index < num_points_) {
                // Update range if this point is closer
                if (range < scan_msg.ranges[index] || scan_msg.ranges[index] == std::numeric_limits<float>::infinity()) {
                    scan_msg.ranges[index] = static_cast<float>(range);
                }
            }
        }

        // Handle infinity values
        if (use_inf_) {
            for (int i = 0; i < num_points_; ++i) {
                if (std::isinf(scan_msg.ranges[i])) {
                    scan_msg.ranges[i] = range_max_ + inf_epsilon_;
                }
            }
        }

        // Publish scan
        scan_pub_->publish(scan_msg);
    }

    // Parameters
    double min_height_, max_height_;
    double angle_min_, angle_max_, angle_increment_;
    double scan_time_;
    double range_min_, range_max_;
    bool use_inf_;
    double inf_epsilon_;
    std::string target_frame_;
    std::string input_topic_;
    std::string output_topic_;
    int num_points_;

    // ROS objects
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToLaserScanNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 