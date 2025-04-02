#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

class LidarCropNode : public rclcpp::Node
{
public:
  LidarCropNode()
  : Node("lidar_crop_node")
  {
    // Paramètres de filtrage (par défaut : ±10 m en X/Y, illimité en Z)
    this->declare_parameter("x_min", -16.0);
    this->declare_parameter("x_max",  16.0);
    this->declare_parameter("y_min", -16.0);
    this->declare_parameter("y_max",  16.0);
    this->declare_parameter("z_min", -10.0);
    this->declare_parameter("z_max",  10.0);

    x_min_ = this->get_parameter("x_min").as_double();
    x_max_ = this->get_parameter("x_max").as_double();
    y_min_ = this->get_parameter("y_min").as_double();
    y_max_ = this->get_parameter("y_max").as_double();
    z_min_ = this->get_parameter("z_min").as_double();
    z_max_ = this->get_parameter("z_max").as_double();

    RCLCPP_INFO(this->get_logger(),
      "LidarCropNode démarré : Filtrage X=[%.1f, %.1f], Y=[%.1f, %.1f], Z=[%.1f, %.1f]",
      x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);

    // Souscription : topic d'entrée
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_points",  // ou autre nom de topic
      rclcpp::QoS(10),
      std::bind(&LidarCropNode::pointCloudCallback, this, std::placeholders::_1)
    );

    // Publication : topic de sortie filtré
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lidar_points_crop",
      rclcpp::QoS(10)
    );
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convertir le message ROS en nuage PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *input_cloud);

    // Filtre CropBox
    pcl::CropBox<pcl::PointXYZ> crop_filter;
    crop_filter.setMin(Eigen::Vector4f(x_min_, y_min_, z_min_, 1.0f));
    crop_filter.setMax(Eigen::Vector4f(x_max_, y_max_, z_max_, 1.0f));
    crop_filter.setInputCloud(input_cloud);

    // Exécuter le filtrage
    pcl::PointCloud<pcl::PointXYZ> cropped_cloud;
    crop_filter.filter(cropped_cloud);

    // Convertir le nuage filtré en message ROS
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(cropped_cloud, output_msg);
    output_msg.header = msg->header;  // on garde le même frame_id / timestamp

    // Publier
    publisher_->publish(output_msg);
  }

  // Paramètres de la box
  double x_min_, x_max_;
  double y_min_, y_max_;
  double z_min_, z_max_;

  // Abonnement / Publication
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarCropNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
