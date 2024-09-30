#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rslidar_ros {
  struct EIGEN_ALIGN16 Point {
      float x;
      float y;
      float z;
      float intensity;
      uint16_t ring = 0;
      double timestamp = 0;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace rslidar_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

namespace cartographer {
  struct EIGEN_ALIGN16 PointXYZIT {
      float x;
      float y;
      float z;
      float intensity;
      float time;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace cartographer
POINT_CLOUD_REGISTER_POINT_STRUCT(cartographer::PointXYZIT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
)

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;

bool time_list(cartographer::PointXYZIT &x, cartographer::PointXYZIT &y) {
  return (x.time < y.time);
};

void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<rslidar_ros::Point> pl_orig;
  pcl::PointCloud<cartographer::PointXYZIT> pl_out;
  pcl::fromROSMsg(*msg, pl_orig);
  size_t plsize = pl_orig.points.size();
  pl_out.reserve(plsize);
  double msg_stamp_orign = rclcpp::Time(msg->header.stamp).seconds();
  double scan_start_time = msg_stamp_orign - 0.1;
  for (const auto& point : pl_orig) {
    cartographer::PointXYZIT added_pt;
    added_pt.x = point.x;
    added_pt.y = point.y;
    added_pt.z = point.z;
    added_pt.intensity = point.intensity;
    // In Cartographer, point.time refer to scan duration form scan start time to point scan time.
    added_pt.time = static_cast<float>(point.timestamp - scan_start_time); 
    if (added_pt.time > 0.0 && msg_stamp_orign > point.timestamp) {
      pl_out.points.push_back(added_pt);
    } 
  }

  std::sort(pl_out.points.begin(), pl_out.points.end(), time_list);

  sensor_msgs::msg::PointCloud2 pl_out_msg;
  pcl::toROSMsg(pl_out, pl_out_msg);
  pl_out_msg.header = msg->header;
  // In Cartographer, pointcloud timestamp = msg.header.stamp + point_cloud.points.back().time
  // rslidar's msg timestamp is already the scan end time.
  double msg_stamp_out = msg_stamp_orign - pl_out.points.back().time;
  int32_t sec = std::floor(msg_stamp_out);
  auto nanosec_d = (msg_stamp_out - sec) * 1e9;
  uint32_t nanosec = nanosec_d;
  pl_out_msg.header.stamp = rclcpp::Time(sec, nanosec);
  
  publisher->publish(pl_out_msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rslidar_conversion_node");
  auto subscriber = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/rslidar_points", 10, lidar_callback);
    publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/rslidar_points_converted", 10);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
