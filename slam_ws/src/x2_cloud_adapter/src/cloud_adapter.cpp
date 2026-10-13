// Converts the X2 Ultra (RoboSense E1R) cloud into the Velodyne-style layout that
// Point-LIO / FAST-LIO consume.
//
// Field offsets are resolved by NAME at runtime rather than hardcoded, so a vendor
// firmware change produces a clear error instead of silently garbled geometry.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace {

std::optional<PointField> findField(const PointCloud2 & m, const std::string & name)
{
  for (const auto & f : m.fields) { if (f.name == name) { return f; } }
  return std::nullopt;
}

// Output layout, matching velodyne_ros::Point (EIGEN_ALIGN16, 32 bytes).
constexpr uint32_t OUT_X = 0, OUT_Y = 4, OUT_Z = 8;
constexpr uint32_t OUT_INTENSITY = 16, OUT_TIME = 20, OUT_RING = 24;
constexpr uint32_t OUT_STEP = 32;

}  // namespace

class CloudAdapter : public rclcpp::Node
{
public:
  CloudAdapter()
  : Node("x2_cloud_adapter")
  {
    in_topic_  = declare_parameter<std::string>(
      "input_topic", "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud");
    out_topic_ = declare_parameter<std::string>("output_topic", "/x2/points");
    // Vendor per-point timestamps are float64 nanoseconds relative to scan start.
    // Point-LIO is configured with timestamp_unit: 0 (seconds), so scale by 1e-9.
    time_scale_ = declare_parameter<double>("time_scale", 1.0e-9);
    frame_override_ = declare_parameter<std::string>("frame_id", "");

    // Subscribe permissively (BEST_EFFORT reader matches the vendor's RELIABLE
    // writer), but PUBLISH deep + RELIABLE: a dropped scan doubles the inter-scan
    // motion, which on a legged platform is how the filter diverges.
    pub_ = create_publisher<PointCloud2>(
      out_topic_, rclcpp::QoS(rclcpp::KeepLast(2000)).reliable());
    sub_ = create_subscription<PointCloud2>(
      in_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CloudAdapter::onCloud, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "x2_cloud_adapter: %s -> %s (time_scale %.3g)",
                in_topic_.c_str(), out_topic_.c_str(), time_scale_);
  }

private:
  void onCloud(const PointCloud2::SharedPtr in)
  {
    const auto fx = findField(*in, "x");
    const auto fy = findField(*in, "y");
    const auto fz = findField(*in, "z");
    const auto fi = findField(*in, "intensity");
    const auto fr = findField(*in, "ring");
    const auto ft = findField(*in, "timestamp");

    if (!fx || !fy || !fz || !fi || !fr || !ft) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "input cloud is missing a required field (need x,y,z,intensity,ring,timestamp)");
      return;
    }
    if (fx->datatype != PointField::FLOAT32 || fi->datatype != PointField::FLOAT32 ||
        fr->datatype != PointField::UINT16 || ft->datatype != PointField::FLOAT64)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "unexpected field datatypes; expected x/y/z/intensity FLOAT32, ring UINT16, "
        "timestamp FLOAT64");
      return;
    }

    const uint32_t n = in->width * in->height;

    PointCloud2 out;
    out.header = in->header;
    if (!frame_override_.empty()) { out.header.frame_id = frame_override_; }
    out.height = 1;
    out.width = n;
    out.is_bigendian = in->is_bigendian;
    out.is_dense = in->is_dense;
    out.point_step = OUT_STEP;
    out.row_step = OUT_STEP * n;
    out.fields.resize(6);
    auto setf = [](PointField & f, const char * nm, uint32_t off, uint8_t dt) {
      f.name = nm; f.offset = off; f.datatype = dt; f.count = 1;
    };
    setf(out.fields[0], "x",         OUT_X,         PointField::FLOAT32);
    setf(out.fields[1], "y",         OUT_Y,         PointField::FLOAT32);
    setf(out.fields[2], "z",         OUT_Z,         PointField::FLOAT32);
    setf(out.fields[3], "intensity", OUT_INTENSITY, PointField::FLOAT32);
    setf(out.fields[4], "time",      OUT_TIME,      PointField::FLOAT32);
    setf(out.fields[5], "ring",      OUT_RING,      PointField::UINT16);
    out.data.assign(static_cast<size_t>(OUT_STEP) * n, 0);

    const uint8_t * src = in->data.data();
    uint8_t * dst = out.data.data();
    const uint32_t istep = in->point_step;

    for (uint32_t i = 0; i < n; ++i) {
      const uint8_t * s = src + static_cast<size_t>(i) * istep;
      uint8_t * d = dst + static_cast<size_t>(i) * OUT_STEP;

      std::memcpy(d + OUT_X,         s + fx->offset, 4);
      std::memcpy(d + OUT_Y,         s + fy->offset, 4);
      std::memcpy(d + OUT_Z,         s + fz->offset, 4);
      std::memcpy(d + OUT_INTENSITY, s + fi->offset, 4);

      double ts_raw;
      std::memcpy(&ts_raw, s + ft->offset, 8);
      const float t = static_cast<float>(ts_raw * time_scale_);
      std::memcpy(d + OUT_TIME, &t, 4);

      uint16_t ring;
      std::memcpy(&ring, s + fr->offset, 2);
      std::memcpy(d + OUT_RING, &ring, 2);

      if (i == n - 1) { last_point_time_ = t; }
      if (ring > max_ring_) { max_ring_ = ring; }
    }

    pub_->publish(out);

    if (++count_ % 100 == 0) {
      // Point-LIO decides given_offset_time from the LAST point's time being > 0.
      RCLCPP_INFO(
        get_logger(), "%zu clouds; %u pts; last-point time %.6f s; max ring %u",
        count_, n, last_point_time_, max_ring_);
    }
  }

  std::string in_topic_, out_topic_, frame_override_;
  double time_scale_{1.0e-9};
  size_t count_{0};
  float last_point_time_{0.f};
  uint16_t max_ring_{0};
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudAdapter>());
  rclcpp::shutdown();
  return 0;
}
