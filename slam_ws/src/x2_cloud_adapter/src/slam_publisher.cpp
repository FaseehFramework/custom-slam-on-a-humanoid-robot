// Publish our SLAM estimate into the vendor's vacant integration slot.
//
// soc0_task_manager and the drp recorder subscribe to /slam/localization/odometry,
// /slam/mapping/odometry and /slam/lidar_odom; with the no_nav profile active nothing
// publishes them (measured: Publisher count 0, Subscription count 1 on all three).
//
// Keeping this a separate relay rather than patching Point-LIO means the vendor
// integration is testable offline, survives an upstream bump, and keeps our patch small.
//
// MEASURED on the robot 2026-10-12: every subscriber is BEST_EFFORT / VOLATILE. We
// publish RELIABLE + TRANSIENT_LOCAL because that is the strictly most compatible
// writer -- a RELIABLE writer also satisfies BEST_EFFORT readers, and a TRANSIENT_LOCAL
// writer also satisfies VOLATILE readers, so this matches today's subscribers and any
// stricter one that appears later. Both are overridable.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <string>
#include <vector>

using nav_msgs::msg::Odometry;

class SlamPublisher : public rclcpp::Node
{
public:
    SlamPublisher() : rclcpp::Node("slam_publisher")
    {
        in_topic_   = declare_parameter<std::string>("in_topic", "/aft_mapped_to_init");
        loc_topic_  = declare_parameter<std::string>("localization_topic", "/slam/localization/odometry");
        map_topic_  = declare_parameter<std::string>("mapping_topic", "/slam/mapping/odometry");
        lid_topic_  = declare_parameter<std::string>("lidar_odom_topic", "/slam/lidar_odom");

        // empty = pass the incoming frame through unchanged
        header_frame_ = declare_parameter<std::string>("header_frame_id", "");
        child_frame_  = declare_parameter<std::string>("child_frame_id", "");

        publish_tf_   = declare_parameter<bool>("publish_tf", false);
        reliable_     = declare_parameter<bool>("reliable", true);
        transient_    = declare_parameter<bool>("transient_local", true);
        const int depth = declare_parameter<int>("depth", 50);

        rclcpp::QoS qos{rclcpp::KeepLast(depth)};
        reliable_ ? qos.reliable() : qos.best_effort();
        transient_ ? qos.transient_local() : qos.durability_volatile();

        pub_loc_ = create_publisher<Odometry>(loc_topic_, qos);
        pub_map_ = create_publisher<Odometry>(map_topic_, qos);
        pub_lid_ = create_publisher<Odometry>(lid_topic_, qos);

        if (publish_tf_) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Point-LIO publishes /aft_mapped_to_init RELIABLE; a BEST_EFFORT reader would
        // match it too, but RELIABLE here means we do not silently drop poses locally.
        sub_ = create_subscription<Odometry>(
                in_topic_, rclcpp::QoS(rclcpp::KeepLast(200)).reliable(),
                std::bind(&SlamPublisher::cb, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "[slam_pub] %s -> {%s, %s, %s}",
                    in_topic_.c_str(), loc_topic_.c_str(), map_topic_.c_str(), lid_topic_.c_str());
        RCLCPP_INFO(get_logger(), "[slam_pub] qos %s/%s depth %d, tf %s",
                    reliable_ ? "RELIABLE" : "BEST_EFFORT",
                    transient_ ? "TRANSIENT_LOCAL" : "VOLATILE",
                    depth, publish_tf_ ? "on" : "off");
    }

private:
    void cb(const Odometry::SharedPtr msg)
    {
        Odometry out = *msg;
        if (!header_frame_.empty()) out.header.frame_id = header_frame_;
        if (!child_frame_.empty())  out.child_frame_id  = child_frame_;

        // Same estimate on all three. The vendor's stack distinguishes localisation
        // (pose in a prior map) from mapping (pose while building one); we have a
        // single estimate and no prior map, so they are identical by construction.
        // Revisit when relocalisation against a saved map exists.
        pub_loc_->publish(out);
        pub_map_->publish(out);
        pub_lid_->publish(out);

        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped t;
            t.header = out.header;
            t.child_frame_id = out.child_frame_id;
            t.transform.translation.x = out.pose.pose.position.x;
            t.transform.translation.y = out.pose.pose.position.y;
            t.transform.translation.z = out.pose.pose.position.z;
            t.transform.rotation = out.pose.pose.orientation;
            tf_->sendTransform(t);
        }

        if (++count_ % 200 == 0) {
            RCLCPP_INFO(get_logger(), "[slam_pub] relayed %zu poses (last frame %s -> %s)",
                        count_, out.header.frame_id.c_str(), out.child_frame_id.c_str());
        }
    }

    std::string in_topic_, loc_topic_, map_topic_, lid_topic_, header_frame_, child_frame_;
    bool publish_tf_{false}, reliable_{true}, transient_{true};
    size_t count_{0};
    rclcpp::Subscription<Odometry>::SharedPtr sub_;
    rclcpp::Publisher<Odometry>::SharedPtr pub_loc_, pub_map_, pub_lid_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamPublisher>());
    rclcpp::shutdown();
    return 0;
}
