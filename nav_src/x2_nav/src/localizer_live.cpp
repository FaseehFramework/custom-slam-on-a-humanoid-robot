// Live relocalisation node. It runs the SAME code as localizer_offline (reloc_core.hpp).
//
// Subscribes Point-LIO's /aft_mapped_to_init + /cloud_registered and pairs them by stamp.
// At startup it runs the --local search around the `init` prior (x, y, z, yaw_deg; the
// operator puts the robot "about" on the home spot), using the first `local_frames` scans.
// Then it tracks. Callbacks only queue frames; one worker thread processes every frame in
// order. So without message loss the output equals `localizer_offline --local` on a
// recording of the same topics (verified on the laptop with bag play).
//
// Publishes:
//   /x2/localization/odometry  nav_msgs/Odometry, frame "map": offset * T_odom for EVERY
//                              odometry message once localised; nothing before
//   /x2/localization/status    std_msgs/String, human-readable state
// It NEVER writes /tf and never commands anything: a passive observer like Point-LIO.
//
// Start the BINARY directly and stop it with SIGINT to that process (pkill -INT -x
// localizer_live). Through `ros2 run`, SIGINT reaches only the python wrapper and the node
// keeps running (seen 2026-10-15).
//   ros2 run x2_nav localizer_live --ros-args -p map_path:=<map.pcd> \
//        -p config_path:=<localizer_x2.yaml> -p init:="[0.0,0.0,0.0,0.0]" -p log_path:=<csv>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

#include "reloc_core.hpp"

using namespace x2reloc;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

static int64_t ns(const builtin_interfaces::msg::Time &t) { return int64_t(t.sec) * 1000000000LL + t.nanosec; }

struct Frame {
  Odometry odom;
  M4D T_odom;
  CloudType::Ptr world;
};

class LocalizerLive : public rclcpp::Node {
public:
  LocalizerLive() : Node("x2_localizer_live") {
    map_path_ = declare_parameter<std::string>("map_path", "");
    cfg_path_ = declare_parameter<std::string>("config_path", "");
    init_ = declare_parameter<std::vector<double>>("init", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    log_path_ = declare_parameter<std::string>("log_path", "");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    if (init_.size() != 4) throw std::runtime_error("init must be [x, y, z, yaw_deg]");

    P_ = Params::load(cfg_path_);
    YAML::Node y = YAML::LoadFile(cfg_path_);
    N_ = y["local_frames"].as<int>();
    min_inl_ = y["local_min_inliers"].as<double>();
    seeds_ = local_seeds(init_[0], init_[1], init_[3], y["local_radius"].as<double>(), y["local_xy_step"].as<double>(),
                         y["local_yaw_range_deg"].as<double>(), y["local_yaw_step_deg"].as<double>());
    map_.reset(new CloudType);
    if (pcl::io::loadPCDFile(map_path_, *map_) != 0 || map_->empty())
      throw std::runtime_error("cannot load map " + map_path_);
    if (!log_path_.empty()) { log_.open(log_path_); log_ << std::fixed; }
    RCLCPP_INFO(get_logger(), "map %s (%zu pts), prior x %.2f y %.2f yaw %.1f, %zu seeds, %d startup frames",
                map_path_.c_str(), map_->size(), init_[0], init_[1], init_[3], seeds_.size(), N_);

    auto qos_in = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    odom_sub_ = create_subscription<Odometry>(odom_topic, qos_in, [this](Odometry::SharedPtr m) { onOdom(m); });
    cloud_sub_ = create_subscription<PointCloud2>(cloud_topic, qos_in, [this](PointCloud2::SharedPtr m) { onCloud(m); });
    auto qos_out = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pose_pub_ = create_publisher<Odometry>("/x2/localization/odometry", qos_out);
    status_pub_ = create_publisher<std_msgs::msg::String>("/x2/localization/status", qos_out);
    worker_ = std::thread(&LocalizerLive::run, this);
  }
  ~LocalizerLive() override {
    stop_ = true;
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

private:
  // ---- pairing (executor thread) ---------------------------------------------------------
  void onOdom(const Odometry::SharedPtr &m) {
    std::lock_guard<std::mutex> lk(pair_mtx_);
    const int64_t k = ns(m->header.stamp);
    auto it = clouds_.find(k);
    if (it != clouds_.end()) { emit(*m, *it->second); clouds_.erase(it); }
    else odoms_[k] = m;
    purge();
  }
  void onCloud(const PointCloud2::SharedPtr &m) {
    std::lock_guard<std::mutex> lk(pair_mtx_);
    const int64_t k = ns(m->header.stamp);
    auto it = odoms_.find(k);
    if (it != odoms_.end()) { emit(*it->second, *m); odoms_.erase(it); }
    else clouds_[k] = m;
    purge();
  }
  void purge() {   // partners that never came (lost message): count them, keep memory bounded
    while (odoms_.size() > 50) { odoms_.erase(odoms_.begin()); ++unpaired_; }
    while (clouds_.size() > 50) { clouds_.erase(clouds_.begin()); ++unpaired_; }
  }
  void emit(const Odometry &o, const PointCloud2 &c) {
    auto f = std::make_shared<Frame>();
    f->odom = o;
    const auto &q = o.pose.pose.orientation;
    const auto &p = o.pose.pose.position;
    f->T_odom = M4D::Identity();
    f->T_odom.block<3, 3>(0, 0) = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    f->T_odom.block<3, 1>(0, 3) = V3D(p.x, p.y, p.z);
    f->world.reset(new CloudType);
    pcl::fromROSMsg(c, *f->world);
    {
      std::lock_guard<std::mutex> lk(q_mtx_);
      q_.push_back(f);
    }
    cv_.notify_one();
  }

  std::shared_ptr<Frame> pop() {
    std::unique_lock<std::mutex> lk(q_mtx_);
    cv_.wait(lk, [this] { return stop_ || !q_.empty(); });
    if (stop_) return nullptr;
    auto f = q_.front();
    q_.pop_front();
    return f;
  }
  void status(const std::string &s) {
    std_msgs::msg::String m;
    m.data = s;
    status_pub_->publish(m);
  }

  // ---- worker: startup search, then track every frame in order ---------------------------
  void run() {
    std::vector<std::shared_ptr<Frame>> window;
    while (!stop_ && !tracker_) {
      auto f = pop();
      if (!f) return;
      window.push_back(f);
      if (int(window.size()) % 10 == 0) status("SEARCHING: collected " + std::to_string(window.size()) + "/" + std::to_string(N_) + " startup scans");
      if (int(window.size()) < N_) continue;
      CloudType::Ptr sub(new CloudType);
      for (auto &w : window) *sub += *w->world;
      const auto t0 = std::chrono::steady_clock::now();
      auto top = search(map_, sub, seeds_, P_);
      const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      char buf[256];
      for (size_t i = 0; i < top.size() && i < 3; ++i) {
        std::snprintf(buf, sizeof buf, "search #%zu: inliers %.1f %% rms %.3f  x %.2f y %.2f yaw %.1f", i + 1,
                      100 * top[i].inl, top[i].rms, top[i].T(0, 3), top[i].T(1, 3), yaw_deg(top[i].T));
        RCLCPP_INFO(get_logger(), "%s", buf);
        if (log_.is_open()) log_ << "# " << buf << std::endl;
      }
      if (!top.empty() && top[0].inl >= min_inl_) {
        tracker_ = std::make_unique<Tracker>(P_, map_path_, top[0].T);
        std::snprintf(buf, sizeof buf, "LOCALISED after %.1f s search: x %.2f y %.2f yaw %.1f (inliers %.1f %%)", secs,
                      top[0].T(0, 3), top[0].T(1, 3), yaw_deg(top[0].T), 100 * top[0].inl);
        RCLCPP_INFO(get_logger(), "%s", buf);
        status(buf);
        for (auto &w : window) track(*w);   // the window frames too, as the offline driver does
      } else {
        std::snprintf(buf, sizeof buf, "SEARCH FAILED (best inliers %.1f %% < %.0f %%) -- retrying on the next %d scans",
                      top.empty() ? 0.0 : 100 * top[0].inl, 100 * min_inl_, N_);
        RCLCPP_WARN(get_logger(), "%s", buf);
        status(buf);
      }
      window.clear();
    }
    while (!stop_) {
      auto f = pop();
      if (!f) return;
      track(*f);
    }
  }

  void track(const Frame &f) {
    auto a = tracker_->step(f.T_odom, f.world);
    const M4D T = tracker_->offset() * f.T_odom;
    Odometry m = f.odom;
    m.header.frame_id = "map";
    Eigen::Quaterniond q(M3D(T.block<3, 3>(0, 0)));
    m.pose.pose.position.x = T(0, 3);
    m.pose.pose.position.y = T(1, 3);
    m.pose.pose.position.z = T(2, 3);
    m.pose.pose.orientation.w = q.w();
    m.pose.pose.orientation.x = q.x();
    m.pose.pose.orientation.y = q.y();
    m.pose.pose.orientation.z = q.z();
    pose_pub_->publish(m);
    ++frames_;
    if (a.tried) {
      ++tries_;
      oks_ += a.ok;
      if (log_.is_open())
        log_ << f.odom.header.stamp.sec + f.odom.header.stamp.nanosec * 1e-9 << " " << a.ok << " " << a.rough << " "
             << a.refine << " " << a.jump_m << " " << a.jump_deg << std::endl;   // flush: survive a hard kill
      char buf[256];
      std::snprintf(buf, sizeof buf, "TRACKING frames %zu, ICP %zu/%zu accepted, last %s refine %.4f jump %.3f m, "
                    "queue %zu, unpaired %zu", frames_, oks_, tries_, a.ok ? "OK" : "REJECTED", a.refine, a.jump_m,
                    qsize(), unpaired_.load());
      status(buf);
      if (tries_ % 10 == 1) RCLCPP_INFO(get_logger(), "%s", buf);   // ~every 10 s, for live_slam.sh --status
    }
  }
  size_t qsize() { std::lock_guard<std::mutex> lk(q_mtx_); return q_.size(); }

  std::string map_path_, cfg_path_, log_path_;
  std::vector<double> init_;
  Params P_;
  int N_ = 30;
  double min_inl_ = 0.6;
  std::vector<Seed> seeds_;
  CloudType::Ptr map_;
  std::unique_ptr<Tracker> tracker_;
  std::ofstream log_;

  std::mutex pair_mtx_;
  std::map<int64_t, Odometry::SharedPtr> odoms_;
  std::map<int64_t, PointCloud2::SharedPtr> clouds_;
  std::atomic<size_t> unpaired_{0};

  std::mutex q_mtx_;
  std::condition_variable cv_;
  std::deque<std::shared_ptr<Frame>> q_;
  std::atomic<bool> stop_{false};
  std::thread worker_;
  size_t frames_ = 0, tries_ = 0, oks_ = 0;

  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char **argv) {
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalizerLive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
