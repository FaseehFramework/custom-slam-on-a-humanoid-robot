// Deterministic offline pose-graph optimisation over a det-harness output bag.
//
//   pgo_offline <result_bag> <config.yaml> <out_dir>
//
// <result_bag> comes from `RECORD_CLOUDS=1 tools/run_point_lio_det.sh`. It holds
// /aft_mapped_to_init (T_world_body, body = IMU) and /cloud_registered (deskewed, world
// frame), with identical stamps per scan.
//
// Why not pgo_node + `ros2 bag play`: pgo_node runs on a 50 ms WALL timer that keeps the
// front of its queue and discards the rest, so the result depends on playback pacing and
// CPU load. This driver feeds EVERY scan, in stamp order, through exactly the calls
// pgo_node's timerCB makes (addKeyPose -> searchForLoopPairs -> smoothAndUpdate), using
// the same SimplePGO code. It uses no DDS, no timers and no /tf writes.
//
// Each cloud is back-projected into the body frame with its same-stamp pose,
// p_b = R^T (p_w - t). That gives the deskewed body-frame scan at scan end that pgo
// expects. Point-LIO's own /cloud_registered_body is NOT deskewed.
//
// Outputs in <out_dir>:
//   map.pcd           key-pose body clouds placed at their final (optimised) poses
//   keyposes.txt      idx stamp  x y z qw qx qy qz (final)  |  same, odometry (local)
//   loops.txt         target source score  t_target t_source
//   online/           bag, /aft_mapped_to_init in the map frame as pgo_node's TF would
//                     have given it at that moment (causal)
//   final/            bag, /aft_mapped_to_init in the map frame after the last
//                     optimisation (each frame chained to its key pose)
// Both bags read with the existing tools (eval_loop_closure.py etc.) unchanged.
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include "simple_pgo.h"

namespace fs = std::filesystem;
static const char *ODOM = "/aft_mapped_to_init";
static const char *CLOUD = "/cloud_registered";

static int64_t ns(const builtin_interfaces::msg::Time &t) { return int64_t(t.sec) * 1000000000LL + t.nanosec; }

struct Frame {
  nav_msgs::msg::Odometry odom;
  M3D r_local;
  V3D t_local;
  int key_idx;   // last key pose at or before this frame
  M3D r_online;
  V3D t_online;  // map-frame pose with the offset known at this moment
};

static void write_bag(const std::string &uri, const std::vector<Frame> &frames,
                      const std::vector<std::pair<M3D, V3D>> &poses) {
  rosbag2_cpp::Writer w;
  rosbag2_storage::StorageOptions so;
  so.uri = uri;
  so.storage_id = "sqlite3";
  w.open(so, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
  w.create_topic({ODOM, "nav_msgs/msg/Odometry", "cdr", ""});
  for (size_t i = 0; i < frames.size(); ++i) {
    nav_msgs::msg::Odometry m = frames[i].odom;  // twist copied unchanged
    m.header.frame_id = "map";
    Eigen::Quaterniond q(poses[i].first);
    m.pose.pose.position.x = poses[i].second.x();
    m.pose.pose.position.y = poses[i].second.y();
    m.pose.pose.position.z = poses[i].second.z();
    m.pose.pose.orientation.w = q.w();
    m.pose.pose.orientation.x = q.x();
    m.pose.pose.orientation.y = q.y();
    m.pose.pose.orientation.z = q.z();
    w.write(m, ODOM, rclcpp::Time(m.header.stamp));
  }
}

int main(int argc, char **argv) {
  if (argc != 4) {
    std::fprintf(stderr, "usage: %s <result_bag> <config.yaml> <out_dir>\n", argv[0]);
    return 1;
  }
  const std::string in = argv[1], cfg_path = argv[2], out = argv[3];

  YAML::Node y = YAML::LoadFile(cfg_path);
  Config cfg;
  cfg.key_pose_delta_deg = y["key_pose_delta_deg"].as<double>();
  cfg.key_pose_delta_trans = y["key_pose_delta_trans"].as<double>();
  cfg.loop_search_radius = y["loop_search_radius"].as<double>();
  cfg.loop_time_tresh = y["loop_time_tresh"].as<double>();
  cfg.loop_score_tresh = y["loop_score_tresh"].as<double>();
  cfg.loop_submap_half_range = y["loop_submap_half_range"].as<int>();
  cfg.submap_resolution = y["submap_resolution"].as<double>();
  cfg.min_loop_detect_duration = y["min_loop_detect_duration"].as<double>();
  cfg.loop_max_angle_deg = y["loop_max_angle_deg"].as<double>();
  cfg.yaw_only_icp = y["yaw_only_icp"].as<bool>();
  std::printf("[pgo_offline] loop gates: score <= %.3f, |dt| > %.0f s, angle < %.0f deg, %s ICP\n",
              cfg.loop_score_tresh, cfg.loop_time_tresh, cfg.loop_max_angle_deg, cfg.yaw_only_icp ? "4-DOF" : "6-DOF");

  // ---- read: odometry by stamp, clouds in stamp order ----------------------------------
  std::map<int64_t, nav_msgs::msg::Odometry> odoms;
  std::map<int64_t, sensor_msgs::msg::PointCloud2> clouds;
  {
    rosbag2_cpp::Reader r;
    rosbag2_storage::StorageOptions so;
    so.uri = in;
    so.storage_id = "sqlite3";
    r.open(so, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
    rclcpp::Serialization<nav_msgs::msg::Odometry> so_odom;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> so_cloud;
    while (r.has_next()) {
      auto bm = r.read_next();
      rclcpp::SerializedMessage sm(*bm->serialized_data);
      if (bm->topic_name == ODOM) {
        nav_msgs::msg::Odometry m;
        so_odom.deserialize_message(&sm, &m);
        odoms[ns(m.header.stamp)] = m;
      } else if (bm->topic_name == CLOUD) {
        sensor_msgs::msg::PointCloud2 m;
        so_cloud.deserialize_message(&sm, &m);
        clouds[ns(m.header.stamp)] = m;
      }
    }
  }
  std::printf("[pgo_offline] read %zu poses, %zu clouds\n", odoms.size(), clouds.size());
  if (clouds.empty()) {
    std::fprintf(stderr, "[pgo_offline] no %s in bag -- run the harness with RECORD_CLOUDS=1\n", CLOUD);
    return 2;
  }

  // ---- feed every pair through SimplePGO, exactly like pgo_node::timerCB ----------------
  SimplePGO pgo(cfg);
  std::vector<Frame> frames;
  std::ofstream loops_txt;
  fs::create_directories(out);
  loops_txt.open(fs::path(out) / "loops.txt");
  size_t unmatched = 0;
  for (auto &[stamp, cloud_msg] : clouds) {
    auto it = odoms.find(stamp);
    if (it == odoms.end()) { ++unmatched; continue; }
    const auto &o = it->second;
    const auto &qo = o.pose.pose.orientation;
    const auto &po = o.pose.pose.position;
    M3D R = Eigen::Quaterniond(qo.w, qo.x, qo.y, qo.z).normalized().toRotationMatrix();
    V3D t(po.x, po.y, po.z);

    CloudType::Ptr world(new CloudType), body(new CloudType);
    pcl::fromROSMsg(cloud_msg, *world);
    Eigen::Matrix4d T_bw = Eigen::Matrix4d::Identity();   // body <- world
    T_bw.block<3, 3>(0, 0) = R.transpose();
    T_bw.block<3, 1>(0, 3) = -R.transpose() * t;
    pcl::transformPointCloud(*world, *body, T_bw.cast<float>());

    CloudWithPose cp;
    cp.pose.setTime(o.header.stamp.sec, o.header.stamp.nanosec);
    cp.pose.r = R;
    cp.pose.t = t;
    cp.cloud = body;
    if (pgo.addKeyPose(cp)) {
      pgo.searchForLoopPairs();
      for (const auto &lp : pgo.cachePairs()) {
        const auto &kp = pgo.keyPoses();
        loops_txt << lp.target_id << " " << lp.source_id << " " << lp.score << " "
                  << std::fixed << kp[lp.target_id].time << " " << kp[lp.source_id].time << "\n";
        std::printf("[pgo_offline] loop: key %zu (t=%.1f) -> key %zu (t=%.1f), score %.4f\n",
                    lp.source_id, kp[lp.source_id].time, lp.target_id, kp[lp.target_id].time, lp.score);
      }
      pgo.smoothAndUpdate();
    }
    Frame f;
    f.odom = o;
    f.r_local = R;
    f.t_local = t;
    f.key_idx = int(pgo.keyPoses().size()) - 1;
    f.r_online = pgo.offsetR() * R;
    f.t_online = pgo.offsetR() * t + pgo.offsetT();
    frames.push_back(f);
  }
  loops_txt.close();

  // ---- outputs ---------------------------------------------------------------------------
  auto &kps = pgo.keyPoses();
  {
    std::ofstream k(fs::path(out) / "keyposes.txt");
    k << std::fixed;
    for (size_t i = 0; i < kps.size(); ++i) {
      Eigen::Quaterniond qg(kps[i].r_global), ql(kps[i].r_local);
      k << i << " " << kps[i].time << "  "
        << kps[i].t_global.x() << " " << kps[i].t_global.y() << " " << kps[i].t_global.z() << " "
        << qg.w() << " " << qg.x() << " " << qg.y() << " " << qg.z() << "  |  "
        << kps[i].t_local.x() << " " << kps[i].t_local.y() << " " << kps[i].t_local.z() << " "
        << ql.w() << " " << ql.x() << " " << ql.y() << " " << ql.z() << "\n";
    }
  }
  CloudType::Ptr map(new CloudType);
  for (auto &kp : kps) {
    CloudType::Ptr wc(new CloudType);
    pcl::transformPointCloud(*kp.body_cloud, *wc, kp.t_global, Eigen::Quaterniond(kp.r_global));
    *map += *wc;
  }
  pcl::io::savePCDFileBinary((fs::path(out) / "map.pcd").string(), *map);

  std::vector<std::pair<M3D, V3D>> online, final_;
  for (auto &f : frames) {
    online.emplace_back(f.r_online, f.t_online);
    const auto &kp = kps[f.key_idx];   // T_final = T_global(k) * T_local(k)^-1 * T_local(f)
    M3D Rc = kp.r_global * kp.r_local.transpose();
    final_.emplace_back(Rc * f.r_local, Rc * (f.t_local - kp.t_local) + kp.t_global);
  }
  for (const char *d : {"online", "final"}) fs::remove_all(fs::path(out) / d);
  write_bag((fs::path(out) / "online").string(), frames, online);
  write_bag((fs::path(out) / "final").string(), frames, final_);

  Eigen::AngleAxisd aa(pgo.offsetR());
  std::printf("[pgo_offline] frames %zu (unmatched clouds %zu), key poses %zu, loops %zu, map points %zu\n",
              frames.size(), unmatched, kps.size(), pgo.historyPairs().size(), map->size());
  std::printf("[pgo_offline] final map<-odom correction: %.4f m, %.3f deg\n",
              pgo.offsetT().norm(), aa.angle() * 180.0 / M_PI);
  return unmatched ? 3 : 0;
}
