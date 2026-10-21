// Deterministic offline relocalisation of one walk against a saved map.
//
//   localizer_offline <map.pcd> <result_bag> <config.yaml> <out_dir> --init x,y,z,yaw_deg
//   localizer_offline <map.pcd> <result_bag> <config.yaml> <out_dir> --search
//
// <result_bag>: `RECORD_CLOUDS=1 tools/run_point_lio_det.sh` output of the walk to localise
// ("B"). Both its odometry frame (camera_init_B) and the map frame are gravity-aligned
// (x2_ultra.yaml gravity_align: true), so the unknown T_map<-camera_init_B is 4-DOF.
//
// --init gives T_map<-camera_init_B directly. Upstream's relocalize service takes
//   T_map<-BODY instead; our body frame is pitched ~-85 deg, so an all-zero request starts
//   ICP ~86 deg off.
// --search finds it with no prior. B's first `search_path_m` of motion, accumulated in
//   camera_init_B, is registered against the map from a grid of (x, y, yaw) seeds. The
//   best inlier ratio wins, and the runner-ups are printed so an ambiguous match is
//   visible.
// --local x,y,z,yaw_deg searches a window around a rough prior, using B's first
//   `local_frames` frames (the start stand-still): what localizer_live does at startup.
//   On failure (best inliers < local_min_inliers) it slides to the next window. Tracking
//   then starts at the first frame of the window that succeeded.
//
// Tracking mirrors localizer_node::timerCB, with one ICP attempt every `every_n_frames`
// (10 = the node's 1 Hz at 10 Hz scans):
//   guess = offset * T_odom(f)
//   ICPLocalizer::align -- THEIR class, their two stages and thresholds
//   success: offset = T_map_body * T_odom(f)^-1; failure: keep the old offset.
// The input is the back-projected scan, p_b = R^T (p_w - t), accumulated over the last
// `accum_scans` frames (upstream: 1). No DDS, no timers, no /tf writes.
//
// Outputs in <out_dir>:
//   init.txt      initial T_map<-camera_init_B (+ search ranking)
//   attempts.txt  stamp ok rough_fitness refine_fitness jump_m jump_deg
//   traj/         bag: /aft_mapped_to_init in the map frame = offset-at-that-moment * odom
//                 (causal, what the node's TF gives). Reads with tools/eval_loop_closure.py.
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
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
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

#include "reloc_core.hpp"

namespace fs = std::filesystem;
using namespace x2reloc;
static const char *ODOM = "/aft_mapped_to_init";
static const char *CLOUD = "/cloud_registered";

static int64_t ns(const builtin_interfaces::msg::Time &t) { return int64_t(t.sec) * 1000000000LL + t.nanosec; }

struct Frame {
  nav_msgs::msg::Odometry odom;
  M4D T_odom;              // camera_init_B <- body
  CloudType::Ptr world;    // deskewed scan in camera_init_B
};

static void print_top(const std::vector<Cand> &top, size_t n_seeds, double inl_dist, std::ofstream &f) {
  std::printf("[loc_offline] %zu seeds; best distinct solutions (inliers within %.2f m):\n", n_seeds, inl_dist);
  f << "search seeds " << n_seeds << " inlier_dist " << inl_dist << "\n";
  for (size_t i = 0; i < top.size() && i < 5; ++i) {
    const M4D &T = top[i].T;
    std::printf("   #%zu  inliers %5.1f %%  rms %.3f m   x %.2f y %.2f z %.2f yaw %.1f deg  tilt %.2f deg\n",
                i + 1, 100 * top[i].inl, top[i].rms, T(0, 3), T(1, 3), T(2, 3), yaw_deg(T),
                std::acos(std::min(1.0, T(2, 2))) * 180.0 / M_PI);
    f << "#" << i + 1 << " " << top[i].inl << " " << top[i].rms << " " << T(0, 3) << " " << T(1, 3) << " "
      << T(2, 3) << " " << yaw_deg(T) << "\n";
  }
}

int main(int argc, char **argv) {
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);  // "Not enough correspondences" from far seeds
  if (argc < 6) {
    std::fprintf(stderr, "usage: %s <map.pcd> <result_bag> <config.yaml> <out_dir> "
                         "(--init x,y,z,yaw_deg | --local x,y,z,yaw_deg | --search)\n", argv[0]);
    return 1;
  }
  const std::string map_path = argv[1], in = argv[2], cfg_path = argv[3], out = argv[4], mode = argv[5];
  const Params P = Params::load(cfg_path);
  YAML::Node y = YAML::LoadFile(cfg_path);

  // ---- read B --------------------------------------------------------------------------
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
  std::vector<Frame> frames;
  size_t unmatched = 0;
  for (auto &[stamp, cm] : clouds) {
    auto it = odoms.find(stamp);
    if (it == odoms.end()) { ++unmatched; continue; }
    const auto &q = it->second.pose.pose.orientation;
    const auto &p = it->second.pose.pose.position;
    Frame f;
    f.odom = it->second;
    f.T_odom = M4D::Identity();
    f.T_odom.block<3, 3>(0, 0) = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    f.T_odom.block<3, 1>(0, 3) = V3D(p.x, p.y, p.z);
    f.world.reset(new CloudType);
    pcl::fromROSMsg(cm, *f.world);
    frames.push_back(std::move(f));
  }
  std::printf("[loc_offline] B: %zu frames (unmatched clouds %zu)\n", frames.size(), unmatched);
  if (frames.empty() || unmatched) return 2;

  CloudType::Ptr map(new CloudType);
  if (pcl::io::loadPCDFile(map_path, *map) != 0 || map->empty()) {
    std::fprintf(stderr, "[loc_offline] cannot load map %s\n", map_path.c_str());
    return 2;
  }
  fs::create_directories(out);
  std::ofstream init_txt(fs::path(out) / "init.txt");

  // ---- initial T_map<-camera_init_B ------------------------------------------------------
  M4D offset;
  size_t first = 0;   // first frame tracked (and written)
  double ix, iy, iz, iyaw;
  if ((mode == "--init" || mode == "--local") && argc >= 7) {
    if (std::sscanf(argv[6], "%lf,%lf,%lf,%lf", &ix, &iy, &iz, &iyaw) != 4) {
      std::fprintf(stderr, "%s expects x,y,z,yaw_deg\n", mode.c_str());
      return 1;
    }
  }
  if (mode == "--init" && argc >= 7) {
    offset = yawT(ix, iy, iz, iyaw * M_PI / 180.0);
    init_txt << "given " << ix << " " << iy << " " << iz << " " << iyaw << "\n";
  } else if (mode == "--local" && argc >= 7) {
    const int N = y["local_frames"].as<int>();
    const double min_inl = y["local_min_inliers"].as<double>();
    auto seeds = local_seeds(ix, iy, iyaw, y["local_radius"].as<double>(), y["local_xy_step"].as<double>(),
                             y["local_yaw_range_deg"].as<double>(), y["local_yaw_step_deg"].as<double>());
    bool found = false;
    for (size_t w = 0; w + N <= frames.size(); w += N) {
      CloudType::Ptr sub(new CloudType);
      for (size_t i = w; i < w + N; ++i) *sub += *frames[i].world;
      auto top = search(map, sub, seeds, P);
      std::printf("[loc_offline] local search, frames %zu-%zu:\n", w, w + N - 1);
      print_top(top, seeds.size(), P.search_inlier_dist, init_txt);
      if (!top.empty() && top[0].inl >= min_inl) { offset = top[0].T; first = w; found = true; break; }
      std::printf("[loc_offline] best inliers below %.2f -- next window\n", min_inl);
    }
    if (!found) { std::fprintf(stderr, "[loc_offline] local search never succeeded\n"); return 4; }
  } else if (mode == "--search") {
    CloudType::Ptr sub(new CloudType);
    double path = 0.0;
    size_t n_sub = 0;
    V3D last = frames[0].T_odom.block<3, 1>(0, 3);
    for (auto &f : frames) {
      V3D t = f.T_odom.block<3, 1>(0, 3);
      path += (t - last).norm();
      last = t;
      *sub += *f.world;
      ++n_sub;
      if (path >= P.search_path_m) break;
    }
    std::printf("[loc_offline] search submap: first %zu frames (%.2f m of path)\n", n_sub, path);
    auto seeds = global_seeds(voxel(map, 0.5), P.search_xy_step, P.search_yaw_step_deg);
    auto top = search(map, sub, seeds, P);
    print_top(top, seeds.size(), P.search_inlier_dist, init_txt);
    if (top.empty()) { std::fprintf(stderr, "[loc_offline] search found nothing\n"); return 4; }
    offset = top[0].T;
  } else {
    std::fprintf(stderr, "mode must be --init x,y,z,yaw_deg, --local x,y,z,yaw_deg or --search\n");
    return 1;
  }
  init_txt << "used " << offset(0, 3) << " " << offset(1, 3) << " " << offset(2, 3) << " " << yaw_deg(offset)
           << " first_frame " << first << "\n";

  // ---- track -------------------------------------------------------------------------------
  Tracker tr(P, map_path, offset);
  std::ofstream att(fs::path(out) / "attempts.txt");
  att << std::fixed;
  std::vector<M4D> traj;
  int tries = 0, oks = 0;
  double max_jump = 0, max_jump_deg = 0;
  for (size_t i = first; i < frames.size(); ++i) {
    const Frame &f = frames[i];
    auto a = tr.step(f.T_odom, f.world);
    if (a.tried) {
      ++tries;
      if (a.ok) {
        ++oks;
        max_jump = std::max(max_jump, a.jump_m);
        max_jump_deg = std::max(max_jump_deg, a.jump_deg);
      }
      att << f.odom.header.stamp.sec + f.odom.header.stamp.nanosec * 1e-9 << " " << a.ok << " " << a.rough << " "
          << a.refine << " " << a.jump_m << " " << a.jump_deg << "\n";
    }
    traj.push_back(tr.offset() * f.T_odom);
  }

  // ---- trajectory bag --------------------------------------------------------------------
  fs::remove_all(fs::path(out) / "traj");
  {
    rosbag2_cpp::Writer w;
    rosbag2_storage::StorageOptions so;
    so.uri = (fs::path(out) / "traj").string();
    so.storage_id = "sqlite3";
    w.open(so, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});
    w.create_topic({ODOM, "nav_msgs/msg/Odometry", "cdr", ""});
    for (size_t i = first; i < frames.size(); ++i) {
      const M4D &T = traj[i - first];
      nav_msgs::msg::Odometry m = frames[i].odom;
      m.header.frame_id = "map";
      Eigen::Quaterniond q(M3D(T.block<3, 3>(0, 0)));
      m.pose.pose.position.x = T(0, 3);
      m.pose.pose.position.y = T(1, 3);
      m.pose.pose.position.z = T(2, 3);
      m.pose.pose.orientation.w = q.w();
      m.pose.pose.orientation.x = q.x();
      m.pose.pose.orientation.y = q.y();
      m.pose.pose.orientation.z = q.z();
      w.write(m, ODOM, rclcpp::Time(m.header.stamp));
    }
  }
  const M4D &off = tr.offset();
  std::printf("[loc_offline] ICP attempts %d, accepted %d (%.0f %%), max accepted jump %.3f m / %.2f deg\n",
              tries, oks, tries ? 100.0 * oks / tries : 0.0, max_jump, max_jump_deg);
  std::printf("[loc_offline] final T_map<-camera_init: x %.3f y %.3f z %.3f yaw %.2f deg\n",
              off(0, 3), off(1, 3), off(2, 3), yaw_deg(off));
  return 0;
}
