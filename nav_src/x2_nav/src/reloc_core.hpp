// Shared relocalisation core for localizer_offline (bag) and localizer_live (ROS node).
// Both executables call exactly this code, so a live run can be checked against an
// offline replay of the same data bit-for-bit (when no message is lost).
//
// Conventions:
//   T_odom   camera_init <- body      (Point-LIO /aft_mapped_to_init; body = IMU)
//   offset   map <- camera_init       (what this code estimates; 4-DOF, gravity-aligned)
//   map pose map <- body = offset * T_odom
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <yaml-cpp/yaml.h>

#include "icp_localizer.h"

namespace x2reloc {

using M4D = Eigen::Matrix4d;

inline M4D inv(const M4D &T) {
  M4D I = M4D::Identity();
  I.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();
  I.block<3, 1>(0, 3) = -T.block<3, 3>(0, 0).transpose() * T.block<3, 1>(0, 3);
  return I;
}
inline M4D yawT(double x, double y, double z, double yaw_rad) {
  M4D T = M4D::Identity();
  T.block<3, 3>(0, 0) = Eigen::AngleAxisd(yaw_rad, V3D::UnitZ()).toRotationMatrix();
  T.block<3, 1>(0, 3) = V3D(x, y, z);
  return T;
}
inline double ang_deg(const M3D &R) { return Eigen::AngleAxisd(R).angle() * 180.0 / M_PI; }
inline double yaw_deg(const M4D &T) { return std::atan2(T(1, 0), T(0, 0)) * 180.0 / M_PI; }
inline CloudType::Ptr voxel(const CloudType::Ptr &in, double leaf) {
  CloudType::Ptr out(new CloudType);
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf, leaf, leaf);
  vg.setInputCloud(in);
  vg.filter(*out);
  return out;
}

struct Params {
  ICPConfig icp;
  int every = 10, accum = 1;
  double search_path_m = 3.0, search_xy_step = 1.0, search_yaw_step_deg = 30.0, search_inlier_dist = 0.2;
  static Params load(const std::string &path) {
    YAML::Node y = YAML::LoadFile(path);
    Params p;
    p.icp.rough_scan_resolution = y["rough_scan_resolution"].as<double>();
    p.icp.rough_map_resolution = y["rough_map_resolution"].as<double>();
    p.icp.rough_max_iteration = y["rough_max_iteration"].as<int>();
    p.icp.rough_score_thresh = y["rough_score_thresh"].as<double>();
    p.icp.refine_scan_resolution = y["refine_scan_resolution"].as<double>();
    p.icp.refine_map_resolution = y["refine_map_resolution"].as<double>();
    p.icp.refine_max_iteration = y["refine_max_iteration"].as<int>();
    p.icp.refine_score_thresh = y["refine_score_thresh"].as<double>();
    p.icp.yaw_only = y["yaw_only"].as<bool>();
    p.icp.lock_z = y["lock_z"].as<bool>();
    p.every = y["every_n_frames"].as<int>();
    p.accum = y["accum_scans"].as<int>();
    p.search_path_m = y["search_path_m"].as<double>();
    p.search_xy_step = y["search_xy_step"].as<double>();
    p.search_yaw_step_deg = y["search_yaw_step_deg"].as<double>();
    p.search_inlier_dist = y["search_inlier_dist"].as<double>();
    return p;
  }
};

struct Seed { double x, y, yaw_deg; };
struct Cand { M4D T; double inl, rms; };

// seeds over the map's xy bounding box (no prior at all)
inline std::vector<Seed> global_seeds(const CloudType::Ptr &map_coarse, double xy_step, double yaw_step) {
  PointType mn, mx;
  pcl::getMinMax3D(*map_coarse, mn, mx);
  std::vector<Seed> s;
  for (double x = mn.x; x <= mx.x; x += xy_step)
    for (double y = mn.y; y <= mx.y; y += xy_step)
      for (double yaw = 0; yaw < 360.0 - 1e-6; yaw += yaw_step) s.push_back({x, y, yaw});
  return s;
}
// seeds in a window around a rough prior (operator placed the robot "about here")
inline std::vector<Seed> local_seeds(double cx, double cy, double cyaw, double radius, double xy_step,
                                     double yaw_range, double yaw_step) {
  std::vector<Seed> s;
  for (double dx = -radius; dx <= radius + 1e-9; dx += xy_step)
    for (double dy = -radius; dy <= radius + 1e-9; dy += xy_step)
      for (double dyaw = -yaw_range; dyaw <= yaw_range + 1e-9; dyaw += yaw_step)
        s.push_back({cx + dx, cy + dy, cyaw + dyaw});
  return s;
}

// Register `sub` (a submap in camera_init) against the map from every seed (z = 0).
// Coarse ICP at 0.5 m, rank by inlier ratio, refine the 20 best distinct coarse results
// with two shrinking gates, de-duplicate the REFINED results, best first.
inline std::vector<Cand> search(const CloudType::Ptr &map, const CloudType::Ptr &sub,
                                const std::vector<Seed> &seeds, const Params &P) {
  CloudType::Ptr sub_c = voxel(sub, 0.5), sub_f = voxel(sub, 0.25);
  CloudType::Ptr map_c = voxel(map, 0.5), map_f = voxel(map, 0.25);
  pcl::IterativeClosestPoint<PointType, PointType> coarse, fine;
  if (P.icp.yaw_only) {
    auto est = std::make_shared<TransformationEstimationYawXYZ<PointType>>(P.icp.lock_z);
    coarse.setTransformationEstimation(est);
    fine.setTransformationEstimation(est);
  }
  coarse.setInputTarget(map_c);
  coarse.setMaxCorrespondenceDistance(1.5);
  coarse.setMaximumIterations(20);
  coarse.setInputSource(sub_c);
  fine.setInputTarget(map_f);
  fine.setMaximumIterations(30);
  fine.setInputSource(sub_f);
  pcl::KdTreeFLANN<PointType> kd;
  kd.setInputCloud(map_f);
  const double s_inl = P.search_inlier_dist;
  auto score = [&](const M4D &T, double &rms) {
    CloudType tmp;
    pcl::transformPointCloud(*sub_f, tmp, T.cast<float>());
    std::vector<int> id(1);
    std::vector<float> d2(1);
    size_t inl = 0;
    double se = 0;
    for (auto &p : tmp.points)
      if (kd.nearestKSearch(p, 1, id, d2) > 0 && d2[0] < s_inl * s_inl) { ++inl; se += d2[0]; }
    rms = inl ? std::sqrt(se / inl) : -1.0;
    return double(inl) / tmp.size();
  };
  std::vector<Cand> cands;
  CloudType dummy;
  for (const Seed &sd : seeds) {
    coarse.align(dummy, yawT(sd.x, sd.y, 0.0, sd.yaw_deg * M_PI / 180.0).cast<float>());
    if (!coarse.hasConverged()) continue;
    double rms;
    M4D T = coarse.getFinalTransformation().cast<double>();
    cands.push_back({T, score(T, rms), rms});
  }
  std::sort(cands.begin(), cands.end(), [](const Cand &a, const Cand &b) { return a.inl > b.inl; });
  auto same = [](const M4D &a, const M4D &b) {
    return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm() < 0.3 &&
           ang_deg(a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0)) < 5.0;
  };
  std::vector<Cand> coarse_top, refined, top;
  for (auto &c : cands) {
    bool dup = false;
    for (auto &t : coarse_top) if (same(t.T, c.T)) { dup = true; break; }
    if (dup) continue;
    coarse_top.push_back(c);
    if (coarse_top.size() >= 20) break;
  }
  for (auto &c : coarse_top) {
    M4D T = c.T;
    for (double gate : {0.5, 0.2}) {
      fine.setMaxCorrespondenceDistance(gate);
      fine.align(dummy, T.cast<float>());
      T = fine.getFinalTransformation().cast<double>();
    }
    double rms;
    double inl = score(T, rms);
    refined.push_back({T, inl, rms});
  }
  std::sort(refined.begin(), refined.end(), [](const Cand &a, const Cand &b) { return a.inl > b.inl; });
  for (auto &c : refined) {
    bool dup = false;
    for (auto &t : top) if (same(t.T, c.T)) { dup = true; break; }
    if (!dup) top.push_back(c);
  }
  return top;
}

// Tracking, mirroring localizer_node::timerCB at every `every` frames.
class Tracker {
public:
  struct Attempt { bool tried = false, ok = false; double rough = -1, refine = -1, jump_m = 0, jump_deg = 0; };

  Tracker(const Params &P, const std::string &map_path, const M4D &init) : P_(P), loc_(P.icp), offset_(init) {
    ok_map_ = loc_.loadMap(map_path);
  }
  bool mapLoaded() const { return ok_map_; }
  const M4D &offset() const { return offset_; }

  // feed one frame (odometry pose + deskewed world-frame scan); returns the attempt record
  Attempt step(const M4D &T_odom, const CloudType::Ptr &world) {
    recent_.push_back({T_odom, world});
    while (int(recent_.size()) > P_.accum) recent_.pop_front();
    Attempt a;
    if (n_++ % size_t(P_.every) != 0) return a;
    a.tried = true;
    CloudType::Ptr input(new CloudType);
    M4D T_bo = inv(T_odom);
    for (auto &r : recent_) {
      CloudType tmp;
      pcl::transformPointCloud(*r.second, tmp, T_bo.cast<float>());
      *input += tmp;
    }
    loc_.setInput(input);
    M4F guess = (offset_ * T_odom).cast<float>();
    a.ok = loc_.align(guess);
    a.rough = loc_.lastRoughScore();
    a.refine = loc_.lastRefineScore();
    if (a.ok) {
      M4D noff = guess.cast<double>() * inv(T_odom);
      // jump = how far the ROBOT's map pose moves (old vs new correction on the same odom
      // pose); comparing correction translations would mix in yaw * distance-from-origin
      a.jump_m = ((noff * T_odom).block<3, 1>(0, 3) - (offset_ * T_odom).block<3, 1>(0, 3)).norm();
      a.jump_deg = ang_deg(offset_.block<3, 3>(0, 0).transpose() * noff.block<3, 3>(0, 0));
      offset_ = noff;
    }
    return a;
  }

private:
  Params P_;
  ICPLocalizer loc_;
  M4D offset_;
  bool ok_map_ = false;
  size_t n_ = 0;
  std::deque<std::pair<M4D, CloudType::Ptr>> recent_;
};

}  // namespace x2reloc
