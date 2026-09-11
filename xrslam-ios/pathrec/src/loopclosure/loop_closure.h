// loop_closure.h — visual loop detection + pose-graph correction for PathRec.
//
// Portable C++ (OpenCV + Eigen) port of ~/research/pathrec/loop_closure.py:
//   ORB + essential-matrix RANSAC detection, dedup, two-stage robust PGO,
//   correction interpolation for the full-rate trajectory.
// Used by the macOS test harness and the iOS app.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <set>
#include <string>
#include <vector>

#include "pose_graph.h"

namespace lc {

struct Config {
    double min_inliers = 30.0;
    double max_pos_dist = 5.0;     // [m]
    double min_time_gap = 15.0;    // [s]
    int max_loops = 60;
    int max_candidates = 400;
    double sig_t_odom = 0.05;      // [m]
    double sig_r_odom_deg = 1.5;   // [deg]
    double sig_t_loop = 0.08;      // [m]
    double sig_r_loop_deg = 1.2;   // [deg]
    bool verbose = false;          // print pass diagnostics to stderr
    // iPhone 12 Pro defaults (same as the Python DEFAULT_* values).
    double k[4] = {475.718439315, 476.682666991, 317.023523238, 231.984642714};
    // q_bc stored in scipy order (x, y, z, w); converted to Eigen internally.
    double q_bc[4] = {-0.7071068, 0.7071068, 0.0, 0.0};
    double p_bc[3] = {0.0236924070003, 0.0306503133762, -0.00907115099061};
};

// Parse intrinsics / q_bc / p_bc from an xrslam device yaml (naive scan,
// same fields as the Python read_device_config). Missing fields keep defaults.
Config parse_device_yaml(const std::string& path);
Config parse_device_yaml_text(const std::string& text);

struct Keyframe {
    double t = 0.0;
    cv::Mat desc;                    // ORB descriptors (CV_8U, Nx32)
    std::vector<cv::KeyPoint> kps;   // same order as desc rows
};

struct LoopEdge {
    int i = 0, j = 0;
    Pose meas;
    int inliers = 0;
    int matches = 0;
    double dpos = 0.0;
};

struct PassResult {
    int loops_total = 0;   // verified loop edges after dedup/cap
    int new_loops = 0;     // new edges verified during this pass
    int candidates = 0;    // untried candidate pairs inspected this pass
    int kept = 0;          // edges used by the final solve
    int rejected = 0;      // edges dropped as gross outliers
    bool corrected = false;
    double corr_mean = 0.0;  // [m]
    double corr_max = 0.0;   // [m]
    std::vector<double> kf_times;         // size = #keyframes
    std::vector<Pose> corrections;        // C_k per keyframe (world-frame)
    std::vector<Pose> final_kf_poses;     // optimized keyframe poses
};

class LoopClosure {
public:
    LoopClosure();

    void reset();
    void set_config(const Config& cfg) { cfg_ = cfg; }
    const Config& config() const { return cfg_; }

    // Store a keyframe: raw (un-rotated BGR or gray) frame + pose at that
    // time. Extracts ORB descriptors + keypoints (call from a worker thread).
    bool add_keyframe(const cv::Mat& bgr, double t, const Pose& pose);
    int keyframe_count() const { return static_cast<int>(kfs_.size()); }

    // Match any not-yet-tried candidate pairs, then (if loops exist) run the
    // two-stage pose-graph solve and return per-keyframe corrections.
    PassResult run_pass();

    const std::vector<LoopEdge>& loops() const { return loops_; }
    const std::vector<Pose>& keyframe_poses() const { return kf_poses_; }

private:
    Config cfg_;
    cv::Ptr<cv::ORB> orb_;
    cv::BFMatcher matcher_;
    std::vector<Keyframe> kfs_;
    std::vector<Pose> kf_poses_;  // fixed odometry keyframe poses
    std::set<std::pair<int, int>> tried_;
    std::vector<LoopEdge> loops_;
};

// Interpolate per-keyframe corrections (C_k, world-frame) onto a sampled
// trajectory (same timestamps reference as kf_times). Port of the Python
// apply_corrections: pos' = Rc*pos + tc, quat' = Rc*quat.
void apply_corrections(const std::vector<double>& ts,
                       const std::vector<Pose>& samples,
                       const std::vector<double>& kf_times,
                       const std::vector<Pose>& C, std::vector<Pose>& out);

}  // namespace lc
