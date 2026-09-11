// loop_closure.cpp — visual loop detection + pose-graph correction.
#include "loop_closure.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace lc {

namespace {

Eigen::Matrix4d to_mat(const Pose& p) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = p.q.toRotationMatrix();
    T.block<3, 1>(0, 3) = p.t;
    return T;
}

Pose from_mat(const Eigen::Matrix4d& T) {
    Pose p;
    p.q = Eigen::Quaterniond(T.block<3, 3>(0, 0));
    p.q.normalize();
    if (p.q.w() < 0) p.q.coeffs() *= -1.0;
    p.t = T.block<3, 1>(0, 3);
    return p;
}

Eigen::Quaterniond eig_quat_from_xyzw(double x, double y, double z, double w) {
    Eigen::Quaterniond q(w, x, y, z);
    q.normalize();
    return q;
}

bool read_bracket_nums(const std::string& text, const std::string& key,
                       std::vector<double>& out) {
    const size_t k = text.find(key);
    if (k == std::string::npos) return false;
    const size_t lb = text.find('[', k);
    if (lb == std::string::npos) return false;
    const size_t rb = text.find(']', lb);
    if (rb == std::string::npos) return false;
    std::string seg = text.substr(lb + 1, rb - lb - 1);
    for (char& ch : seg) {
        if (ch == ',' || ch == ';') ch = ' ';
    }
    std::istringstream iss(seg);
    out.clear();
    double v = 0.0;
    while (iss >> v) out.push_back(v);
    return !out.empty();
}

}  // namespace

Config parse_device_yaml_text(const std::string& text) {
    Config cfg;
    std::vector<double> nums;
    if (read_bracket_nums(text, "intrinsics:", nums) && nums.size() >= 4) {
        for (int i = 0; i < 4; ++i) cfg.k[i] = nums[i];
    }
    if (read_bracket_nums(text, "q_bc:", nums) && nums.size() >= 4) {
        for (int i = 0; i < 4; ++i) cfg.q_bc[i] = nums[i];
    }
    if (read_bracket_nums(text, "p_bc:", nums) && nums.size() >= 3) {
        for (int i = 0; i < 3; ++i) cfg.p_bc[i] = nums[i];
    }
    return cfg;
}

Config parse_device_yaml(const std::string& path) {
    std::ifstream f(path);
    if (!f) return Config();
    std::stringstream ss;
    ss << f.rdbuf();
    return parse_device_yaml_text(ss.str());
}

LoopClosure::LoopClosure()
    : orb_(cv::ORB::create(600)), matcher_(cv::NORM_HAMMING) {}

void LoopClosure::reset() {
    kfs_.clear();
    kf_poses_.clear();
    tried_.clear();
    loops_.clear();
}

bool LoopClosure::add_keyframe(const cv::Mat& bgr, double t, const Pose& pose) {
    if (bgr.empty()) return false;
    cv::Mat gray;
    if (bgr.channels() == 3) {
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = bgr;
    }
    Keyframe kf;
    kf.t = t;
    orb_->detectAndCompute(gray, cv::noArray(), kf.kps, kf.desc);
    kfs_.push_back(std::move(kf));
    kf_poses_.push_back(pose);
    return true;
}

PassResult LoopClosure::run_pass() {
    PassResult out;
    const int n = static_cast<int>(kfs_.size());
    out.kf_times.resize(n);
    out.corrections.resize(n);
    for (int i = 0; i < n; ++i) out.kf_times[i] = kfs_[i].t;
    if (n < 5) return out;

    // ---- candidate pairs (fresh, not-yet-tried; nearest first) ----
    struct Cand {
        double d;
        int i, j;
    };
    std::vector<Cand> cands;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (kfs_[j].t - kfs_[i].t < cfg_.min_time_gap) continue;
            const double d = (kf_poses_[i].t - kf_poses_[j].t).norm();
            if (d > cfg_.max_pos_dist) continue;
            if (tried_.count({i, j})) continue;
            cands.push_back({d, i, j});
        }
    }
    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b) { return a.d < b.d; });
    if (static_cast<int>(cands.size()) > cfg_.max_candidates) {
        cands.resize(cfg_.max_candidates);
        if (cfg_.verbose)
            fprintf(stderr, "[detect] capped candidates to %d\n",
                    cfg_.max_candidates);
    }
    out.candidates = static_cast<int>(cands.size());
    if (cfg_.verbose)
        fprintf(stderr, "[detect] %zu candidate pair(s) this pass\n",
                cands.size());

    const cv::Matx33d Kmat(cfg_.k[0], 0.0, cfg_.k[2], 0.0, cfg_.k[1],
                           cfg_.k[3], 0.0, 0.0, 1.0);
    Eigen::Matrix4d T_bc = Eigen::Matrix4d::Identity();
    T_bc.block<3, 3>(0, 0) = eig_quat_from_xyzw(cfg_.q_bc[0], cfg_.q_bc[1],
                                                cfg_.q_bc[2], cfg_.q_bc[3])
                                 .toRotationMatrix();
    T_bc.block<3, 1>(0, 3) =
        Eigen::Vector3d(cfg_.p_bc[0], cfg_.p_bc[1], cfg_.p_bc[2]);
    const Eigen::Matrix4d T_bc_inv = T_bc.inverse();

    int new_loops = 0;
    for (const Cand& c : cands) {
        tried_.insert({c.i, c.j});
        const Keyframe& ka = kfs_[c.i];
        const Keyframe& kb = kfs_[c.j];
        if (ka.desc.empty() || kb.desc.empty() || ka.desc.rows < 20 ||
            kb.desc.rows < 20) {
            continue;
        }

        std::vector<std::vector<cv::DMatch>> knn;
        matcher_.knnMatch(ka.desc, kb.desc, knn, 2);
        std::vector<cv::DMatch> good;
        good.reserve(knn.size());
        for (const auto& m : knn) {
            if (m.size() == 2 && m[0].distance < 0.75f * m[1].distance) {
                good.push_back(m[0]);
            }
        }
        if (good.size() < 25) continue;

        std::vector<cv::Point2f> pts1, pts2;
        pts1.reserve(good.size());
        pts2.reserve(good.size());
        for (const auto& m : good) {
            pts1.push_back(ka.kps[m.queryIdx].pt);
            pts2.push_back(kb.kps[m.trainIdx].pt);
        }
        cv::Mat maskE;
#if CV_VERSION_MAJOR >= 5
        const cv::Mat E = cv::findEssentialMat(
            pts1, pts2, Kmat, cv::RANSAC, 0.999, 2.0, 1000, maskE);
#else
        const cv::Mat E = cv::findEssentialMat(
            pts1, pts2, Kmat, cv::RANSAC, 0.999, 2.0, maskE);
#endif
        if (E.empty() || maskE.empty()) continue;
        const int inliers = cv::countNonZero(maskE);
        if (inliers < static_cast<int>(cfg_.min_inliers)) continue;

        auto mask_at = [&](int k) -> bool {
            if (maskE.rows == static_cast<int>(good.size()) &&
                maskE.cols == 1) {
                return maskE.at<uchar>(k, 0) != 0;
            }
            if (maskE.rows == 1 &&
                maskE.cols == static_cast<int>(good.size())) {
                return maskE.at<uchar>(0, k) != 0;
            }
            return false;
        };
        std::vector<cv::Point2f> p1s, p2s;
        p1s.reserve(inliers);
        p2s.reserve(inliers);
        for (int k = 0; k < static_cast<int>(good.size()); ++k) {
            if (mask_at(k)) {
                p1s.push_back(pts1[k]);
                p2s.push_back(pts2[k]);
            }
        }

        cv::Mat R_rec, t_rec;
        const int ok = cv::recoverPose(E, p1s, p2s, Kmat, R_rec, t_rec);
        if (ok <= 0) continue;
        Eigen::Matrix3d Rr;
        for (int r = 0; r < 3; ++r) {
            for (int cc = 0; cc < 3; ++cc) {
                Rr(r, cc) = R_rec.at<double>(r, cc);
            }
        }
        const Eigen::Vector3d tr(t_rec.at<double>(0), t_rec.at<double>(1),
                                 t_rec.at<double>(2));

        // Same convention as the Python reference:
        // T_c1c2 = [R_rec^T | -R_rec^T t_rec]; T_b1b2 = T_bc*T_c1c2*T_bc^-1.
        Eigen::Matrix4d T_c1c2 = Eigen::Matrix4d::Identity();
        T_c1c2.block<3, 3>(0, 0) = Rr.transpose();
        T_c1c2.block<3, 1>(0, 3) = -(Rr.transpose() * tr);
        Eigen::Matrix4d T_b1b2 = T_bc * T_c1c2 * T_bc_inv;
        const double tn = T_b1b2.block<3, 1>(0, 3).norm();
        if (tn > 1e-9) {
            T_b1b2.block<3, 1>(0, 3) *= c.d / tn;  // scale from odometry
        }
        LoopEdge L;
        L.i = c.i;
        L.j = c.j;
        L.meas = from_mat(T_b1b2);
        L.inliers = inliers;
        L.matches = static_cast<int>(good.size());
        L.dpos = c.d;
        loops_.push_back(L);
        ++new_loops;
    }

    // ---- dedup (keep strongest per match run) + cap ----
    std::sort(loops_.begin(), loops_.end(),
              [](const LoopEdge& a, const LoopEdge& b) {
                  return a.inliers > b.inliers;
              });
    std::vector<LoopEdge> deduped;
    for (const LoopEdge& L : loops_) {
        bool ok = true;
        for (const LoopEdge& K : deduped) {
            if (std::abs(L.i - K.i) < 3 && std::abs(L.j - K.j) < 3) {
                ok = false;
                break;
            }
        }
        if (ok) deduped.push_back(L);
    }
    std::sort(deduped.begin(), deduped.end(),
              [](const LoopEdge& a, const LoopEdge& b) {
                  if (a.i != b.i) return a.i < b.i;
                  return a.j < b.j;
              });
    loops_ = std::move(deduped);
    if (static_cast<int>(loops_.size()) > cfg_.max_loops) {
        loops_.resize(cfg_.max_loops);
    }

    out.new_loops = new_loops;
    out.loops_total = static_cast<int>(loops_.size());
    if (cfg_.verbose) {
        fprintf(stderr, "[detect] %d new verified; %d after dedup\n",
                new_loops, out.loops_total);
    }
    if (loops_.empty()) return out;

    // ---- two-stage pose-graph solve ----
    auto build_edges = [&](const std::vector<LoopEdge>& LL) {
        std::vector<Edge> es;
        es.reserve((n - 1) + LL.size());
        for (int k = 0; k + 1 < n; ++k) {
            Edge e;
            e.i = k;
            e.j = k + 1;
            e.meas = from_mat(to_mat(kf_poses_[k]).inverse() *
                              to_mat(kf_poses_[k + 1]));
            e.sig_t = cfg_.sig_t_odom;
            e.sig_r = cfg_.sig_r_odom_deg * M_PI / 180.0;
            es.push_back(e);
        }
        for (const LoopEdge& L : LL) {
            Edge e;
            e.i = L.i;
            e.j = L.j;
            e.meas = L.meas;
            e.sig_t = cfg_.sig_t_loop;
            e.sig_r = cfg_.sig_r_loop_deg * M_PI / 180.0;
            es.push_back(e);
        }
        return es;
    };

    const std::vector<Edge> edges_all = build_edges(loops_);
    const SolveResult r1 =
        solve_pose_graph(kf_poses_, edges_all, Loss::SoftL1);
    if (!r1.ok) return out;

    std::vector<LoopEdge> keptE;
    int rejected = 0;
    for (const LoopEdge& L : loops_) {
        double rb = 0.0;
        const double tb =
            edge_error(r1.poses[L.i], r1.poses[L.j], L.meas, &rb);
        if (tb <= 0.60 && rb <= 6.0) {
            keptE.push_back(L);
        } else {
            ++rejected;
            if (cfg_.verbose) {
                fprintf(stderr,
                        "[pgo]   drop kf %4d <-> %4d: %.2f m / %.1f deg "
                        "(inliers %d)\n",
                        L.i, L.j, tb, rb, L.inliers);
            }
        }
    }
    if (keptE.empty()) keptE = loops_;

    const SolveResult r2 = solve_pose_graph(
        r1.poses, build_edges(keptE), Loss::Linear);

    double sum = 0.0;
    double mx = 0.0;
    for (int k = 0; k < n; ++k) {
        const Eigen::Matrix4d Ck =
            to_mat(r2.poses[k]) * to_mat(kf_poses_[k]).inverse();
        out.corrections[k] = from_mat(Ck);
        const double d = Ck.block<3, 1>(0, 3).norm();
        sum += d;
        mx = std::max(mx, d);
    }
    out.corr_mean = sum / std::max(n, 1);
    out.corr_max = mx;
    out.kept = static_cast<int>(keptE.size());
    out.rejected = rejected;
    out.corrected = true;
    out.final_kf_poses = r2.poses;
    if (cfg_.verbose) {
        fprintf(stderr,
                "[pgo] stage1 residual %.1f; stage2 residual %.1f "
                "(%d iters); %d kept / %d rejected\n",
                r1.residual_norm, r2.residual_norm, r2.iterations,
                out.kept, out.rejected);
    }
    return out;
}

void apply_corrections(const std::vector<double>& ts,
                       const std::vector<Pose>& samples,
                       const std::vector<double>& kf_times,
                       const std::vector<Pose>& C, std::vector<Pose>& out) {
    out = samples;
    const int m = static_cast<int>(kf_times.size());
    if (m == 0 || C.size() != static_cast<size_t>(m) || ts.empty()) return;

    std::vector<Eigen::Vector3d> crv(m);
    std::vector<Eigen::Vector3d> ct(m);
    for (int k = 0; k < m; ++k) {
        const Eigen::AngleAxisd aa(C[k].q);
        crv[k] = aa.angle() * aa.axis();
        ct[k] = C[k].t;
    }

    for (size_t s = 0; s < ts.size(); ++s) {
        const double t = ts[s];
        int k0 = 0, k1 = 0;
        double a = 0.0;
        if (t <= kf_times.front()) {
            k0 = k1 = 0;
        } else if (t >= kf_times.back()) {
            k0 = k1 = m - 1;
        } else {
            const auto it =
                std::lower_bound(kf_times.begin(), kf_times.end(), t);
            k1 = static_cast<int>(it - kf_times.begin());
            k0 = k1 - 1;
            const double dt = kf_times[k1] - kf_times[k0];
            a = (t - kf_times[k0]) / (dt > 1e-9 ? dt : 1e-9);
        }
        const Eigen::Vector3d rv = crv[k0] * (1.0 - a) + crv[k1] * a;
        const Eigen::Vector3d tt = ct[k0] * (1.0 - a) + ct[k1] * a;
        const double ang = rv.norm();
        const Eigen::Matrix3d Rc =
            ang > 1e-12
                ? Eigen::AngleAxisd(ang, rv / ang).toRotationMatrix()
                : Eigen::Matrix3d::Identity();
        Eigen::Quaterniond qo(Rc * samples[s].q.toRotationMatrix());
        qo.normalize();
        if (qo.w() < 0) qo.coeffs() *= -1.0;
        out[s].t = Rc * samples[s].t + tt;
        out[s].q = qo;
    }
}

}  // namespace lc
