// pose_graph.h — SE(3) pose-graph optimization (portable C++/Eigen).
//
// Port of the Python reference implementation in ~/research/pathrec/loop_closure.py
// (scipy least_squares trf, soft_l1 stage + linear polish stage).
// No OpenCV dependency: compiles for both the macOS test harness and the
// iOS PathRec app.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace lc {

struct Pose {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

struct Edge {
    int i = 0;      // from node
    int j = 0;      // to node
    Pose meas;      // measured pose of j expressed in i's frame (T_meas)
    double sig_t = 0.05;  // translation sigma [m]
    double sig_r = 0.02;  // rotation sigma [rad]
};

enum class Loss {
    Linear,
    SoftL1,  // rho(s) = 2*(sqrt(1+s)-1), matching scipy soft_l1 with f_scale=1
};

struct SolveResult {
    std::vector<Pose> poses;
    double residual_norm = 0.0;  // ||r|| (unweighted, after solve)
    int iterations = 0;
    bool ok = false;
};

// Residual of a single edge: translation [m], rotation [deg] (optional out).
double edge_error(const Pose& Ti, const Pose& Tj, const Pose& Tmeas,
                  double* rot_deg = nullptr);

// Node 0 is fixed (gauge). Poses of nodes 0..n-1 given as initial guess.
// Levenberg-Marquardt with grouped finite-difference Jacobian and IRLS
// weights for the robust loss.
SolveResult solve_pose_graph(const std::vector<Pose>& poses0,
                             const std::vector<Edge>& edges,
                             Loss loss,
                             int max_iter = 200,
                             double lambda0 = 1e-4);

}  // namespace lc
