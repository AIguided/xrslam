// pose_graph.cpp — SE(3) pose-graph optimization (portable C++/Eigen).
#include "pose_graph.h"

#include <Eigen/SparseCholesky>

#include <algorithm>
#include <cmath>
#include <cstdio>

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

Eigen::Vector3d log_rot(const Eigen::Matrix3d& R) {
    const Eigen::AngleAxisd aa(R);
    return aa.angle() * aa.axis();
}

// Right-multiply perturbation: t' = t + dt, q' = q * exp(dw)
void perturb(Pose& p, const Eigen::Matrix<double, 6, 1>& d) {
    p.t += d.head<3>();
    const Eigen::Vector3d w = d.tail<3>();
    const double th = w.norm();
    if (th > 1e-14) {
        p.q = (p.q * Eigen::Quaterniond(Eigen::AngleAxisd(th, w / th)))
                  .normalized();
    }
    if (p.q.w() < 0) p.q.coeffs() *= -1.0;
}

double loss_value(double s2, Loss loss) {
    if (loss == Loss::SoftL1) return 2.0 * (std::sqrt(1.0 + s2) - 1.0);
    return s2;
}

double loss_weight(double s2, Loss loss) {
    if (loss == Loss::SoftL1) return 1.0 / std::sqrt(1.0 + s2);
    return 1.0;
}

}  // namespace

double edge_error(const Pose& Ti, const Pose& Tj, const Pose& Tmeas,
                  double* rot_deg) {
    const Eigen::Matrix4d e =
        to_mat(Tmeas).inverse() * to_mat(Ti).inverse() * to_mat(Tj);
    if (rot_deg) {
        *rot_deg = log_rot(e.block<3, 3>(0, 0)).norm() * 180.0 / M_PI;
    }
    return e.block<3, 1>(0, 3).norm();
}

SolveResult solve_pose_graph(const std::vector<Pose>& poses0,
                             const std::vector<Edge>& edges, Loss loss,
                             int max_iter, double lambda0) {
    SolveResult out;
    out.poses = poses0;
    const int n = static_cast<int>(poses0.size());
    const int E = static_cast<int>(edges.size());
    if (n < 2 || E == 0) {
        out.ok = false;
        return out;
    }
    const int ncols = 6 * (n - 1);
    std::vector<Pose> P = poses0;

    auto residuals = [&](const std::vector<Pose>& Ps, Eigen::VectorXd& r) {
        r.resize(6 * E);
        for (int e = 0; e < E; ++e) {
            const Edge& ed = edges[e];
            const Eigen::Matrix4d err = to_mat(ed.meas).inverse() *
                                        to_mat(Ps[ed.i]).inverse() *
                                        to_mat(Ps[ed.j]);
            r.segment<3>(6 * e) = err.block<3, 1>(0, 3) / ed.sig_t;
            r.segment<3>(6 * e + 3) =
                log_rot(err.block<3, 3>(0, 0)) / ed.sig_r;
        }
    };

    // Incidence: edges touching each node; nodes sharing an edge.
    std::vector<std::vector<int>> nodeEdges(n);
    std::vector<std::vector<char>> adj(n, std::vector<char>(n, 0));
    for (int e = 0; e < E; ++e) {
        nodeEdges[edges[e].i].push_back(e);
        nodeEdges[edges[e].j].push_back(e);
        adj[edges[e].i][edges[e].j] = 1;
        adj[edges[e].j][edges[e].i] = 1;
    }

    // Greedy column coloring: columns conflict when they belong to the same
    // node or to nodes sharing an edge. Columns in a group can be perturbed
    // simultaneously in a single finite-difference evaluation.
    std::vector<int> color(ncols, -1);
    int ngroups = 0;
    for (int c = 0; c < ncols; ++c) {
        const int k = c / 6 + 1;
        std::vector<char> used;
        for (int c2 = 0; c2 < c; ++c2) {
            const int k2 = c2 / 6 + 1;
            if (k == k2 || adj[k][k2]) {
                const int col = color[c2];
                if (col >= 0) {
                    if (static_cast<int>(used.size()) <= col)
                        used.resize(col + 1, 0);
                    used[col] = 1;
                }
            }
        }
        int c0 = 0;
        while (c0 < static_cast<int>(used.size()) && used[c0]) ++c0;
        color[c] = c0;
        ngroups = std::max(ngroups, c0 + 1);
    }

    auto jacobian = [&](const std::vector<Pose>& Ps, const Eigen::VectorXd& r0,
                        Eigen::SparseMatrix<double>& J) {
        const double h = 1e-6;
        std::vector<Eigen::Triplet<double>> trips;
        Eigen::VectorXd r1(6 * E);
        for (int g = 0; g < ngroups; ++g) {
            std::vector<Pose> Ps2 = Ps;
            for (int c = 0; c < ncols; ++c) {
                if (color[c] != g) continue;
                const int k = c / 6 + 1;
                Eigen::Matrix<double, 6, 1> d =
                    Eigen::Matrix<double, 6, 1>::Zero();
                d[c % 6] = h;
                perturb(Ps2[k], d);
            }
            residuals(Ps2, r1);
            for (int c = 0; c < ncols; ++c) {
                if (color[c] != g) continue;
                const int k = c / 6 + 1;
                for (int e : nodeEdges[k]) {
                    for (int row = 0; row < 6; ++row) {
                        const int gr = 6 * e + row;
                        const double v = (r1[gr] - r0[gr]) / h;
                        if (v != 0.0) trips.emplace_back(gr, c, v);
                    }
                }
            }
        }
        J.resize(6 * E, ncols);
        J.setFromTriplets(trips.begin(), trips.end());
    };

    Eigen::VectorXd r0;
    residuals(P, r0);
    double cost = 0.0;
    for (int i = 0; i < r0.size(); ++i)
        cost += loss_value(r0[i] * r0[i], loss);

    double lambda = lambda0;
    int it_done = 0;
    for (int it = 0; it < max_iter; ++it) {
        Eigen::VectorXd w(6 * E);
        for (int i = 0; i < r0.size(); ++i)
            w[i] = loss_weight(r0[i] * r0[i], loss);

        Eigen::SparseMatrix<double> J;
        jacobian(P, r0, J);
        for (int k = 0; k < J.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator itJ(J, k); itJ;
                 ++itJ) {
                itJ.valueRef() *= std::sqrt(w[itJ.row()]);
            }
        }
        Eigen::VectorXd sw(6 * E);
        for (int i = 0; i < r0.size(); ++i) sw[i] = std::sqrt(w[i]) * r0[i];

        Eigen::SparseMatrix<double> H = J.transpose() * J;
        const Eigen::VectorXd g = J.transpose() * sw;
        for (int c = 0; c < ncols; ++c) H.coeffRef(c, c) += lambda;

        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt;
        ldlt.compute(H);
        if (ldlt.info() != Eigen::Success) {
            lambda = std::min(lambda * 10.0, 1e12);
            it_done = it + 1;
            if (lambda >= 1e12) break;
            continue;
        }
        Eigen::VectorXd delta = ldlt.solve(-g);
        if (ldlt.info() != Eigen::Success || !delta.allFinite()) {
            lambda = std::min(lambda * 10.0, 1e12);
            it_done = it + 1;
            if (lambda >= 1e12) break;
            continue;
        }

        std::vector<Pose> P2 = P;
        for (int k = 1; k < n; ++k)
            perturb(P2[k], delta.segment<6>(6 * (k - 1)));
        Eigen::VectorXd r1;
        residuals(P2, r1);
        double cost1 = 0.0;
        for (int i = 0; i < r1.size(); ++i)
            cost1 += loss_value(r1[i] * r1[i], loss);

        it_done = it + 1;
        if (cost1 < cost) {
            const double rel =
                (cost - cost1) / std::max(std::fabs(cost), 1e-12);
            P.swap(P2);
            r0.swap(r1);
            cost = cost1;
            lambda = std::max(lambda * 0.25, 1e-10);
            if (delta.lpNorm<Eigen::Infinity>() < 1e-8 || rel < 1e-12) break;
        } else {
            lambda = std::min(lambda * 4.0, 1e12);
            if (lambda >= 1e12) break;
        }
    }

    out.poses = std::move(P);
    out.residual_norm = r0.norm();
    out.iterations = it_done;
    out.ok = true;
    return out;
}

}  // namespace lc
