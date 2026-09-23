#include <xrslam/extra/opencv_image.h>
#include <xrslam/extra/poisson_disk_filter.h>

#include <cstdio>

using namespace cv;

namespace xrslam::extra {

static std::vector<Point2f> to_opencv(const std::vector<vector<2>> &v) {
    std::vector<Point2f> r(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        r[i].x = (float)v[i].x();
        r[i].y = (float)v[i].y();
    }
    return r;
}

OpenCvImage::OpenCvImage() = default;

double OpenCvImage::evaluate(const vector<2> &u, int level) const {
    double f;
    const vector<2> &s = scale_levels[level];
    vector<2> su{u.x() * s.x(), u.y() * s.y()};
    interpolator_levels[level].Evaluate(su.y(), su.x(), &f);
    return f;
}

double OpenCvImage::evaluate(const vector<2> &u, vector<2> &ddu,
                             int level) const {
    double f;
    const vector<2> &s = scale_levels[level];
    vector<2> su{u.x() * s.x(), u.y() * s.y()};
    interpolator_levels[level].Evaluate(su.y(), su.x(), &f, &ddu.y(), &ddu.x());
    ddu.x() *= s.x();
    ddu.y() *= s.y();
    return f;
}

void OpenCvImage::detect_keypoints(std::vector<vector<2>> &keypoints,
                                   size_t max_points,
                                   double keypoint_distance) const {

    std::vector<KeyPoint> cvkeypoints;

    gftt(max_points)->detect(image, cvkeypoints);

    if (cvkeypoints.size() > 0) {
        std::sort(cvkeypoints.begin(), cvkeypoints.end(),
                  [](const auto &a, const auto &b) {
                      return a.response > b.response;
                  });
        std::vector<vector<2>> new_keypoints;
        for (size_t i = 0; i < cvkeypoints.size(); ++i) {
            new_keypoints.emplace_back(cvkeypoints[i].pt.x,
                                       cvkeypoints[i].pt.y);
        }

        PoissonDiskFilter<2> filter(keypoint_distance);
        filter.preset_points(keypoints);
        filter.insert_points(new_keypoints);

        new_keypoints.erase(
            std::remove_if(new_keypoints.begin(), new_keypoints.end(),
                           [this](const auto &keypoint) {
                               return keypoint.x() < 20 || keypoint.y() < 20 ||
                                      keypoint.x() >= image.cols - 20 ||
                                      keypoint.y() >= image.rows - 20;
                           }),
            new_keypoints.end());

        keypoints.insert(keypoints.end(), new_keypoints.begin(),
                         new_keypoints.end());
    }
}

void OpenCvImage::track_keypoints(const Image *next_image,
                                  const std::vector<vector<2>> &curr_keypoints,
                                  std::vector<vector<2>> &next_keypoints,
                                  std::vector<char> &result_status) const {
    std::vector<Point2f> curr_cvpoints = to_opencv(curr_keypoints);
    std::vector<Point2f> next_cvpoints;
    if (next_keypoints.size() > 0) {
        next_cvpoints = to_opencv(next_keypoints);
    } else {
        next_keypoints.resize(curr_keypoints.size());
        next_cvpoints = curr_cvpoints;
    }

    const OpenCvImage *next_cvimage =
        dynamic_cast<const OpenCvImage *>(next_image);

    result_status.resize(curr_keypoints.size(), 0);
    if (next_cvimage && curr_cvpoints.size() > 0) {
        Mat cvstatus, cverr;
        calcOpticalFlowPyrLK(
            image_pyramid, next_cvimage->image_pyramid, curr_cvpoints,
            next_cvpoints, cvstatus, cverr, Size(21, 21), (int)level_num(),
            TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01),
            OPTFLOW_USE_INITIAL_FLOW);
        for (size_t i = 0; i < next_cvpoints.size(); ++i) {
            result_status[i] = cvstatus.at<unsigned char>((int)i);
            if (next_cvpoints[i].x < 20 ||
                next_cvpoints[i].x >= image.cols - 20 ||
                next_cvpoints[i].y < 20 ||
                next_cvpoints[i].y >= image.rows - 20) {
                result_status[i] = 0;
            }
            if (result_status[i]) {
                auto p = (next_cvpoints[i] - curr_cvpoints[i]);
                auto v = vector<2>(p.x, p.y);
                if (v.norm() > image.rows / 4) {
                    result_status[i] = 0;
                }
            }
        }

        {
            std::vector<uchar> reverse_status;
            std::vector<float> reverse_err;
            std::vector<Point2f> reverse_pts = curr_cvpoints;
            calcOpticalFlowPyrLK(
                next_cvimage->image_pyramid, image_pyramid, next_cvpoints,
                reverse_pts, reverse_status, reverse_err, Size(21, 21),
                (int)level_num(),
                TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01),
                OPTFLOW_USE_INITIAL_FLOW);

            for (size_t i = 0; i < reverse_status.size(); ++i) {
                if (result_status[i]) {
                    if (!reverse_status[i] ||
                        cv::norm(curr_cvpoints[i] - reverse_pts[i]) > 0.5) {
                        result_status[i] = 0;
                    }
                }
            }
        }
    }

    std::vector<size_t> l;
    std::vector<Point2f> p, q;
    for (size_t i = 0; i < result_status.size(); ++i) {
        if (result_status[i] != 0) {
            l.push_back(i);
            p.push_back(curr_cvpoints[i]);
            q.push_back(next_cvpoints[i]);
        }
    }

    for (size_t i = 0; i < curr_keypoints.size(); ++i) {
        if (result_status[i]) {
            next_keypoints[i].x() = next_cvpoints[i].x;
            next_keypoints[i].y() = next_cvpoints[i].y;
        }
    }
}

void OpenCvImage::preprocess(double clipLimit, int width, int height) {
    // Guard: on session start this CLAHE path has requested garbage-sized
    // copyMakeBorder allocations (~900 GB), and this static build cannot
    // unwind C++ exceptions (cv::error faults, killing the process). Skip
    // preprocessing for degenerate dimensions instead of dying; the frame
    // simply carries no features and tracking re-initializes later.
    if (image.cols <= 0 || image.rows <= 0 ||
        image.cols > 8192 || image.rows > 8192) {
        fprintf(stderr,
                "[xrslam][preprocess] skip clahe: bad image %dx%d type=%d "
                "t=%.3f\n",
                image.cols, image.rows, image.type(), t);
        return;
    }
    // OpenCV CLAHE is not safe in the bundled iOS 4.13.0 static framework:
    // the first valid 640x480, 8x8-tile frame deterministically enters
    // copyMakeBorder with corrupt internal state and requests ~900 GB. Keep
    // the native grayscale image, then build the same optical-flow pyramid;
    // feature detection/tracking continues without contrast equalization.
    // Re-enable only after replacing or rebuilding the OpenCV framework.
    image_pyramid.clear();
    buildOpticalFlowPyramid(image, image_pyramid, Size(21, 21),
                            (int)level_num(), true);
}

void OpenCvImage::correct_distortion(const matrix<3> &intrinsics,
                                     const vector<4> &coeffs) {
    Mat new_image;
    Mat K(3, 3, CV_32FC1), cvcoeffs(1, 4, CV_32FC1);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K.at<float>(i, j) = (float)intrinsics(i, j);
        }
    }
    for (int i = 0; i < 4; ++i) {
        cvcoeffs.at<float>(i) = (float)coeffs(i);
    }
    undistort(image, new_image, K, cvcoeffs);
    image = new_image;
}

CLAHE *OpenCvImage::clahe(double clipLimit, int width, int height) {
    // thread_local (not static): the previous process-wide instance shared
    // its internal Mats and Ptr refcount across feature-tracker worker
    // threads and XRSLAM sessions, leaving a corruption window on session
    // churn. One instance per worker thread removes that shared state.
    thread_local Ptr<CLAHE> s_clahe =
        createCLAHE(clipLimit, cv::Size(width, height));
    return s_clahe.get();
}

GFTTDetector *OpenCvImage::gftt(size_t max_points) {
    thread_local Ptr<GFTTDetector> s_gftt =
        GFTTDetector::create(max_points, 1.0e-3, 20, 3, true);
    return s_gftt.get();
}

FastFeatureDetector *OpenCvImage::fast() {
    thread_local Ptr<FastFeatureDetector> s_fast = FastFeatureDetector::create();
    return s_fast.get();
}

ORB *OpenCvImage::orb() {
    thread_local Ptr<ORB> s_orb = ORB::create();
    return s_orb.get();
}

void OpenCvImage::release_image_buffer() {
    image.release();
    raw.release();
    image_pyramid.clear();
    image_levels.clear();
    interpolator_levels.clear();
    grid_levels.clear();
    scale_levels.clear();
}

} // namespace xrslam::extra
