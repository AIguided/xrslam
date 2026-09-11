#import <opencv2/opencv.hpp>
#import "XRSLAM.h"
#import "XRSLAM_iOS.h"
#import <UIKit/UIKit.h>
#import <Foundation/Foundation.h>
#import <Eigen/Eigen>
#import "loopclosure/loop_closure.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#import <opencv2/imgcodecs/ios.h>
#pragma clang diagnostic pop

@implementation XRSLAM {
    UIImage *uiimage;
    cv::Mat cvimage;
    cv::Mat cvimage_rgb;
    std::string slam_config_content;
    std::string device_config_content;
    std::string license_path;
    BOOL device_supported;
    NSInteger display_rotation;
    // On-device loop closure (all engine access on _lcQueue).
    lc::LoopClosure _lcEngine;
    dispatch_queue_t _lcQueue;
    std::vector<double> _lcKfTimes;
    std::vector<lc::Pose> _lcCorrections;
    bool _lcConfigured;
}

- (id)init:(NSString *)model {
    if (self = [super init]) {
        std::cout << "Device: " << [model UTF8String] << std::endl;
        display_rotation = 1; // default: rotate preview 90° CW
        _lcConfigured = false;
        _lcQueue = dispatch_queue_create("pathrec.loopclosure",
                                         DISPATCH_QUEUE_SERIAL);

        NSString *slam_params =
            [[NSBundle mainBundle] pathForResource:@"configs/slam_params"
                                            ofType:@"yaml"];
        NSString *slam_config_ns =
            [NSString stringWithContentsOfFile:slam_params
                                      encoding:NSUTF8StringEncoding
                                         error:NULL];
        NSString *device_params = [[NSBundle mainBundle]
            pathForResource:[@"configs/" stringByAppendingString:model]
                     ofType:@"yaml"];
        device_supported = [[NSFileManager defaultManager]
            fileExistsAtPath:device_params];
        if (!device_supported) {
            std::cout << "XRSLAM does NOT support this device! (no configs/"
                      << [model UTF8String] << ".yaml)" << std::endl;
            return self;
        }
        NSString *device_config_ns =
            [NSString stringWithContentsOfFile:device_params
                                      encoding:NSUTF8StringEncoding
                                         error:NULL];
        NSString *license_file = [[NSBundle mainBundle]
            pathForResource:
                @"configs/SENSESLAMSDK_165BA1A4-F959-4790-A891-C85DBF5D26EA"
                   ofType:@"lic"];

        slam_config_content = std::string([slam_config_ns UTF8String]);
        device_config_content = std::string([device_config_ns UTF8String]);
        // XRSLAMCreate expects configs as *content* but the license as
        // *path*. The open-source release ships no .lic file; pass an
        // empty path.
        license_path = license_file ? std::string([license_file UTF8String])
                                    : std::string("");

        // Loop-closure engine config from the same device yaml the estimator
        // uses (intrinsics + camera-IMU extrinsics for essential-matrix math).
        {
            lc::Config lccfg = lc::parse_device_yaml_text(device_config_content);
            lccfg.verbose = true; // diagnostics to the devicectl console
            _lcEngine.set_config(lccfg);
            _lcConfigured = true;
        }

        [self createSystem];
    }
    return self;
}

- (void)createSystem {
    void *yaml_config = nil;
    int num = XRSLAMCreate(slam_config_content.c_str(),
                           device_config_content.c_str(),
                           license_path.c_str(), "SenseSLAMSDK",
                           &yaml_config);
    std::cout << "init xr slam: " << num << std::endl;
}

- (void)resetSystem {
    XRSLAMDestroy();
    [self createSystem];
}

- (void)setDisplayRotation:(NSInteger)rotation {
    display_rotation = ((rotation % 4) + 4) % 4;
}

- (void)dealloc {
}

- (void)processBuffer:(CMSampleBufferRef)buffer {
    CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(buffer);
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);

    int w = (int)CVPixelBufferGetWidth(pixelBuffer);
    int h = (int)CVPixelBufferGetHeight(pixelBuffer);
    int pixelPerRow = (int)CVPixelBufferGetBytesPerRow(pixelBuffer);
    unsigned char *baseAddress =
        (unsigned char *)CVPixelBufferGetBaseAddress(pixelBuffer);

    cv::Mat raw_image = cv::Mat(h, w, CV_8UC4, baseAddress, pixelPerRow);

    cv::Mat rgb_image;
    cv::cvtColor(raw_image, cvimage, cv::COLOR_BGRA2GRAY);
    cv::cvtColor(raw_image, rgb_image, cv::COLOR_BGRA2RGB);
    cvimage_rgb = rgb_image;

    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);

    // Display-only rotation; the estimator already consumed the raw frame.
    switch (display_rotation) {
    case 1: {
        cv::Mat rotated;
        cv::rotate(rgb_image, rotated, cv::ROTATE_90_CLOCKWISE);
        uiimage = MatToUIImage(rotated);
    } break;
    case 2: {
        cv::Mat rotated;
        cv::rotate(rgb_image, rotated, cv::ROTATE_180);
        uiimage = MatToUIImage(rotated);
    } break;
    case 3: {
        cv::Mat rotated;
        cv::rotate(rgb_image, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
        uiimage = MatToUIImage(rotated);
    } break;
    default:
        uiimage = MatToUIImage(rgb_image);
        break;
    }
}

- (void)trackCamera:(double)t buffer:(CMSampleBufferRef)buffer {
    [self processBuffer:buffer];

    XRSLAMImage image;
    image.camera_id = 0;
    image.ext = nullptr;
    image.timeStamp = t;
    image.data = cvimage.data;
    image.stride = cvimage.step[0];
    image.channel = cvimage.channels();
    XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA, &image);
    XRSLAMRunOneFrame();
}

- (void)trackGyroscope:(double)t x:(double)x y:(double)y z:(double)z {
    XRSLAMGyroscope gyro;
    gyro.timestamp = t;
    gyro.data[0] = x;
    gyro.data[1] = y;
    gyro.data[2] = z;
    XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE, &gyro);
}

- (void)trackAccelerometer:(double)t x:(double)x y:(double)y z:(double)z {
    XRSLAMAcceleration acc;
    acc.timestamp = t;
    acc.data[0] = x;
    acc.data[1] = y;
    acc.data[2] = z;
    XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION, &acc);
}

- (SysState)get_system_state {
    XRSLAMState result;
    XRSLAMGetResult(XRSLAM_RESULT_STATE, &result);
    if (result == XRSLAM_STATE_INITIALIZING) {
        return SysState::SYS_INITIALIZING;
    } else if (result == XRSLAM_STATE_TRACKING_SUCCESS) {
        return SysState::SYS_TRACKING;
    } else if (result == XRSLAM_STATE_TRACKING_FAIL) {
        return SysState::SYS_CRASH;
    }
    return SysState::SYS_UNKNOWN;
}

- (UIImage *)getCurrentImage {
    return uiimage;
}

- (NSData *)getCurrentImageJPEG:(double)quality {
    if (cvimage_rgb.empty()) {
        return nil;
    }
    std::vector<unsigned char> buf;
    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back((int)(quality * 100.0));
    cv::imencode(".jpg", cvimage_rgb, buf, params);
    return [NSData dataWithBytes:buf.data() length:buf.size()];
}

- (NSArray<NSValue *> *)getFeaturePoints {
    XRSLAMFeatures features;
    XRSLAMGetResult(XRSLAM_RESULT_FEATURES, &features);
    const double w = cvimage.cols;
    const double h = cvimage.rows;
    NSMutableArray<NSValue *> *result =
        [NSMutableArray arrayWithCapacity:features.pos.size()];
    for (auto &p : features.pos) {
        double x = p.x, y = p.y, nx = x, ny = y;
        switch (display_rotation) {
        case 1: nx = (h - 1.0) - y; ny = x; break;             // 90 CW
        case 2: nx = (w - 1.0) - x; ny = (h - 1.0) - y; break; // 180
        case 3: nx = y; ny = (w - 1.0) - x; break;             // 90 CCW
        default: break;
        }
        [result addObject:[NSValue valueWithCGPoint:CGPointMake(nx, ny)]];
    }
    return result;
}

- (NSArray<NSNumber *> *)getBodyPose {
    XRSLAMPose pose;
    XRSLAMGetResult(XRSLAM_RESULT_BODY_POSE, &pose);
    return @[
        @(pose.timestamp), @(pose.translation[0]), @(pose.translation[1]),
        @(pose.translation[2]), @(pose.quaternion[0]), @(pose.quaternion[1]),
        @(pose.quaternion[2]), @(pose.quaternion[3])
    ];
}

#pragma mark - Loop closure

- (void)lcAddKeyframeWithTime:(double)t
                            x:(double)x
                            y:(double)y
                            z:(double)z
                           qx:(double)qx
                           qy:(double)qy
                           qz:(double)qz
                           qw:(double)qw {
    if (!_lcConfigured || cvimage_rgb.empty()) return;
    // cvimage_rgb is the latest processed frame (same buffer the keyframe
    // JPEG is encoded from); clone it because the camera thread overwrites
    // it. Note: the ORB luminance conversion in add_keyframe treats this
    // buffer as BGR, which makes the grayscale match
    // cv::imread(saved_kf.jpg, IMREAD_GRAYSCALE) of the saved files.
    cv::Mat frame = cvimage_rgb.clone();
    lc::Pose pose;
    pose.t = Eigen::Vector3d(x, y, z);
    pose.q = Eigen::Quaterniond(qw, qx, qy, qz);
    pose.q.normalize();
    dispatch_async(_lcQueue, ^{
        self->_lcEngine.add_keyframe(frame, t, pose);
    });
}

- (void)lcRunPassWithCompletion:
    (void (^)(NSDictionary * _Nullable))completion {
    if (!_lcConfigured) {
        dispatch_async(dispatch_get_main_queue(), ^{
            completion(nil);
        });
        return;
    }
    dispatch_async(_lcQueue, ^{
        const lc::PassResult res = self->_lcEngine.run_pass();
        self->_lcKfTimes = res.kf_times;
        self->_lcCorrections = res.corrections;
        if (res.kf_times.empty()) {
            dispatch_async(dispatch_get_main_queue(), ^{
                completion(nil);
            });
            return;
        }
        NSMutableDictionary *d = [NSMutableDictionary dictionary];
        d[@"newLoops"] = @(res.new_loops);
        d[@"candidates"] = @(res.candidates);
        d[@"loopsTotal"] = @(res.loops_total);
        d[@"kept"] = @(res.kept);
        d[@"rejected"] = @(res.rejected);
        d[@"corrected"] = @(res.corrected);
        d[@"corrMean"] = @(res.corr_mean);
        d[@"corrMax"] = @(res.corr_max);
        d[@"kfTimes"] = [NSData dataWithBytes:res.kf_times.data()
                                       length:res.kf_times.size() *
                                              sizeof(double)];
        if (!res.corrections.empty()) {
            std::vector<double> packed;
            packed.reserve(res.corrections.size() * 7);
            for (const lc::Pose &C : res.corrections) {
                packed.push_back(C.t.x());
                packed.push_back(C.t.y());
                packed.push_back(C.t.z());
                packed.push_back(C.q.x());
                packed.push_back(C.q.y());
                packed.push_back(C.q.z());
                packed.push_back(C.q.w());
            }
            d[@"corrections"] = [NSData dataWithBytes:packed.data()
                                               length:packed.size() *
                                                      sizeof(double)];
        }
        dispatch_async(dispatch_get_main_queue(), ^{
            completion(d);
        });
    });
}

- (void)lcApplyToTrajectoryTimes:(NSData *)ts
                       positions:(NSData *)pos
                     quaternions:(NSData *)quat
                      completion:(void (^)(NSData * _Nullable))completion {
    dispatch_async(_lcQueue, ^{
        NSData *out = nil;
        const bool have =
            !self->_lcCorrections.empty() &&
            ts.length % sizeof(double) == 0;
        if (have) {
            const size_t n = ts.length / sizeof(double);
            if (pos.length == n * 3 * sizeof(double) &&
                quat.length == n * 4 * sizeof(double) && n > 0) {
                const double *tp = (const double *)ts.bytes;
                const double *pp = (const double *)pos.bytes;
                const double *qp = (const double *)quat.bytes;
                std::vector<double> tv(n);
                std::vector<lc::Pose> sv(n);
                for (size_t i = 0; i < n; ++i) {
                    tv[i] = tp[i];
                    sv[i].t = Eigen::Vector3d(pp[3 * i], pp[3 * i + 1],
                                              pp[3 * i + 2]);
                    sv[i].q = Eigen::Quaterniond(qp[4 * i + 3], qp[4 * i],
                                                 qp[4 * i + 1],
                                                 qp[4 * i + 2]);
                    sv[i].q.normalize();
                }
                std::vector<lc::Pose> ov;
                lc::apply_corrections(tv, sv, self->_lcKfTimes,
                                      self->_lcCorrections, ov);
                std::vector<double> packed;
                packed.reserve(ov.size() * 7);
                for (size_t i = 0; i < ov.size(); ++i) {
                    packed.push_back(tv[i]);
                    packed.push_back(ov[i].t.x());
                    packed.push_back(ov[i].t.y());
                    packed.push_back(ov[i].t.z());
                    packed.push_back(ov[i].q.x());
                    packed.push_back(ov[i].q.y());
                    packed.push_back(ov[i].q.z());
                    packed.push_back(ov[i].q.w());
                }
                out = [NSData dataWithBytes:packed.data()
                                     length:packed.size() * sizeof(double)];
            }
        }
        dispatch_async(dispatch_get_main_queue(), ^{
            completion(out);
        });
    });
}

- (void)lcReset {
    if (!_lcQueue) return;
    dispatch_async(_lcQueue, ^{
        self->_lcEngine.reset();
        self->_lcKfTimes.clear();
        self->_lcCorrections.clear();
    });
}

@end
