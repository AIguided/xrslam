#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>

NS_ASSUME_NONNULL_BEGIN

@interface XRSLAM : NSObject

typedef NS_ENUM(NSUInteger, SysState) {
    SYS_INITIALIZING = 0,
    SYS_TRACKING,
    SYS_CRASH,
    SYS_UNKNOWN
};

- (id)init:(NSString *)model;
- (void)processBuffer:(CMSampleBufferRef)buffer;
- (void)trackGyroscope:(double)t x:(double)x y:(double)y z:(double)z;
- (void)trackAccelerometer:(double)t x:(double)x y:(double)y z:(double)z;
- (void)trackCamera:(double)t buffer:(CMSampleBufferRef)buffer;
- (void)resetSystem;
- (SysState)get_system_state;
- (nullable UIImage *)getCurrentImage;

// Latest raw (un-rotated) camera frame as JPEG data; for keyframe capture.
- (nullable NSData *)getCurrentImageJPEG:(double)quality;

// Display-only rotation: 0=raw, 1=90CW, 2=180, 3=90CCW. Rotates the
// preview image and feature overlay coordinates; the estimator always
// consumes the raw sensor frame.
- (void)setDisplayRotation:(NSInteger)rotation;

// 2D tracked features in the processed image coordinate frame.
- (NSArray<NSValue *> *)getFeaturePoints;

// Full body pose: [t, tx, ty, tz, qx, qy, qz, qw].
- (NSArray<NSNumber *> *)getBodyPose;

// ---- On-device loop closure (visual detection + pose-graph correction) ----

// Feed a keyframe: latest raw frame + body pose at time t (systemUptime).
// Runs ORB extraction on an internal serial queue (fire and forget).
- (void)lcAddKeyframeWithTime:(double)t
                            x:(double)x
                            y:(double)y
                            z:(double)z
                           qx:(double)qx
                           qy:(double)qy
                           qz:(double)qz
                           qw:(double)qw;

// Detect loops among untried keyframe pairs, run the two-stage pose-graph
// solve, and return a result dictionary (nil if < 5 keyframes):
//   newLoops, loopsTotal, kept, rejected, corrected, corrMean, corrMax,
//   kfTimes (NSData, M doubles), corrections (NSData, 7*M doubles).
// Completion is called on the main thread.
- (void)lcRunPassWithCompletion:(void (^)(NSDictionary * _Nullable result))completion;

// Apply the latest corrections to a recorded trajectory (flat arrays:
// times N, positions 3N, quaternions 4N in xyzw order). Returns corrected
// values as N samples of 8 doubles each (t, x, y, z, qx, qy, qz, qw), or
// nil when there are no corrections. Completion is called on the main thread.
- (void)lcApplyToTrajectoryTimes:(NSData *)ts
                       positions:(NSData *)pos
                     quaternions:(NSData *)quat
                      completion:(void (^)(NSData * _Nullable corrected))completion;

// Clear all keyframes + loop state (call on session start).
- (void)lcReset;

@end

NS_ASSUME_NONNULL_END
