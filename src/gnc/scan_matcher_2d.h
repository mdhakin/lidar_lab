
#ifndef SCAN_MATCHER_2D_H
#define SCAN_MATCHER_2D_H

#include "lidar.h"

struct Pose2D {
    double xM = 0.0;
    double yM = 0.0;
    double headingRad = 0.0;
};

struct ScanMatchResult {
    Pose2D relativePose;

    double matchingErrorM = 0.0;
    double confidence = 0.0;

    unsigned int matchedPoints = 0;
    bool valid = false;
};

struct ScanMatcherConfig {
    double maxTranslationM = 0.50;
    double maxRotationRad = 0.7853981634;

    double maxCorrespondenceDistanceM = 0.20;
    unsigned int minimumMatches = 40;

    unsigned int maxIterations = 30;
    double convergenceTolerance = 1e-5;
};

class ScanMatcher2D {
public:
    explicit ScanMatcher2D(
        const ScanMatcherConfig& config = {});

    // Estimates the transform mapping current scan
    // coordinates into reference scan coordinates.
    ScanMatchResult match(
        const LidarScan& reference,
        const LidarScan& current) const;

private:
    ScanMatcherConfig config_;
};

#endif
