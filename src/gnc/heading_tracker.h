
#ifndef HEADING_TRACKER_H
#define HEADING_TRACKER_H

#include "lidar.h"

struct HeadingEstimate {
  float rotationDeg = 0.0f;
  float matchingError = 0.0f;
  float confidence = 0.0f;
  bool valid = false;
};

struct HeadingTrackerConfig {
  float maxRotationDeg = 180.0f;
  float coarseStepDeg = 2.0f;
  float fineStepDeg = 0.25f;

  float maxRangeDifferenceM = 0.30f;
  unsigned int minimumMatches = 40;
};

class HeadingTracker {
 public:
  explicit HeadingTracker(
      const HeadingTrackerConfig& config = {});

  void setReference(const LidarScan& scan);
  void reset();

  bool hasReference() const;

  HeadingEstimate estimate(
      const LidarScan& currentScan) const;

 private:
  HeadingTrackerConfig config_;
  LidarScan referenceScan_;
  bool referenceValid_ = false;
};

#endif
