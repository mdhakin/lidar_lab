
#ifndef TARGET_SELECTOR_H
#define TARGET_SELECTOR_H

#include "lidar.h"

struct NavigationTarget {
  float headingDeg = 0.0f;
  float distanceM = 0.0f;
  float confidence = 0.0f;
  bool valid = false;
};

struct TargetSelectorConfig {
  float sectorWidthDeg = 10.0f;
  float maxTargetDistanceM = 3.0f;
  float minimumClearanceM = 0.75f;
  float safetyMarginM = 0.30f;
  unsigned int minimumPointsPerSector = 3;
};

class TargetSelector {
 public:
  explicit TargetSelector(const TargetSelectorConfig& config = {});

  NavigationTarget selectTarget(const LidarScan& scan) const;

 private:
  TargetSelectorConfig config_;
};

#endif
