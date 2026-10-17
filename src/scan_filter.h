#ifndef SCAN_FILTER_H
#define SCAN_FILTER_H

#include <cstddef>
#include <deque>

#include "lidar.h"

struct ScanFilterConfig {
  size_t historySize = 5;
  size_t requiredScans = 3;

  float angleToleranceDeg = 1.5f;
  float distanceToleranceM = 0.08f;
};

class ScanFilter {
 public:
  explicit ScanFilter(
      const ScanFilterConfig& config = {});

  void addScan(const LidarScan& scan);

  LidarScan getStableScan() const;

  void clear();

  size_t historyCount() const;

 private:
  bool hasSupportingPoint(
      const LidarScan& scan,
      const LidarPoint& point) const;

  ScanFilterConfig config_;
  std::deque<LidarScan> history_;
};

#endif