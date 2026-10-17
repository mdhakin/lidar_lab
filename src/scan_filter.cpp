
#include "scan_filter.h"

#include <algorithm>
#include <cmath>
#include <utility>

ScanFilter::ScanFilter(const ScanFilterConfig& config) : config_(config) {
  // Keep configuration values within valid bounds.
  config_.historySize = std::max<size_t>(1, config_.historySize);

  config_.requiredScans = std::clamp(config_.requiredScans, size_t{1}, config_.historySize);

  config_.angleToleranceDeg = std::max(0.0f, config_.angleToleranceDeg);

  config_.distanceToleranceM = std::max(0.0f, config_.distanceToleranceM);
}

void ScanFilter::addScan(const LidarScan& scan) {
  history_.push_back(scan);

  // Retain only the most recent scans.
  while (history_.size() > config_.historySize) {
    history_.pop_front();
  }
}

bool ScanFilter::hasSupportingPoint(const LidarScan& scan, const LidarPoint& point) const {
  for (const LidarPoint& candidate : scan.points) {
    // Handle angular wraparound at 0/360 degrees.
    float angleDifference = std::fabs(candidate.angleDeg - point.angleDeg);

    angleDifference = std::fmod(angleDifference, 360.0f);

    angleDifference = std::min(angleDifference, 360.0f - angleDifference);

    const float distanceDifference = std::fabs(candidate.distanceM - point.distanceM);

    if (angleDifference <= config_.angleToleranceDeg && distanceDifference <= config_.distanceToleranceM) {
      return true;
    }
  }

  return false;
}

LidarScan ScanFilter::getStableScan() const {
  LidarScan stableScan;

  if (history_.empty()) {
    return stableScan;
  }

  // The newest scan provides all candidate points.
  const LidarScan& currentScan = history_.back();
  stableScan.sequence = currentScan.sequence;
  stableScan.points.reserve(currentScan.points.size());

  for (const LidarPoint& point : currentScan.points) {
    // The current observation counts as one.
    size_t supportCount = 1;

    // Search previous scans for supporting observations.
    // Each previous scan contributes at most one vote.
    for (size_t i = 0; i + 1 < history_.size(); ++i) {
      if (hasSupportingPoint(history_[i], point)) {
        ++supportCount;
      }
    }

    if (supportCount >= config_.requiredScans) {
      // Preserve the current measurement unchanged.
      stableScan.points.push_back(point);
    }
  }

  return stableScan;
}

void ScanFilter::clear() { history_.clear(); }

size_t ScanFilter::historyCount() const { return history_.size(); }
