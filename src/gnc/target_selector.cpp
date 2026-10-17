
#include "target_selector.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

float normalizeAngle(float angle) {
  angle = std::fmod(angle, 360.0f);

  if (angle < 0.0f) {
    angle += 360.0f;
  }

  return angle;
}

}  // namespace

TargetSelector::TargetSelector(const TargetSelectorConfig& config) : config_(config) {}

NavigationTarget TargetSelector::selectTarget(const LidarScan& scan) const {
  NavigationTarget bestTarget;

  if (scan.points.empty() || config_.sectorWidthDeg <= 0.0f || config_.maxTargetDistanceM <= 0.0f) {
    return bestTarget;
  }

  const int sectorCount = static_cast<int>(std::ceil(360.0f / config_.sectorWidthDeg));

  std::vector<std::vector<float>> sectors(sectorCount);

  // Group valid measurements by angular sector.
  for (const LidarPoint& point : scan.points) {
    if (!std::isfinite(point.angleDeg) || !std::isfinite(point.distanceM) || point.distanceM <= 0.0f) {
      continue;
    }

    const float angle = normalizeAngle(point.angleDeg);

    int index = static_cast<int>(angle / config_.sectorWidthDeg);

    index = std::min(index, sectorCount - 1);

    sectors[index].push_back(point.distanceM);
  }

  float bestClearance = 0.0f;

  for (int i = 0; i < sectorCount; ++i) {
    std::vector<float>& ranges = sectors[i];

    if (ranges.size() < config_.minimumPointsPerSector) {
      continue;
    }

    std::sort(ranges.begin(), ranges.end());

    const size_t percentileIndex = static_cast<size_t>(0.20f * static_cast<float>(ranges.size() - 1));

    const float clearance = ranges[percentileIndex];

    if (clearance < config_.minimumClearanceM) {
      continue;
    }

    if (clearance <= bestClearance) {
      continue;
    }

    bestClearance = clearance;

    const float startAngle = static_cast<float>(i) * config_.sectorWidthDeg;

    const float endAngle = std::min(startAngle + config_.sectorWidthDeg, 360.0f);

    bestTarget.headingDeg = (startAngle + endAngle) * 0.5f;

    bestTarget.distanceM = std::min(clearance - config_.safetyMarginM, config_.maxTargetDistanceM);

    bestTarget.confidence = static_cast<float>(ranges.size()) / static_cast<float>(scan.points.size());

    bestTarget.valid = bestTarget.distanceM > 0.0f;
  }

  return bestTarget;
}
