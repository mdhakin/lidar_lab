
#include "heading_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {

constexpr float kFullCircle = 360.0f;
constexpr float kBinWidthDeg = 1.0f;
constexpr int kBinCount = 360;
constexpr float kInvalidRange = -1.0f;

float normalizeAngle(float angle) {
  angle = std::fmod(angle, kFullCircle);
  if (angle < 0.0f) {
    angle += kFullCircle;
  }
  return angle;
}

float signedAngle(float angle) {
  angle = normalizeAngle(angle);
  return angle > 180.0f ? angle - 360.0f : angle;
}

using RangeBins = std::vector<float>;

RangeBins buildBins(const LidarScan& scan) {
  RangeBins bins(kBinCount, kInvalidRange);

  for (const auto& point : scan.points) {
    if (!std::isfinite(point.angleDeg) || !std::isfinite(point.distanceM) || point.distanceM <= 0.0f) {
      continue;
    }

    const int index = static_cast<int>(normalizeAngle(point.angleDeg) / kBinWidthDeg);

    // Keep the nearest return in each angular bin.
    float& range = bins[index];

    if (range < 0.0f || point.distanceM < range) {
      range = point.distanceM;
    }
  }

  return bins;
}

struct MatchResult {
  float rotationDeg = 0.0f;
  float error = std::numeric_limits<float>::infinity();
  unsigned int matches = 0;
  float overlap = 0.0f;
  bool valid = false;
};

MatchResult evaluateRotation(const RangeBins& reference, const RangeBins& current, float rotationDeg,
                             float maxRangeDifferenceM, unsigned int minimumMatches) {
  MatchResult result;
  result.rotationDeg = rotationDeg;

  float totalError = 0.0f;
  unsigned int matches = 0;
  unsigned int compared = 0;

  for (int i = 0; i < kBinCount; ++i) {
    if (current[i] < 0.0f) {
      continue;
    }

    // Positive rotation means clockwise.
    // A fixed environmental feature moves in the opposite
    // direction in the sensor's local angular frame.
    //
    // This assumes sensor angles increase clockwise.
    // We will verify that convention experimentally.
    const float referenceAngle = static_cast<float>(i) + rotationDeg;

    const int referenceIndex = static_cast<int>(std::round(normalizeAngle(referenceAngle))) % kBinCount;

    if (reference[referenceIndex] < 0.0f) {
      continue;
    }

    ++compared;

    const float difference = std::fabs(current[i] - reference[referenceIndex]);

    if (difference <= maxRangeDifferenceM) {
      totalError += difference;
      ++matches;
    }
  }

  if (matches < minimumMatches || compared == 0) {
    return result;
  }

  result.matches = matches;
  result.overlap = static_cast<float>(matches) / static_cast<float>(compared);

  // Penalize candidates that only match a small subset.
  result.error = (totalError / static_cast<float>(matches)) + (1.0f - result.overlap) * maxRangeDifferenceM;

  result.valid = true;
  return result;
}

}  // namespace

HeadingTracker::HeadingTracker(const HeadingTrackerConfig& config) : config_(config) {}

void HeadingTracker::setReference(const LidarScan& scan) {
  referenceScan_ = scan;
  referenceValid_ = !scan.points.empty();
}

void HeadingTracker::reset() {
  referenceScan_.points.clear();
  referenceValid_ = false;
}

bool HeadingTracker::hasReference() const { return referenceValid_; }

HeadingEstimate HeadingTracker::estimate(const LidarScan& currentScan) const {
  HeadingEstimate estimate;

  if (!referenceValid_ || currentScan.points.empty()) {
    return estimate;
  }

  if (config_.coarseStepDeg <= 0.0f || config_.fineStepDeg <= 0.0f || config_.maxRotationDeg <= 0.0f ||
      config_.maxRangeDifferenceM <= 0.0f) {
    return estimate;
  }

  const RangeBins reference = buildBins(referenceScan_);
  const RangeBins current = buildBins(currentScan);

  MatchResult best;

  // Coarse search.
  for (float rotation = -config_.maxRotationDeg; rotation <= config_.maxRotationDeg;
       rotation += config_.coarseStepDeg) {
    const MatchResult candidate =
        evaluateRotation(reference, current, rotation, config_.maxRangeDifferenceM, config_.minimumMatches);

    if (candidate.valid && candidate.error < best.error) {
      best = candidate;
    }
  }

  if (!best.valid) {
    return estimate;
  }

  const float coarseRotation = best.rotationDeg;

  // Fine search around the best coarse candidate.
  for (float rotation = coarseRotation - config_.coarseStepDeg; rotation <= coarseRotation + config_.coarseStepDeg;
       rotation += config_.fineStepDeg) {
    if (std::fabs(rotation) > config_.maxRotationDeg) {
      continue;
    }

    const MatchResult candidate =
        evaluateRotation(reference, current, rotation, config_.maxRangeDifferenceM, config_.minimumMatches);

    if (candidate.valid && candidate.error < best.error) {
      best = candidate;
    }
  }

  estimate.rotationDeg = signedAngle(best.rotationDeg);
  estimate.matchingError = best.error;

  // Heuristic score, not a calibrated probability.
  estimate.confidence = std::clamp(best.overlap * (1.0f - best.error / config_.maxRangeDifferenceM), 0.0f, 1.0f);

  estimate.valid = best.valid;

  return estimate;
}
