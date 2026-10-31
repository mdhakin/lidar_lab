
#include "scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

struct Correspondence {
  Point2D source;
  Point2D target;
};

double normalizeAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

Point2D transformPoint(const Point2D& p, const Pose2D& pose) {
  const double c = std::cos(pose.headingRad);
  const double s = std::sin(pose.headingRad);

  return {c * p.x - s * p.y + pose.xM, s * p.x + c * p.y + pose.yM};
}

Pose2D compose(const Pose2D& a, const Pose2D& b) {
  const Point2D t = transformPoint({b.xM, b.yM}, a);

  return {t.x, t.y, normalizeAngle(a.headingRad + b.headingRad)};
}

std::vector<Point2D> toCartesian(const LidarScan& scan) {
  std::vector<Point2D> points;
  points.reserve(scan.points.size());

  for (const auto& p : scan.points) {
    if (!std::isfinite(p.angleDeg) || !std::isfinite(p.distanceM) || p.distanceM <= 0.0f) {
      continue;
    }

    const double angle = static_cast<double>(p.angleDeg) * kPi / 180.0;

    const double distance = p.distanceM;

    points.push_back({distance * std::cos(angle), distance * std::sin(angle)});
  }

  return points;
}

double distanceSquared(const Point2D& a, const Point2D& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;

  return dx * dx + dy * dy;
}

// Finds a rigid transformation mapping source points
// to their corresponding target points.
Pose2D solveRigidTransform(const std::vector<Correspondence>& pairs) {
  Point2D sourceCenter{};
  Point2D targetCenter{};

  const double count = static_cast<double>(pairs.size());

  for (const auto& pair : pairs) {
    sourceCenter.x += pair.source.x;
    sourceCenter.y += pair.source.y;

    targetCenter.x += pair.target.x;
    targetCenter.y += pair.target.y;
  }

  sourceCenter.x /= count;
  sourceCenter.y /= count;

  targetCenter.x /= count;
  targetCenter.y /= count;

  double dot = 0.0;
  double cross = 0.0;

  for (const auto& pair : pairs) {
    const double sx = pair.source.x - sourceCenter.x;
    const double sy = pair.source.y - sourceCenter.y;

    const double tx = pair.target.x - targetCenter.x;
    const double ty = pair.target.y - targetCenter.y;

    dot += sx * tx + sy * ty;
    cross += sx * ty - sy * tx;
  }

  const double angle = std::atan2(cross, dot);

  const double c = std::cos(angle);
  const double s = std::sin(angle);

  const double x = targetCenter.x - (c * sourceCenter.x - s * sourceCenter.y);

  const double y = targetCenter.y - (s * sourceCenter.x + c * sourceCenter.y);

  return {x, y, angle};
}

}  // namespace

ScanMatcher2D::ScanMatcher2D(const ScanMatcherConfig& config) : config_(config) {}

ScanMatchResult ScanMatcher2D::match(const LidarScan& reference, const LidarScan& current) const {
  ScanMatchResult result;

  const auto referencePoints = toCartesian(reference);
  const auto currentPoints = toCartesian(current);

  if (referencePoints.size() < config_.minimumMatches || currentPoints.size() < config_.minimumMatches ||
      config_.maxIterations == 0 || config_.maxCorrespondenceDistanceM <= 0.0) {
    return result;
  }

  Pose2D estimate{};

  const double maxDistanceSq = config_.maxCorrespondenceDistanceM * config_.maxCorrespondenceDistanceM;

  for (unsigned int iteration = 0; iteration < config_.maxIterations; ++iteration) {
    std::vector<Correspondence> pairs;
    pairs.reserve(currentPoints.size());

    for (const auto& currentPoint : currentPoints) {
      const Point2D transformed = transformPoint(currentPoint, estimate);

      double bestDistanceSq = maxDistanceSq;
      const Point2D* nearest = nullptr;

      for (const auto& referencePoint : referencePoints) {
        const double d2 = distanceSquared(transformed, referencePoint);

        if (d2 < bestDistanceSq) {
          bestDistanceSq = d2;
          nearest = &referencePoint;
        }
      }

      if (nearest != nullptr) {
        pairs.push_back({transformed, *nearest});
      }
    }

    if (pairs.size() < config_.minimumMatches) {
      return result;
    }

    const Pose2D correction = solveRigidTransform(pairs);

    estimate = compose(correction, estimate);

    const double translation = std::hypot(correction.xM, correction.yM);

    const double rotation = std::fabs(correction.headingRad);

    if (translation < config_.convergenceTolerance && rotation < config_.convergenceTolerance) {
      break;
    }
  }

  // Evaluate the final transformation independently.
  double totalSquaredError = 0.0;
  unsigned int matched = 0;

  for (const auto& currentPoint : currentPoints) {
    const Point2D transformed = transformPoint(currentPoint, estimate);

    double bestDistanceSq = maxDistanceSq;

    for (const auto& referencePoint : referencePoints) {
      bestDistanceSq = std::min(bestDistanceSq, distanceSquared(transformed, referencePoint));
    }

    if (bestDistanceSq < maxDistanceSq) {
      totalSquaredError += bestDistanceSq;
      ++matched;
    }
  }

  if (matched < config_.minimumMatches) {
    return result;
  }

  // Reject transformations outside configured bounds.
  if (std::hypot(estimate.xM, estimate.yM) > config_.maxTranslationM ||
      std::fabs(estimate.headingRad) > config_.maxRotationRad) {
    return result;
  }

  result.relativePose = estimate;
  result.matchedPoints = matched;

  result.matchingErrorM = std::sqrt(totalSquaredError / matched);

  // Heuristic match coverage, not a probability.
  result.confidence = static_cast<double>(matched) / static_cast<double>(currentPoints.size());

  result.valid = true;

  return result;
}
