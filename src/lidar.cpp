#include "lidar.h"

#include <algorithm>
#include <cstddef>
#include <cstdio>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
using namespace sl;

class Lidar::Impl {
 public:
  ILidarDriver* driver = nullptr;
  IChannel* channel = nullptr;
  bool connected = false;
};

Lidar::Lidar() : impl_(std::make_unique<Impl>()) {}

Lidar::~Lidar() {
  stop();

  if (!impl_) {
    return;
  }

  if (impl_->driver != nullptr) {
    impl_->driver->stop();
    impl_->driver->setMotorSpeed(0);

    delete impl_->driver;
    impl_->driver = nullptr;
  }

  impl_->channel = nullptr;
  impl_->connected = false;
}

bool Lidar::connect(const std::string& port, int baudRate) {
  if (!impl_) {
    return false;
  }

  impl_->driver = *createLidarDriver();

  if (impl_->driver == nullptr) {
    return false;
  }

  impl_->channel = *createSerialPortChannel(port.c_str(), baudRate);

  if (impl_->channel == nullptr) {
    delete impl_->driver;
    impl_->driver = nullptr;
    return false;
  }

  if (SL_IS_FAIL(impl_->driver->connect(impl_->channel))) {
    delete impl_->driver;
    impl_->driver = nullptr;
    impl_->channel = nullptr;
    return false;
  }

  sl_lidar_response_device_health_t health;

  if (SL_IS_FAIL(impl_->driver->getHealth(health))) {
    return false;
  }

  if (health.status == SL_LIDAR_STATUS_ERROR) {
    return false;
  }

  impl_->driver->setMotorSpeed();

  if (SL_IS_FAIL(impl_->driver->startScan(0, 1))) {
    return false;
  }

  impl_->connected = true;

  return true;
}

bool Lidar::acquireScan(LidarScan& scan) {
  if (!impl_ || !impl_->connected || impl_->driver == nullptr) {
    return false;
  }

  sl_lidar_response_measurement_node_hq_t nodes[8192];

  for (int attempt = 0; attempt < 10; ++attempt) {
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    const sl_result result = impl_->driver->grabScanDataHq(nodes, count);

    if (result == SL_RESULT_OPERATION_TIMEOUT) {
      continue;
    }

    if (SL_IS_FAIL(result)) {
      std::fprintf(stderr, "grabScanDataHq failed: 0x%08X\n", static_cast<unsigned int>(result));
      return false;
    }

    if (count == 0) {
      continue;
    }

    const sl_result ascendResult = impl_->driver->ascendScanData(nodes, count);

    if (SL_IS_FAIL(ascendResult)) {
      std::fprintf(stderr, "ascendScanData failed: 0x%08X\n", static_cast<unsigned int>(ascendResult));
      return false;
    }

    scan.points.clear();
    scan.points.reserve(count);

    for (size_t i = 0; i < count; ++i) {
      const float distanceMm = nodes[i].dist_mm_q2 / 4.0f;

      const float quality = static_cast<float>(nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

      if (distanceMm <= 0.0f || quality <= 0.0f) {
        continue;
      }

      LidarPoint point;

      point.angleDeg = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;

      point.distanceM = distanceMm / 1000.0f;
      point.quality = quality;

      scan.points.push_back(point);
    }

    return !scan.points.empty();
  }

  return false;
}
bool Lidar::start() {
  if (!impl_ || !impl_->connected) {
    return false;
  }

  if (running_) {
    return true;
  }

  running_ = true;

  acquisitionThread_ = std::thread(&Lidar::acquisitionLoop, this);

  return true;
}

void Lidar::stop() {
  running_ = false;

  if (acquisitionThread_.joinable()) {
    acquisitionThread_.join();
  }
}

void Lidar::acquisitionLoop() {
  uint64_t sequence = 0;

  while (running_) {
    LidarScan scan;

    if (!acquireScan(scan)) {
      continue;
    }

    scan.sequence = ++sequence;

    {
      std::lock_guard<std::mutex> lock(scanMutex_);
      latestScan_ = std::move(scan);
    }
  }
}

bool Lidar::getLatestScan(LidarScan& scan) const {
  std::lock_guard<std::mutex> lock(scanMutex_);

  if (latestScan_.points.empty()) {
    return false;
  }

  scan = latestScan_;
  return true;
}

bool Lidar::isRunning() const { return running_; }
bool Lidar::isConnected() const { return impl_ && impl_->connected; }