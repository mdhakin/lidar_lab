#ifndef LIDAR_H
#define LIDAR_H

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct LidarPoint {
  float angleDeg = 0.0f;
  float distanceM = 0.0f;
  float quality = 0.0f;
};

struct LidarScan {
  std::vector<LidarPoint> points;
};

class Lidar {
 public:
  Lidar();
  ~Lidar();

  Lidar(const Lidar&) = delete;
  Lidar& operator=(const Lidar&) = delete;

  bool connect(const std::string& port, int baudRate);

  bool start();
  void stop();

  bool getLatestScan(LidarScan& scan) const;

  bool isConnected() const;
  bool isRunning() const;

 private:
  void acquisitionLoop();
  bool acquireScan(LidarScan& scan);

  class Impl;
  std::unique_ptr<Impl> impl_;

  mutable std::mutex scanMutex_;
  LidarScan latestScan_;

  std::atomic<bool> running_{false};
  std::thread acquisitionThread_;
};

#endif