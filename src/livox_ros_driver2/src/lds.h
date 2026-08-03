//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

// livox lidar data source

#ifndef LIVOX_ROS_DRIVER_LDS_H_
#define LIVOX_ROS_DRIVER_LDS_H_

#include <atomic>
#include <array>
#include <chrono>
#include <map>
#include <mutex>
#include <string>

#include "comm/semaphore.h"
#include "comm/comm.h"
#include "comm/cache_index.h"

namespace livox_ros {
/**
 * Lidar data source abstract.
 */
class Lds {
 public:
  explicit Lds(double publish_freq);
  virtual ~Lds();

  void StorageImuData(ImuData* imu_data);
  void StoragePointData(PointFrame* frame);

  int8_t GetHandle(const uint8_t lidar_type, const PointPacket* lidar_point);
  void PushLidarData(PointPacket* lidar_data, const uint8_t index, const uint64_t base_time);

  static void ResetLidar(LidarDevice *lidar);
  void ResetLds();

  void RequestExit();

  void ConfigureTopicNaming(const std::string& topic_namespace,
                            const std::string& single_lidar_name,
                            const std::map<std::string, std::string>& serial_names,
                            double device_timeout_sec);
  void RegisterLidarIdentity(uint8_t index, const std::string& serial);
  void MarkLidarSeen(uint8_t index);
  void RefreshLidarPresence();
  std::string GetLidarBaseTopic(uint8_t index) const;

  bool IsAllQueueEmpty();
  bool IsAllQueueReadStop();

  void CleanRequestExit() { request_exit_.store(false); }
  bool IsRequestExit() const { return request_exit_.load(); }
  virtual void PrepareExit(void);

  // get publishing frequency
  double GetLdsFrequency() { return publish_freq_; }

 public:
  uint8_t lidar_count_;                 /**< Lidar access handle. */
  LidarDevice lidars_[kMaxSourceLidar]{}; /**< Stable slot for each lidar IP. */
  Semaphore pcd_semaphore_;
  Semaphore imu_semaphore_;
  static CacheIndex cache_index_;
 protected:
  double publish_freq_;
 private:
  static bool IsLogicalName(const std::string& value);
  static std::string SerialTopicToken(const std::string& serial);
  void RecomputeLogicalNamesLocked();

  std::atomic_bool request_exit_;
  mutable std::mutex identity_mutex_;
  std::string topic_namespace_{"hardware"};
  std::string single_lidar_name_{"body_lidar"};
  std::map<std::string, std::string> serial_names_;
  double device_timeout_sec_{3.0};
  std::array<std::string, kMaxSourceLidar> serials_{};
  std::array<std::string, kMaxSourceLidar> logical_names_{};
  std::array<std::chrono::steady_clock::time_point, kMaxSourceLidar>
      last_seen_{};
  std::array<bool, kMaxSourceLidar> online_{};
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER_LDS_H_
