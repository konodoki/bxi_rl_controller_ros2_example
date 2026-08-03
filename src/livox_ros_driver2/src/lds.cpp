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

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <stdexcept>

#include "lds.h"
#include "comm/ldq.h"

namespace livox_ros {

CacheIndex Lds::cache_index_;

/* Member function --------------------------------------------------------- */
Lds::Lds(double publish_freq)
    : lidar_count_(kMaxSourceLidar),
      pcd_semaphore_(0),
      imu_semaphore_(0),
      publish_freq_(publish_freq),
      request_exit_(false) {
  ResetLds();
}

Lds::~Lds() {
  ResetLds();
  printf("Lidar data source destroyed.\n");
}

void Lds::ResetLidar(LidarDevice *lidar) {
  //cache_index_.ResetIndex(lidar);
  DeInitQueue(&lidar->data);
  lidar->imu_data.Clear();

  lidar->lidar_type = 0;
  lidar->handle = 0;
  lidar->user_configured = false;
  lidar->connect_state.store(kConnectStateOff);
}

void Lds::ResetLds() {
  lidar_count_ = kMaxSourceLidar;
  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    ResetLidar(&lidars_[i]);
  }
}

void Lds::RequestExit() {
  request_exit_.store(true);
  // Wake both distributor threads even if no lidar has produced data.
  pcd_semaphore_.Signal();
  imu_semaphore_.Signal();
}

bool Lds::IsLogicalName(const std::string& value) {
  if (value.empty() ||
      !std::isalpha(static_cast<unsigned char>(value.front()))) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](char ch) {
    const auto value = static_cast<unsigned char>(ch);
    return std::isalnum(value) || ch == '_';
  });
}

std::string Lds::SerialTopicToken(const std::string& serial) {
  std::string token = "SN_";
  token.reserve(token.size() + serial.size());
  for (char ch : serial) {
    const auto value = static_cast<unsigned char>(ch);
    token.push_back(std::isalnum(value) || ch == '_' ? ch : '_');
  }
  return token;
}

void Lds::ConfigureTopicNaming(
    const std::string& topic_namespace,
    const std::string& single_lidar_name,
    const std::map<std::string, std::string>& serial_names,
    double device_timeout_sec) {
  std::string normalized_namespace = topic_namespace;
  while (!normalized_namespace.empty() &&
         normalized_namespace.front() == '/') {
    normalized_namespace.erase(normalized_namespace.begin());
  }
  while (!normalized_namespace.empty() &&
         normalized_namespace.back() == '/') {
    normalized_namespace.pop_back();
  }
  if (normalized_namespace.empty()) {
    throw std::invalid_argument("topic_namespace must not be empty");
  }
  if (!IsLogicalName(single_lidar_name)) {
    throw std::invalid_argument(
        "single_lidar_name must start with a letter and contain only "
        "letters, digits, or underscores");
  }
  if (device_timeout_sec <= 0.0) {
    throw std::invalid_argument("device_timeout_sec must be positive");
  }

  std::map<std::string, std::string> logical_owners;
  for (const auto& mapping : serial_names) {
    if (mapping.first.empty()) {
      throw std::invalid_argument("lidar serial mapping must not be empty");
    }
    if (!IsLogicalName(mapping.second)) {
      throw std::invalid_argument(
          "mapped lidar name must start with a letter and contain only "
          "letters, digits, or underscores: " + mapping.second);
    }
    auto inserted = logical_owners.emplace(mapping.second, mapping.first);
    if (!inserted.second && inserted.first->second != mapping.first) {
      throw std::invalid_argument("logical lidar name is assigned to more than "
                                  "one serial: " + mapping.second);
    }
  }

  std::lock_guard<std::mutex> lock(identity_mutex_);
  topic_namespace_ = normalized_namespace;
  single_lidar_name_ = single_lidar_name;
  serial_names_ = serial_names;
  device_timeout_sec_ = device_timeout_sec;
  RecomputeLogicalNamesLocked();
}

void Lds::RegisterLidarIdentity(uint8_t index, const std::string& serial) {
  if (index >= kMaxSourceLidar) {
    return;
  }
  std::lock_guard<std::mutex> lock(identity_mutex_);
  serials_[index] = serial.empty()
      ? "IP_" + ReplacePeriodByUnderline(IpNumToString(lidars_[index].handle))
      : serial;
  last_seen_[index] = std::chrono::steady_clock::now();
  online_[index] = true;
  RecomputeLogicalNamesLocked();
}

void Lds::MarkLidarSeen(uint8_t index) {
  if (index >= kMaxSourceLidar) {
    return;
  }
  std::lock_guard<std::mutex> lock(identity_mutex_);
  last_seen_[index] = std::chrono::steady_clock::now();
  if (!online_[index]) {
    online_[index] = true;
    RecomputeLogicalNamesLocked();
  }
}

void Lds::RefreshLidarPresence() {
  const auto now = std::chrono::steady_clock::now();
  bool changed = false;
  std::lock_guard<std::mutex> lock(identity_mutex_);
  for (uint8_t index = 0; index < kMaxSourceLidar; ++index) {
    if (online_[index] && !serials_[index].empty() &&
        std::chrono::duration<double>(now - last_seen_[index]).count() >
            device_timeout_sec_) {
      online_[index] = false;
      changed = true;
    }
  }
  if (changed) {
    RecomputeLogicalNamesLocked();
  }
}

std::string Lds::GetLidarBaseTopic(uint8_t index) const {
  std::lock_guard<std::mutex> lock(identity_mutex_);
  const std::string& logical_name = logical_names_[index];
  return "/" + topic_namespace_ + "/" +
         (logical_name.empty() ? single_lidar_name_ : logical_name);
}

void Lds::RecomputeLogicalNamesLocked() {
  size_t online_count = 0;
  for (uint8_t index = 0; index < kMaxSourceLidar; ++index) {
    if (online_[index] && !serials_[index].empty()) {
      ++online_count;
    }
  }

  bool single_name_reserved = false;
  for (const auto& mapping : serial_names_) {
    if (mapping.second == single_lidar_name_) {
      single_name_reserved = true;
      break;
    }
  }

  for (uint8_t index = 0; index < kMaxSourceLidar; ++index) {
    if (serials_[index].empty()) {
      continue;
    }
    auto mapped = serial_names_.find(serials_[index]);
    if (mapped != serial_names_.end()) {
      logical_names_[index] = mapped->second;
    } else if (online_[index] && online_count == 1 &&
               !single_name_reserved) {
      logical_names_[index] = single_lidar_name_;
    } else {
      logical_names_[index] = SerialTopicToken(serials_[index]);
    }
  }
}

bool Lds::IsAllQueueEmpty() {
  for (int i = 0; i < lidar_count_; i++) {
    if (!QueueIsEmpty(&lidars_[i].data)) {
      return false;
    }
  }
  return true;
}

bool Lds::IsAllQueueReadStop() {
  for (int i = 0; i < lidar_count_; i++) {
    uint32_t data_size = QueueUsedSize(&lidars_[i].data);
    if (data_size) {
      return false;
    }
  }
  return true;
}

void Lds::StorageImuData(ImuData* imu_data) {
  uint32_t device_num = 0;
  if (imu_data->lidar_type == kLivoxLidarType) {
    device_num = imu_data->handle;
  } else {
    printf("Storage imu data failed, unknown lidar type:%u.\n", imu_data->lidar_type);
    return;
  }

  uint8_t index = 0;
  int ret = cache_index_.GetIndex(imu_data->lidar_type, device_num, index);
  if (ret != 0) {
    printf("Storage point data failed, can not get index, lidar type:%u, device_num:%u.\n", imu_data->lidar_type, device_num);
    return;
  }

  LidarDevice *p_lidar = &lidars_[index];
  MarkLidarSeen(index);
  p_lidar->connect_state.store(kConnectStateSampling,
                               std::memory_order_release);
  LidarImuDataQueue* imu_queue = &p_lidar->imu_data;
  imu_queue->Push(imu_data);
  if (!imu_queue->Empty()) {
    if (imu_semaphore_.GetCount() <= 0) {
      imu_semaphore_.Signal();
    }
  }
}

void Lds::StoragePointData(PointFrame* frame) {
  if (frame == nullptr) {
    return;
  }

  uint8_t lidar_number = frame->lidar_num;
  for (uint i = 0; i < lidar_number; ++i) {
    PointPacket& lidar_point = frame->lidar_point[i];
    //printf("StoragePointData, lidar_type:%u, point_num:%lu.\n", lidar_point.lidar_type, lidar_point.points_num);

    uint64_t base_time = frame->base_time[i];

    uint8_t index = 0;
    int8_t ret = cache_index_.GetIndex(lidar_point.lidar_type, lidar_point.handle, index);
    if (ret != 0) {
      printf("Storage point data failed, lidar type:%u, handle:%u.\n", lidar_point.lidar_type, lidar_point.handle);
      continue;
    }
    MarkLidarSeen(index);
    // Incoming data is authoritative proof that the device is sampling, even
    // when it does not support one of the optional configuration commands.
    lidars_[index].connect_state.store(kConnectStateSampling,
                                       std::memory_order_release);
    PushLidarData(&lidar_point, index, base_time);
  }
}

void Lds::PushLidarData(PointPacket* lidar_data, const uint8_t index, const uint64_t base_time) {
  if (lidar_data == nullptr) {
    return;
  }

  LidarDevice *p_lidar = &lidars_[index];
  LidarDataQueue *queue = &p_lidar->data;

  if (nullptr == queue->storage_packet) {
    uint32_t queue_size = CalculatePacketQueueSize(publish_freq_);
    InitQueue(queue, queue_size);
    printf("Lidar[%u] storage queue size: %u\n", index, queue_size);
  }

  if (!QueueIsFull(queue)) {
    QueuePushAny(queue, (uint8_t *)lidar_data, base_time);
    if (!QueueIsEmpty(queue)) {
      if (pcd_semaphore_.GetCount() <= 0) {
        pcd_semaphore_.Signal();
      }
    }
  } else {
    if (pcd_semaphore_.GetCount() <= 0) {
        pcd_semaphore_.Signal();
    }
  }
}

void Lds::PrepareExit(void) {}

}  // namespace livox_ros
