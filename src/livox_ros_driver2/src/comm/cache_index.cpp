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

#include "cache_index.h"
#include "livox_lidar_def.h"

namespace livox_ros {

CacheIndex::CacheIndex() {
  std::array<bool, kMaxLidarCount> index_cache = {0};
  index_cache_.swap(index_cache);
}

int8_t CacheIndex::GetFreeIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index) {
  std::string key;
  int8_t ret = GenerateIndexKey(livox_lidar_type, handle, key);
  if (ret != 0) {
    return -1;
  }
  std::lock_guard<std::mutex> lock(index_mutex_);
  auto existing = map_index_.find(key);
  if (existing != map_index_.end()) {
    index = existing->second;
    return 0;
  }

  printf("GetFreeIndex key:%s.\n", key.c_str());
  for (size_t i = 0; i < kMaxSourceLidar; ++i) {
    if (!index_cache_[i]) {
      index_cache_[i] = true;
      map_index_[key] = static_cast<uint8_t>(i);
      index = static_cast<uint8_t>(i);
      return 0;
    }
  }
  return -1;
}

int8_t CacheIndex::GenerateIndexKey(const uint8_t livox_lidar_type, const uint32_t handle, std::string& key) {
  if (livox_lidar_type == kLivoxLidarType) {
    key = "livox_lidar_" + std::to_string(handle);
  } else {
    printf("Can not generate index, the livox lidar type is unknown, the livox lidar type:%u\n", livox_lidar_type);
    return -1;
  }
  return 0;
}

int8_t CacheIndex::GetIndex(const uint8_t livox_lidar_type,
                            const uint32_t handle, uint8_t& index,
                            bool log_if_missing) {
  std::string key;
  int8_t ret = GenerateIndexKey(livox_lidar_type, handle, key);
  if (ret != 0) {
    return -1;
  }

  std::lock_guard<std::mutex> lock(index_mutex_);
  auto existing = map_index_.find(key);
  if (existing != map_index_.end()) {
    index = existing->second;
    return 0;
  }
  if (log_if_missing) {
    printf("Can not get index, the livox lidar type:%u, handle:%u\n",
           livox_lidar_type, handle);
  }
  return -1;
}

void CacheIndex::ResetIndex(LidarDevice *lidar) {
  std::string key;
  int8_t ret = GenerateIndexKey(lidar->lidar_type, lidar->handle, key);
  if (ret != 0) {
    printf("Reset index failed, can not generate index key, lidar type:%u, handle:%u.\n", lidar->lidar_type, lidar->handle);
    return;
  }

  std::lock_guard<std::mutex> lock(index_mutex_);
  auto existing = map_index_.find(key);
  if (existing != map_index_.end()) {
    uint8_t index = existing->second;
    map_index_.erase(existing);
    index_cache_[index] = 0;
  }
}

} // namespace
