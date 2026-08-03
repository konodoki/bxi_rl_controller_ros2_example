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

#include "livox_lidar_callback.h"

#include "livox_lidar_api.h"
#include <chrono>
#include <string>
#include <thread>
#include <iostream>

namespace livox_ros {

namespace {

void CompleteConfigStep(LdsLidar* lds_lidar, LidarDevice* lidar_device,
                        uint32_t step) {
  std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
  lidar_device->livox_config.set_bits &= ~step;
  if (lidar_device->livox_config.set_bits == 0) {
    lidar_device->connect_state.store(kConnectStateSampling,
                                      std::memory_order_release);
  }
}

void LogCommandFailure(const char* command, uint32_t handle,
                       livox_status status,
                       const LivoxLidarAsyncControlResponse* response) {
  std::cout << "failed to " << command << ", ip: " << IpNumToString(handle)
            << ", status: " << status;
  if (response != nullptr) {
    std::cout << ", return code: " << response->ret_code
              << ", error key: " << response->error_key;
  }
  std::cout << std::endl;
}

std::string LidarSerial(const LivoxLidarInfo* info) {
  if (info == nullptr) {
    return {};
  }
  std::string serial(info->sn, sizeof(info->sn));
  const auto nul = serial.find('\0');
  if (nul != std::string::npos) {
    serial.resize(nul);
  }
  return serial;
}

}  // namespace

void LivoxLidarCallback::LidarInfoChangeCallback(const uint32_t handle,
                                                  const LivoxLidarInfo* info,
                                                  void* client_data) {
  if (client_data == nullptr) {
    std::cout << "lidar info change callback failed, client data is nullptr" << std::endl;
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  uint8_t index = 0;
  LidarDevice* lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    if (!lds_lidar->IsAutoConnectMode()) {
      std::cout << "ignore unconfigured lidar because auto_connect is false, ip: "
                << IpNumToString(handle) << std::endl;
      return;
    }

    int8_t ret = lds_lidar->cache_index_.GetFreeIndex(kLivoxLidarType, handle, index);
    if (ret != 0) {
      std::cout << "failed to add lidar device, lidar ip: " << IpNumToString(handle) << std::endl;
      return;
    }
    lidar_device = &(lds_lidar->lidars_[index]);
    UserLivoxLidarConfig default_config{};
    default_config.handle = handle;
    default_config.pcl_data_type = -1;
    default_config.pattern_mode = -1;
    default_config.blind_spot_set = -1;
    default_config.dual_emit_en = -1;
    lidar_device->livox_config = default_config;
    lidar_device->user_configured = false;
  } else if (lds_lidar->cache_index_.GetIndex(
                 kLivoxLidarType, handle, index, false) != 0) {
    std::cout << "failed to resolve lidar index, ip: "
              << IpNumToString(handle) << std::endl;
    return;
  }

  lidar_device->lidar_type = kLivoxLidarType;
  lidar_device->handle = handle;
  const std::string serial = LidarSerial(info);
  lds_lidar->RegisterLidarIdentity(index, serial);
  lidar_device->connect_state.store(kConnectStateConfig,
                                     std::memory_order_release);

  const UserLivoxLidarConfig& config = lidar_device->livox_config;
  uint32_t pending_steps = kConfigWorkMode;
  if (lidar_device->user_configured) {
    if (config.pcl_data_type != -1) {
      pending_steps |= kConfigDataType;
    }
    if (config.pattern_mode != -1) {
      pending_steps |= kConfigScanPattern;
    }
    if (config.blind_spot_set != -1) {
      pending_steps |= kConfigBlindSpot;
    }
    if (config.dual_emit_en != -1) {
      pending_steps |= kConfigDualEmit;
    }
  }
  {
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
    lidar_device->livox_config.set_bits = pending_steps;
  }

  std::cout << (lidar_device->user_configured ? "connect configured lidar" :
                "auto-connect discovered lidar")
            << ", ip: " << IpNumToString(handle)
            << ", sn: " << (serial.empty() ? "unknown" : serial)
            << ", device type: "
            << (info == nullptr ? -1 : static_cast<int>(info->dev_type))
            << std::endl;

  if (lidar_device->user_configured) {
    if (config.pcl_data_type != -1) {
      const livox_status status = SetLivoxLidarPclDataType(
          handle, static_cast<LivoxLidarPointDataType>(config.pcl_data_type),
          LivoxLidarCallback::SetDataTypeCallback, lds_lidar);
      if (status != kLivoxLidarStatusSuccess) {
        LogCommandFailure("queue data type configuration", handle, status,
                          nullptr);
        CompleteConfigStep(lds_lidar, lidar_device, kConfigDataType);
      }
    }
    if (config.pattern_mode != -1) {
      const livox_status status = SetLivoxLidarScanPattern(
          handle, static_cast<LivoxLidarScanPattern>(config.pattern_mode),
          LivoxLidarCallback::SetPatternModeCallback, lds_lidar);
      if (status != kLivoxLidarStatusSuccess) {
        LogCommandFailure("queue scan pattern configuration", handle, status,
                          nullptr);
        CompleteConfigStep(lds_lidar, lidar_device, kConfigScanPattern);
      }
    }
    if (config.blind_spot_set != -1) {
      const livox_status status = SetLivoxLidarBlindSpot(
          handle, config.blind_spot_set,
          LivoxLidarCallback::SetBlindSpotCallback, lds_lidar);
      if (status != kLivoxLidarStatusSuccess) {
        LogCommandFailure("queue blind spot configuration", handle, status,
                          nullptr);
        CompleteConfigStep(lds_lidar, lidar_device, kConfigBlindSpot);
      }
    }
    if (config.dual_emit_en != -1) {
      const livox_status status = SetLivoxLidarDualEmit(
          handle, config.dual_emit_en != 0,
          LivoxLidarCallback::SetDualEmitCallback, lds_lidar);
      if (status != kLivoxLidarStatusSuccess) {
        LogCommandFailure("queue dual emit configuration", handle, status,
                          nullptr);
        CompleteConfigStep(lds_lidar, lidar_device, kConfigDualEmit);
      }
    }

    // set extrinsic params into lidar
    LivoxLidarInstallAttitude attitude {
      config.extrinsic_param.roll,
      config.extrinsic_param.pitch,
      config.extrinsic_param.yaw,
      config.extrinsic_param.x,
      config.extrinsic_param.y,
      config.extrinsic_param.z
    };
    const livox_status status = SetLivoxLidarInstallAttitude(
        config.handle, &attitude, LivoxLidarCallback::SetAttitudeCallback,
        lds_lidar);
    if (status != kLivoxLidarStatusSuccess) {
      LogCommandFailure("queue install attitude configuration", handle,
                        status, nullptr);
    }
  }

  std::cout << "begin to change work mode to 'Normal', handle: " << handle << std::endl;
  const livox_status work_mode_status = SetLivoxLidarWorkMode(
      handle, kLivoxLidarNormal, WorkModeChangedCallback, lds_lidar);
  if (work_mode_status != kLivoxLidarStatusSuccess) {
    LogCommandFailure("queue Normal work mode", handle, work_mode_status,
                      nullptr);
    CompleteConfigStep(lds_lidar, lidar_device, kConfigWorkMode);
  }

  const livox_status imu_status = EnableLivoxLidarImuData(
      handle, LivoxLidarCallback::EnableLivoxLidarImuDataCallback, lds_lidar);
  if (imu_status != kLivoxLidarStatusSuccess) {
    LogCommandFailure("queue lidar IMU enable", handle, imu_status, nullptr);
  }
}

void LivoxLidarCallback::WorkModeChangedCallback(livox_status status,
                                                 uint32_t handle,
                                                 LivoxLidarAsyncControlResponse *response,
                                                 void *client_data) {
  LidarDevice* lidar_device = GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  if (status == kLivoxLidarStatusTimeout) {
    std::cout << "change work mode timeout, handle: " << handle
              << ", try again..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeChangedCallback,
                          client_data);
    return;
  }
  if (status == kLivoxLidarStatusSuccess) {
    std::cout << "successfully changed work mode, handle: " << handle << std::endl;
  } else {
    LogCommandFailure("change work mode", handle, status, response);
  }
  CompleteConfigStep(lds_lidar, lidar_device, kConfigWorkMode);
}

void LivoxLidarCallback::SetDataTypeCallback(livox_status status, uint32_t handle,
                                             LivoxLidarAsyncControlResponse *response,
                                             void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    std::cout << "failed to set data type since no lidar device found, handle: "
              << handle << std::endl;
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    std::cout << "successfully set data type, handle: " << handle << std::endl;
    CompleteConfigStep(lds_lidar, lidar_device, kConfigDataType);
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarPclDataType(handle, static_cast<LivoxLidarPointDataType>(config.pcl_data_type),
                             LivoxLidarCallback::SetDataTypeCallback, client_data);
    std::cout << "set data type timeout, handle: " << handle
              << ", try again..." << std::endl;
  } else {
    LogCommandFailure("set data type", handle, status, response);
    CompleteConfigStep(lds_lidar, lidar_device, kConfigDataType);
  }
}

void LivoxLidarCallback::SetPatternModeCallback(livox_status status, uint32_t handle,
                                                LivoxLidarAsyncControlResponse *response,
                                                void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    std::cout << "failed to set pattern mode since no lidar device found, handle: "
              << handle << std::endl;
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    std::cout << "successfully set pattern mode, handle: " << handle << std::endl;
    CompleteConfigStep(lds_lidar, lidar_device, kConfigScanPattern);
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarScanPattern(handle, static_cast<LivoxLidarScanPattern>(config.pattern_mode),
                             LivoxLidarCallback::SetPatternModeCallback, client_data);
    std::cout << "set pattern mode timeout, handle: " << handle
              << ", try again..." << std::endl;
  } else {
    LogCommandFailure("set pattern mode", handle, status, response);
    CompleteConfigStep(lds_lidar, lidar_device, kConfigScanPattern);
  }
}

void LivoxLidarCallback::SetBlindSpotCallback(livox_status status, uint32_t handle,
                                              LivoxLidarAsyncControlResponse *response,
                                              void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    std::cout << "failed to set blind spot since no lidar device found, handle: "
              << handle << std::endl;
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    std::cout << "successfully set blind spot, handle: " << handle << std::endl;
    CompleteConfigStep(lds_lidar, lidar_device, kConfigBlindSpot);
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarBlindSpot(handle, config.blind_spot_set,
                           LivoxLidarCallback::SetBlindSpotCallback, client_data);
    std::cout << "set blind spot timeout, handle: " << handle
              << ", try again..." << std::endl;
  } else {
    LogCommandFailure("set blind spot", handle, status, response);
    CompleteConfigStep(lds_lidar, lidar_device, kConfigBlindSpot);
  }
}

void LivoxLidarCallback::SetDualEmitCallback(livox_status status, uint32_t handle,
                                             LivoxLidarAsyncControlResponse *response,
                                             void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    std::cout << "failed to set dual emit mode since no lidar device found, handle: "
              << handle << std::endl;
    return;
  }

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);
  if (status == kLivoxLidarStatusSuccess) {
    std::cout << "successfully set dual emit mode, handle: " << handle << std::endl;
    CompleteConfigStep(lds_lidar, lidar_device, kConfigDualEmit);
  } else if (status == kLivoxLidarStatusTimeout) {
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarDualEmit(handle, config.dual_emit_en,
                          LivoxLidarCallback::SetDualEmitCallback, client_data);
    std::cout << "set dual emit mode timeout, handle: " << handle
              << ", try again..." << std::endl;
  } else {
    LogCommandFailure("set dual emit mode", handle, status, response);
    CompleteConfigStep(lds_lidar, lidar_device, kConfigDualEmit);
  }
}

void LivoxLidarCallback::SetAttitudeCallback(livox_status status, uint32_t handle,
                                             LivoxLidarAsyncControlResponse *response,
                                             void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    std::cout << "failed to set dual emit mode since no lidar device found, handle: "
              << handle << std::endl;
    return;
  }

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);
  if (status == kLivoxLidarStatusSuccess) {
    std::cout << "successfully set lidar attitude, ip: " << IpNumToString(handle) << std::endl;
  } else if (status == kLivoxLidarStatusTimeout) {
    std::cout << "set lidar attitude timeout, ip: " << IpNumToString(handle)
              << ", try again..." << std::endl;
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    LivoxLidarInstallAttitude attitude {
      config.extrinsic_param.roll,
      config.extrinsic_param.pitch,
      config.extrinsic_param.yaw,
      config.extrinsic_param.x,
      config.extrinsic_param.y,
      config.extrinsic_param.z
    };
    SetLivoxLidarInstallAttitude(config.handle, &attitude,
                                 LivoxLidarCallback::SetAttitudeCallback, lds_lidar);
  } else {
    std::cout << "failed to set lidar attitude, ip: " << IpNumToString(handle) << std::endl;
  }
}

void LivoxLidarCallback::EnableLivoxLidarImuDataCallback(livox_status status, uint32_t handle,
                                                         LivoxLidarAsyncControlResponse *response,
                                                         void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data);
  if (lidar_device == nullptr) {
    std::cout << "failed to set pattern mode since no lidar device found, handle: "
              << handle << std::endl;
    return;
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);

  if (status == kLivoxLidarStatusSuccess) {
    std::cout << "successfully enable Livox Lidar imu, ip: " << IpNumToString(handle) << std::endl;
  } else if (status == kLivoxLidarStatusTimeout) {
    std::cout << "enable Livox Lidar imu timeout, ip: " << IpNumToString(handle)
              << ", try again..." << std::endl;
    EnableLivoxLidarImuData(handle, LivoxLidarCallback::EnableLivoxLidarImuDataCallback, lds_lidar);
  } else {
    LogCommandFailure("enable lidar IMU", handle, status, response);
  }
}

LidarDevice* LivoxLidarCallback::GetLidarDevice(const uint32_t handle, void* client_data) {
  if (client_data == nullptr) {
    std::cout << "failed to get lidar device, client data is nullptr" << std::endl;
    return nullptr;
  }

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data);
  uint8_t index = 0;
  int8_t ret = lds_lidar->cache_index_.GetIndex(kLivoxLidarType, handle,
                                                 index, false);
  if (ret != 0) {
    return nullptr;
  }

  return &(lds_lidar->lidars_[index]);
}

} // namespace livox_ros
