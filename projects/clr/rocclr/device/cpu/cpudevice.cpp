/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#include "device/cpu/cpudevice.hpp"

#include "device/cpu/cpuprogram.hpp"
#include "os/os.hpp"
#include "platform/context.hpp"
#include "platform/runtime.hpp"
#include "utils/debug.hpp"
#include "utils/flags.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace amd::cpu {

namespace {

//! Target ID of the host CPU device registered in the global ISA table.
constexpr char kHostIsaTargetId[] = "spvcpu";

//! Returns the host ISA registered in the global ISA table.
const amd::Isa* findHostIsa() {
  for (const amd::Isa* isa = amd::Isa::begin(); isa != amd::Isa::end(); ++isa) {
    if (std::strcmp(isa->targetId(), kHostIsaTargetId) == 0) {
      return isa;
    }
  }
  return nullptr;
}

}  // namespace

NullDevice::NullDevice() : amd::Device() { online_ = false; }

bool NullDevice::init() {
  const amd::Isa* isa = findHostIsa();
  if (isa == nullptr) {
    LogError("Host ISA is not registered in the runtime ISA table!");
    return false;
  }

  // Register the offline target only when no online device has the host ISA.
  for (const auto* device : getDevices(CL_DEVICE_TYPE_GPU, false)) {
    if (&device->isa() == isa) {
      return true;
    }
  }

  std::unique_ptr<NullDevice> device(new NullDevice());
  if (device == nullptr || !device->create(*isa)) {
    LogError("Unable to create the offline CPU device!");
    return false;
  }
  device.release()->registerDevice();
  return true;
}

void NullDevice::tearDown() {}

bool NullDevice::create(const amd::Isa& isa) {
  if (&isa != findHostIsa()) {
    LogError("CPU device requires the host ISA!");
    return false;
  }

  online_ = false;
  info_.type_ = CL_DEVICE_TYPE_GPU;
  info_.vendorId_ = AmdVendor;

  settings_ = new device::Settings();
  if (!ValidateComgr()) {
    LogError("Code object manager initialization failed for the CPU target!");
    return false;
  }
  if (settings_ == nullptr || !amd::Device::create(isa)) {
    LogError("Unable to setup the CPU target!");
    return false;
  }

  fillDeviceInfo();
  info_.extensions_ = getExtensionString();
  return true;
}

Device::Device() : NullDevice() {}

bool Device::init() {
  const amd::Isa* isa = findHostIsa();
  if (isa == nullptr) {
    LogError("Host ISA is not registered in the runtime ISA table!");
    return false;
  }

  Device* device = new Device();
  if (device == nullptr) {
    LogError("Error allocating a new instance of the host device!");
    return false;
  }

  if (!device->create(*isa)) {
    LogError("Unable to create the host device!");
    delete device;
    return false;
  }

  // Runs under the non-recursive spin lock in amd::Runtime::init(). Do not
  // call back into the runtime.
  device->registerDevice();
  return true;
}

void Device::tearDown() { NullDevice::tearDown(); }

bool Device::create(const amd::Isa& isa) {
  if (!NullDevice::create(isa)) {
    LogError("Unable to setup the host device!");
    return false;
  }

  online_ = true;
  info_.available_ = 1;
  // Copy the system-memory capabilities from the online device. The offline
  // target has no SVM devices.
  info_.svmCapabilities_ = CL_DEVICE_SVM_COARSE_GRAIN_BUFFER |
                           CL_DEVICE_SVM_FINE_GRAIN_BUFFER | CL_DEVICE_SVM_FINE_GRAIN_SYSTEM;

  std::vector<amd::Device*> devices{this};
  context_ = new amd::Context(devices, amd::Context::Info());
  if (context_ == nullptr) {
    return false;
  }

  return true;
}

device::Program* NullDevice::createProgram(amd::Program& owner, amd::option::Options* options) {
  device::Program* program = new cpu::Program(*this, owner);
  if (program == nullptr) {
    LogError("Memory allocation has failed!");
  }
  return program;
}

device::Memory* Device::createMemory(amd::Memory& owner) const {
  Memory* memory = new Memory(*const_cast<Device*>(this), owner);
  if (memory != nullptr && !memory->isValid()) {
    delete memory;
    return nullptr;
  }
  return memory;
}

void NullDevice::fillDeviceInfo() {
  info_.littleEndian_ = 1;
  info_.available_ = 0;
  info_.hostUnifiedMemory_ = 1;
  info_.executionCapabilities_ = CL_EXEC_KERNEL;

  ::strncpy(info_.name_, "CPU", sizeof(info_.name_) - 1);
  ::strncpy(info_.vendor_, "AMD", sizeof(info_.vendor_) - 1);
  ::strncpy(info_.boardName_, "CPU", sizeof(info_.boardName_) - 1);
  ::snprintf(info_.driverVersion_, sizeof(info_.driverVersion_), "%d.%d", 1, 0);

  // The host device reports one compute unit executing a single work item at a
  // time. The wavefront width must stay non-zero: the HIP runtime computes the
  // maximum blocks per compute unit as maxThreadsPerCU_ / wavefrontWidth_.
  info_.maxComputeUnits_ = 1;
  info_.maxPhysicalComputeUnits_ = 1;
  info_.wavefrontWidth_ = 1;
  info_.simdWidth_ = 1;
  info_.maxThreadsPerCU_ = 1;
  info_.maxWorkGroupSize_ = 1;
  info_.preferredWorkGroupSize_ = 1;
  info_.maxWorkItemDimensions_ = 3;
  for (size_t dim = 0; dim < 3; ++dim) {
    info_.maxWorkItemSizes_[dim] = 1;
    info_.maxGridDim_[dim] = UINT32_MAX;
  }
  info_.preferredVectorWidthChar_ = 1;
  info_.preferredVectorWidthShort_ = 1;
  info_.preferredVectorWidthInt_ = 1;
  info_.preferredVectorWidthLong_ = 1;
  info_.preferredVectorWidthFloat_ = 1;
  info_.preferredVectorWidthDouble_ = 1;
  info_.nativeVectorWidthChar_ = 1;
  info_.nativeVectorWidthShort_ = 1;
  info_.nativeVectorWidthInt_ = 1;
  info_.nativeVectorWidthLong_ = 1;
  info_.nativeVectorWidthFloat_ = 1;
  info_.nativeVectorWidthDouble_ = 1;
  info_.singleFPConfig_ = CL_FP_ROUND_TO_NEAREST;
  info_.doubleFPConfig_ = CL_FP_ROUND_TO_NEAREST;
  info_.halfFPConfig_ = CL_FP_ROUND_TO_NEAREST;

  // The host device provides system memory only. Report half of the host
  // physical memory, matching what the ROCr backend reports for system memory.
  info_.localMemType_ = CL_NONE;
  info_.localMemSize_ = 0;
  info_.localMemSizePerCU_ = 0;
  info_.globalMemSize_ = amd::Os::hostTotalPhysicalMemory() / 2;
  info_.globalMemSize_ = std::max(info_.globalMemSize_, static_cast<uint64_t>(1 * Gi));
  info_.maxMemAllocSize_ = amd::alignDown(info_.globalMemSize_, sizeof(uint64_t));
  info_.maxPhysicalMemAllocSize_ = amd::Os::getPhysicalMemSize();
  info_.maxConstantBufferSize_ = 64 * Ki;
  info_.preferredConstantBufferSize_ = 64 * Ki;
  info_.maxParameterSize_ = 1024;
  info_.globalMemCacheLineSize_ = 64;
  info_.memBaseAddrAlign_ = 128;
  info_.minDataTypeAlignSize_ = 128;

  // Offline targets cannot allocate or execute.
  info_.svmCapabilities_ = 0;
  info_.queueProperties_ = CL_QUEUE_PROFILING_ENABLE;

  info_.timeStampFrequency_ = 1000000;  // 1MHz, matching the POSIX clock resolution.
  info_.profilingTimerResolution_ = 1;
  info_.maxEngineClockFrequency_ = 0;
  info_.maxMemoryClockFrequency_ = 0;

  // Advertise only the extensions the host device can support.
  settings_->enableExtension(ClKhrFp64);
  settings_->enableExtension(ClAmdFp64);
  settings_->enableExtension(ClKhrGlobalInt32BaseAtomics);
  settings_->enableExtension(ClKhrGlobalInt32ExtendedAtomics);
  settings_->enableExtension(ClKhrLocalInt32BaseAtomics);
  settings_->enableExtension(ClKhrLocalInt32ExtendedAtomics);
  settings_->enableExtension(ClKhrInt64BaseAtomics);
  settings_->enableExtension(ClKhrInt64ExtendedAtomics);
  settings_->enableExtension(ClKhrByteAddressableStore);
}

}  // namespace amd::cpu
