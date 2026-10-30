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
#include <thread>
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

  if (!threadPool_.start(std::thread::hardware_concurrency())) {
    LogError("Unable to start the CPU thread pool!");
    return false;
  }

  online_ = true;
  info_.available_ = 1;
  // Copy the system-memory capabilities from the online device. The offline
  // target has no SVM devices.
  info_.svmCapabilities_ = 0;
  info_.svmCapabilities_ |= CL_DEVICE_SVM_COARSE_GRAIN_BUFFER;
  info_.svmCapabilities_ |= CL_DEVICE_SVM_FINE_GRAIN_BUFFER;
  info_.svmCapabilities_ |= CL_DEVICE_SVM_FINE_GRAIN_SYSTEM;
  info_.svmCapabilities_ |= CL_DEVICE_SVM_ATOMICS;

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
  return createBufferMemory(owner, owner.getAlignment());
}

Memory* Device::createBufferMemory(amd::Memory& owner, size_t alignment) const {
  if (owner.asBuffer() == nullptr || owner.isInterop()) {
    LogError("CPU device only supports system-memory buffers!");
    return nullptr;
  }

  const cl_mem_flags memFlags = owner.getMemFlags();
  if ((memFlags & (CL_MEM_VA_RANGE_AMD | CL_MEM_BUS_ADDRESSABLE_AMD |
                   CL_MEM_EXTERNAL_PHYSICAL_AMD | ROCCLR_MEM_INTERPROCESS |
                   ROCCLR_MEM_HSA_SIGNAL_MEMORY)) ||
      owner.ipcShared() ||
      owner.getSvmPtr() == reinterpret_cast<void*>(amd::Memory::MemoryType::kArenaMemoryPtr)) {
    LogError("Unsupported CPU buffer memory requirements!");
    return nullptr;
  }

  Memory* memory = new Memory(*this, owner, alignment);
  if (memory == nullptr || !memory->isValid()) {
    LogError("Failed creating CPU buffer memory!");
    delete memory;
    return nullptr;
  }

  if (owner.getType() == CL_MEM_OBJECT_PIPE) {
    const size_t pipeInit[3] = {0, 0, owner.asPipe()->getMaxNumPackets()};
    std::memcpy(reinterpret_cast<void*>(memory->virtualAddress()), pipeInit, sizeof(pipeInit));
  }
  return memory;
}

void* Device::svmAlloc(amd::Context& context, size_t size, size_t alignment,
                       cl_svm_mem_flags flags, void* svmPtr) const {
  void* svmPtrUsed = reinterpret_cast<void*>(amd::Memory::MemoryType::kSvmMemoryPtr);
  if (svmPtr != nullptr) {
    amd::Memory* existing = amd::MemObjMap::FindMemObj(svmPtr);
    if (existing != nullptr) {
      return existing->getSvmPtr();
    }
    if (!(flags & CL_MEM_USE_HOST_PTR)) {
      LogError("Cannot find the supplied CPU SVM pointer!");
      return nullptr;
    }
    svmPtrUsed = svmPtr;
  }

  if (size == 0 || (alignment != 0 && (alignment & (alignment - 1)) != 0)) {
    LogError("Invalid CPU SVM allocation size or alignment!");
    return nullptr;
  }

  amd::Buffer* mem = new (context) amd::Buffer(context, flags, size, svmPtrUsed, alignment);
  if (mem == nullptr) {
    LogError("Failed creating a CPU SVM memory object!");
    return nullptr;
  }
  if (!mem->create(nullptr)) {
    LogError("Failed creating a CPU SVM hidden buffer!");
    mem->release();
    return nullptr;
  }

  // Use the regular allocation path to register both the device backing and svmBase.
  device::Memory* memory = mem->getDeviceMemory(*this);
  if (memory == nullptr) {
    mem->release();
    return nullptr;
  }
  amd::MemObjMap::AddMemObj(mem->getSvmPtr(), mem);
  return mem->getSvmPtr();
}

void Device::svmFree(void* ptr) const {
  amd::Memory* mem = amd::MemObjMap::FindMemObj(ptr);
  if (mem != nullptr) {
    amd::MemObjMap::RemoveMemObj(mem->getSvmPtr());
    mem->release();
  }
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

  // One compute unit, one work item at a time. The wavefront width must not be
  // zero. TODO: The limits below match gfx906. Derive them from the host.
  info_.maxComputeUnits_ = 60;
  info_.maxPhysicalComputeUnits_ = 60;
  info_.wavefrontWidth_ = 64;
  info_.simdWidth_ = 16;
  info_.simdPerCU_ = 4;
  info_.simdInstructionWidth_ = 1;
  info_.maxThreadsPerCU_ = 2560;
  info_.maxWorkGroupSize_ = 1024;
  info_.preferredWorkGroupSize_ = 256;
  info_.maxWorkItemDimensions_ = 3;
  info_.cooperativeGroups_ = 1;
  for (size_t dim = 0; dim < 3; ++dim) {
    info_.maxWorkItemSizes_[dim] = 1024;
    info_.maxGridDim_[dim] = INT32_MAX;
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

  // System memory only, reported as half of the host physical memory.
  // TODO: The local memory sizes match gfx906. Set them from the cost of
  // __shared__ on the host.
  info_.localMemType_ = CL_LOCAL;
  info_.localMemSize_ = 64 * Ki;
  info_.localMemSizePerCU_ = 64 * Ki;
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

  // Linear memory layout. Set unconditionally so that hipMallocPitch and
  // hipMalloc3D produce a nonzero, aligned pitch even without image support.
  info_.imagePitchAlignment_ = 256;
  info_.imageBaseAddressAlignment_ = 256;

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
