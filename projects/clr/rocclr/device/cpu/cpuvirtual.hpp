/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CPUVIRTUAL_HPP_
#define CPUVIRTUAL_HPP_

#include "device/device.hpp"

#include <cstdint>

//! CPU Device Implementation
namespace amd::cpu {

class Device;

//! Host execution environment for one command queue. It runs supported memory
//! commands synchronously in process memory.
class VirtualDevice : public device::VirtualDevice {
 public:
  VirtualDevice(Device& device);

  virtual ~VirtualDevice();

  const Device& dev() const { return dev_; }

  virtual uint64_t getQueueID() { return 0; }

  virtual bool isFenceDirty() const { return false; }

  virtual void HiddenHeapInit() {}

  virtual void submitReadMemory(amd::ReadMemoryCommand& cmd);
  virtual void submitWriteMemory(amd::WriteMemoryCommand& cmd);
  virtual void submitCopyMemory(amd::CopyMemoryCommand& cmd);
  virtual void submitCopyMemoryP2P(amd::CopyMemoryP2PCommand& cmd) { Unsupported("CopyMemoryP2P"); }
  virtual void submitBatchCopyMemory(amd::BatchCopyMemoryCommand& cmd) {
    Unsupported("BatchCopyMemory");
  }
  virtual void SubmitBatchWriteMemory(amd::BatchWriteMemoryCommand& cmd) {
    Unsupported("BatchWriteMemory");
  }
  virtual void SubmitBatchReadMemory(amd::BatchReadMemoryCommand& cmd) {
    Unsupported("BatchReadMemory");
  }
  virtual void submitMapMemory(amd::MapMemoryCommand& cmd) { Unsupported("MapMemory"); }
  virtual void submitUnmapMemory(amd::UnmapMemoryCommand& cmd) { Unsupported("UnmapMemory"); }
  virtual void submitKernel(amd::NDRangeKernelCommand& cmd);
  virtual void submitNativeFn(amd::NativeFnCommand& cmd) { Unsupported("NativeFn"); }
  virtual void submitMarker(amd::Marker& cmd);
  virtual void submitAccumulate(amd::AccumulateCommand& cmd) { Unsupported("Accumulate"); }
  virtual void submitExternalSemaphoreCmd(amd::ExternalSemaphoreCmd& cmd) {
    Unsupported("ExternalSemaphore");
  }
  virtual void submitFillMemory(amd::FillMemoryCommand& cmd);
  virtual void submitMigrateMemObjects(amd::MigrateMemObjectsCommand& cmd) {
    Unsupported("MigrateMemObjects");
  }
  virtual void submitAcquireExtObjects(amd::AcquireExtObjectsCommand& cmd) {
    Unsupported("AcquireExtObjects");
  }
  virtual void submitReleaseExtObjects(amd::ReleaseExtObjectsCommand& cmd) {
    Unsupported("ReleaseExtObjects");
  }
  virtual void submitPerfCounter(amd::PerfCounterCommand& cmd) { Unsupported("PerfCounter"); }
  virtual void submitThreadTraceMemObjects(amd::ThreadTraceMemObjectsCommand& cmd) {
    Unsupported("ThreadTraceMemObjects");
  }
  virtual void submitThreadTrace(amd::ThreadTraceCommand& cmd) { Unsupported("ThreadTrace"); }
  virtual void flush(amd::Command* list = NULL, bool wait = false);
  virtual void submitSvmFreeMemory(amd::SvmFreeMemoryCommand& cmd) { Unsupported("SvmFreeMemory"); }
  virtual void submitSvmCopyMemory(amd::SvmCopyMemoryCommand& cmd) { Unsupported("SvmCopyMemory"); }
  virtual void submitSvmFillMemory(amd::SvmFillMemoryCommand& cmd) { Unsupported("SvmFillMemory"); }
  virtual void submitSvmMapMemory(amd::SvmMapMemoryCommand& cmd) { Unsupported("SvmMapMemory"); }
  virtual void submitSvmUnmapMemory(amd::SvmUnmapMemoryCommand& cmd) {
    Unsupported("SvmUnmapMemory");
  }
  virtual void submitSignal(amd::SignalCommand& cmd) { Unsupported("Signal"); }
  virtual void submitMakeBuffersResident(amd::MakeBuffersResidentCommand& cmd) {
    Unsupported("MakeBuffersResident");
  }

 private:
  //! Fills a memory object with a pattern
  bool fillMemory(cl_command_type type, amd::Memory* amdMemory, const void* pattern,
                  size_t patternSize, const amd::Coord3D& surface, const amd::Coord3D& origin,
                  const amd::Coord3D& size);

  //! The host backend implements no command.
  void Unsupported(const char* command);

  const Device& dev_;

  //! Disable default copy constructor
  VirtualDevice(const VirtualDevice&);

  //! Disable default operator=
  VirtualDevice& operator=(const VirtualDevice&);
};

}  // namespace amd::cpu

#endif /*CPUVIRTUAL_HPP_*/
