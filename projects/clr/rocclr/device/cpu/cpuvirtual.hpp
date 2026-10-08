/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CPUVIRTUAL_HPP_
#define CPUVIRTUAL_HPP_

#include "device/device.hpp"

//! CPU Device Implementation
namespace amd::cpu {

//! The host execution environment for one command queue.
//!
//! The host device does not execute device code, so this only has to be a
//! valid object to hand to amd::HostQueue: creating it is what makes the queue
//! accept commands, and every command submission is rejected. Anything that
//! needs real work (kernels, memory transfers, signals) is unimplemented on
//! purpose.
class VirtualDevice : public device::VirtualDevice {
 public:
  VirtualDevice(amd::Device& device) : device::VirtualDevice(device) {}

  virtual ~VirtualDevice() {}

  virtual uint64_t getQueueID() { return 0; }

  virtual bool isFenceDirty() const { return false; }

  virtual void HiddenHeapInit() {}

  virtual void submitReadMemory(amd::ReadMemoryCommand& cmd) { Unsupported("ReadMemory"); }
  virtual void submitWriteMemory(amd::WriteMemoryCommand& cmd) { Unsupported("WriteMemory"); }
  virtual void submitCopyMemory(amd::CopyMemoryCommand& cmd) { Unsupported("CopyMemory"); }
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
  virtual void submitKernel(amd::NDRangeKernelCommand& cmd) { Unsupported("Kernel"); }
  virtual void submitNativeFn(amd::NativeFnCommand& cmd) { Unsupported("NativeFn"); }
  virtual void submitMarker(amd::Marker& cmd) { Unsupported("Marker"); }
  virtual void submitAccumulate(amd::AccumulateCommand& cmd) { Unsupported("Accumulate"); }
  virtual void submitExternalSemaphoreCmd(amd::ExternalSemaphoreCmd& cmd) {
    Unsupported("ExternalSemaphore");
  }
  virtual void submitFillMemory(amd::FillMemoryCommand& cmd) { Unsupported("FillMemory"); }
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
  virtual void flush(amd::Command* list = NULL, bool wait = false) {}
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
  //! The host backend implements no command.
  void Unsupported(const char* command);

  //! Disable default copy constructor
  VirtualDevice(const VirtualDevice&);

  //! Disable default operator=
  VirtualDevice& operator=(const VirtualDevice&);
};

}  // namespace amd::cpu

#endif /*CPUVIRTUAL_HPP_*/
