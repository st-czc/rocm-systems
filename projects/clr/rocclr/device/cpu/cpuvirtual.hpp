/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CPUVIRTUAL_HPP_
#define CPUVIRTUAL_HPP_

#include "device/device.hpp"
#include "platform/command.hpp"

#include <cstdint>
#include <functional>

//! CPU Device Implementation
namespace amd::cpu {

class Device;
class Pool;
class Queue;

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
  virtual void submitCopyMemoryP2P(amd::CopyMemoryP2PCommand& cmd) {
    submitUnsupported(cmd, "CopyMemoryP2P");
  }
  virtual void submitBatchCopyMemory(amd::BatchCopyMemoryCommand& cmd);
  virtual void SubmitBatchWriteMemory(amd::BatchWriteMemoryCommand& cmd);
  virtual void SubmitBatchReadMemory(amd::BatchReadMemoryCommand& cmd);
  virtual void submitMapMemory(amd::MapMemoryCommand& cmd) { submitUnsupported(cmd, "MapMemory"); }
  virtual void submitUnmapMemory(amd::UnmapMemoryCommand& cmd) {
    submitUnsupported(cmd, "UnmapMemory");
  }
  virtual void submitKernel(amd::NDRangeKernelCommand& cmd);
  virtual void submitNativeFn(amd::NativeFnCommand& cmd) { submitUnsupported(cmd, "NativeFn"); }
  virtual void submitMarker(amd::Marker& cmd);
  virtual void submitAccumulate(amd::AccumulateCommand& cmd) {
    submitUnsupported(cmd, "Accumulate");
  }
  virtual void submitExternalSemaphoreCmd(amd::ExternalSemaphoreCmd& cmd) {
    submitUnsupported(cmd, "ExternalSemaphore");
  }
  virtual void submitFillMemory(amd::FillMemoryCommand& cmd);
  virtual void submitMigrateMemObjects(amd::MigrateMemObjectsCommand& cmd) {
    submitUnsupported(cmd, "MigrateMemObjects");
  }
  virtual void submitAcquireExtObjects(amd::AcquireExtObjectsCommand& cmd) {
    submitUnsupported(cmd, "AcquireExtObjects");
  }
  virtual void submitReleaseExtObjects(amd::ReleaseExtObjectsCommand& cmd) {
    submitUnsupported(cmd, "ReleaseExtObjects");
  }
  virtual void submitPerfCounter(amd::PerfCounterCommand& cmd) {
    submitUnsupported(cmd, "PerfCounter");
  }
  virtual void submitThreadTraceMemObjects(amd::ThreadTraceMemObjectsCommand& cmd) {
    submitUnsupported(cmd, "ThreadTraceMemObjects");
  }
  virtual void submitThreadTrace(amd::ThreadTraceCommand& cmd) {
    submitUnsupported(cmd, "ThreadTrace");
  }
  virtual void flush(amd::Command* list = NULL, bool wait = false);
  virtual void submitSvmFreeMemory(amd::SvmFreeMemoryCommand& cmd) {
    submitUnsupported(cmd, "SvmFreeMemory");
  }
  virtual void submitSvmCopyMemory(amd::SvmCopyMemoryCommand& cmd) {
    submitUnsupported(cmd, "SvmCopyMemory");
  }
  virtual void submitSvmFillMemory(amd::SvmFillMemoryCommand& cmd) {
    submitUnsupported(cmd, "SvmFillMemory");
  }
  virtual void submitSvmMapMemory(amd::SvmMapMemoryCommand& cmd) {
    submitUnsupported(cmd, "SvmMapMemory");
  }
  virtual void submitSvmUnmapMemory(amd::SvmUnmapMemoryCommand& cmd) {
    submitUnsupported(cmd, "SvmUnmapMemory");
  }
  virtual void submitSignal(amd::SignalCommand& cmd) { submitUnsupported(cmd, "Signal"); }
  virtual void submitMakeBuffersResident(amd::MakeBuffersResidentCommand& cmd) {
    submitUnsupported(cmd, "MakeBuffersResident");
  }

 private:
  //! Fills a memory object with a pattern
  bool fillMemory(cl_command_type type, amd::Memory* amdMemory, const void* pattern,
                  size_t patternSize, const amd::Coord3D& surface, const amd::Coord3D& origin,
                  const amd::Coord3D& size);

  //! The host backend implements no command.
  void Unsupported(const char* command);

  //! Submits one command as a single-ticket packet. work runs on a pool worker
  //! and does no bookkeeping: the decision it runs was made before submission.
  void submit(amd::Command& cmd, std::function<void()> work);

  //! Submits a command whose work splits across numTickets workers. task takes
  //! the ticket, handed out in descending order.
  void submit(amd::Command& cmd, uint32_t numTickets, std::function<void(int)> task);

  //! Fails a command the backend does not implement, without submitting it.
  void submitUnsupported(amd::Command& cmd, const char* command);

  //! Advances every command of the batch to COMPLETE and drops its batch reference.
  static void updateBatchState(amd::Command* list);

  const Device& dev_;
  Pool& pool_;
  Queue* queue_;

  //! Disable default copy constructor
  VirtualDevice(const VirtualDevice&);

  //! Disable default operator=
  VirtualDevice& operator=(const VirtualDevice&);
};

}  // namespace amd::cpu

#endif /*CPUVIRTUAL_HPP_*/
