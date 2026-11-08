// PROTOTYPE: VirtualCPU translates rocclr commands into packets for the
// emulated HSA runtime. Ordering, dependency waits and completion signalling
// all live below it, in Queue and Signal -- the same split rocclr gets from
// the real HSA runtime.
#ifndef PROTOTYPE_CPUVIRTUAL_HPP_
#define PROTOTYPE_CPUVIRTUAL_HPP_

#include <cstdint>

#include "device/device.hpp"
#include "platform/command.hpp"

namespace amd {
namespace device {
namespace prototype {
class Pool;
class Queue;
}  // namespace prototype
}  // namespace device
}  // namespace amd

namespace amd::device::prototype {

class VirtualCPU : public VirtualDevice {
 public:
  VirtualCPU(amd::Device& device, Pool& pool);
  ~VirtualCPU() override;

  uint64_t getQueueID() override { return queue_id_; }

  void submitReadMemory(amd::ReadMemoryCommand& cmd) override { submit(cmd); }
  void submitWriteMemory(amd::WriteMemoryCommand& cmd) override { submit(cmd); }
  void submitCopyMemory(amd::CopyMemoryCommand& cmd) override { submit(cmd); }
  void submitCopyMemoryP2P(amd::CopyMemoryP2PCommand& cmd) override { submit(cmd); }
  void submitBatchCopyMemory(amd::BatchCopyMemoryCommand& cmd) override { submit(cmd); }
  void SubmitBatchWriteMemory(amd::BatchWriteMemoryCommand& cmd) override { submit(cmd); }
  void SubmitBatchReadMemory(amd::BatchReadMemoryCommand& cmd) override { submit(cmd); }
  void submitMapMemory(amd::MapMemoryCommand& cmd) override { submit(cmd); }
  void submitUnmapMemory(amd::UnmapMemoryCommand& cmd) override { submit(cmd); }
  void submitKernel(amd::NDRangeKernelCommand& command) override;
  void submitNativeFn(amd::NativeFnCommand& cmd) override { submit(cmd); }
  void submitMarker(amd::Marker& cmd) override;
  void submitAccumulate(amd::AccumulateCommand& cmd) override { submit(cmd); }
  void submitExternalSemaphoreCmd(amd::ExternalSemaphoreCmd& cmd) override { submit(cmd); }
  void submitFillMemory(amd::FillMemoryCommand& cmd) override { submit(cmd); }
  void submitMigrateMemObjects(amd::MigrateMemObjectsCommand& cmd) override { submit(cmd); }
  void submitAcquireExtObjects(amd::AcquireExtObjectsCommand& cmd) override { submit(cmd); }
  void submitReleaseExtObjects(amd::ReleaseExtObjectsCommand& cmd) override { submit(cmd); }
  void submitPerfCounter(amd::PerfCounterCommand& cmd) override { submit(cmd); }
  void submitThreadTraceMemObjects(amd::ThreadTraceMemObjectsCommand& cmd) override { submit(cmd); }
  void submitThreadTrace(amd::ThreadTraceCommand& cmd) override { submit(cmd); }
  void submitSvmFreeMemory(amd::SvmFreeMemoryCommand& cmd) override { submit(cmd); }
  void submitSvmCopyMemory(amd::SvmCopyMemoryCommand& cmd) override { submit(cmd); }
  void submitSvmFillMemory(amd::SvmFillMemoryCommand& cmd) override { submit(cmd); }
  void submitSvmMapMemory(amd::SvmMapMemoryCommand& cmd) override { submit(cmd); }
  void submitSvmUnmapMemory(amd::SvmUnmapMemoryCommand& cmd) override { submit(cmd); }
  void submitSignal(amd::SignalCommand& cmd) override { submit(cmd); }
  void submitMakeBuffersResident(amd::MakeBuffersResidentCommand& cmd) override { submit(cmd); }

  //! Direct dispatch submits at enqueue time; nothing is batched here.
  void flush(amd::Command* list = nullptr, bool wait = false) override {}

  bool isFenceDirty() const override { return false; }
  void HiddenHeapInit() override {}

 private:
  void submit(amd::Command& cmd);
  static void updateBatchState(amd::Command* list);

  Pool& pool_;
  Queue* queue_;
  uint64_t queue_id_;
};

}  // namespace amd::device::prototype

#endif  // PROTOTYPE_CPUVIRTUAL_HPP_
