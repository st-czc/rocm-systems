// PROTOTYPE: VirtualCPU translates rocclr commands into packets for the
// emulated HSA agent. Ordering, dependency waits and completion signalling
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
class Agent;
class Queue;
}  // namespace prototype
}  // namespace device
}  // namespace amd

namespace amd::device::prototype {

class VirtualCPU : public VirtualDevice {
 public:
  VirtualCPU(amd::Device& device, Agent& agent);
  ~VirtualCPU() override;

  uint64_t getQueueID() override { return queue_id_; }

  void submitReadMemory(amd::ReadMemoryCommand& cmd) override { Submit(cmd); }
  void submitWriteMemory(amd::WriteMemoryCommand& cmd) override { Submit(cmd); }
  void submitCopyMemory(amd::CopyMemoryCommand& cmd) override { Submit(cmd); }
  void submitCopyMemoryP2P(amd::CopyMemoryP2PCommand& cmd) override { Submit(cmd); }
  void submitBatchCopyMemory(amd::BatchCopyMemoryCommand& cmd) override { Submit(cmd); }
  void SubmitBatchWriteMemory(amd::BatchWriteMemoryCommand& cmd) override { Submit(cmd); }
  void SubmitBatchReadMemory(amd::BatchReadMemoryCommand& cmd) override { Submit(cmd); }
  void submitMapMemory(amd::MapMemoryCommand& cmd) override { Submit(cmd); }
  void submitUnmapMemory(amd::UnmapMemoryCommand& cmd) override { Submit(cmd); }
  void submitKernel(amd::NDRangeKernelCommand& command) override { Submit(command); }
  void submitNativeFn(amd::NativeFnCommand& cmd) override { Submit(cmd); }
  void submitMarker(amd::Marker& cmd) override;
  void submitAccumulate(amd::AccumulateCommand& cmd) override { Submit(cmd); }
  void submitExternalSemaphoreCmd(amd::ExternalSemaphoreCmd& cmd) override { Submit(cmd); }
  void submitFillMemory(amd::FillMemoryCommand& cmd) override { Submit(cmd); }
  void submitMigrateMemObjects(amd::MigrateMemObjectsCommand& cmd) override { Submit(cmd); }
  void submitAcquireExtObjects(amd::AcquireExtObjectsCommand& cmd) override { Submit(cmd); }
  void submitReleaseExtObjects(amd::ReleaseExtObjectsCommand& cmd) override { Submit(cmd); }
  void submitPerfCounter(amd::PerfCounterCommand& cmd) override { Submit(cmd); }
  void submitThreadTraceMemObjects(amd::ThreadTraceMemObjectsCommand& cmd) override { Submit(cmd); }
  void submitThreadTrace(amd::ThreadTraceCommand& cmd) override { Submit(cmd); }
  void submitSvmFreeMemory(amd::SvmFreeMemoryCommand& cmd) override { Submit(cmd); }
  void submitSvmCopyMemory(amd::SvmCopyMemoryCommand& cmd) override { Submit(cmd); }
  void submitSvmFillMemory(amd::SvmFillMemoryCommand& cmd) override { Submit(cmd); }
  void submitSvmMapMemory(amd::SvmMapMemoryCommand& cmd) override { Submit(cmd); }
  void submitSvmUnmapMemory(amd::SvmUnmapMemoryCommand& cmd) override { Submit(cmd); }
  void submitSignal(amd::SignalCommand& cmd) override { Submit(cmd); }
  void submitMakeBuffersResident(amd::MakeBuffersResidentCommand& cmd) override { Submit(cmd); }

  //! Direct dispatch submits at enqueue time; nothing is batched here.
  void flush(amd::Command* list = nullptr, bool wait = false) override {}

  bool isFenceDirty() const override { return false; }
  void HiddenHeapInit() override {}

 private:
  void Submit(amd::Command& cmd);
  static void UpdateBatchState(amd::Command* list);

  Agent& agent_;
  Queue* queue_;
  uint64_t queue_id_;
};

}  // namespace amd::device::prototype

#endif  // PROTOTYPE_CPUVIRTUAL_HPP_
