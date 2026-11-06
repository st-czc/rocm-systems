// PROTOTYPE: VirtualCPU translates rocclr commands into packets for the
// emulated HSA agent.
#include "device/prototype/cpuvirtual.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <random>

#include "device/prototype/hsaemu.hpp"
#include "os/os.hpp"

namespace amd::device::prototype {
namespace {

std::atomic<uint64_t> g_queue_id{0};

void SimulateWork() {
  thread_local std::mt19937 rng{std::random_device{}()};
  std::uniform_int_distribution<int> dist(1, 20);
  std::this_thread::sleep_for(std::chrono::milliseconds(dist(rng)));
}

}  // namespace

VirtualCPU::VirtualCPU(amd::Device& device, Agent& agent)
    : VirtualDevice(device), agent_(agent), queue_(agent.NewQueue()), queue_id_(g_queue_id++) {}

VirtualCPU::~VirtualCPU() {}

//! The counterpart of VirtualGPU::updateCommandsState. Only the batch head is
//! given this function; running it advances every command of the batch to
//! COMPLETE and drops the batch's own reference on each.
void VirtualCPU::UpdateBatchState(amd::Command* list) {
  const uint64_t now = Os::timeNanos();
  for (amd::Command* current = list; current != nullptr;) {
    amd::Command* next = current->getNext();
    if (current->status() == CL_SUBMITTED) {
      current->setStatus(CL_RUNNING, now);
      current->setStatus(CL_COMPLETE, now);
    }
    current->release();
    current = next;
  }
}

void VirtualCPU::Submit(amd::Command& cmd) {
  auto packet = std::make_shared<Packet>();

  for (amd::Event* event : cmd.eventWaitList()) {
    packet->AddDep(agent_.SignalFor(event));
  }
  packet->SetDone(agent_.SignalFor(&cmd));
  packet->work = [] { SimulateWork(); };

  queue_->Submit(std::move(packet));
}

void VirtualCPU::submitMarker(amd::Marker& cmd) {
  auto packet = std::make_shared<Packet>();

  for (amd::Event* event : cmd.eventWaitList()) {
    packet->AddDep(agent_.SignalFor(event));
  }
  Signal* done = agent_.SignalFor(&cmd);
  packet->SetDone(done);

  // Only the batch head carries completion: advancing the chain is what moves
  // every command of the batch to CL_COMPLETE.
  if (cmd.GetBatchHead() != nullptr) {
    amd::Command* head = cmd.GetBatchHead();
    packet->work = [head] { UpdateBatchState(head); };
  } else {
    packet->work = [] {};
  }

  queue_->Submit(std::move(packet));

  // An internal marker from finish()/terminate() is the caller's sync point.
  if (cmd.CpuWaitRequested()) {
    done->Wait();
  }
}

}  // namespace amd::device::prototype
