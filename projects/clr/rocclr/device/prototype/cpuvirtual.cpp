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
  std::vector<amd::ReferenceCountedObject*> wait_list;
  for (amd::Event* event : cmd.eventWaitList()) {
    wait_list.push_back(event);
  }
  auto packet = agent_.NewPacket(&cmd, wait_list);

  packet->work = [] { SimulateWork(); };

  queue_->Submit(packet);
}

void VirtualCPU::submitMarker(amd::Marker& cmd) {
  std::vector<amd::ReferenceCountedObject*> wait_list;
  for (amd::Event* event : cmd.eventWaitList()) {
    wait_list.push_back(event);
  }
  auto packet = agent_.NewPacket(&cmd, wait_list);

  packet->work = nullptr;

  Signal* done = packet->done;

  if (cmd.CpuWaitRequested()) {
    done->retain();
    queue_->Submit(packet);
    done->Wait();
    done->release();
    if (auto* head = cmd.GetBatchHead()) {
      UpdateBatchState(head);
    }
  } else {
    if (auto* head = cmd.GetBatchHead()) {
      done->OnDone([head] { UpdateBatchState(head); });
    }
    queue_->Submit(packet);
  }
}

}  // namespace amd::device::prototype
