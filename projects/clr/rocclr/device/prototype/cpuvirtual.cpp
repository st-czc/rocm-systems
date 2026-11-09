// PROTOTYPE: VirtualCPU translates rocclr commands into packets for the
// emulated HSA runtime.
#include "device/prototype/cpuvirtual.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <cstdio>

#include "device/prototype/hsaemu.hpp"
#include "os/os.hpp"
#include "platform/kernel.hpp"

namespace amd::device::prototype {
namespace {

std::atomic<uint64_t> g_queue_id{0};

void simulateWork(void* p, int i) {
  thread_local std::mt19937 rng{std::random_device{}()};
  std::uniform_int_distribution<int> dist(1, 20);
  std::printf("Packet[%p]: A %4d\n", p, i);
  std::this_thread::sleep_for(std::chrono::milliseconds(dist(rng)));
  std::printf("Packet[%p]: B %4d\n", p, i);
}

}  // namespace

VirtualCPU::VirtualCPU(amd::Device& device, Pool& pool)
    : VirtualDevice(device), pool_(pool), queue_(pool.newQueue(this)), queue_id_(g_queue_id++) {}

VirtualCPU::~VirtualCPU() { pool_.deleteQueue(this); }

//! The counterpart of VirtualGPU::updateCommandsState. Only the batch head is
//! given this function; running it advances every command of the batch to
//! COMPLETE and drops the batch's own reference on each.
void VirtualCPU::updateBatchState(amd::Command* list) {
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

void VirtualCPU::submit(amd::Command& cmd) {
  auto packet = pool_.newPacket(&cmd, 1, cmd.eventWaitList());

  packet->task = [packet](int i) { simulateWork(static_cast<void*>(packet), i); };

  queue_->submit(packet);
}

void VirtualCPU::submitKernel(NDRangeKernelCommand& cmd) {
  auto packet = pool_.newPacket(&cmd, 32, cmd.eventWaitList());

  packet->task = [packet](int i) { simulateWork(static_cast<void*>(packet), i); };

  queue_->submit(packet);
}

void VirtualCPU::submitMarker(amd::Marker& cmd) {
  auto packet = pool_.newPacket(&cmd, 0, cmd.eventWaitList());

  Signal* signal = packet->signal;

  if (cmd.CpuWaitRequested()) {
    signal->retain();
    queue_->submit(packet);
    signal->wait();
    signal->release();
    if (auto* head = cmd.GetBatchHead()) {
      updateBatchState(head);
    }
  } else {
    // Mirrors device/rocm: the non-CpuWait marker registers an async handler
    // (rocvirtual.cpp:805-871) and the batch update runs on ROCr's async-events
    // thread, never on a thread that executes work. onDone itself may fire on a
    // worker, on the app thread, or on the async thread, so it only posts.
    if (auto* head = cmd.GetBatchHead()) {
      Pool* pool = &pool_;
      signal->onDone([pool, head] { pool->postAsync([head] { updateBatchState(head); }); });
    }
    queue_->submit(packet);
  }
}

}  // namespace amd::device::prototype
