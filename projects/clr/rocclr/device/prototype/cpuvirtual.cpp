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
    : VirtualDevice(device), pool_(pool), queue_(pool.newQueue()), queue_id_(g_queue_id++) {}

VirtualCPU::~VirtualCPU() {}

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
  std::vector<amd::ReferenceCountedObject*> wait_list;
  for (amd::Event* event : cmd.eventWaitList()) {
    wait_list.push_back(event);
  }
  auto packet = pool_.newPacket(&cmd, 1, wait_list);

  packet->task = [packet](int i) { simulateWork(static_cast<void*>(packet), i); };

  queue_->submit(packet);
}

void VirtualCPU::submitKernel(NDRangeKernelCommand& cmd) {
  std::vector<amd::ReferenceCountedObject*> wait_list;
  for (amd::Event* event : cmd.eventWaitList()) {
    wait_list.push_back(event);
  }
  auto packet = pool_.newPacket(&cmd, 32, wait_list);

  packet->task = [packet](int i) { simulateWork(static_cast<void*>(packet), i); };

  queue_->submit(packet);
}

void VirtualCPU::submitMarker(amd::Marker& cmd) {
  std::vector<amd::ReferenceCountedObject*> wait_list;
  for (amd::Event* event : cmd.eventWaitList()) {
    wait_list.push_back(event);
  }
  auto packet = pool_.newPacket(&cmd, 0, wait_list);

  Signal* done = packet->done;

  if (cmd.CpuWaitRequested()) {
    done->retain();
    queue_->submit(packet);
    done->wait();
    done->release();
    if (auto* head = cmd.GetBatchHead()) {
      updateBatchState(head);
    }
  } else {
    if (auto* head = cmd.GetBatchHead()) {
      done->onDone([head] { updateBatchState(head); });
    }
    queue_->submit(packet);
  }
}

}  // namespace amd::device::prototype
