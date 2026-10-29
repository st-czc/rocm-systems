/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>

#include "device/devkernel.hpp"
#include "device/devsignal.hpp"

//! CPU Device Implementation
namespace amd::cpu {

class Program;

//! Work-group state passed last to Native CPU entry points. Its layout mirrors
//! native_cpu::state in the Native CPU device library.
struct NativeCpuState {
  size_t global_id[3];
  size_t global_range[3];
  size_t workgroup_size[3];
  size_t workgroup_id[3];
  size_t local_id[3];
  size_t num_groups[3];
  size_t global_offset[3];
  uint32_t num_sub_groups;
  uint32_t sub_group_id;
  uint32_t sub_group_local_id;
  uint32_t sub_group_size;
};

//! Per-kernel subhandler. It takes one pointer per argument and the work-group
//! state, and it runs one work-group per call.
//! [HIPCPU x-range] It also takes the dimension-0 work-group range to run and
//! the dynamic shared memory size; one call runs that range.
using Subhandler = void (*)(void** args, NativeCpuState* state, size_t x_begin,
                            size_t x_end, size_t dynamicSharedMemBytes);

//! Host kernel handle. It carries the work-group metadata that the runtime
//! queries.
class Kernel : public device::Kernel {
 public:
  Kernel(const std::string& name, const Program* prog);

  virtual ~Kernel() {}

  //! Builds the abstraction-layer state the runtime queries for this kernel.
  bool init();

  //! The subhandler resolved from the loaded device image.
  void setEntry(Subhandler entry) { entry_ = entry; }
  Subhandler entry() const { return entry_; }

 private:
  Subhandler entry_ = nullptr;

  //! Disable default copy constructor
  Kernel(const Kernel&);

  //! Disable default operator=
  Kernel& operator=(const Kernel&);
};

}  // namespace amd::cpu

namespace amd::device {

//! Host signal backed by process memory, usable only by waiters in this
//! process.
class HostSignal : public Signal {
 public:
  HostSignal() : value_(0), ws_(WaitState::Blocked) {}

  virtual ~HostSignal() {}

  bool Init(const amd::Device& dev, uint64_t init, WaitState ws) override {
    std::scoped_lock lock(lock_);
    value_ = init;
    ws_ = ws;
    return true;
  }

  uint64_t Wait(uint64_t value, Condition c, uint64_t timeout) override;

  void Reset(uint64_t value) override {
    std::scoped_lock lock(lock_);
    value_ = value;
    cv_.notify_all();
  }

  uint64_t Load() override {
    std::scoped_lock lock(lock_);
    return value_;
  }

 private:
  std::mutex lock_;
  std::condition_variable cv_;
  uint64_t value_;
  WaitState ws_;
};

}  // namespace amd::device
