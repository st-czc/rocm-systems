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

//! A host kernel handle. The host backend does not execute device code, so the
//! kernel object carries only the workgroup metadata the runtime queries.
class Kernel : public device::Kernel {
 public:
  Kernel(const std::string& name, const Program* prog);

  virtual ~Kernel() {}

  //! Returns the program associated with this kernel
  const Program* program() const;

 private:
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
