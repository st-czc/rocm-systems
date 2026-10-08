/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#include "device/cpu/cpukernel.hpp"

#include "device/cpu/cpuprogram.hpp"

#include <chrono>

namespace amd::cpu {

Kernel::Kernel(const std::string& name, const Program* prog)
    : device::Kernel(prog->device(), name, *prog) {}

const Program* Kernel::program() const { return static_cast<const Program*>(&prog_); }

}  // namespace amd::cpu

namespace amd::device {

uint64_t HostSignal::Wait(uint64_t value, Condition c, uint64_t timeout) {
  auto satisfied = [&](uint64_t current) {
    switch (c) {
      case Condition::Eq:
        return current == value;
      case Condition::Ne:
        return current != value;
      case Condition::Lt:
        return current < value;
      case Condition::Gte:
        return current >= value;
    }
    return false;
  };

  std::unique_lock lock(lock_);
  if (satisfied(value_)) {
    return value_;
  }

  // Timeout of zero waits indefinitely, matching the underlying signal semantics.
  if (timeout == 0) {
    cv_.wait(lock, [&] { return satisfied(value_); });
  } else if (!cv_.wait_for(lock, std::chrono::nanoseconds(timeout),
                           [&] { return satisfied(value_); })) {
    return -1;
  }

  return value_;
}

}  // namespace amd::device
