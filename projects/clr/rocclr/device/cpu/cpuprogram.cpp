/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#include "device/cpu/cpuprogram.hpp"

#include "device/cpu/cpudevice.hpp"
#include "utils/debug.hpp"

namespace amd::cpu {

Program::Program(NullDevice& device, amd::Program& owner) : device::Program(device, owner) {}

Program::~Program() {}

bool Program::createBinary(amd::option::Options* options) {
  // The host backend does not build device code.
  LogWarning("Host device does not support program compilation!");
  return false;
}

}  // namespace amd::cpu
