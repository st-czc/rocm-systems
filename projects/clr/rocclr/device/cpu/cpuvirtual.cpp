/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#include "device/cpu/cpuvirtual.hpp"

#include "utils/debug.hpp"

namespace amd::cpu {

void VirtualDevice::Unsupported(const char* command) {
  LogPrintfError("The spvcpu device does not implement the %s command!", command);
}

}  // namespace amd::cpu
