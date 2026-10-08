/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#include "device/cpu/cpumemory.hpp"

#include "device/cpu/cpudevice.hpp"
#include "os/alloc.hpp"
#include "utils/debug.hpp"

#include <algorithm>

namespace amd::cpu {

Memory::Memory(Device& dev, amd::Memory& owner)
    : device::Memory(owner), dev_(dev), ptr_(nullptr), allocated_(false) {
  void* svmPtr = owner.getSvmPtr();
  const bool allocateSvm =
      svmPtr == reinterpret_cast<void*>(amd::Memory::MemoryType::kSvmMemoryPtr);
  if (svmPtr != nullptr && !allocateSvm) {
    // Other devices must use the same SVM storage.
    ptr_ = static_cast<address>(svmPtr);
  } else if (owner.getHostMem() != nullptr && !allocateSvm) {
    // The application supplied the storage (USE_HOST_PTR and friends).
    ptr_ = static_cast<address>(owner.getHostMem());
  } else {
    allocated_ = allocate(owner.getSize(), owner.getAlignment());
  }

  if (ptr_ != nullptr) {
    flags_ |= HostMemoryDirectAccess;
    if (svmPtr != nullptr) {
      owner.setSvmPtr(ptr_);
      if (owner.getHostMem() != ptr_) {
        owner.setHostMem(ptr_);
      }
    }
  }
}

Memory::~Memory() { release(); }

bool Memory::allocate(size_t size, size_t alignment) {
  if (size == 0) {
    return true;
  }

  // posix_memalign requires a nonzero power-of-two multiple of sizeof(void*).
  alignment = std::max(alignment, static_cast<size_t>(dev_.info().memBaseAddrAlign_));
  alignment = std::max(alignment, sizeof(void*));
  ptr_ = static_cast<address>(amd::AlignedMemory::allocate(size, alignment));
  if (ptr_ == nullptr) {
    LogError("Unable to allocate host memory!");
    return false;
  }

  return true;
}

void Memory::release() {
  if (allocated_ && (ptr_ != nullptr)) {
    amd::AlignedMemory::deallocate(ptr_);
    ptr_ = nullptr;
    allocated_ = false;
  }
}

}  // namespace amd::cpu
