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
#include <cstring>

namespace amd::cpu {

Memory::Memory(const Device& dev, amd::Memory& owner, size_t alignment)
    : device::Memory(owner), dev_(dev), ptr_(nullptr),
      ownership_(StorageOwnership::Borrowed) {
  if (owner.parent() != nullptr) {
    device::Memory* parent = owner.parent()->getDeviceMemory(dev_);
    if (parent == nullptr || parent->virtualAddress() == 0) {
      LogError("Unable to create CPU parent buffer memory!");
      return;
    }
    ptr_ = reinterpret_cast<address>(parent->virtualAddress()) + owner.getOrigin();
    flags_ |= HostMemoryDirectAccess;
    owner.setHostMem(ptr_);
    return;
  }

  void* svmPtr = owner.getSvmPtr();
  const bool allocateSvm =
      svmPtr == reinterpret_cast<void*>(amd::Memory::MemoryType::kSvmMemoryPtr);
  if (svmPtr != nullptr && !allocateSvm) {
    // Other devices must use the same SVM storage.
    ptr_ = static_cast<address>(svmPtr);
  } else if (owner.getHostMem() != nullptr && !allocateSvm &&
             !(owner.getMemFlags() & CL_MEM_COPY_HOST_PTR)) {
    // Borrow host backing supplied by the application or the owner.
    ptr_ = static_cast<address>(owner.getHostMem());
  } else {
    if (!allocate(owner.getSize(),
                  std::max(alignment, static_cast<size_t>(owner.getAlignment())))) {
      return;
    }
    ownership_ = StorageOwnership::AlignedAllocation;
    if (ptr_ != nullptr && (owner.getMemFlags() & CL_MEM_COPY_HOST_PTR) &&
        owner.getHostMem() != nullptr) {
      std::memcpy(ptr_, owner.getHostMem(), owner.getSize());
    }
  }

  if (ptr_ != nullptr) {
    flags_ |= HostMemoryDirectAccess;
    if (svmPtr != nullptr) {
      owner.setSvmPtr(ptr_);
    }
    // Publish the actual storage after the initial copy. setHostMem replaces
    // the previous host backing.
    if (owner.getHostMem() != ptr_) {
      owner.setHostMem(ptr_);
    }
  }
}

Memory::~Memory() {
  dev_.removeVACache(this);
  release();
}

bool Memory::allocate(size_t size, size_t alignment) {
  if (size == 0) {
    return false;
  }

  // posix_memalign requires a nonzero power-of-two multiple of sizeof(void*).
  alignment = std::max(alignment, static_cast<size_t>(dev_.info().memBaseAddrAlign_));
  alignment = std::max(alignment, sizeof(void*));
  if ((alignment & (alignment - 1)) != 0) {
    LogError("CPU memory alignment must be a power of two!");
    return false;
  }
  ptr_ = static_cast<address>(amd::AlignedMemory::allocate(size, alignment));
  if (ptr_ == nullptr) {
    LogError("Unable to allocate host memory!");
    return false;
  }

  return true;
}

void Memory::release() {
  if (ptr_ == nullptr) {
    return;
  }
  if (ownership_ == StorageOwnership::AlignedAllocation) {
    amd::AlignedMemory::deallocate(ptr_);
  }
  ptr_ = nullptr;
  ownership_ = StorageOwnership::Borrowed;
}

}  // namespace amd::cpu
