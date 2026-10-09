/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CPUMEMORY_HPP_
#define CPUMEMORY_HPP_

#include "device/device.hpp"

#include <cstdint>

//! CPU Device Implementation
namespace amd::cpu {

class Device;

//! CPU-side backing for an amd::Memory object. The virtual address is the
//! process address. For SVM allocations it is also the public SVM pointer. The
//! object borrows host pointers and frees only its own allocations.
class Memory : public device::Memory {
 public:
  //! Constructor (from an amd::Memory object)
  Memory(const Device& dev, amd::Memory& owner, size_t alignment = 0);

  //! Default destructor
  virtual ~Memory();

  //! Reports whether the backing storage was created successfully.
  bool isValid() const { return ptr_ != nullptr; }

  //! Returns the host pointer as the device virtual address
  virtual uint64_t virtualAddress() const { return reinterpret_cast<uint64_t>(ptr_); }

  //! Returns CPU pointer to the device memory
  virtual const address cpuSrd() const { return static_cast<const address>(ptr_); }

  //! Immediate blocking write from the device cache to the owner's backing store.
  virtual void syncHostFromCache(device::VirtualDevice* vDev, SyncFlags syncFlags = SyncFlags()) {}

  //! Updates the device memory from the owner's host allocation.
  void syncCacheFromHost(device::VirtualDevice& vDev, SyncFlags syncFlags = SyncFlags()) {}

  //! Map the device memory to CPU visible
  virtual void* cpuMap(device::VirtualDevice& vDev, uint flags = 0, uint startLayer = 0,
                       uint numLayers = 0, size_t* rowPitch = NULL, size_t* slicePitch = NULL) {
    return ptr_;
  }

  //! Unmap the device memory
  virtual void cpuUnmap(device::VirtualDevice& vDev) {}

 private:
  enum class StorageOwnership { Borrowed, AlignedAllocation };

  //! Allocates the host memory
  bool allocate(size_t size, size_t alignment);

  //! Releases the host memory
  void release();

  //! Disable default copy constructor
  Memory(const Memory&);

  //! Disable default operator=
  Memory& operator=(const Memory&);

  const Device& dev_;  //!< The host device that owns this memory
  address ptr_;        //!< Host allocation backing the memory object
  StorageOwnership ownership_;  //!< Selects whether and how ptr_ is released
};

}  // namespace amd::cpu

#endif /*CPUMEMORY_HPP_*/
