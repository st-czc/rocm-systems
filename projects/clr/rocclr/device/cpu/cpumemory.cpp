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
#include <tuple>

namespace amd::cpu {

namespace {

// Channel format fields shared by the buffer resource descriptor (V#) and the
// image resource descriptor (T#). Returns {dstSel, dataFormat, numFormat}.
std::tuple<uint32_t, uint32_t, uint32_t> getSrdFormat(const amd::Image& image) {
  const cl_image_format& format = image.getImageFormat();

  uint32_t channels = 0;
  switch (format.image_channel_order) {
    case CL_R:
      channels = 1;
      break;
    case CL_RG:
      channels = 2;
      break;
    case CL_RGBA:
      channels = 4;
      break;
    default:
      return {};
  }

  uint32_t numFormat = 0;
  switch (format.image_channel_data_type) {
    case CL_UNORM_INT8:
    case CL_UNORM_INT16:
      numFormat = 0;
      break;
    case CL_SNORM_INT8:
    case CL_SNORM_INT16:
      numFormat = 1;
      break;
    case CL_UNSIGNED_INT8:
    case CL_UNSIGNED_INT16:
    case CL_UNSIGNED_INT32:
      numFormat = 4;
      break;
    case CL_SIGNED_INT8:
    case CL_SIGNED_INT16:
    case CL_SIGNED_INT32:
      numFormat = 5;
      break;
    case CL_HALF_FLOAT:
    case CL_FLOAT:
      numFormat = 7;
      break;
    default:
      return {};
  }

  uint32_t bits = 0;
  switch (format.image_channel_data_type) {
    case CL_UNORM_INT8:
    case CL_SNORM_INT8:
    case CL_UNSIGNED_INT8:
    case CL_SIGNED_INT8:
      bits = 8;
      break;
    case CL_UNORM_INT16:
    case CL_SNORM_INT16:
    case CL_UNSIGNED_INT16:
    case CL_SIGNED_INT16:
    case CL_HALF_FLOAT:
      bits = 16;
      break;
    default:
      bits = 32;
      break;
  }

  uint32_t dataFormat = 0;
  if (channels == 1) {
    dataFormat = (bits == 8) ? 1 : (bits == 16) ? 2 : 4;
  } else if (channels == 2) {
    dataFormat = (bits == 8) ? 3 : (bits == 16) ? 5 : 11;
  } else {
    dataFormat = (bits == 8) ? 10 : (bits == 16) ? 12 : 14;
  }

  uint32_t dstSel = 0;
  if (channels > 1) {
    dstSel |= 1u << 2;
  }
  if (channels > 2) {
    dstSel |= 2u << 4;
  }
  if (channels > 3) {
    dstSel |= 3u << 6;
  }

  return {dstSel, dataFormat, numFormat};
}

}  // namespace

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

  // HIP guarantees that device allocations are aligned to at least 256 bytes.
  constexpr size_t kDeviceAllocationAlignment = 256;

  // posix_memalign requires a nonzero power-of-two multiple of sizeof(void*).
  alignment = std::max(alignment, static_cast<size_t>(dev_.info().memBaseAddrAlign_));
  alignment = std::max(alignment, kDeviceAllocationAlignment);
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

Image::Image(const Device& dev, amd::Memory& owner) : Memory(dev, owner) {
  std::memset(srd_, 0, sizeof(srd_));

  const amd::Image* image = owner.asImage();
  if (image == nullptr) {
    return;
  }

  const uint64_t base = virtualAddress();
  const auto [dstSel, dataFormat, numFormat] = getSrdFormat(*image);

  if (image->getType() == CL_MEM_OBJECT_IMAGE1D_BUFFER) {
    const size_t elementSize = image->getImageFormat().getElementSize();
    srd_[0] = static_cast<uint32_t>(base);
    srd_[1] = static_cast<uint32_t>(base >> 32) | (static_cast<uint32_t>(elementSize) << 16);
    srd_[2] = static_cast<uint32_t>(image->getWidth());
    srd_[3] = dstSel | (numFormat << 8) | (dataFormat << 12);
    return;
  }

  uint32_t type = 0x8;
  switch (image->getType()) {
    case CL_MEM_OBJECT_IMAGE2D:
      type = 0x9;
      break;
    case CL_MEM_OBJECT_IMAGE3D:
      type = 0xa;
      break;
    case CL_MEM_OBJECT_IMAGE1D_ARRAY:
      type = 0xc;
      break;
    case CL_MEM_OBJECT_IMAGE2D_ARRAY:
      type = 0xd;
      break;
    default:
      break;
  }

  const uint32_t width = static_cast<uint32_t>(image->getWidth());
  const uint32_t height = static_cast<uint32_t>(image->getHeight());
  const uint32_t depth = static_cast<uint32_t>(image->getDepth());
  if (width == 0 || height == 0 || depth == 0) {
    return;
  }

  srd_[0] = static_cast<uint32_t>(base);
  srd_[1] = static_cast<uint32_t>(base >> 32) | (dataFormat << 20) | (numFormat << 26);
  srd_[2] = ((width - 1) & 0x3fffu) | (((height - 1) & 0x3fffu) << 14);
  srd_[3] = dstSel | (type << 28);
  srd_[4] = (depth - 1) & 0x1fffu;
  srd_[10] = width;
}

}  // namespace amd::cpu
