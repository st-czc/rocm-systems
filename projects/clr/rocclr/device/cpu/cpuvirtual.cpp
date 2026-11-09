/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#include "device/cpu/cpuvirtual.hpp"

#include "device/blit.hpp"
#include "device/cpu/cpudevice.hpp"
#include "device/cpu/cpukernel.hpp"
#include "device/cpu/threadpool.hpp"
#include "os/os.hpp"
#include "utils/debug.hpp"

#include <cstdint>
#include <mutex>
#include <vector>

namespace amd::cpu {

class BlitManager final : public device::HostBlitManager {
 public:
  BlitManager(VirtualDevice& virtualDevice) : HostBlitManager(virtualDevice) {}

  virtual bool streamOpsWrite(device::Memory& memory, uint64_t value, size_t offset,
                              size_t sizeBytes) const override {
    return false;
  }

  virtual bool streamOpsIncrement(device::Memory& memory, uint64_t value, size_t offset,
                                  size_t sizeBytes) const override {
    return false;
  }

  virtual bool streamOpsDecrement(device::Memory& memory, uint64_t value, size_t offset,
                                  size_t sizeBytes) const override {
    return false;
  }

  virtual bool streamOpsWait(device::Memory& memory, uint64_t value, size_t offset,
                             size_t sizeBytes, uint64_t flags, uint64_t mask) const override {
    return false;
  }

  virtual bool batchMemOps(const void* paramArray, size_t paramSize,
                           uint32_t count) const override {
    return false;
  }

  virtual bool initHeap(device::Memory* heapToInitialize, device::Memory* initialBlocks,
                        uint heapSize, uint numberOfInitialBlocks) const override {
    return false;
  }

  virtual bool fillBuffer(device::Memory& memory, const void* pattern, size_t patternSize,
                          const amd::Coord3D& surface, const amd::Coord3D& origin,
                          const amd::Coord3D& size, bool entire, bool forceBlit) const override {
    const size_t width = size[0];
    const size_t height = size[1];
    const size_t depth = size[2];

    amd::Coord3D region{surface[1], surface[2], depth};
    amd::Coord3D start(origin);
    amd::BufferRect rect;
    if (!rect.create(static_cast<size_t*>(start), static_cast<size_t*>(region), surface[0], 0)) {
      LogError("Misaligned fill surface!");
      return false;
    }

    // Fill one row at a time so that the row pitch of the surface is honored
    for (size_t slice = 0; slice < depth; ++slice) {
      for (size_t row = 0; row < height; ++row) {
        if (!HostBlitManager::fillBuffer(memory, pattern, patternSize, surface,
                                         amd::Coord3D{rect.offset(0, row, slice), 0, 0},
                                         amd::Coord3D{width, 1, 1}, entire, forceBlit)) {
          return false;
        }
      }
    }
    return true;
  }
};

VirtualDevice::VirtualDevice(Device& device)
    : device::VirtualDevice(device),
      dev_(device),
      pool_(device.pool()),
      queue_(device.pool().newQueue(this)) {
  blitMgr_ = new BlitManager(*this);
  LogPrintfInfo("CPU VirtualDevice created: virtual=%p device=%p", this, &device);
}

VirtualDevice::~VirtualDevice() {
  delete blitMgr_;
  pool_.deleteQueue(this);
}

void VirtualDevice::submit(amd::Command& cmd, std::function<void()> work) {
  submit(cmd, 1, [work = std::move(work)](int) { work(); });
}

void VirtualDevice::submit(amd::Command& cmd, uint32_t numTickets,
                           std::function<void(int)> task) {
  // Make sure VirtualDevice has an exclusive access to the resources
  std::scoped_lock lock(execution());

  Packet* packet = pool_.newPacket(&cmd, numTickets, cmd.eventWaitList());
  packet->task = std::move(task);
  queue_->submit(packet);
}

//! An unimplemented command must still retire, or its batch never completes.
void VirtualDevice::submitUnsupported(amd::Command& cmd, const char* command) {
  std::scoped_lock lock(execution());

  Unsupported(command);
  cmd.setStatus(CL_INVALID_OPERATION);
}

void VirtualDevice::submitReadMemory(amd::ReadMemoryCommand& cmd) {
  // Make sure VirtualDevice has an exclusive access to the resources
  std::scoped_lock lock(execution());

  // profilingBegin(cmd, true);

  size_t offset = 0;
  // Find if virtual address is a CL allocation
  device::Memory* hostMemory = dev_.findMemoryFromVA(cmd.destination(), &offset);

  Memory* devMem = static_cast<Memory*>(cmd.source().getDeviceMemory(dev_, false));

  // Synchronize data with other memory instances if necessary
  if (devMem != nullptr) {
    devMem->syncCacheFromHost(*this);
  }

  void* dst = cmd.destination();
  amd::Coord3D size = cmd.size();

  cl_command_type type = cmd.type();
  bool imageBuffer = false;

  // Force buffer read for IMAGE1D_BUFFER
  if ((type == CL_COMMAND_READ_IMAGE) && (cmd.source().getType() == CL_MEM_OBJECT_IMAGE1D_BUFFER)) {
    type = CL_COMMAND_READ_BUFFER;
    imageBuffer = true;
  }

  switch (type) {
    case CL_COMMAND_READ_BUFFER: {
      if (devMem == nullptr) {
        cmd.setStatus(CL_OUT_OF_RESOURCES);
        return;
      }

      amd::Coord3D origin(cmd.origin()[0]);
      if (imageBuffer) {
        size_t elemSize = cmd.source().asImage()->getImageFormat().getElementSize();
        origin.c[0] *= elemSize;
        size.c[0] *= elemSize;
      }
      const address src = static_cast<address>(devMem->cpuMap(*this)) + origin[0];
      if (hostMemory != nullptr) {
        // Accelerated transfer without pinning
        const address dstMem = static_cast<address>(hostMemory->cpuMap(*this)) + offset;
        submit(cmd, [=]() { std::memcpy(dstMem, src, size[0]); });
      } else {
        submit(cmd, [=]() { std::memcpy(dst, src, size[0]); });
      }
      break;
    }
    case CL_COMMAND_READ_BUFFER_RECT: {
      if (devMem == nullptr) {
        cmd.setStatus(CL_OUT_OF_RESOURCES);
        return;
      }

      const address src = static_cast<address>(devMem->cpuMap(*this));
      if (hostMemory != nullptr) {
        amd::BufferRect hostbufferRect;
        amd::Coord3D region(0);
        amd::Coord3D hostOrigin(cmd.hostRect().start_ + offset);
        hostbufferRect.create(hostOrigin.c, size.c, cmd.hostRect().rowPitch_,
                              cmd.hostRect().slicePitch_);
        const address dstMem = static_cast<address>(hostMemory->cpuMap(*this));
        const amd::BufferRect srcRect(cmd.bufRect());
        const amd::BufferRect dstRect(hostbufferRect);
        submit(cmd, [=]() {
          for (size_t z = 0; z < size[2]; ++z) {
            for (size_t y = 0; y < size[1]; ++y) {
              std::memcpy(dstMem + dstRect.offset(0, y, z), src + srcRect.offset(0, y, z), size[0]);
            }
          }
        });
      } else {
        const amd::BufferRect srcRect(cmd.bufRect());
        const amd::BufferRect dstRect(cmd.hostRect());
        submit(cmd, [=]() {
          for (size_t z = 0; z < size[2]; ++z) {
            for (size_t y = 0; y < size[1]; ++y) {
              std::memcpy(static_cast<address>(dst) + dstRect.offset(0, y, z),
                          src + srcRect.offset(0, y, z), size[0]);
            }
          }
        });
      }
      break;
    }
    case CL_COMMAND_READ_IMAGE: {
      if (devMem == nullptr) {
        cmd.setStatus(CL_OUT_OF_RESOURCES);
        return;
      }

      const amd::Image* image = cmd.source().asImage();
      const size_t elementSize = image->getImageFormat().getElementSize();
      size_t rowPitch = cmd.rowPitch();
      size_t slicePitch = cmd.slicePitch();
      if (rowPitch == 0) {
        rowPitch = size[0] * elementSize;
      }
      if (slicePitch == 0) {
        slicePitch = size[0] * size[1] * elementSize;
      }
      const size_t imageRowPitch = image->getRowPitch();
      const size_t imageSlicePitch = image->getSlicePitch();
      const size_t firstRow = cmd.origin()[0] * elementSize + imageRowPitch * cmd.origin()[1] +
                              imageSlicePitch * cmd.origin()[2];
      const address src = static_cast<address>(devMem->cpuMap(*this));
      submit(cmd, [=]() {
        for (size_t z = 0; z < size[2]; ++z) {
          for (size_t y = 0; y < size[1]; ++y) {
            std::memcpy(static_cast<address>(dst) + rowPitch * y + slicePitch * z,
                        src + firstRow + imageRowPitch * y + imageSlicePitch * z,
                        size[0] * elementSize);
          }
        }
      });
      break;
    }
    default:
      submitUnsupported(cmd, "ReadMemory");
      break;
  }
  // profilingEnd();
}

void VirtualDevice::submitWriteMemory(amd::WriteMemoryCommand& cmd) {
  // Make sure VirtualDevice has an exclusive access to the resources
  std::scoped_lock lock(execution());

  // profilingBegin(cmd, true);

  size_t offset = 0;
  // Find if virtual address is a CL allocation
  device::Memory* hostMemory = dev_.findMemoryFromVA(cmd.source(), &offset);

  Memory* devMem = static_cast<Memory*>(cmd.destination().getDeviceMemory(dev_, false));

  // Synchronize memory from host if necessary
  device::Memory::SyncFlags syncFlags;
  syncFlags.skipEntire_ = cmd.isEntireMemory();
  if (devMem != nullptr) {
    devMem->syncCacheFromHost(*this, syncFlags);
  }

  const char* src = static_cast<const char*>(cmd.source());
  amd::Coord3D size = cmd.size();

  cl_command_type type = cmd.type();
  bool imageBuffer = false;

  // Force buffer write for IMAGE1D_BUFFER
  if ((type == CL_COMMAND_WRITE_IMAGE) &&
      (cmd.destination().getType() == CL_MEM_OBJECT_IMAGE1D_BUFFER)) {
    type = CL_COMMAND_WRITE_BUFFER;
    imageBuffer = true;
  }

  switch (type) {
    case CL_COMMAND_WRITE_BUFFER: {
      if (devMem == nullptr || devMem->virtualAddress() == 0) {
        cmd.setStatus(CL_OUT_OF_RESOURCES);
        return;
      }

      amd::Coord3D origin(cmd.origin()[0]);
      if (imageBuffer) {
        size_t elemSize = cmd.destination().asImage()->getImageFormat().getElementSize();
        origin.c[0] *= elemSize;
        size.c[0] *= elemSize;
      }
      address dst = static_cast<address>(devMem->cpuMap(*this)) + origin[0];
      if (hostMemory != nullptr) {
        // Accelerated transfer without pinning
        const address srcMem = static_cast<address>(hostMemory->cpuMap(*this)) + offset;
        submit(cmd, [=, &cmd]() {
          std::memcpy(dst, srcMem, size[0]);
          cmd.destination().signalWrite(&dev_);
        });
      } else {
        submit(cmd, [=, &cmd]() {
          std::memcpy(dst, src, size[0]);
          cmd.destination().signalWrite(&dev_);
        });
      }
      break;
    }
    case CL_COMMAND_WRITE_BUFFER_RECT: {
      if (devMem == nullptr || devMem->virtualAddress() == 0) {
        cmd.setStatus(CL_OUT_OF_RESOURCES);
        return;
      }

      const address dst = static_cast<address>(devMem->cpuMap(*this));
      amd::BufferRect hostbufferRect;
      if (hostMemory != nullptr) {
        amd::Coord3D region(0);
        amd::Coord3D hostOrigin(cmd.hostRect().start_ + offset);
        hostbufferRect.create(hostOrigin.c, size.c, cmd.hostRect().rowPitch_,
                              cmd.hostRect().slicePitch_);
        const address srcMem = static_cast<address>(hostMemory->cpuMap(*this));
        const amd::BufferRect srcRect(hostbufferRect);
        const amd::BufferRect dstRect(cmd.bufRect());
        submit(cmd, [=, &cmd]() {
          for (size_t z = 0; z < size[2]; ++z) {
            for (size_t y = 0; y < size[1]; ++y) {
              std::memcpy(dst + dstRect.offset(0, y, z), srcMem + srcRect.offset(0, y, z), size[0]);
            }
          }
          cmd.destination().signalWrite(&dev_);
        });
      } else {
        const amd::BufferRect srcRect(cmd.hostRect());
        const amd::BufferRect dstRect(cmd.bufRect());
        submit(cmd, [=, &cmd]() {
          for (size_t z = 0; z < size[2]; ++z) {
            for (size_t y = 0; y < size[1]; ++y) {
              std::memcpy(dst + dstRect.offset(0, y, z),
                          reinterpret_cast<address>(const_cast<char*>(src)) + srcRect.offset(0, y, z),
                          size[0]);
            }
          }
          cmd.destination().signalWrite(&dev_);
        });
      }
      break;
    }
    case CL_COMMAND_WRITE_IMAGE: {
      if (devMem == nullptr || devMem->virtualAddress() == 0) {
        cmd.setStatus(CL_OUT_OF_RESOURCES);
        return;
      }

      const amd::Image* image = cmd.destination().asImage();
      const size_t elementSize = image->getImageFormat().getElementSize();
      const size_t imageRowPitch = image->getRowPitch();
      const size_t imageSlicePitch = image->getSlicePitch();
      const size_t firstRow = cmd.origin()[0] * elementSize + imageRowPitch * cmd.origin()[1] +
                              imageSlicePitch * cmd.origin()[2];
      address dst = static_cast<address>(devMem->cpuMap(*this));
      if (hostMemory != nullptr) {
        const address srcMem = static_cast<address>(hostMemory->cpuMap(*this)) + offset;
        submit(cmd, [=, &cmd]() {
          for (size_t z = 0; z < size[2]; ++z) {
            for (size_t y = 0; y < size[1]; ++y) {
              std::memcpy(dst + firstRow + imageRowPitch * y + imageSlicePitch * z,
                          srcMem + size[0] * elementSize * (y + size[1] * z),
                          size[0] * elementSize);
            }
          }
          cmd.destination().signalWrite(&dev_);
        });
      } else {
        const size_t rowPitch = size[0] * elementSize;
        submit(cmd, [=, &cmd]() {
          for (size_t z = 0; z < size[2]; ++z) {
            for (size_t y = 0; y < size[1]; ++y) {
              std::memcpy(dst + firstRow + imageRowPitch * y + imageSlicePitch * z,
                          reinterpret_cast<address>(const_cast<char*>(src)) + rowPitch * y +
                              rowPitch * size[1] * z,
                          rowPitch);
            }
          }
          cmd.destination().signalWrite(&dev_);
        });
      }
      break;
    }
    default:
      submitUnsupported(cmd, "WriteMemory");
      break;
  }
  // profilingEnd();
}

void VirtualDevice::submitCopyMemory(amd::CopyMemoryCommand& cmd) {
  // Make sure VirtualDevice has an exclusive access to the resources
  std::scoped_lock lock(execution());

  // profilingBegin(cmd, true);

  cl_command_type type = cmd.type();
  bool entire = cmd.isEntireMemory();
  const amd::Coord3D size = cmd.size();

  Memory* srcDevMem = static_cast<Memory*>(cmd.source().getDeviceMemory(dev_, false));
  Memory* dstDevMem = static_cast<Memory*>(cmd.destination().getDeviceMemory(dev_, false));
  if (srcDevMem == nullptr || dstDevMem == nullptr) {
    LogError("submitCopyMemory failed!");
    cmd.setStatus(CL_INVALID_OPERATION);
    return;
  }

  // Synchronize source and destination memory
  device::Memory::SyncFlags syncFlags;
  syncFlags.skipEntire_ = entire;
  dstDevMem->syncCacheFromHost(*this, syncFlags);
  srcDevMem->syncCacheFromHost(*this);

  switch (type) {
    case CL_COMMAND_SVM_MEMCPY:
    case CL_COMMAND_COPY_BUFFER: {
      const address src = static_cast<address>(srcDevMem->cpuMap(*this)) + cmd.srcOrigin()[0];
      address dst = static_cast<address>(dstDevMem->cpuMap(*this)) + cmd.dstOrigin()[0];
      submit(cmd, [=, &cmd]() {
        std::memcpy(dst, src, size[0]);
        cmd.destination().signalWrite(&dev_);
      });
      break;
    }
    case CL_COMMAND_COPY_BUFFER_RECT: {
      const address src = static_cast<address>(srcDevMem->cpuMap(*this));
      address dst = static_cast<address>(dstDevMem->cpuMap(*this));
      const amd::BufferRect srcRect(cmd.srcRect());
      const amd::BufferRect dstRect(cmd.dstRect());
      submit(cmd, [=, &cmd]() {
        for (size_t z = 0; z < size[2]; ++z) {
          for (size_t y = 0; y < size[1]; ++y) {
            std::memcpy(dst + dstRect.offset(0, y, z), src + srcRect.offset(0, y, z), size[0]);
          }
        }
        cmd.destination().signalWrite(&dev_);
      });
      break;
    }
    case CL_COMMAND_COPY_IMAGE: {
      const address src = static_cast<address>(srcDevMem->cpuMap(*this));
      address dst = static_cast<address>(dstDevMem->cpuMap(*this));
      const size_t elementSize = cmd.source().asImage()->getImageFormat().getElementSize();
      const size_t srcRowPitch = cmd.source().asImage()->getRowPitch();
      const size_t srcSlicePitch = cmd.source().asImage()->getSlicePitch();
      const size_t dstRowPitch = cmd.destination().asImage()->getRowPitch();
      const size_t dstSlicePitch = cmd.destination().asImage()->getSlicePitch();
      const size_t srcFirst = cmd.srcOrigin()[0] * elementSize + srcRowPitch * cmd.srcOrigin()[1] +
                              srcSlicePitch * cmd.srcOrigin()[2];
      const size_t dstFirst = cmd.dstOrigin()[0] * elementSize + dstRowPitch * cmd.dstOrigin()[1] +
                              dstSlicePitch * cmd.dstOrigin()[2];
      submit(cmd, [=, &cmd]() {
        for (size_t z = 0; z < size[2]; ++z) {
          for (size_t y = 0; y < size[1]; ++y) {
            std::memcpy(dst + dstFirst + dstRowPitch * y + dstSlicePitch * z,
                        src + srcFirst + srcRowPitch * y + srcSlicePitch * z,
                        size[0] * elementSize);
          }
        }
        cmd.destination().signalWrite(&dev_);
      });
      break;
    }
    case CL_COMMAND_COPY_IMAGE_TO_BUFFER: {
      const address src = static_cast<address>(srcDevMem->cpuMap(*this));
      const amd::Image* image = cmd.source().asImage();
      const size_t elementSize = image->getImageFormat().getElementSize();
      const size_t imageRowPitch = image->getRowPitch();
      const size_t imageSlicePitch = image->getSlicePitch();
      const size_t srcFirst = cmd.srcOrigin()[0] * elementSize +
                              imageRowPitch * cmd.srcOrigin()[1] +
                              imageSlicePitch * cmd.srcOrigin()[2];
      size_t dstRowPitch = cmd.dstRect().rowPitch_;
      if (dstRowPitch == 0) {
        dstRowPitch = size[0] * elementSize;
      }
      size_t dstSlicePitch = cmd.dstRect().slicePitch_;
      if (dstSlicePitch == 0) {
        dstSlicePitch = dstRowPitch * size[1];
      }
      const address dst = static_cast<address>(dstDevMem->cpuMap(*this)) + cmd.dstOrigin()[0];
      submit(cmd, [=, &cmd]() {
        for (size_t z = 0; z < size[2]; ++z) {
          for (size_t y = 0; y < size[1]; ++y) {
            std::memcpy(dst + dstRowPitch * y + dstSlicePitch * z,
                        src + srcFirst + imageRowPitch * y + imageSlicePitch * z,
                        size[0] * elementSize);
          }
        }
        cmd.destination().signalWrite(&dev_);
      });
      break;
    }
    case CL_COMMAND_COPY_BUFFER_TO_IMAGE: {
      const address src = static_cast<address>(srcDevMem->cpuMap(*this));
      const amd::Image* image = cmd.destination().asImage();
      const size_t elementSize = image->getImageFormat().getElementSize();
      const size_t imageRowPitch = image->getRowPitch();
      const size_t imageSlicePitch = image->getSlicePitch();
      const size_t dstFirst = cmd.dstOrigin()[0] * elementSize +
                              imageRowPitch * cmd.dstOrigin()[1] +
                              imageSlicePitch * cmd.dstOrigin()[2];
      size_t srcRowPitch = cmd.srcRect().rowPitch_;
      if (srcRowPitch == 0) {
        srcRowPitch = size[0] * elementSize;
      }
      size_t srcSlicePitch = cmd.srcRect().slicePitch_;
      if (srcSlicePitch == 0) {
        srcSlicePitch = srcRowPitch * size[1];
      }
      address dst = static_cast<address>(dstDevMem->cpuMap(*this));
      submit(cmd, [=, &cmd]() {
        for (size_t z = 0; z < size[2]; ++z) {
          for (size_t y = 0; y < size[1]; ++y) {
            std::memcpy(dst + dstFirst + imageRowPitch * y + imageSlicePitch * z,
                        src + srcRowPitch * y + srcSlicePitch * z,
                        size[0] * elementSize);
          }
        }
        cmd.destination().signalWrite(&dev_);
      });
      break;
    }
    default:
      submitUnsupported(cmd, "CopyMemory");
      break;
  }
  // profilingEnd();
}

void VirtualDevice::submitBatchCopyMemory(amd::BatchCopyMemoryCommand& cmd) {
  // Make sure VirtualDevice has an exclusive access to the resources
  std::scoped_lock lock(execution());

  const auto& copyOps = cmd.copyOps();
  if (copyOps.empty()) {
    cmd.setStatus(CL_COMPLETE, amd::Os::timeNanos());
    return;
  }

  struct CopySpan {
    address src;
    address dst;
    size_t size;
  };
  std::vector<CopySpan> spans;
  spans.reserve(copyOps.size());
  std::vector<amd::Memory*> written;
  written.reserve(copyOps.size());

  // Synchronize source and destination memory
  device::Memory::SyncFlags syncFlags;
  syncFlags.skipEntire_ = false;
  for (const auto& op : copyOps) {
    Memory* srcDevMem = static_cast<Memory*>(op.srcMemory->getDeviceMemory(dev_, false));
    Memory* dstDevMem = static_cast<Memory*>(op.dstMemory->getDeviceMemory(dev_, false));
    if (srcDevMem == nullptr || dstDevMem == nullptr) {
      LogError("submitBatchCopyMemory: Invalid memory objects!");
      cmd.setStatus(CL_INVALID_MEM_OBJECT);
      return;
    }
    dstDevMem->syncCacheFromHost(*this, syncFlags);
    srcDevMem->syncCacheFromHost(*this);
    spans.push_back({static_cast<address>(srcDevMem->cpuMap(*this)) + op.srcOffset,
                     static_cast<address>(dstDevMem->cpuMap(*this)) + op.dstOffset, op.size});
    written.push_back(op.dstMemory);
  }

  submit(cmd, [spans = std::move(spans), written = std::move(written), this]() {
    for (const auto& span : spans) {
      std::memcpy(span.dst, span.src, span.size);
    }
    for (auto* memory : written) {
      memory->signalWrite(&dev_);
    }
  });
}

void VirtualDevice::SubmitBatchWriteMemory(amd::BatchWriteMemoryCommand& cmd) {
  // Make sure VirtualDevice has an exclusive access to the resources
  std::scoped_lock lock(execution());

  const std::vector<amd::BatchWriteMemoryOp>& writeOps = cmd.WriteOps();
  if (writeOps.empty()) {
    cmd.setStatus(CL_COMPLETE, amd::Os::timeNanos());
    return;
  }

  struct WriteSpan {
    const void* src;
    address dst;
    size_t size;
  };
  std::vector<WriteSpan> spans;
  spans.reserve(writeOps.size());
  std::vector<amd::Memory*> written;
  written.reserve(writeOps.size());
  for (const auto& op : writeOps) {
    Memory* dstDevMem = static_cast<Memory*>(op.dst_memory->getDeviceMemory(dev_, false));
    if (dstDevMem == nullptr) {
      LogError("SubmitBatchWriteMemory: Invalid memory objects!");
      cmd.setStatus(CL_INVALID_MEM_OBJECT);
      return;
    }
    spans.push_back({op.src_host,
                     static_cast<address>(dstDevMem->cpuMap(*this)) + op.dst_offset, op.size});
    written.push_back(op.dst_memory);
  }

  submit(cmd, [spans = std::move(spans), written = std::move(written), this]() {
    for (const auto& span : spans) {
      std::memcpy(span.dst, span.src, span.size);
    }
    for (auto* memory : written) {
      memory->signalWrite(&dev_);
    }
  });
}

void VirtualDevice::SubmitBatchReadMemory(amd::BatchReadMemoryCommand& cmd) {
  // Make sure VirtualDevice has an exclusive access to the resources
  std::scoped_lock lock(execution());

  const std::vector<amd::BatchReadMemoryOp>& readOps = cmd.ReadOps();
  if (readOps.empty()) {
    cmd.setStatus(CL_COMPLETE, amd::Os::timeNanos());
    return;
  }

  struct ReadSpan {
    address src;
    void* dst;
    size_t size;
  };
  std::vector<ReadSpan> spans;
  spans.reserve(readOps.size());
  for (const auto& op : readOps) {
    Memory* srcDevMem = static_cast<Memory*>(op.src_memory->getDeviceMemory(dev_, false));
    if (srcDevMem == nullptr) {
      LogError("SubmitBatchReadMemory: Invalid memory objects!");
      cmd.setStatus(CL_INVALID_MEM_OBJECT);
      return;
    }
    spans.push_back({static_cast<address>(srcDevMem->cpuMap(*this)) + op.src_offset, op.dst_host,
                     op.size});
  }

  submit(cmd, [spans = std::move(spans)]() {
    for (const auto& span : spans) {
      std::memcpy(span.dst, span.src, span.size);
    }
  });
}

bool VirtualDevice::fillMemory(cl_command_type type, amd::Memory* amdMemory, const void* pattern,
                               size_t patternSize, const amd::Coord3D& surface,
                               const amd::Coord3D& origin, const amd::Coord3D& size) {
  Memory* memory = dev().getCPUMemory(amdMemory);

  bool entire = amdMemory->isEntirelyCovered(origin, size);
  // No necessary to synchronize memory from host for CPU

  bool result = false;

  // Find the the right fill operation
  switch (type) {
    case CL_COMMAND_SVM_MEMFILL:
    case CL_COMMAND_FILL_BUFFER: {
      result = blitMgr().fillBuffer(*memory, pattern, patternSize, surface, origin, size, entire);
      break;
    }    case CL_COMMAND_FILL_IMAGE: {
      // result = blitMgr().fillImage(*memory, pattern, origin, size, entire);
      Unsupported("FillImage");
      break;
    }
    default:
      LogError("Unsupported command type for FillMemory!");
      break;
  }

  if (!result) {
    LogError("submitFillMemory failed!");
    return false;
  }

  amdMemory->signalWrite(&dev());
  return true;
}

void VirtualDevice::submitFillMemory(amd::FillMemoryCommand& cmd) {
  // Make sure VirtualDevice has an exclusive access to the resources
  std::scoped_lock lock(execution());

  // profilingBegin(cmd, true);

  submit(cmd, [this, &cmd]() {
    fillMemory(cmd.type(), &cmd.memory(), cmd.pattern(), cmd.patternSize(), cmd.surface(),
               cmd.origin(), cmd.size());
  });
  // profilingEnd();
}

void VirtualDevice::submitMarker(amd::Marker& cmd) {
  const bool cpuWait = cmd.CpuWaitRequested();
  amd::Command* head = nullptr;
  Signal* signal = nullptr;
  {
    std::scoped_lock lock(execution());

    head = cmd.GetBatchHead();
    Packet* packet = pool_.newPacket(&cmd, 0, cmd.eventWaitList());
    signal = packet->signal;

    if (cpuWait) {
      // The wait below outlives the packet, which drops its own reference.
      signal->retain();
    } else if (head != nullptr) {
      // onDone may fire on a worker, on the app thread, or on the async thread,
      // so it only posts.
      signal->onDone([this, head] {
        pool_.postAsync([head] { updateBatchState(head); });
      });
    }
    queue_->submit(packet);
  }

  // The only thread that blocks on a signal is the app thread, as specified.
  if (cpuWait) {
    signal->wait();
    signal->release();
    if (head != nullptr) {
      updateBatchState(head);
    }
  }
}

void VirtualDevice::updateBatchState(amd::Command* list) {
  const uint64_t now = amd::Os::timeNanos();
  for (amd::Command* current = list; current != nullptr;) {
    amd::Command* next = current->getNext();
    // A command may already be COMPLETE (failed validation) or RUNNING (a submit
    // advanced it); both are expected. A negative status is terminal and is
    // released without being advanced.
    const int32_t status = current->status();
    if (status == CL_SUBMITTED || status == CL_RUNNING) {
      if (status == CL_SUBMITTED) {
        current->setStatus(CL_RUNNING, now);
      }
      current->setStatus(CL_COMPLETE, now);
    }
    current->release();
    current = next;
  }
}

void VirtualDevice::flush(amd::Command* list, bool) {}

void VirtualDevice::submitKernel(amd::NDRangeKernelCommand& cmd) {
  const amd::Kernel& kernel = cmd.kernel();
  const amd::NDRangeContainer& sizes = cmd.sizes();
  const amd::NDRange& gws = sizes.global();
  const amd::NDRange& lws = cmd.sizes().local();
  const amd::NDRange& off = sizes.offset();
  const device::Kernel* devKernel = kernel.getDeviceKernel(dev_);

  LogPrintfInfo(
      "CPU VirtualDevice::submitKernel reached: command=%p kernel=%p name=%s parameters=%p "
      "status=%d",
      &cmd, &kernel, kernel.name().c_str(), cmd.parameters(), cmd.status());

  const amd::KernelSignature& signature = kernel.signature();
  for (uint32_t i = 0; i < signature.numParameters(); ++i) {
    const amd::KernelParameterDescriptor& desc = signature.at(i);
    const void* slot = cmd.parameters() + desc.offset_;
    LogPrintfInfo(
        "CPU VirtualDevice::submitKernel arg %u: type=%d offset=%zu size=%zu value=0x%llx", i,
        static_cast<int>(desc.type_), desc.offset_, desc.size_,
        static_cast<unsigned long long>(*static_cast<const uint64_t*>(slot)));
  }

  const Kernel* hostKernel = static_cast<const Kernel*>(devKernel);
  if (hostKernel == nullptr || hostKernel->entry() == nullptr) {
    LogError("submitKernel: the kernel has no entry point!");
    cmd.setStatus(CL_INVALID_OPERATION);
    return;
  }

  // Subhandler arguments: one pointer per argument, the value for a pointer and
  // the address of the captured value otherwise.
  std::vector<void*> args(signature.numParameters());
  for (uint32_t i = 0; i < signature.numParameters(); ++i) {
    const amd::KernelParameterDescriptor& desc = signature.at(i);
    const_address slot = cmd.parameters() + desc.offset_;
    args[i] = (desc.type_ == T_POINTER) ? *reinterpret_cast<void* const*>(slot)
                                        : const_cast<address>(slot);
  }

  NativeCpuState state = {};
  for (size_t dim = 0; dim < 3; ++dim) {
    const size_t global = (dim < sizes.dimensions()) ? gws[dim] : 1;
    const size_t local = (dim < sizes.dimensions()) ? lws[dim] : 1;
    state.global_range[dim] = global;
    state.workgroup_size[dim] = local;
    state.num_groups[dim] = (local != 0) ? (global + local - 1) / local : 1;
    state.global_offset[dim] = (dim < sizes.dimensions()) ? off[dim] : 0;
  }

  LogPrintfInfo("CPU VirtualDevice::submitKernel dispatching: entry=%p groups={%zu,%zu,%zu}",
                reinterpret_cast<void*>(hostKernel->entry()), state.num_groups[0],
                state.num_groups[1], state.num_groups[2]);

  const Subhandler entry = hostKernel->entry();
  const size_t sharedMemBytes = static_cast<size_t>(cmd.sharedMemBytes());
  const size_t totalGroups =
      state.num_groups[0] * state.num_groups[1] * state.num_groups[2];
  if (totalGroups == 0) {
    cmd.setStatus(CL_INVALID_OPERATION);
    return;
  }

  // One ticket per worker, or one per group when there are fewer groups than
  // workers. Tickets are handed out in descending order, so the remainder lands
  // on the high tickets that start first.
  const uint32_t numTickets = static_cast<uint32_t>(std::min<size_t>(
      totalGroups, std::max<uint32_t>(pool_.numWorkers(), 1)));
  const size_t groupsPerTicket = totalGroups / numTickets;
  const size_t remainder = totalGroups % numTickets;

  // The subhandler reads the argument vector directly. The task runs after this
  // function returns, so the task owns the vector: a bare pointer would dangle.
  submit(cmd, numTickets,
         [args = std::move(args), entry, state, numTickets, groupsPerTicket, remainder,
          sharedMemBytes](int ticket) mutable {
           // Tickets are handed out high to low, so the larger share goes to the high
           // tickets that start first: the last `remainder` tickets take one extra.
           const size_t t = static_cast<size_t>(ticket);
           const bool isLarge = t >= numTickets - remainder;
           const size_t begin = groupsPerTicket * t + (isLarge ? t - (numTickets - remainder) : 0);
           const size_t count = groupsPerTicket + (isLarge ? 1 : 0);

           NativeCpuState ticketState = state;
           entry(args.data(), &ticketState, begin, begin + count, sharedMemBytes);
         });
}

void VirtualDevice::Unsupported(const char* command) {
  LogPrintfError("The spvcpu device does not implement the %s command!", command);
}

}  // namespace amd::cpu
