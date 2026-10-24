/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#include "device/cpu/cpuvirtual.hpp"

#include "device/blit.hpp"
#include "device/cpu/cpudevice.hpp"
#include "device/cpu/cpukernel.hpp"
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
};

VirtualDevice::VirtualDevice(Device& device)
    : device::VirtualDevice(device), dev_(device) {
  blitMgr_ = new BlitManager(*this);
  LogPrintfInfo("CPU VirtualDevice created: virtual=%p device=%p", this, &device);
}

VirtualDevice::~VirtualDevice() { delete blitMgr_; }

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
  bool result = false;
  bool imageBuffer = false;

  // Force buffer read for IMAGE1D_BUFFER
  if ((type == CL_COMMAND_READ_IMAGE) && (cmd.source().getType() == CL_MEM_OBJECT_IMAGE1D_BUFFER)) {
    type = CL_COMMAND_READ_BUFFER;
    imageBuffer = true;
  }

  cmd.setStatus(CL_RUNNING, amd::Os::timeNanos());

  switch (type) {
    case CL_COMMAND_READ_BUFFER: {
      if (devMem == nullptr) {
        break;
      }

      amd::Coord3D origin(cmd.origin()[0]);
      if (imageBuffer) {
        size_t elemSize = cmd.source().asImage()->getImageFormat().getElementSize();
        origin.c[0] *= elemSize;
        size.c[0] *= elemSize;
      }
      if (hostMemory != nullptr) {
        // Accelerated transfer without pinning
        amd::Coord3D dstOrigin(offset);
        result = blitMgr().copyBuffer(*devMem, *hostMemory, origin, dstOrigin, size,
                                      cmd.isEntireMemory(), cmd.copyMetadata());
      } else {
        result = blitMgr().readBuffer(*devMem, dst, origin, size, cmd.isEntireMemory(),
                                      cmd.copyMetadata());
      }
      break;
    }
    case CL_COMMAND_READ_BUFFER_RECT: {
      if (devMem == nullptr) {
        break;
      }

      amd::BufferRect hostbufferRect;
      amd::Coord3D region(0);
      amd::Coord3D hostOrigin(cmd.hostRect().start_ + offset);
      hostbufferRect.create(hostOrigin.c, size.c, cmd.hostRect().rowPitch_,
                            cmd.hostRect().slicePitch_);
      if (hostMemory != nullptr) {
        result = blitMgr().copyBufferRect(*devMem, *hostMemory, cmd.bufRect(), hostbufferRect, size,
                                          cmd.isEntireMemory(), cmd.copyMetadata());
      } else {
        result = blitMgr().readBufferRect(*devMem, dst, cmd.bufRect(), cmd.hostRect(), size,
                                          cmd.isEntireMemory(), cmd.copyMetadata());
      }
      break;
    }
    case CL_COMMAND_READ_IMAGE: {
      if (devMem == nullptr) {
        break;
      }

      result = blitMgr().readImage(*devMem, dst, cmd.origin(), size, cmd.rowPitch(),
                                   cmd.slicePitch(), cmd.isEntireMemory(), cmd.copyMetadata());
      break;
    }
    default:
      break;
  }

  if (!result) {
    LogError("submitReadMemory failed!");
    cmd.setStatus(CL_OUT_OF_RESOURCES);
  } else {
    cmd.setStatus(CL_COMPLETE, amd::Os::timeNanos());
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
  bool result = false;
  bool imageBuffer = false;

  // Force buffer write for IMAGE1D_BUFFER
  if ((type == CL_COMMAND_WRITE_IMAGE) &&
      (cmd.destination().getType() == CL_MEM_OBJECT_IMAGE1D_BUFFER)) {
    type = CL_COMMAND_WRITE_BUFFER;
    imageBuffer = true;
  }

  cmd.setStatus(CL_RUNNING, amd::Os::timeNanos());

  switch (type) {
    case CL_COMMAND_WRITE_BUFFER: {
      amd::Coord3D origin(cmd.origin()[0]);
      if (imageBuffer) {
        size_t elemSize = cmd.destination().asImage()->getImageFormat().getElementSize();
        origin.c[0] *= elemSize;
        size.c[0] *= elemSize;
      }
      if (devMem == nullptr || devMem->virtualAddress() == 0) {
        break;
      }

      if (hostMemory != nullptr) {
        // Accelerated transfer without pinning
        amd::Coord3D srcOrigin(offset);
        result = blitMgr().copyBuffer(*hostMemory, *devMem, srcOrigin, origin, size,
                                      cmd.isEntireMemory(), cmd.copyMetadata());
      } else {
        result = blitMgr().writeBuffer(src, *devMem, origin, size, cmd.isEntireMemory(),
                                       cmd.copyMetadata());
      }
      break;
    }
    case CL_COMMAND_WRITE_BUFFER_RECT: {
      amd::BufferRect hostbufferRect;
      amd::Coord3D region(0);
      amd::Coord3D hostOrigin(cmd.hostRect().start_ + offset);
      hostbufferRect.create(hostOrigin.c, size.c, cmd.hostRect().rowPitch_,
                            cmd.hostRect().slicePitch_);
      if (devMem == nullptr || devMem->virtualAddress() == 0) {
        break;
      }

      if (hostMemory != nullptr) {
        result = blitMgr().copyBufferRect(*hostMemory, *devMem, hostbufferRect, cmd.bufRect(), size,
                                          cmd.isEntireMemory(), cmd.copyMetadata());
      } else {
        result = blitMgr().writeBufferRect(src, *devMem, cmd.hostRect(), cmd.bufRect(), size,
                                           cmd.isEntireMemory(), cmd.copyMetadata());
      }
      break;
    }
    case CL_COMMAND_WRITE_IMAGE: {
      if (devMem == nullptr || devMem->virtualAddress() == 0) {
        break;
      }

      if (hostMemory != nullptr) {
        amd::Coord3D srcOrigin(offset);
        result = blitMgr().copyBufferToImage(*hostMemory, *devMem, srcOrigin, cmd.origin(), size,
                                             cmd.isEntireMemory(), cmd.rowPitch(),
                                             cmd.slicePitch(), cmd.copyMetadata());
      } else {
        result = blitMgr().writeImage(src, *devMem, cmd.origin(), size, cmd.rowPitch(),
                                      cmd.slicePitch(), cmd.isEntireMemory(), cmd.copyMetadata());
      }
      break;
    }
    default:
      break;
  }

  if (!result) {
    LogError("submitWriteMemory failed!");
    cmd.setStatus(CL_OUT_OF_RESOURCES);
  } else {
    cmd.destination().signalWrite(&dev_);
    cmd.setStatus(CL_COMPLETE, amd::Os::timeNanos());
  }
  // profilingEnd();
}

void VirtualDevice::submitCopyMemory(amd::CopyMemoryCommand& cmd) {
  // Make sure VirtualDevice has an exclusive access to the resources
  std::scoped_lock lock(execution());

  // profilingBegin(cmd, true);

  cl_command_type type = cmd.type();
  bool entire = cmd.isEntireMemory();
  bool result = false;

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

  cmd.setStatus(CL_RUNNING, amd::Os::timeNanos());

  switch (type) {
    case CL_COMMAND_SVM_MEMCPY:
    case CL_COMMAND_COPY_BUFFER: {
      result = blitMgr().copyBuffer(*srcDevMem, *dstDevMem, cmd.srcOrigin(), cmd.dstOrigin(),
                                    cmd.size(), entire, cmd.copyMetadata());
      break;
    }
    case CL_COMMAND_COPY_BUFFER_RECT: {
      result = blitMgr().copyBufferRect(*srcDevMem, *dstDevMem, cmd.srcRect(), cmd.dstRect(),
                                        cmd.size(), entire, cmd.copyMetadata());
      break;
    }
    case CL_COMMAND_COPY_IMAGE: {
      result = blitMgr().copyImage(*srcDevMem, *dstDevMem, cmd.srcOrigin(), cmd.dstOrigin(),
                                   cmd.size(), entire, cmd.copyMetadata());
      break;
    }
    case CL_COMMAND_COPY_IMAGE_TO_BUFFER: {
      result = blitMgr().copyImageToBuffer(*srcDevMem, *dstDevMem, cmd.srcOrigin(), cmd.dstOrigin(),
                                           cmd.size(), entire, cmd.dstRect().rowPitch_,
                                           cmd.dstRect().slicePitch_, cmd.copyMetadata());
      break;
    }
    case CL_COMMAND_COPY_BUFFER_TO_IMAGE: {
      result = blitMgr().copyBufferToImage(*srcDevMem, *dstDevMem, cmd.srcOrigin(), cmd.dstOrigin(),
                                            cmd.size(), entire, cmd.srcRect().rowPitch_,
                                            cmd.srcRect().slicePitch_, cmd.copyMetadata());
      break;
    }
    default:
      break;
  }

  if (!result) {
    LogError("submitCopyMemory failed!");
    cmd.setStatus(CL_INVALID_OPERATION);
  } else {
    cmd.destination().signalWrite(&dev_);
    cmd.setStatus(CL_COMPLETE, amd::Os::timeNanos());
  }
  // profilingEnd();
}

void VirtualDevice::submitMarker(amd::Marker& cmd) { flush(cmd.GetBatchHead()); }

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
  LogPrintfInfo(
      "CPU VirtualDevice::submitKernel ndrange: dims=%zu offset={%zu,%zu,%zu} "
      "global={%zu,%zu,%zu} local={%zu,%zu,%zu} sharedMem=%zu",
      sizes.dimensions(), off[0], off[1], off[2], gws[0], gws[1], gws[2], lws[0], lws[1], lws[2],
      cmd.sharedMemBytes());
  LogPrintfInfo(
      "CPU VirtualDevice::submitKernel kernel info: devKernel=%p params=%u paramsSize=%zu "
      "wgSize=%zu",
      devKernel, kernel.signature().numParameters(), kernel.signature().paramsSize(),
      devKernel != nullptr ? devKernel->workGroupInfo()->size_ : 0);

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

  // Set the dynamic shared memory size before the work-groups are dispatched.
  if (hostKernel->dynamicSharedSetter() != nullptr) {
    hostKernel->dynamicSharedSetter()(static_cast<uint64_t>(cmd.sharedMemBytes()));
  }

  // One dispatch call per work-group. The work-group function iterates the work
  // items.
  for (size_t z = 0; z < state.num_groups[2]; ++z) {
    for (size_t y = 0; y < state.num_groups[1]; ++y) {
      for (size_t x = 0; x < state.num_groups[0]; ++x) {
        state.workgroup_id[0] = x;
        state.workgroup_id[1] = y;
        state.workgroup_id[2] = z;
        hostKernel->entry()(args.data(), &state);
      }
    }
  }

  cmd.setStatus(CL_COMPLETE, amd::Os::timeNanos());
}

void VirtualDevice::flush(amd::Command* list, bool) {
  std::scoped_lock lock(execution());

  amd::Command* current = list;
  while (current != nullptr) {
    amd::Command* next = current->getNext();
    if (current->status() == CL_SUBMITTED || current->status() == CL_RUNNING) {
      const uint64_t timeStamp = amd::Os::timeNanos();
      if (current->status() == CL_SUBMITTED) {
        current->setStatus(CL_RUNNING, timeStamp);
      }
      current->setStatus(CL_COMPLETE, timeStamp);
    } else if (current->status() > CL_COMPLETE) {
      LogPrintfError("Unexpected command status - %d.", current->status());
    }

    current->release();
    current = next;
  }
}

void VirtualDevice::Unsupported(const char* command) {
  LogPrintfError("The spvcpu device does not implement the %s command!", command);
}

}  // namespace amd::cpu
