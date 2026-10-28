/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CPUDEVICE_HPP_
#define CPUDEVICE_HPP_

#include "device/device.hpp"
#include "device/devprogram.hpp"
#include "device/devsignal.hpp"

#include "cpukernel.hpp"
#include "cpumemory.hpp"
#include "cputhreadpool.hpp"
#include "cpuvirtual.hpp"

#include <string>
#include <vector>

//! CPU Device Implementation
namespace amd::cpu {

//! Offline CPU target. Runtime resources are disabled.
class NullDevice : public amd::Device {
 public:
  NullDevice();
  virtual ~NullDevice() = default;

  //! Creates an offline CPU target without an execution context.
  bool create(const amd::Isa& isa);

  //! Register an offline CPU target if no online device has the host ISA.
  static bool init();
  static void tearDown();

  virtual device::VirtualDevice* createVirtualDevice(amd::CommandQueue* queue = nullptr) override {
    ShouldNotReachHere();
    return nullptr;
  }

  //! Creates a CPU program object; compilation is still unsupported.
  virtual device::Program* createProgram(amd::Program& owner,
                                         amd::option::Options* options = nullptr) override;

  virtual device::Memory* createMemory(amd::Memory& owner) const override {
    ShouldNotReachHere();
    return nullptr;
  }

  virtual device::Memory* createMemory(size_t size, size_t alignment = 0) const override {
    ShouldNotReachHere();
    return nullptr;
  }

  //! Sampler object allocation
  virtual bool createSampler(const amd::Sampler& owner,  //!< abstraction layer sampler object
                             device::Sampler** sampler  //!< device sampler object
  ) const override {
    ShouldNotReachHere();
    return false;
  }

  virtual device::Memory* createView(
      amd::Memory& owner,           //!< Owner memory object
      const device::Memory& parent  //!< Parent device memory object for the view
  ) const override {
    ShouldNotReachHere();
    return nullptr;
  }

  virtual device::Signal* createSignal() const override {
    ShouldNotReachHere();
    return nullptr;
  }

  virtual bool bindExternalDevice(uint flags, void* const pDevice[], void* pContext,
                                  bool validateOnly) override {
    ShouldNotReachHere();
    return false;
  }

  virtual bool unbindExternalDevice(uint flags, void* const pDevice[], void* pContext,
                                    bool validateOnly) override {
    ShouldNotReachHere();
    return false;
  }

  virtual bool globalFreeMemory(size_t* freeMemory) const override {
    ShouldNotReachHere();
    return false;
  }

  virtual bool amdFileRead(amd::Os::FileDesc handle, void* devicePtr, uint64_t size,
                           int64_t file_offset, uint64_t* size_copied, int32_t* status) override {
    ShouldNotReachHere();
    return false;
  }

  virtual bool amdFileWrite(amd::Os::FileDesc handle, void* devicePtr, uint64_t size,
                            int64_t file_offset, uint64_t* size_copied, int32_t* status) override {
    ShouldNotReachHere();
    return false;
  }

  virtual void* svmAlloc(amd::Context& context, size_t size, size_t alignment,
                         cl_svm_mem_flags flags, void* svmPtr) const override {
    ShouldNotReachHere();
    return nullptr;
  }

  virtual void svmFree(void* ptr) const override { ShouldNotReachHere(); }

  virtual void* virtualAlloc(void* addr, size_t size, size_t alignment) override {
    ShouldNotReachHere();
    return nullptr;
  }

  virtual bool virtualFree(void* addr) override {
    ShouldNotReachHere();
    return false;
  }

  virtual cl_int virtualMap(void* va, size_t size, amd::Memory* phys) override {
    ShouldNotReachHere();
    return CL_INVALID_OPERATION;
  }

  virtual cl_int virtualUnmap(void* va, size_t size) override {
    ShouldNotReachHere();
    return CL_INVALID_OPERATION;
  }

  virtual bool SetMemAccess(void* va_addr, size_t va_size, VmmAccess access_flags,
                            VmmLocationType = VmmLocationType::kDevice,
                            int numaNode = -1) override {
    ShouldNotReachHere();
    return false;
  }

  virtual bool GetMemAccess(void* va_addr, VmmAccess* access_flags_ptr) const override {
    ShouldNotReachHere();
    return false;
  }

  virtual bool ValidateMemAccess(amd::Memory& mem, bool read_write) const override {
    ShouldNotReachHere();
    return false;
  }

  virtual bool importExtSemaphore(void** extSemaphore, const amd::Os::FileDesc& handle,
                                  amd::ExternalSemaphoreHandleType sem_handle_type) override {
    ShouldNotReachHere();
    return false;
  }

  virtual void DestroyExtSemaphore(void* extSemaphore) override { ShouldNotReachHere(); }

  virtual bool SetClockMode(const cl_set_device_clock_mode_input_amd setClockModeInput,
                            cl_set_device_clock_mode_output_amd* pSetClockModeOutput) override {
    ShouldNotReachHere();
    return false;
  }

#if defined(__clang__)
#if __has_feature(address_sanitizer)
  virtual device::UriLocator* createUriLocator() const override {
    ShouldNotReachHere();
    return nullptr;
  }
#endif
#endif

 protected:
  //! Fills common CPU target information.
  void fillDeviceInfo();

 private:
  NullDevice(const NullDevice&) = delete;
  NullDevice& operator=(const NullDevice&) = delete;
};

//! Online CPU device backed by process memory, reported as a compute device
//! so HIP discovers it.
class Device : public NullDevice {
 public:
  Device();
  bool create(const amd::Isa& isa);
  static bool init();
  static void tearDown();

  virtual device::VirtualDevice* createVirtualDevice(amd::CommandQueue* queue = nullptr) override {
    return new VirtualDevice(*this);
  }

  using NullDevice::createMemory;
  virtual device::Memory* createMemory(amd::Memory& owner) const override;

  //! Returns the CPU memory object for the given memory
  Memory* getCPUMemory(amd::Memory* mem) const {
    return static_cast<Memory*>(mem->getDeviceMemory(*this));
  }

  //! Returns the launch thread pool shared by all virtual devices
  ThreadPool& threadPool() const { return threadPool_; }

  virtual void* svmAlloc(amd::Context& context, size_t size, size_t alignment,
                         cl_svm_mem_flags flags = CL_MEM_READ_WRITE,
                         void* svmPtr = nullptr) const override;
  virtual void svmFree(void* ptr) const override;

  virtual device::Signal* createSignal() const override { return new device::HostSignal; }

  // Preserve the online device's existing query and placeholder behavior.
  virtual device::Memory* createView(amd::Memory& owner,
                                     const device::Memory& parent) const override {
    return nullptr;
  }

  virtual bool bindExternalDevice(uint flags, void* const pDevice[], void* pContext,
                                  bool validateOnly) override {
    return true;
  }

  virtual bool unbindExternalDevice(uint flags, void* const pDevice[], void* pContext,
                                    bool validateOnly) override {
    return true;
  }

  virtual bool globalFreeMemory(size_t* freeMemory) const override { return false; }

  virtual void* virtualAlloc(void* addr, size_t size, size_t alignment) override {
    return nullptr;
  }

  virtual bool virtualFree(void* addr) override { return true; }

  virtual cl_int virtualMap(void* va, size_t size, amd::Memory* phys) override {
    return CL_INVALID_OPERATION;
  }

  virtual cl_int virtualUnmap(void* va, size_t size) override { return CL_INVALID_OPERATION; }

  virtual bool SetMemAccess(void* va_addr, size_t va_size, VmmAccess access_flags,
                            VmmLocationType = VmmLocationType::kDevice,
                            int numaNode = -1) override {
    return true;
  }

  virtual bool GetMemAccess(void* va_addr, VmmAccess* access_flags_ptr) const override {
    return true;
  }

  virtual bool ValidateMemAccess(amd::Memory& mem, bool read_write) const override { return true; }

  virtual bool importExtSemaphore(void** extSemaphore, const amd::Os::FileDesc& handle,
                                  amd::ExternalSemaphoreHandleType sem_handle_type) override {
    return false;
  }

  virtual void DestroyExtSemaphore(void* extSemaphore) override {}

  virtual bool SetClockMode(const cl_set_device_clock_mode_input_amd setClockModeInput,
                            cl_set_device_clock_mode_output_amd* pSetClockModeOutput) override {
    return true;
  }

#if defined(__clang__)
#if __has_feature(address_sanitizer)
  virtual device::UriLocator* createUriLocator() const override { return nullptr; }
#endif
#endif

 private:
  //! Creates buffer backing with the requested system-memory alignment.
  Memory* createBufferMemory(amd::Memory& owner, size_t alignment) const;

  //! The launch thread pool owned by this device
  mutable ThreadPool threadPool_;

  //! Disable default copy constructor
  Device(const Device&);

  //! Disable default operator=
  Device& operator=(const Device&);
};

}  // namespace amd::cpu

#endif /*CPUDEVICE_HPP_*/
