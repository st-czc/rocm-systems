/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#include "device/cpu/cpuprogram.hpp"

#include "device/cpu/cpudevice.hpp"
#include "device/cpu/cpukernel.hpp"
#include "os/os.hpp"
#include "utils/debug.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

namespace amd::cpu {

//! Suffix of the per-kernel subhandler exported by the Native CPU pipeline.
static constexpr std::string_view kSubhandlerSuffix = ".SYCLNCPU";

//! ELF device image for this host. The fat binary marks it with the AMDGPU
//! machine and the HSA OS ABI so the loader rejects it. The runtime removes the
//! mark before it hands the image to the loader.
static constexpr size_t kOsAbiOffset = 7;
static constexpr size_t kMachineOffset = 18;
static constexpr unsigned char kElfOsAbiAmdgpuHsa = 64;
static constexpr uint16_t kElfMachineAmdgpu = 224;

Program::Program(NullDevice& device, amd::Program& owner) : device::Program(device, owner) {
  LogPrintfInfo("CPU Program created: program=%p owner=%p device=%p", this, &owner, &device);
}

Program::~Program() {
  LogPrintfInfo("CPU Program destroyed: program=%p loaded=%d kernels=%zu", this,
                isCodeObjectLoaded(), kernels().size());
  releaseClBinary();
  if (imageHandle_ != nullptr) {
    amd::Os::unloadLibrary(imageHandle_);
  }
}

bool Program::initClBinary(const char* binaryIn, size_t size, amd::Os::FileDesc fdesc,
                           size_t foffset, std::string uri) {
  if (!device::Program::initClBinary(binaryIn, size, fdesc, foffset, uri)) {
    return false;
  }

  const binary_t attached = clBinary()->data();
  LogPrintfInfo("CPU Program::initClBinary: program=%p in=%p size=%zu attached=%p size=%zu", this,
                static_cast<const void*>(binaryIn), size, attached.first, attached.second);
  if (attached.first == nullptr || attached.second == 0) {
    return true;
  }

  // Copy the image before taking ownership. On a second pass the attached
  // binary is image_ itself.
  const char* bytes = static_cast<const char*>(attached.first);
  std::vector<char> marked(bytes, bytes + attached.second);
  if (!markImage(marked)) {
    return false;
  }

  image_ = std::move(marked);

  // ClBinary borrows the buffer. Pass allocated=false so its release() does not
  // free the program's buffer.
  return clBinary()->setBinary(image_.data(), image_.size(), false, fdesc, foffset, uri);
}

bool Program::createKernels(void* binary, size_t binSize, bool useUniformWorkGroupSize,
                            bool internalKernel) {
  // Find the size of global variables from the binary
  if (!FindGlobalVarSize(binary, binSize)) {
    buildLog_ += "Error: Cannot Find Global Var Sizes\n";
    return false;
  }

  LogPrintfInfo("CPU Program::createKernels: program=%p binary=%p size=%zu metadata=%zu", this,
                binary, binSize, kernelMetadataMap_.size());

  for (const auto& kernelMeta : kernelMetadataMap_) {
    const std::string& name = kernelMeta.first;
    LogPrintfInfo("CPU Program::createKernels: program=%p kernel=%s", this, name.c_str());

    Kernel* kernel = new Kernel(name, this);
    if (!kernel->init()) {
      buildLog_ += "Error: Cannot initialize the kernel " + name + ".\n";
      delete kernel;
      return false;
    }
    if (codeObjectVer() < 5) {
      kernel->setUniformWorkGroupSize(useUniformWorkGroupSize);
    }
    kernel->setInternalKernelFlag(internalKernel);
    addKernel(kernel);
  }

  LogPrintfInfo("CPU Program::createKernels result: program=%p kernels=%zu", this,
                kernels().size());
  return true;
}

bool Program::markImage(std::vector<char>& image) {
  if (image.size() < kMachineOffset + sizeof(uint16_t)) {
    return false;
  }

  auto& osabi = reinterpret_cast<unsigned char&>(image[kOsAbiOffset]);
  uint16_t machine = 0;
  std::memcpy(&machine, image.data() + kMachineOffset, sizeof(machine));

  // The first pass records the original header values for repairImage. Later
  // passes see this function's output.
  if (!savedElfHeader_.valid) {
    savedElfHeader_ = {true, osabi, machine};
  }
  osabi = kElfOsAbiAmdgpuHsa;
  machine = kElfMachineAmdgpu;
  std::memcpy(image.data() + kMachineOffset, &machine, sizeof(machine));
  LogPrintfInfo("CPU Program::markImage: program=%p ei_osabi %u -> %u, e_machine %u -> %u", this,
                savedElfHeader_.osabi, kElfOsAbiAmdgpuHsa, savedElfHeader_.machine,
                kElfMachineAmdgpu);
  return true;
}

void Program::repairImage(std::vector<char>& image) {
  if (!savedElfHeader_.valid) return;
  auto& osabi = reinterpret_cast<unsigned char&>(image[kOsAbiOffset]);
  osabi = savedElfHeader_.osabi;
  std::memcpy(image.data() + kMachineOffset, &savedElfHeader_.machine,
              sizeof(savedElfHeader_.machine));
  LogPrintfInfo("CPU Program::repairImage: program=%p ei_osabi -> %u, e_machine -> %u", this,
                savedElfHeader_.osabi, savedElfHeader_.machine);
}

void* Program::loadImage(void* binary, size_t binSize) {
  // The device image is a host shared library. The loader takes a path, so
  // repair the header, write the bytes to a file, load it, and delete the file.
  std::vector<char> image(static_cast<const char*>(binary),
                          static_cast<const char*>(binary) + binSize);
  repairImage(image);

  const std::string path = amd::Os::getTempFileName();

  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file.is_open()) {
    buildLog_ += "Error: Cannot create a file for the device image.\n";
    return nullptr;
  }
  file.write(image.data(), image.size());
  file.close();
  if (!file) {
    buildLog_ += "Error: Cannot write the device image.\n";
    std::remove(path.c_str());
    return nullptr;
  }

  void* handle = amd::Os::loadLibrary(path.c_str());
  std::remove(path.c_str());
  if (handle == nullptr) {
    buildLog_ += "Error: Cannot load the device image.\n";
    LogPrintfError("CPU Program::loadImage failed: program=%p path=%s", this, path.c_str());
    return nullptr;
  }

  LogPrintfInfo("CPU Program::loadImage: program=%p path=%s size=%zu handle=%p", this, path.c_str(),
                binSize, handle);
  return handle;
}

bool Program::setKernels(void* binary, size_t binSize, amd::Os::FileDesc fdesc, size_t foffset,
                         std::string uri) {
  LogPrintfInfo("CPU Program::setKernels: program=%p binary=%p size=%zu fdesc=%p offset=%zu uri=%s",
                this, binary, binSize, reinterpret_cast<void*>(fdesc), foffset, uri.c_str());

  imageHandle_ = loadImage(binary, binSize);
  if (imageHandle_ == nullptr) {
    return false;
  }

  for (const auto& it : kernels()) {
    Kernel* kernel = static_cast<Kernel*>(it.second);
    const std::string symbol = kernel->name() + std::string(kSubhandlerSuffix);

    auto entry = reinterpret_cast<Subhandler>(amd::Os::getSymbol(imageHandle_, symbol.c_str()));
    if (entry == nullptr) {
      buildLog_ += "Error: Cannot find the entry point " + symbol + ".\n";
      return false;
    }
    kernel->setEntry(entry);
    LogPrintfInfo("CPU Program::setKernels: program=%p kernel=%s entry=%s at %p", this,
                  kernel->name().c_str(), symbol.c_str(), reinterpret_cast<void*>(entry));
  }

  LogPrintfInfo("CPU Program::setKernels result: program=%p handle=%p kernels=%zu", this,
                imageHandle_, kernels().size());
  return true;
}

bool Program::createBinary(amd::option::Options* options) {
  // The host backend does not build device code.
  LogPrintfError("CPU Program::createBinary rejected: program=%p options=%p", this, options);
  return false;
}

}  // namespace amd::cpu
