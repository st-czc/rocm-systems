/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <string>
#include <vector>

#include "device/devkernel.hpp"
#include "device/devprogram.hpp"

//! CPU Device Implementation
namespace amd::cpu {

class NullDevice;

//! Host program object tracking binary attachment only.
class Program : public device::Program {
 public:
  //! Default constructor
  Program(NullDevice& device, amd::Program& owner);

  //! Default destructor
  virtual ~Program();

 protected:
  //! Collects the kernel entry points the device image exports.
  bool createKernels(void* binary, size_t binSize, bool useUniformWorkGroupSize,
                     bool internalKernel) override;

  //! Loads the device image into the process and resolves every kernel entry.
  bool setKernels(void* binary, size_t binSize, amd::Os::FileDesc fdesc = amd::Os::FDescInit(),
                  size_t foffset = 0, std::string uri = std::string()) override;

  //! Create a binary for the host device. Not supported.
  bool createBinary(amd::option::Options* options) override;

  //! Replaces the attached binary with a marked, writable copy of itself.
  bool initClBinary(const char* binaryIn, size_t size,
                    amd::Os::FileDesc fdesc = amd::Os::FDescInit(), size_t foffset = 0,
                    std::string uri = std::string()) override;

 private:
  //! Header mark that stops the loader from accepting the image. Inverse of
  //! repairImage.
  bool markImage(std::vector<char>& image);

  //! Checks that the host can run the image and removes the header mark.
  void repairImage(std::vector<char>& image);

  //! Makes the in-memory device image loadable and returns its dlopen handle.
  void* loadImage(void* binary, size_t binSize);

  //! The dlopen handle of the device image, owned by this program.
  void* imageHandle_ = nullptr;

  //! Marked copy of the device image, owned by the program while ClBinary points
  //! to it.
  std::vector<char> image_;

  //! Original header values that markImage overwrites. repairImage restores
  //! them.
  struct SavedELFHeader {
    bool valid;
    unsigned char osabi;
    uint16_t machine;
  } savedElfHeader_ = {false, 0, 0};

  //! Disable default copy constructor
  Program(const Program&) = delete;

  //! Disable operator=
  Program& operator=(const Program&) = delete;
};

}  // namespace amd::cpu
