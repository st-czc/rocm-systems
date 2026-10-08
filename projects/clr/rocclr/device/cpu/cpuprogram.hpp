/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <string>

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
  //! Create a binary for the host device. Not supported.
  bool createBinary(amd::option::Options* options) override;

 private:
  //! Disable default copy constructor
  Program(const Program&) = delete;

  //! Disable operator=
  Program& operator=(const Program&) = delete;
};

}  // namespace amd::cpu
