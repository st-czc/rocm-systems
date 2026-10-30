/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef GRIDSYNC_HPP_
#define GRIDSYNC_HPP_

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>

//! CPU Device Implementation
namespace amd::cpu {

//! Grid-wide barrier shared by the threads that run one cooperative launch.
//! The state spans threads, so it belongs to the launch and not to a thread.
class GridSync {
 public:
  explicit GridSync(size_t participants) : remaining_(participants) {}

  //! Blocks until every participant has called gridSync() for the current
  //! generation.
  void gridSync();

 private:
  std::mutex lock_;
  std::condition_variable cv_;
  size_t arrived_ = 0;
  const size_t remaining_;
  uint64_t generation_ = 0;
};

//! Makes grid available to the barrier calls made by this thread.
void setCurrentGridSync(GridSync* grid);

//! The barrier for the current launch on this thread; null outside a launch.
GridSync* currentGridSync();

//! The barrier entry point the device image calls. Every work-group calls it
//! once per barrier, from the thread that runs the work-group.
void gridSyncEntry();

//! Address of the pointer the device image holds for gridSyncEntry().
//! Returns null when the image never references grid sync.
void** gridSyncHandle(void* imageHandle);

}  // namespace amd::cpu

#endif /*GRIDSYNC_HPP_*/
