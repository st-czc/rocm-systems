/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#include "device/cpu/gridsync.hpp"

#include "os/os.hpp"

#include <cstdio>

namespace amd::cpu {

namespace {

//! Name of the pointer the device image exports for the grid barrier.
constexpr char kGridSyncHandleSymbol[] = "runtime_grid_sync_handle";

thread_local GridSync* tlsGridSync = nullptr;

}  // namespace

void GridSync::gridSync() {
  std::unique_lock lock(lock_);
  const uint64_t generation = generation_;
  const size_t arrived = ++arrived_;
  std::fprintf(stderr, "[hipcpu] grid.sync gen=%llu arrived=%zu/%zu\n",
               static_cast<unsigned long long>(generation), arrived, remaining_);
  if (arrived == remaining_) {
    arrived_ = 0;
    ++generation_;
    lock.unlock();
    cv_.notify_all();
    return;
  }
  cv_.wait(lock, [&] { return generation_ != generation; });
}

void setCurrentGridSync(GridSync* grid) { tlsGridSync = grid; }

GridSync* currentGridSync() { return tlsGridSync; }

void gridSyncEntry() { tlsGridSync->gridSync(); }

void** gridSyncHandle(void* imageHandle) {
  return static_cast<void**>(amd::Os::getSymbol(imageHandle, kGridSyncHandleSymbol));
}

}  // namespace amd::cpu
