/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#include "device/cpu/cputhreadpool.hpp"

#include "os/os.hpp"
#include "utils/debug.hpp"

#include <system_error>

namespace amd::cpu {

ThreadPool::~ThreadPool() {
  {
    std::lock_guard launch(launchLock_);
    std::lock_guard lock(lock_);
    stop_ = true;
  }
  workCv_.notify_all();
  for (auto& worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

bool ThreadPool::start(size_t numThreads) {
  if (!workers_.empty()) {
    return true;
  }
  if (numThreads == 0) {
    numThreads = 1;
  }

  try {
    workers_.reserve(numThreads);
    for (size_t tid = 0; tid < numThreads; ++tid) {
      workers_.emplace_back(&ThreadPool::workerLoop, this, tid);
    }
  } catch (const std::system_error& error) {
    LogPrintfError("Failed to start the CPU thread pool: %s", error.what());
    {
      std::scoped_lock lock(lock_);
      stop_ = true;
    }
    workCv_.notify_all();
    for (auto& worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    workers_.clear();
    return false;
  }

  LogPrintfInfo("CPU thread pool started: threads=%zu", workers_.size());
  return true;
}

void ThreadPool::run(const std::function<void(size_t, size_t)>& job) {
  std::lock_guard launch(launchLock_);
  std::unique_lock lock(lock_);
  job_ = &job;
  remaining_ = workers_.size();
  ++generation_;
  workCv_.notify_all();
  doneCv_.wait(lock, [this] { return remaining_ == 0; });
  job_ = nullptr;
}

void ThreadPool::workerLoop(size_t tid) {
  uint64_t seen = 0;
  std::unique_lock lock(lock_);
  for (;;) {
    workCv_.wait(lock, [&] { return stop_ || generation_ != seen; });
    if (stop_) {
      return;
    }
    seen = generation_;
    const auto* job = job_;
    const size_t numThreads = workers_.size();
    lock.unlock();
    (*job)(tid, numThreads);
    lock.lock();
    if (--remaining_ == 0) {
      doneCv_.notify_all();
    }
  }
}

}  // namespace amd::cpu
