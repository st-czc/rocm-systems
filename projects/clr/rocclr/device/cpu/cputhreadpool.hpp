/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CPUTHREADPOOL_HPP_
#define CPUTHREADPOOL_HPP_

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

//! CPU Device Implementation
namespace amd::cpu {

//! Fixed-size worker set for kernel launches. The pool runs one launch at a
//! time. run() passes the job to every worker. run() returns after all workers
//! finish.
class ThreadPool {
 public:
  ThreadPool() = default;
  ~ThreadPool();

  //! Starts numThreads workers. Returns false if a worker cannot be created.
  bool start(size_t numThreads);

  //! Runs job(tid, numThreads) once on every worker and waits for completion.
  void run(const std::function<void(size_t tid, size_t numThreads)>& job);

 private:
  void workerLoop(size_t tid);

  //! Disable default copy constructor
  ThreadPool(const ThreadPool&);

  //! Disable default operator=
  ThreadPool& operator=(const ThreadPool&);

  std::mutex launchLock_;
  std::mutex lock_;
  std::condition_variable workCv_;
  std::condition_variable doneCv_;
  std::vector<std::thread> workers_;
  const std::function<void(size_t, size_t)>* job_ = nullptr;
  uint64_t generation_ = 0;
  size_t remaining_ = 0;
  bool stop_ = false;
};

}  // namespace amd::cpu

#endif /*CPUTHREADPOOL_HPP_*/
