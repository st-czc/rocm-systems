/*
 * Copyright (c) Advanced Micro Devices, Inc., or its affiliates.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef THREADPOOL_HPP_
#define THREADPOOL_HPP_

#include "top.hpp"
#include "platform/command.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

namespace amd::cpu {

class Pool;
class Queue;

//! One-shot: armed (1) drops to done (0) once. A subscriber added after the
//! drop runs immediately, so park-then-subscribe never loses a wakeup.
class Signal : public amd::ReferenceCountedObject {
 public:
  using Callback = std::function<void()>;

  Signal() = default;
  Signal(const Signal&) = delete;
  Signal& operator=(const Signal&) = delete;

  void notify();
  bool isDone() const { return value_.load(std::memory_order_acquire) == 0; }
  void wait();
  void onDone(Callback on_done);

 private:
  std::mutex lock_;
  std::atomic<int32_t> value_{1};
  std::vector<Callback> callbacks_;
};

using EventWaitList = Event::EventWaitList;

using SignalWaitList = std::vector<Signal*>;

using Ticket = std::optional<uint32_t>;

//! Single-owner unit of work split into num_tickets tickets. submit() hands it
//! to the runtime, which deletes it on completion; read fields before submit().
struct Packet {
  Queue* queue{};

  std::function<void(int)> task;

  Event* event{};
  Signal* signal{};
  SignalWaitList signal_wait_list;

  std::atomic<uint32_t> num_owners;
  std::atomic<uint32_t> num_tickets;

  Packet(Event* event, uint32_t num_tickets);
  ~Packet();

  Packet(const Packet&) = delete;
  Packet& operator=(const Packet&) = delete;

  //! Claims the next ticket in descending order; nullopt when none remain.
  Ticket getTicket() {
    uint32_t ticket = num_tickets.load(std::memory_order_relaxed);
    do {
      if (ticket == 0) {
        return std::nullopt;
      }
    } while (!num_tickets.compare_exchange_weak(ticket, ticket - 1, std::memory_order_acq_rel,
                                                std::memory_order_relaxed));
    return ticket - 1;
  }
};

//! In-order queue: exactly one packet runs at a time.
class Queue {
 public:
  explicit Queue(Pool& pool);

  void submit(Packet* packet);
  Packet* nextTaskPacket(bool busy_done);

  //! Nothing running, queued, or parked. An undrained queue must not be
  //! destroyed: a parked head's callbacks capture this.
  bool isDrained();

 private:
  Packet* nextPacket(bool busy_done);

  Pool& pool_;
  std::mutex lock_;
  std::deque<Packet*> packets_;
  bool busy_ = false;
  bool waiting_ready_ = false;
  size_t num_waiting_ = 0;
};

//! Serial completion thread; bookkeeping and callbacks run here so workers only
//! run packet->task.
class AsyncHandler {
 public:
  using Handler = std::function<void()>;

  AsyncHandler();
  ~AsyncHandler();
  AsyncHandler(const AsyncHandler&) = delete;
  AsyncHandler& operator=(const AsyncHandler&) = delete;

  void post(Handler handler);

 private:
  void loop();

  std::mutex lock_;
  std::condition_variable cv_;
  std::deque<Handler> handlers_;
  bool exit_ = false;
  //! Last, so the members above are initialized before the thread starts.
  std::thread thread_;
};

//! One per device; lives for the program.
class Pool {
 public:
  explicit Pool(uint32_t workers);
  ~Pool();
  Pool(const Pool&) = delete;
  Pool& operator=(const Pool&) = delete;

  void submit(Packet* packet);

  uint32_t numWorkers() const { return static_cast<uint32_t>(workers_.size()); }

  //! owner is an opaque stream key; pass it back to deleteQueue.
  Queue* newQueue(const void* owner);
  //! Destroys |owner|'s queue, or defers it until drained.
  void deleteQueue(const void* owner);

  //! An in-flight event adds a dependency edge; a completed one adds nothing.
  //! Must be followed by submit(): only completion removes the table entry.
  Packet* newPacket(Event* event, uint32_t num_tickets, const EventWaitList& event_wait_list);
  void donePacket(Packet* packet);

  void postAsync(AsyncHandler::Handler handler) { async_.post(std::move(handler)); }

 private:
  static void loop(Pool* pool);

  //! Destroys drained entries in dying_queues_; caller holds queue_lock_.
  void sweepDyingQueues();

  std::mutex queue_lock_;
  std::unordered_map<const void*, std::unique_ptr<Queue>> stream_queues_;
  //! Queues whose stream is gone but which had not drained yet.
  std::vector<std::unique_ptr<Queue>> dying_queues_;

  std::mutex event_lock_;
  std::unordered_map<const Event*, Signal*> event_signals_;

  std::mutex packet_lock_;
  std::condition_variable cv_;
  std::deque<Packet*> ready_packets_;

  AsyncHandler async_;

  bool shutdown_ = false;
  std::vector<std::thread> workers_;
};

}  // namespace amd::cpu

#endif  // THREADPOOL_HPP_
