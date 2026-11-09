// PROTOTYPE: software stand-in for the slice of HSA that rocclr's ROCm
// backend relies on. Self-contained: no rocclr headers, so it can be tested
// on its own.
#ifndef PROTOTYPE_HSAEMU_HPP_
#define PROTOTYPE_HSAEMU_HPP_

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

//! Standalone builds (hsaemu_test.cpp) get a local stand-in with the same
//! contract: starts at 1, deletes itself when the last reference goes away.
#ifdef PROTOTYPE_STANDALONE
namespace amd {
class ReferenceCountedObject {
 public:
  ReferenceCountedObject() : referenceCount_(1) {}
  ReferenceCountedObject(const ReferenceCountedObject&) = delete;
  ReferenceCountedObject& operator=(const ReferenceCountedObject&) = delete;

  uint32_t referenceCount() const { return referenceCount_.load(std::memory_order_acquire); }

  void retain() { referenceCount_.fetch_add(1, std::memory_order_acq_rel); }

  uint32_t release() {
    uint32_t count = referenceCount_.fetch_sub(1, std::memory_order_acq_rel) - 1;
    if (count == 0) {
      referenceCount_.store(1, std::memory_order_relaxed);
      delete this;
    }
    return count;
  }

 protected:
  virtual ~ReferenceCountedObject() = default;

 private:
  std::atomic<uint32_t> referenceCount_;
};
}  // namespace amd
#else
#include "top.hpp"
#include "platform/command.hpp"
#endif

namespace amd::device::prototype {

class Pool;
class Queue;

//! hsa_signal_t: armed (1) drops to done (0) once. Subscribers registered
//! after the drop run immediately, so a park-then-subscribe never loses a
//! wakeup.
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

#ifdef PROTOTYPE_STANDALONE
using Event = amd::ReferenceCountedObject;
using EventWaitList = std::vector<Event*>;
#else
using EventWaitList = Event::EventWaitList;
#endif

using SignalWaitList = std::vector<Signal*>;

using Ticket = std::optional<uint32_t>;

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

//! In-order queue, the counterpart of an hsa_queue_t.
class Queue {
 public:
  explicit Queue(Pool& pool);

  void submit(Packet* packet);
  Packet* nextTaskPacket(bool busy_done);

  //! Nothing running, queued or parked. A parked head's onDone callbacks
  //! capture this, so destroying a queue that is not drained leaves them
  //! pointing at freed memory.
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

//! ROCr's async-events thread (Runtime::AsyncEventsLoop): one thread, handlers
//! run serially on it. Completion bookkeeping goes here so it never occupies a
//! worker -- the pool is for compute only.
class AsyncHandler {
 public:
  using Handler = std::function<void()>;

  AsyncHandler();
  ~AsyncHandler();
  AsyncHandler(const AsyncHandler&) = delete;
  AsyncHandler& operator=(const AsyncHandler&) = delete;

  //! Runs |handler| on the async thread. Never blocks.
  void post(Handler handler);

  //! ROCr's InAsyncSignalHandler(): true on the async thread. A handler that
  //! would destroy what it is running under defers instead.
  static bool inHandler();

 private:
  void loop();

  std::mutex lock_;
  std::condition_variable cv_;
  std::deque<Handler> handlers_;
  bool exit_ = false;
  //! Last, so everything above is initialized before the thread starts.
  std::thread thread_;
};

//! Worker pool plus the queues and signals it owns.
class Pool {
 public:
  explicit Pool(uint32_t workers);
  ~Pool();
  Pool(const Pool&) = delete;
  Pool& operator=(const Pool&) = delete;

  void submit(Packet* packet);

  //! |owner| is the stream that owns the queue, opaque here. Pass it back to
  //! deleteQueue.
  Queue* newQueue(const void* owner);
  //! Destroys |owner|'s queue, or defers it until the queue drains.
  void deleteQueue(const void* owner);

  Packet* newPacket(Event* event, uint32_t num_tickets, const EventWaitList& event_wait_list);
  void donePacket(Packet* packet);

  //! Runs |handler| on the async thread, off the worker pool.
  void postAsync(AsyncHandler::Handler handler) { async_.post(std::move(handler)); }

 private:
  static void loop(Pool* pool);

  //! Destroys whatever in dying_queues_ has drained. Caller holds queue_lock_.
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

}  // namespace amd::device::prototype

#endif  // PROTOTYPE_HSAEMU_HPP_
