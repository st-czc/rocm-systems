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
#endif

namespace amd::device::prototype {

class Pool;

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
  std::vector<Callback> waiters_;
};

using Key = amd::ReferenceCountedObject;
using KeyList = std::vector<Key*>;

class Queue;

using Ticket = std::optional<uint32_t>;

struct Packet {
  amd::ReferenceCountedObject* cmd = nullptr;
  std::vector<Signal*> deps;
  std::function<void(int)> task;
  Queue* queue = nullptr;
  Signal* done = nullptr;

  std::atomic<uint32_t> owners = 1;
  std::atomic<uint32_t> num_tickets;

  Packet() = default;
  Packet(const Packet&) = delete;
  Packet& operator=(const Packet&) = delete;

  Ticket getTicket() {
    if (num_tickets.load(std::memory_order_relaxed) == 0) {
      return std::nullopt;
    }
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

//! In-order queue, the counterpart of an hsa_queue_t. One packet runs at a
//! time in submission order; a head whose deps are not done parks the queue.
class Queue {
 public:
  explicit Queue(Pool& pool);

  void submit(Packet* packet);
  Packet* nextPacket(bool isDone);
  Packet* nextTaskPacket(bool isDone);

 private:
  Pool& pool_;
  std::mutex lock_;
  std::deque<Packet*> packets_;
  bool busy_ = false;           //!< a packet is running on a worker
  size_t num_waiting_ = 0;      //!< subscriptions outstanding for the head; nonzero = parked
  bool waiting_ready_ = false;  //!< the head's deps all fired; dispatch without a rescan
};

//! Worker pool plus the queues and signals it owns.
class Pool {
 public:
  explicit Pool(uint32_t workers);
  ~Pool();
  Pool(const Pool&) = delete;
  Pool& operator=(const Pool&) = delete;

  void submit(Packet* packet);

  Queue* newQueue();

  Packet* newPacket(Key* cmd, uint32_t num_tickets, const KeyList& wait_list);
  void donePacket(Packet* packet);

 private:
  static void loop(Pool* pool);

  std::mutex lock_;
  std::condition_variable cv_;
  std::deque<Packet*> packets_;
  //! Completion signal per in-flight command. The entry holds a reference on
  //! the command, so the key address cannot be recycled while the entry lives.
  std::unordered_map<const amd::ReferenceCountedObject*, Signal*> event_signals_;
  std::vector<std::unique_ptr<Queue>> queues_;
  std::vector<std::thread> workers_;
  bool shutdown_ = false;
};

}  // namespace amd::device::prototype

#endif  // PROTOTYPE_HSAEMU_HPP_
