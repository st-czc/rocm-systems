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

//! Standalone builds (hsaemu_test.cpp) cannot pull in rocclr's headers, so they
//! get a local stand-in with the same contract: starts at 1, deletes itself
//! when the last reference goes away.
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

class Agent;  // forward declaration for Packet::Retire

//! hsa_signal_t: starts armed (1), drops to done (0) once, never back.
//! Subscribers registered after the drop run immediately, which is what makes
//! a park-then-subscribe sequence free of lost wakeups.
//! Lifetime follows amd::ReferenceCountedObject, like ProfilingSignal: whoever
//! holds a pointer retains it, and the last release deletes the signal.
class Signal : public amd::ReferenceCountedObject {
 public:
  using Callback = std::function<void()>;

  Signal() = default;
  Signal(const Signal&) = delete;
  Signal& operator=(const Signal&) = delete;

  void Notify();
  bool IsDone() const { return value_.load(std::memory_order_acquire) == 0; }

  //! Blocks the calling thread. Workers must never call this.
  void Wait();

  //! Runs |on_done| once, inline if the signal is already done.
  void OnDone(Callback on_done);

 private:
  std::atomic<int32_t> value_{1};
  std::mutex lock_;
  std::condition_variable cv_;
  std::vector<Callback> waiters_;
};

using Key = amd::ReferenceCountedObject;
using KeyList = std::vector<Key*>;

class Queue;

using Ticket = std::optional<uint32_t>;

struct Packet {
  amd::ReferenceCountedObject* cmd = nullptr;
  std::vector<Signal*> deps;
  std::function<void(int)> work;
  Queue* queue = nullptr;
  Signal* done = nullptr;

  std::atomic<uint32_t> owners = 1;
  std::atomic<uint32_t> num_tickets;

  Packet() = default;
  Packet(const Packet&) = delete;
  Packet& operator=(const Packet&) = delete;

  Ticket get_ticket() {
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

//! In-order queue, the counterpart of an hsa_queue_t. Exactly one packet runs
//! at a time, in submission order. A head whose deps are not done parks the
//! queue and releases its worker.
class Queue {
 public:
  explicit Queue(Agent& agent);

  void Submit(Packet* packet);
  Packet* NextPacket(bool isDone);
  Packet* NextWorkPacket(bool isDone);

 private:
  Agent& agent_;
  std::mutex lock_;
  std::deque<Packet*> packets_;
  bool busy_ = false;       //!< a packet is running on a worker
  size_t num_waiting_ = 0;  //!< subscriptions outstanding for the head; nonzero = parked
  //! The head's outstanding deps all fired, so it may dispatch without a rescan.
  bool waiting_ready_ = false;
};

//! Worker pool plus the queues and signals it owns. One agent per device.
class Agent {
 public:
  explicit Agent(uint32_t workers);
  ~Agent();
  Agent(const Agent&) = delete;
  Agent& operator=(const Agent&) = delete;

  //! Runs |task| on a worker. Never blocks.
  void Submit(Packet* task);

  Queue* NewQueue();

  Packet* NewPacket(Key* cmd, uint32_t num_tickets, const KeyList& wait_list);
  void DonePacket(Packet* packet);

 private:
  static void Loop(Agent* agent);

  std::mutex lock_;
  std::condition_variable cv_;
  std::deque<Packet*> tasks_;
  //! Completion signal per command address. An entry exists exactly while its
  //! command has not completed, and it holds a reference on the command, so the
  //! key cannot be recycled while the entry is reachable.
  std::unordered_map<const amd::ReferenceCountedObject*, Signal*> event_signals_;
  std::vector<std::unique_ptr<Queue>> queues_;
  std::vector<std::thread> workers_;
  bool shutdown_ = false;
};

}  // namespace amd::device::prototype

#endif  // PROTOTYPE_HSAEMU_HPP_
