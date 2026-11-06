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
#include <thread>
#include <unordered_map>
#include <vector>

namespace amd::device::prototype {

//! hsa_signal_t: starts armed (1), drops to done (0) once, never back.
//! Subscribers registered after the drop run immediately, which is what makes
//! a park-then-subscribe sequence free of lost wakeups.
class Signal {
 public:
  Signal() = default;
  Signal(const Signal&) = delete;
  Signal& operator=(const Signal&) = delete;

  void Notify();
  bool IsDone() const { return value_.load(std::memory_order_acquire) == 0; }

  //! Blocks the calling thread. Workers must never call this.
  void Wait();

  //! Runs |on_done| once, inline if the signal is already done.
  void OnDone(std::function<void()> on_done);

 private:
  std::atomic<int32_t> value_{1};
  std::mutex lock_;
  std::condition_variable cv_;
  std::vector<std::function<void()>> waiters_;
};

//! One entry in a queue: the work, the signals it waits for, and the signal
//! it raises when it is done.
struct Packet {
  std::function<void()> work;
  std::vector<Signal*> deps;
  Signal* done = nullptr;
};

class Agent;

//! In-order queue, the counterpart of an hsa_queue_t. Exactly one packet runs
//! at a time, in submission order. A head whose deps are not done parks the
//! queue and releases its worker.
class Queue {
 public:
  explicit Queue(Agent& agent);

  void Submit(std::shared_ptr<Packet> packet);

 private:
  //! Starts the head if it may run. Idempotent and thread safe.
  void Pump();
  void Execute(Packet* packet);

  Agent& agent_;
  std::mutex lock_;
  std::deque<std::shared_ptr<Packet>> packets_;
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
  void Submit(std::function<void()> task);

  //! Returns once no work is queued or running.
  void WaitIdle();

  Queue* NewQueue();
  Signal* NewSignal();
  //! Signal standing in for |event|; allocated on first use.
  Signal* SignalFor(const void* event);

 private:
  static void Loop(Agent* agent);

  std::mutex lock_;
  std::condition_variable cv_;
  std::condition_variable idle_cv_;
  std::deque<std::function<void()>> tasks_;
  std::vector<std::unique_ptr<Signal>> signals_;
  std::unordered_map<const void*, Signal*> event_signals_;
  std::vector<std::unique_ptr<Queue>> queues_;
  std::vector<std::thread> workers_;
  uint64_t pending_ = 0;
  bool shutdown_ = false;
};

}  // namespace amd::device::prototype

#endif  // PROTOTYPE_HSAEMU_HPP_
