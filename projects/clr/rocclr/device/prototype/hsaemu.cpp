// PROTOTYPE: software stand-in for the slice of HSA that rocclr's ROCm
// backend relies on.
#include "device/prototype/hsaemu.hpp"

namespace amd::device::prototype {

// ================================================================================================
void Signal::Notify() {
  std::vector<std::function<void()>> waiters;
  {
    std::lock_guard<std::mutex> lock(lock_);
    if (value_.load(std::memory_order_relaxed) != 0) {
      value_.store(0, std::memory_order_release);
      waiters.swap(waiters_);
    }
  }
  cv_.notify_all();
  for (auto& waiter : waiters) {
    waiter();
  }
}

void Signal::Wait() {
  std::unique_lock<std::mutex> lock(lock_);
  cv_.wait(lock, [this] { return value_.load(std::memory_order_acquire) == 0; });
}

void Signal::OnDone(std::function<void()> on_done) {
  {
    std::lock_guard<std::mutex> lock(lock_);
    if (value_.load(std::memory_order_acquire) != 0) {
      waiters_.push_back(std::move(on_done));
      return;
    }
  }
  on_done();
}

// ================================================================================================
Queue::Queue(Agent& agent) : agent_(agent) {}

void Queue::Submit(std::shared_ptr<Packet> packet) {
  {
    std::lock_guard<std::mutex> lock(lock_);
    packets_.push_back(std::move(packet));
  }
  Pump();
}

void Queue::Pump() {
  std::shared_ptr<Packet> packet;
  std::vector<Signal*> unmet;

  {
    std::lock_guard<std::mutex> lock(lock_);
    if (busy_ || packets_.empty() || (num_waiting_ != 0)) {
      return;
    }
    if (waiting_ready_) {
      waiting_ready_ = false;
    } else {
      Packet* head = packets_.front().get();
      for (Signal* dep : head->deps) {
        if (!dep->IsDone()) {
          unmet.push_back(dep);
        }
      }
    }
    if (unmet.empty()) {
      packet = packets_.front();
      packets_.pop_front();
      busy_ = true;
    } else {
      num_waiting_ = unmet.size();
    }
  }

  if (packet) {
    agent_.Submit([this, packet] {
      Execute(packet.get());
      {
        std::lock_guard<std::mutex> lock(lock_);
        busy_ = false;
      }
      Pump();
    });
    return;
  }

  // Only unmet deps are subscribed, so an already-done dep cannot re-enter
  // here. The counter is armed before subscribing: a dep that completes inside
  // the loop fires inline and decrements it, and only the last one re-pumps.
  for (Signal* dep : unmet) {
    dep->OnDone([this] {
      {
        std::lock_guard<std::mutex> lock(lock_);
        if (--num_waiting_ != 0) {
          return;
        }
        waiting_ready_ = true;
      }
      Pump();
    });
  }
}

void Queue::Execute(Packet* packet) {
  if (packet->work) {
    packet->work();
  }
  if (packet->done != nullptr) {
    packet->done->Notify();
  }
}

// ================================================================================================
Agent::Agent(uint32_t workers) {
  for (uint32_t i = 0; i < workers; ++i) {
    workers_.emplace_back(&Agent::Loop, this);
  }
}

Agent::~Agent() {
  {
    std::lock_guard<std::mutex> lock(lock_);
    shutdown_ = true;
  }
  cv_.notify_all();
  for (auto& worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

void Agent::Submit(std::function<void()> task) {
  {
    std::lock_guard<std::mutex> lock(lock_);
    tasks_.push_back(std::move(task));
    ++pending_;
  }
  cv_.notify_one();
}

void Agent::WaitIdle() {
  std::unique_lock<std::mutex> lock(lock_);
  idle_cv_.wait(lock, [this] { return pending_ == 0; });
}

void Agent::Loop(Agent* agent) {
  for (;;) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(agent->lock_);
      agent->cv_.wait(lock, [agent] { return agent->shutdown_ || !agent->tasks_.empty(); });
      if (agent->tasks_.empty()) {
        return;
      }
      task = std::move(agent->tasks_.front());
      agent->tasks_.pop_front();
    }
    task();
    {
      std::lock_guard<std::mutex> lock(agent->lock_);
      if (--agent->pending_ == 0) {
        agent->idle_cv_.notify_all();
      }
    }
  }
}

Queue* Agent::NewQueue() {
  std::lock_guard<std::mutex> lock(lock_);
  queues_.emplace_back(new Queue(*this));
  return queues_.back().get();
}

Signal* Agent::NewSignal() {
  std::lock_guard<std::mutex> lock(lock_);
  signals_.emplace_back(new Signal());
  return signals_.back().get();
}

Signal* Agent::SignalFor(const void* event) {
  std::lock_guard<std::mutex> lock(lock_);
  auto it = event_signals_.find(event);
  if (it != event_signals_.end()) {
    return it->second;
  }
  signals_.emplace_back(new Signal());
  event_signals_.emplace(event, signals_.back().get());
  return signals_.back().get();
}

}  // namespace amd::device::prototype
