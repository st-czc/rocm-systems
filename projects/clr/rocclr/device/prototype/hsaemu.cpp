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
  // The work has run, so the command is complete: drop its entry before waking
  // anyone, so that a packet submitted afterwards sees "no entry" and reads it
  // as "already complete".
  packet->Retire();
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

Signal* Agent::NewSignal() { return new Signal(); }

// ================================================================================================
void Agent::PreparePacket(Packet& packet, amd::ReferenceCountedObject& cmd,
                          const std::vector<amd::ReferenceCountedObject*>& wait_list) {
  std::lock_guard<std::mutex> lock(lock_);

  // A wait-list member that is absent from the table has already completed:
  // its entry is removed only after its work ran, so there is no signal left
  // to wait on and no dependency edge is created.
  for (amd::ReferenceCountedObject* event : wait_list) {
    auto it = event_signals_.find(event);
    if (it != event_signals_.end()) {
      packet.AddDep(it->second);
    }
  }

  // The single reference `new` returns belongs to the packet: it is what the
  // entry is valid for, and what ~Packet releases. The caller reads it back
  // from packet.done rather than from a return value, so ownership stays plain.
  Signal* done = new Signal();
  packet.done = done;
  // Assignment, not emplace: an entry for this address cannot still exist
  // (it would be holding the command alive), and silently losing the insert
  // would strand |done|.
  event_signals_[&cmd] = done;

  // The table keeps an entry's key alive through the packet that owns the
  // pair; Packet::Retire drops this reference together with the entry.
  cmd.retain();
  packet.agent = this;
  packet.cmd = &cmd;
}

// ================================================================================================
void Agent::RetireEntry(amd::ReferenceCountedObject* cmd) {
  {
    std::lock_guard<std::mutex> lock(lock_);
    event_signals_.erase(cmd);
  }
  // Outside the lock: releasing the last reference destroys the command, and a
  // destructor must not run while the agent's table is locked.
  cmd->release();
}

// ================================================================================================
Packet::~Packet() {
  Retire();
  for (Signal* dep : deps) {
    dep->release();
  }
  if (done != nullptr) {
    done->release();
  }
}

// ================================================================================================
void Packet::Retire() {
  if (agent == nullptr) {
    return;
  }
  // Take the fields first: RetireEntry releases |cmd|, which may destroy it.
  Agent* owner = agent;
  amd::ReferenceCountedObject* command = cmd;
  agent = nullptr;
  cmd = nullptr;
  owner->RetireEntry(command);
}

}  // namespace amd::device::prototype
