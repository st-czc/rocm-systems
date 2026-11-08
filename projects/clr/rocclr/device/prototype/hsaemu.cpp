// PROTOTYPE: software stand-in for the slice of HSA that rocclr's ROCm
// backend relies on.
#include "device/prototype/hsaemu.hpp"
#include <cstdint>

namespace amd::device::prototype {

// ================================================================================================
void Signal::Notify() {
  std::vector<Callback> waiters;
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

void Signal::OnDone(Callback on_done) {
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

void Queue::Submit(Packet* packet) {
  {
    std::lock_guard<std::mutex> lock(lock_);
    packets_.push_back(packet);
  }
  Pump();
}

Packet* Queue::Pump(bool isAgent) {
  bool dispatch = false;
  Packet* packet = nullptr;
  {
    std::lock_guard<std::mutex> lock(lock_);
    if (isAgent) {
      busy_ = false;
    }
    if (busy_ || packets_.empty() || (num_waiting_ != 0)) {
      return {};
    }
    packet = packets_.front();
    if (waiting_ready_) {
      for (Signal* dep : packet->deps) {
        dep->release();
      }
      packet->deps.clear();
      waiting_ready_ = false;
    } else {
      // reset the deps when packet become head
      auto& deps = packet->deps;
      size_t num_waiting = deps.size();
      size_t new_num_waiting = 0;
      for (size_t idx = 0; idx < num_waiting; idx++) {
        auto* dep = packet->deps[idx];
        if (dep->IsDone()) {
          dep->release();
        } else {
          deps[new_num_waiting++] = dep;
        }
      }
      deps.resize(new_num_waiting);
    }
    if (packet->deps.empty()) {
      packets_.pop_front();
      busy_ = true;
      dispatch = true;
    } else {
      num_waiting_ = packet->deps.size();
    }
  }

  if (dispatch) {
    // Fast path for marker
    if (packet->num_tickets == 0) {
      agent_.DonePacket(packet);
      {
        std::lock_guard<std::mutex> lock(lock_);
        busy_ = false;
      }
      return Pump(isAgent);
    }
    packet->queue = this;
    if (isAgent && packet->num_tickets == 1) {
      // Avoid submit to the agent_, less lock
      return packet;
    } else {
      agent_.Submit(packet);
      return {};
    }
  }

  auto num_waiting = packet->deps.size();
  for (size_t idx = 0; idx < num_waiting; idx++) {
    packet->deps[idx]->OnDone([this] {
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
  return {};
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

void Agent::Submit(Packet* task) {
  uint32_t num_tickets;
  {
    std::lock_guard<std::mutex> lock(lock_);
    tasks_.push_back(std::move(task));
    num_tickets = task->num_tickets;
  }
  if (num_tickets == 1) {
    cv_.notify_one();
  } else {
    cv_.notify_all();
  }
}

void Agent::Loop(Agent* agent) {
  for (;;) {
    Queue* queue = nullptr;
    Packet* task = nullptr;
    uint32_t ticket = 0;
    {
      std::unique_lock<std::mutex> lock(agent->lock_);
      agent->cv_.wait(lock, [agent] { return agent->shutdown_ || !agent->tasks_.empty(); });
      if (agent->shutdown_) {
        return;
      }
      task = agent->tasks_.front();
      ticket = task->num_tickets.fetch_sub(1);
      ticket -= 1;
      if (ticket == 0) {
        agent->tasks_.pop_front();
      }
      queue = task->queue;
    }
    // Get the next task from queue and execute it if possible
    while (task) {
      task->work(ticket);
      if (task->num_rest_tickets.fetch_sub(1) == 1) {
        agent->DonePacket(task);
        task = queue->Pump(true);
        ticket = 0;
      } else {
        task = nullptr;
      }
    }
  }
}

Queue* Agent::NewQueue() {
  std::lock_guard<std::mutex> lock(lock_);
  queues_.emplace_back(new Queue(*this));
  return queues_.back().get();
}

Packet* Agent::NewPacket(Key* cmd, uint32_t num_tickets, const KeyList& wait_list) {
  auto* packet = new Packet;
  cmd->retain();
  packet->cmd = cmd;
  Signal* done = new Signal();
  packet->num_tickets = num_tickets;
  packet->num_rest_tickets = num_tickets;
  packet->done = done;
  packet->work = nullptr;
  packet->deps.reserve(wait_list.size());
  {
    std::lock_guard<std::mutex> lock(lock_);
    for (Key* event : wait_list) {
      auto it = event_signals_.find(event);
      if (it != event_signals_.end()) {
        it->second->retain();
        packet->deps.push_back(it->second);
      }
    }
    event_signals_[cmd] = done;
  }
  return packet;
}

void Agent::DonePacket(Packet* packet) {
  {
    std::lock_guard<std::mutex> lock(lock_);
    event_signals_.erase(packet->cmd);
  }
  packet->done->Notify();
  packet->cmd->release();
  packet->done->release();
  delete packet;
}
}  // namespace amd::device::prototype
