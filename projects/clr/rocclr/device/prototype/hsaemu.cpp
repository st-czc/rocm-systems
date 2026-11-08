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
  packet->queue = this;
  {
    std::lock_guard<std::mutex> lock(lock_);
    packets_.push_back(packet);
  }
  agent_.Submit(NextWorkPacket(false));
}

Packet* Queue::NextWorkPacket(bool isDone) {
  Packet* packet = NextPacket(isDone);
  while (packet && packet->num_tickets == 0) {
    agent_.DonePacket(packet);
    packet = NextPacket(true);
  }
  return packet;
}

Packet* Queue::NextPacket(bool isDone) {
  Packet* packet = nullptr;
  {
    std::lock_guard<std::mutex> lock(lock_);
    if (isDone) {
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
    } else {
      num_waiting_ = packet->deps.size();
    }
  }

  auto num_waiting = packet->deps.size();
  if (num_waiting == 0) {
    return packet;
  }

  for (size_t idx = 0; idx < num_waiting; idx++) {
    packet->deps[idx]->OnDone([this] {
      {
        std::lock_guard<std::mutex> lock(lock_);
        if (--num_waiting_ != 0) {
          return;
        }
        waiting_ready_ = true;
      }
      agent_.Submit(NextWorkPacket(false));
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
  if (task == nullptr) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(lock_);
    tasks_.push_back(task);
  }
  cv_.notify_one();
}

void Agent::Loop(Agent* agent) {
  for (;;) {
    Queue* queue = nullptr;
    Packet* task = nullptr;
    Ticket ticket = std::nullopt;
    {
      std::unique_lock<std::mutex> lock(agent->lock_);
      agent->cv_.wait(lock, [agent] { return agent->shutdown_ || !agent->tasks_.empty(); });
      if (agent->shutdown_) {
        return;
      }
      task = agent->tasks_.front();
      ticket = task->get_ticket();
      if (ticket.value_or(0) == 0) {
        // last or null ticket, take ownership from tasks_
        agent->tasks_.pop_front();
      } else {
        task->owners += 1;
      }
      queue = task->queue;
    }
    bool notified = false;
    // Get the next task from queue and execute it if possible
    while (task) {
      while (ticket.has_value()) {
        if (!notified && ticket.value() != 0) {
          agent->cv_.notify_one();
          notified = true;
        }
        task->work(ticket.value());
        ticket = task->get_ticket();
      }
      if (task->owners.fetch_sub(1) == 1) {
        agent->DonePacket(task);
        task = queue->NextWorkPacket(true);
        if (task != nullptr) {
          ticket = task->get_ticket();
          if (ticket != 0) {
            task->owners += 1;
            agent->Submit(task);
            notified = true;
          }
        }
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
