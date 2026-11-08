// PROTOTYPE: software stand-in for the slice of HSA that rocclr's ROCm
// backend relies on.

#include <cstdint>
#include <future>

#include "device/prototype/hsaemu.hpp"

namespace amd::device::prototype {

void Signal::notify() {
  std::vector<Callback> waiters;
  {
    std::lock_guard<std::mutex> lock(lock_);
    if (value_.load(std::memory_order_relaxed) != 0) {
      value_.store(0, std::memory_order_release);
      waiters.swap(waiters_);
    }
  }
  for (auto& waiter : waiters) {
    waiter();
  }
}

void Signal::wait() {
  auto ready = std::make_shared<std::promise<void>>();
  auto future = ready->get_future();
  onDone([ready] { ready->set_value(); });
  future.wait();
}

void Signal::onDone(Callback on_done) {
  if (!isDone()) {
    std::lock_guard<std::mutex> lock(lock_);
    if (!isDone()) {
      waiters_.push_back(std::move(on_done));
      return;
    }
  }
  on_done();
}

Queue::Queue(Pool& pool) : pool_(pool) {}

void Queue::submit(Packet* packet) {
  packet->queue = this;
  {
    std::lock_guard<std::mutex> lock(lock_);
    packets_.push_back(packet);
  }
  pool_.submit(nextTaskPacket(false));
}

Packet* Queue::nextTaskPacket(bool isDone) {
  Packet* packet = nextPacket(isDone);
  while (packet && packet->num_tickets == 0) {
    pool_.donePacket(packet);
    packet = nextPacket(true);
  }
  return packet;
}

Packet* Queue::nextPacket(bool isDone) {
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
      auto& deps = packet->deps;
      size_t num_waiting = deps.size();
      size_t new_num_waiting = 0;
      for (size_t idx = 0; idx < num_waiting; idx++) {
        auto* dep = packet->deps[idx];
        if (dep->isDone()) {
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
    packet->deps[idx]->onDone([this] {
      {
        std::lock_guard<std::mutex> lock(lock_);
        if (--num_waiting_ != 0) {
          return;
        }
        waiting_ready_ = true;
      }
      pool_.submit(nextTaskPacket(false));
    });
  }
  return {};
}

Pool::Pool(uint32_t workers) {
  for (uint32_t i = 0; i < workers; ++i) {
    workers_.emplace_back(&Pool::loop, this);
  }
}

Pool::~Pool() {
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

void Pool::submit(Packet* packet) {
  if (packet == nullptr) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(lock_);
    packets_.push_back(packet);
  }
  cv_.notify_one();
}

void Pool::loop(Pool* pool) {
  for (;;) {
    Queue* queue = nullptr;
    Packet* packet = nullptr;
    Ticket ticket = std::nullopt;
    {
      std::unique_lock<std::mutex> lock(pool->lock_);
      pool->cv_.wait(lock, [pool] { return pool->shutdown_ || !pool->packets_.empty(); });
      if (pool->shutdown_) {
        return;
      }
      packet = pool->packets_.front();
      ticket = packet->getTicket();
      if (ticket.value_or(0) == 0) {
        pool->packets_.pop_front();
      } else {
        packet->owners += 1;
      }
      queue = packet->queue;
    }
    bool notified = false;
    while (packet) {
      while (ticket.has_value()) {
        if (!notified && ticket.value() != 0) {
          pool->cv_.notify_one();
          notified = true;
        }
        packet->task(ticket.value());
        ticket = packet->getTicket();
      }
      if (packet->owners.fetch_sub(1) == 1) {
        pool->donePacket(packet);
        packet = queue->nextTaskPacket(true);
        if (packet != nullptr) {
          ticket = packet->getTicket();
          if (ticket != 0) {
            packet->owners += 1;
            pool->submit(packet);
            notified = true;
          }
        }
      } else {
        packet = nullptr;
      }
    }
  }
}

Queue* Pool::newQueue() {
  std::lock_guard<std::mutex> lock(lock_);
  queues_.emplace_back(new Queue(*this));
  return queues_.back().get();
}

Packet* Pool::newPacket(Key* cmd, uint32_t num_tickets, const KeyList& wait_list) {
  auto* packet = new Packet;
  cmd->retain();
  packet->cmd = cmd;
  Signal* done = new Signal();
  packet->num_tickets = num_tickets;
  packet->done = done;
  packet->task = nullptr;
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

void Pool::donePacket(Packet* packet) {
  {
    std::lock_guard<std::mutex> lock(lock_);
    event_signals_.erase(packet->cmd);
  }
  packet->done->notify();
  packet->cmd->release();
  packet->done->release();
  delete packet;
}
}  // namespace amd::device::prototype
