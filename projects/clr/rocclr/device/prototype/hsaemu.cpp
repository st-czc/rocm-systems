// PROTOTYPE: software stand-in for the slice of HSA that rocclr's ROCm
// backend relies on.

#include <cstdint>
#include <future>

#include "device/prototype/hsaemu.hpp"

namespace amd::device::prototype {

void Signal::notify() {
  {
    std::lock_guard<std::mutex> lock(lock_);
    value_.store(0, std::memory_order_release);
  }
  for (auto& callback : callbacks_) {
    callback();
  }
  callbacks_.clear();
}

void Signal::wait() {
  if (isDone()) return;
  auto ready = std::make_shared<std::promise<void>>();
  auto future = ready->get_future();
  onDone([ready = std::move(ready)] { ready->set_value(); });
  future.wait();
}

void Signal::onDone(Callback on_done) {
  if (!isDone()) {
    std::lock_guard<std::mutex> lock(lock_);
    if (!isDone()) {
      callbacks_.push_back(std::move(on_done));
      return;
    }
  }
  on_done();
}

void Queue::submit(Packet* packet) {
  packet->queue = this;
  {
    std::lock_guard<std::mutex> lock(lock_);
    packets_.push_back(packet);
  }
  pool_.submit(nextTaskPacket(false));
}

Packet* Queue::nextTaskPacket(bool busy_done) {
  Packet* packet = nextPacket(busy_done);
  while (packet && packet->num_tickets == 0) {
    pool_.donePacket(packet);
    packet = nextPacket(true);
  }
  return packet;
}

Packet* Queue::nextPacket(bool busy_done) {
  Packet* packet = nullptr;
  {
    std::lock_guard<std::mutex> lock(lock_);
    if (busy_done) {
      busy_ = false;
    }
    if (busy_ || packets_.empty() || (num_waiting_ != 0)) {
      return nullptr;
    }
    packet = packets_.front();
    if (waiting_ready_) {
      for (Signal* signal_wait : packet->signal_wait_list) {
        signal_wait->release();
      }
      packet->signal_wait_list.clear();
      waiting_ready_ = false;
    } else {
      auto& signal_wait_list = packet->signal_wait_list;
      size_t num_waiting = signal_wait_list.size();
      size_t new_num_waiting = 0;
      for (size_t idx = 0; idx < num_waiting; idx++) {
        auto* signal_wait = packet->signal_wait_list[idx];
        if (signal_wait->isDone()) {
          signal_wait->release();
        } else {
          signal_wait_list[new_num_waiting++] = signal_wait;
        }
      }
      signal_wait_list.resize(new_num_waiting);
    }
    if (packet->signal_wait_list.empty()) {
      packets_.pop_front();
      busy_ = true;
    } else {
      num_waiting_ = packet->signal_wait_list.size();
    }
  }

  auto num_waiting = packet->signal_wait_list.size();
  if (num_waiting == 0) {
    return packet;
  }

  for (size_t idx = 0; idx < num_waiting; idx++) {
    packet->signal_wait_list[idx]->onDone([this] {
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
  return nullptr;
}

namespace {
thread_local bool tls_in_async_handler = false;
}  // namespace

AsyncHandler::AsyncHandler() : thread_(&AsyncHandler::loop, this) {}

AsyncHandler::~AsyncHandler() {
  {
    std::lock_guard<std::mutex> lock(lock_);
    exit_ = true;
  }
  cv_.notify_one();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void AsyncHandler::post(Handler handler) {
  {
    std::lock_guard<std::mutex> lock(lock_);
    handlers_.push_back(std::move(handler));
  }
  cv_.notify_one();
}

bool AsyncHandler::inHandler() { return tls_in_async_handler; }

void AsyncHandler::loop() {
  tls_in_async_handler = true;
  for (;;) {
    Handler handler;
    {
      std::unique_lock<std::mutex> lock(lock_);
      cv_.wait(lock, [this] { return exit_ || !handlers_.empty(); });
      // Drain before exiting: a pending handler owns references that its
      // posting packet has already dropped.
      if (handlers_.empty()) {
        return;
      }
      handler = std::move(handlers_.front());
      handlers_.pop_front();
    }
    handler();
  }
}

Pool::Pool(uint32_t workers) {
  for (uint32_t i = 0; i < workers; ++i) {
    workers_.emplace_back(&Pool::loop, this);
  }
}

Pool::~Pool() {
  {
    std::lock_guard<std::mutex> lock(packet_lock_);
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
  if (packet == nullptr) return;
  {
    std::lock_guard<std::mutex> lock(packet_lock_);
    ready_packets_.push_back(packet);
  }
  cv_.notify_one();
}

void Pool::loop(Pool* pool) {
  for (;;) {
    Queue* queue = nullptr;
    Packet* packet = nullptr;
    Ticket ticket = std::nullopt;
    {
      std::unique_lock<std::mutex> lock(pool->packet_lock_);
      pool->cv_.wait(lock, [pool] { return pool->shutdown_ || !pool->ready_packets_.empty(); });
      if (pool->shutdown_) {
        return;
      }
      packet = pool->ready_packets_.front();
      ticket = packet->getTicket();
      if (ticket.value_or(0) == 0) {
        pool->ready_packets_.pop_front();
      } else {
        packet->num_owners += 1;
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
      if (packet->num_owners.fetch_sub(1) == 1) {
        pool->donePacket(packet);
        packet = queue->nextTaskPacket(true);
        if (packet != nullptr) {
          ticket = packet->getTicket();
          if (ticket.value_or(0) != 0) {
            packet->num_owners += 1;
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

Queue::Queue(Pool& pool) : pool_(pool) {}

Queue* Pool::newQueue() {
  std::lock_guard<std::mutex> lock(queue_lock_);
  queues_.emplace_back(new Queue(*this));
  return queues_.back().get();
}

Packet::Packet(Event* event, uint32_t num_tickets)
    : event(event), num_owners(1), num_tickets(num_tickets) {
  event->retain();
  signal = new Signal;
}

Packet::~Packet() {
  signal->release();
  event->release();
}

Packet* Pool::newPacket(Event* event, uint32_t num_tickets, const EventWaitList& event_wait_list) {
  auto* packet = new Packet(event, num_tickets);
  packet->signal_wait_list.reserve(event_wait_list.size());
  {
    std::lock_guard<std::mutex> lock(event_lock_);
    for (Event* event_wait : event_wait_list) {
      auto it = event_signals_.find(event_wait);
      if (it != event_signals_.end()) {
        it->second->retain();
        packet->signal_wait_list.push_back(it->second);
      }
    }
    event_signals_[event] = packet->signal;
  }
  return packet;
}

void Pool::donePacket(Packet* packet) {
  {
    std::lock_guard<std::mutex> lock(event_lock_);
    event_signals_.erase(packet->event);
  }
  packet->signal->notify();
  delete packet;
}
}  // namespace amd::device::prototype
