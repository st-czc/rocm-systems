// PROTOTYPE tests: the emulated HSA agent with divisible (multi-ticket) tasks.
//
// The public surface is: newQueue, newPacket(cmd, num_tickets, event_wait_list),
// submit, and per-packet done signals. A test waits on a packet's done signal
// (retained first, since donePacket deletes the packet). Command objects are
// declared before the Pool so the Pool's workers are joined before the commands
// are destroyed.
#include "device/prototype/hsaemu.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

using namespace amd::device::prototype;

namespace {

std::atomic<int> g_failures{0};

void check(bool ok, const char* what) {
  if (!ok) {
    printf("  FAIL %s\n", what);
    ++g_failures;
  }
}

void pass(const char* what) { printf("PASS %s\n", what); }

//! Submits |packet| and blocks until its done signal fires.
void submitAndWait(Queue* queue, Packet* packet) {
  Signal* done = packet->signal;
  done->retain();
  queue->submit(packet);
  done->wait();
  done->release();
}

//! One-shot gate.
class Latch {
 public:
  void open() {
    {
      std::lock_guard<std::mutex> lock(lock_);
      open_ = true;
    }
    cv_.notify_all();
  }

  void wait() {
    std::unique_lock<std::mutex> lock(lock_);
    cv_.wait(lock, [this] { return open_; });
  }

 private:
  std::mutex lock_;
  std::condition_variable cv_;
  bool open_ = false;
};

//! Stands in for amd::Command.
struct TestCmd : amd::ReferenceCountedObject {
  ~TestCmd() override = default;
};

// ---- single-ticket packets run in order and always with ticket 0 ----
void testSingleTicketFifo() {
  constexpr int kPackets = 500;
  std::vector<TestCmd> cmds(kPackets);
  Pool pool(4);
  Queue* queue = pool.newQueue();

  std::atomic<int> seq{0};
  std::atomic<bool> ordered{true};
  std::atomic<bool> bad_ticket{false};

  std::vector<Packet*> packets;
  packets.reserve(kPackets);
  for (int i = 0; i < kPackets; ++i) {
    auto* p = pool.newPacket(&cmds[i], 1, {});
    p->task = [&seq, &ordered, &bad_ticket, i](int t) {
      if (t != 0) bad_ticket = true;
      if (seq.load() != i) ordered = false;
      seq.store(i + 1);
    };
    packets.push_back(p);
  }

  Signal* last = packets.back()->signal;
  last->retain();
  for (auto* p : packets) queue->submit(p);
  last->wait();
  last->release();

  check(ordered.load() && !bad_ticket.load() && seq.load() == kPackets,
        "single-ticket packets ran in order with ticket 0");
  pass("single-ticket fifo");
}

// ---- a multi-ticket packet runs every ticket exactly once ----
void testMultiTicketFanout() {
  constexpr int kTickets = 64;
  TestCmd cmd;
  Pool pool(4);
  Queue* queue = pool.newQueue();

  std::atomic<int64_t> total{0};
  std::atomic<int64_t> sum{0};
  auto* p = pool.newPacket(&cmd, kTickets, {});
  p->task = [&total, &sum](int i) {
    total.fetch_add(1);
    sum.fetch_add(i);
  };
  submitAndWait(queue, p);

  int64_t expected = static_cast<int64_t>(kTickets) * (kTickets - 1) / 2;
  check(total.load() == kTickets && sum.load() == expected,
        "all 64 tickets executed exactly once");
  pass("multi-ticket fanout");
}

// ---- a single-ticket command after a kernel gets ticket 0, not a stale one ----
void testKernelThenCommandTicket() {
  TestCmd kernel_cmd;
  TestCmd cmd;
  Pool pool(4);
  Queue* queue = pool.newQueue();

  auto* kp = pool.newPacket(&kernel_cmd, 4, {});
  kp->task = [](int i) {
    // Larger i sleeps longer, so the worker holding ticket 3 finishes last and
    // drives the continuation to the next packet.
    std::this_thread::sleep_for(std::chrono::milliseconds(i * 20));
  };
  queue->submit(kp);

  std::atomic<int> cmd_ticket{-1};
  auto* cp = pool.newPacket(&cmd, 1, {});
  cp->task = [&cmd_ticket](int i) { cmd_ticket = i; };
  submitAndWait(queue, cp);

  check(cmd_ticket.load() == 0, "single-ticket command after kernel ran with ticket 0");
  pass("kernel-then-command ticket");
}

// ---- an in-flight dependency creates an edge and the waiter honours it ----
void testPendingDepCreatesEdge() {
  TestCmd gate_cmd;
  TestCmd waiter_cmd;
  Pool pool(3);
  Queue* queue = pool.newQueue();

  Latch started;
  Latch release;
  auto* gate = pool.newPacket(&gate_cmd, 1, {});
  gate->task = [&started, &release](int) {
    started.open();
    release.wait();
  };
  Signal* gate_done = gate->signal;
  gate_done->retain();
  queue->submit(gate);
  started.wait();  // gate is now in-flight

  std::atomic<bool> ran{false};
  auto* watchdog = pool.newPacket(&waiter_cmd, 1, {&gate_cmd});
  bool has_edge = !watchdog->signal_wait_list.empty();
  watchdog->task = [&ran](int) { ran = true; };
  Signal* wd_done = watchdog->signal;
  wd_done->retain();
  queue->submit(watchdog);

  check(!ran.load(), "dependent parked while dependency in-flight");

  release.open();
  gate_done->wait();
  gate_done->release();
  wd_done->wait();
  wd_done->release();

  check(has_edge, "pending dependency created an edge");
  check(ran.load(), "dependent ran after dependency completed");
  pass("pending dependency creates edge");
}

// ---- a completed dependency creates no edge ----
void testCompletedDepCreatesNoEdge() {
  TestCmd finished;
  TestCmd later_cmd;
  Pool pool(2);
  Queue* queue = pool.newQueue();

  auto* first = pool.newPacket(&finished, 1, {});
  first->task = [](int) {};
  submitAndWait(queue, first);

  std::atomic<bool> ran{false};
  auto* later = pool.newPacket(&later_cmd, 1, {&finished});
  bool no_edge = later->signal_wait_list.empty();
  later->task = [&ran](int) { ran = true; };
  submitAndWait(queue, later);

  check(no_edge, "waiting on a completed event added no edge");
  check(ran.load(), "the later packet ran");
  pass("completed dependency creates no edge");
}

// ---- a zero-ticket packet (marker) completes ----
void testMarker() {
  TestCmd cmd;
  Pool pool(2);
  Queue* queue = pool.newQueue();

  auto* p = pool.newPacket(&cmd, 0, {});
  Signal* done = p->signal;
  done->retain();
  queue->submit(p);
  done->wait();
  check(done->isDone(), "marker's done fired");
  done->release();
  pass("marker");
}

// ---- concurrent submission from several threads all completes ----
void testConcurrentSubmit() {
  constexpr int kThreads = 4, kPerThread = 25, kTotal = kThreads * kPerThread;
  std::vector<TestCmd> cmds(kTotal);
  Pool pool(4);
  std::vector<Queue*> queues;
  for (int i = 0; i < 4; ++i) queues.push_back(pool.newQueue());

  std::atomic<int> completed{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t] {
      for (int i = 0; i < kPerThread; ++i) {
        int idx = t * kPerThread + i;
        auto* p = pool.newPacket(&cmds[idx], 1, {});
        p->task = [&completed](int) { completed.fetch_add(1); };
        queues[idx % 4]->submit(p);
      }
    });
  }
  for (auto& th : threads) th.join();

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (completed.load() != kTotal && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  check(completed.load() == kTotal, "all concurrent packets completed");
  pass("concurrent submit");
}

// ---- getTicket CAS hands out each ticket exactly once under contention ----
void testGetTicketRace() {
  constexpr int kIters = 2000;
  constexpr int kThreads = 8;
  TestCmd cmd;
  for (int iter = 0; iter < kIters; ++iter) {
    Packet p(&cmd, 1);
    std::atomic<int> valid{0};
    std::atomic<int> ready{0};
    std::atomic<bool> go{false};
    std::vector<std::thread> th;
    for (int t = 0; t < kThreads; ++t) {
      th.emplace_back([&] {
        ready.fetch_add(1);
        while (!go.load(std::memory_order_acquire)) {}
        if (p.getTicket().has_value()) valid.fetch_add(1);
      });
    }
    while (ready.load() != kThreads) {}
    go.store(true, std::memory_order_release);
    for (auto& t : th) t.join();
    if (valid.load() != 1) {
      check(false, "getTicket handed out duplicate/missing ticket under contention");
      return;
    }
  }
  pass("getTicket race");
}

// ---- a continuation packet runs its work exactly once ----
void testContinuationRunsOnce() {
  TestCmd first;
  TestCmd second;
  Pool pool(2);
  Queue* queue = pool.newQueue();

  std::atomic<int> first_count{0};
  std::atomic<int> second_count{0};

  auto* p1 = pool.newPacket(&first, 1, {});
  p1->task = [&first_count](int) { first_count.fetch_add(1); };
  queue->submit(p1);

  auto* p2 = pool.newPacket(&second, 1, {});
  p2->task = [&second_count](int) { second_count.fetch_add(1); };
  Signal* done = p2->signal;
  done->retain();
  queue->submit(p2);
  done->wait();
  done->release();

  check(first_count.load() == 1 && second_count.load() == 1,
        "each single-ticket packet ran exactly once");
  pass("continuation runs once");
}

// ---- a multi-ticket packet with slow work engages multiple workers ----
void testMultiTicketParallelism() {
  constexpr int kTickets = 4;
  TestCmd cmd;
  Pool pool(4);
  Queue* queue = pool.newQueue();

  std::atomic<int> active{0};
  std::atomic<int> max_active{0};
  auto* p = pool.newPacket(&cmd, kTickets, {});
  p->task = [&active, &max_active](int) {
    int now = active.fetch_add(1) + 1;
    int seen = max_active.load(std::memory_order_relaxed);
    while (now > seen &&
           !max_active.compare_exchange_weak(seen, now, std::memory_order_relaxed)) {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    active.fetch_sub(1);
  };
  submitAndWait(queue, p);

  check(max_active.load() >= 2, "multi-ticket packet engaged multiple workers concurrently");
  pass("multi-ticket parallelism");
}

// ---- a head parked on two signal_wait_list dispatches once both complete, in any order ----
void testMultiDepPark() {
  TestCmd dep1_cmd;
  TestCmd dep2_cmd;
  TestCmd waiter_cmd;
  Pool pool(2);
  Queue* q1 = pool.newQueue();
  Queue* q2 = pool.newQueue();
  Queue* q3 = pool.newQueue();

  Latch release_dep1;
  Latch release_dep2;

  auto* dep1 = pool.newPacket(&dep1_cmd, 1, {});
  dep1->task = [&release_dep1](int) { release_dep1.wait(); };
  auto* dep2 = pool.newPacket(&dep2_cmd, 1, {});
  dep2->task = [&release_dep2](int) { release_dep2.wait(); };

  Signal* dep1_done = dep1->signal;
  Signal* dep2_done = dep2->signal;
  dep1_done->retain();
  dep2_done->retain();
  q1->submit(dep1);
  q2->submit(dep2);

  std::atomic<int> ran{0};
  auto* waiter = pool.newPacket(&waiter_cmd, 1, {&dep1_cmd, &dep2_cmd});
  bool both_edges = waiter->signal_wait_list.size() == 2;
  waiter->task = [&ran](int) { ran.fetch_add(1); };
  Signal* wd = waiter->signal;
  wd->retain();
  q3->submit(waiter);

  check(!ran.load(), "waiter parked while both signal_wait_list pending");

  release_dep2.open();
  dep2_done->wait();
  check(!ran.load(), "waiter still parked after only one dep completed");

  release_dep1.open();
  dep1_done->wait();
  wd->wait();
  wd->release();
  dep1_done->release();
  dep2_done->release();

  check(both_edges, "waiter has two dependency edges");
  check(ran.load() == 1, "waiter ran exactly once after both signal_wait_list completed");
  pass("multi-dep park");
}

}  // namespace

int main() {
  testSingleTicketFifo();
  testMultiTicketFanout();
  testKernelThenCommandTicket();
  testPendingDepCreatesEdge();
  testCompletedDepCreatesNoEdge();
  testMarker();
  testConcurrentSubmit();
  testGetTicketRace();
  testContinuationRunsOnce();
  testMultiTicketParallelism();
  testMultiDepPark();

  if (g_failures.load() != 0) {
    printf("\n%d check(s) failed\n", g_failures.load());
    return 1;
  }
  printf("\nall tests passed\n");
  return 0;
}
