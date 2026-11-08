// PROTOTYPE tests: the emulated HSA agent with divisible (multi-ticket) tasks.
//
// The public surface is: NewQueue, NewPacket(cmd, num_tickets, wait_list),
// Submit, and per-packet done signals. There is no global WaitIdle, so a test
// waits on a packet's done signal (retained first, since DonePacket deletes the
// packet). Command objects are declared before the Agent so the Agent's workers
// are joined before the commands are destroyed.
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

void Check(bool ok, const char* what) {
  if (!ok) {
    printf("  FAIL %s\n", what);
    ++g_failures;
  }
}

void Pass(const char* what) { printf("PASS %s\n", what); }

//! Submit |packet| and block until its done signal fires.
void SubmitAndWait(Queue* queue, Packet* packet) {
  Signal* done = packet->done;
  done->retain();
  queue->Submit(packet);
  done->Wait();
  done->release();
}

//! One-shot gate.
class Latch {
 public:
  void Open() {
    {
      std::lock_guard<std::mutex> lock(lock_);
      open_ = true;
    }
    cv_.notify_all();
  }

  void Wait() {
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
void TestSingleTicketFifo() {
  constexpr int kPackets = 500;
  std::vector<TestCmd> cmds(kPackets);
  Agent agent(4);
  Queue* queue = agent.NewQueue();

  std::atomic<int> seq{0};
  std::atomic<bool> ordered{true};
  std::atomic<bool> bad_ticket{false};

  std::vector<Packet*> packets;
  packets.reserve(kPackets);
  for (int i = 0; i < kPackets; ++i) {
    auto* p = agent.NewPacket(&cmds[i], 1, {});
    p->work = [&seq, &ordered, &bad_ticket, i](int t) {
      if (t != 0) bad_ticket = true;
      if (seq.load() != i) ordered = false;
      seq.store(i + 1);
    };
    packets.push_back(p);
  }

  Signal* last = packets.back()->done;
  last->retain();
  for (auto* p : packets) queue->Submit(p);
  last->Wait();
  last->release();

  Check(ordered.load() && !bad_ticket.load() && seq.load() == kPackets,
        "single-ticket packets ran in order with ticket 0");
  Pass("single-ticket fifo");
}

// ---- a multi-ticket packet runs every ticket exactly once ----
void TestMultiTicketFanout() {
  constexpr int kTickets = 64;
  TestCmd cmd;
  Agent agent(4);
  Queue* queue = agent.NewQueue();

  std::atomic<int64_t> total{0};
  std::atomic<int64_t> sum{0};
  auto* p = agent.NewPacket(&cmd, kTickets, {});
  p->work = [&total, &sum](int i) {
    total.fetch_add(1);
    sum.fetch_add(i);
  };
  SubmitAndWait(queue, p);

  int64_t expected = static_cast<int64_t>(kTickets) * (kTickets - 1) / 2;
  Check(total.load() == kTickets && sum.load() == expected,
        "all 64 tickets executed exactly once");
  Pass("multi-ticket fanout");
}

// ---- a single-ticket command after a kernel gets ticket 0, not a stale one ----
void TestKernelThenCommandTicket() {
  TestCmd kernel_cmd;
  TestCmd cmd;
  Agent agent(4);
  Queue* queue = agent.NewQueue();

  auto* kp = agent.NewPacket(&kernel_cmd, 4, {});
  kp->work = [](int i) {
    // Larger i sleeps longer, so the worker holding ticket 3 finishes last and
    // drives the continuation to the next packet.
    std::this_thread::sleep_for(std::chrono::milliseconds(i * 20));
  };
  queue->Submit(kp);

  std::atomic<int> cmd_ticket{-1};
  auto* cp = agent.NewPacket(&cmd, 1, {});
  cp->work = [&cmd_ticket](int i) { cmd_ticket = i; };
  SubmitAndWait(queue, cp);

  Check(cmd_ticket.load() == 0, "single-ticket command after kernel ran with ticket 0");
  Pass("kernel-then-command ticket");
}

// ---- an in-flight dependency creates an edge and the waiter honours it ----
void TestPendingDepCreatesEdge() {
  TestCmd gate_cmd;
  TestCmd waiter_cmd;
  Agent agent(3);
  Queue* queue = agent.NewQueue();

  Latch started;
  Latch release;
  auto* gate = agent.NewPacket(&gate_cmd, 1, {});
  gate->work = [&started, &release](int) {
    started.Open();
    release.Wait();
  };
  Signal* gate_done = gate->done;
  gate_done->retain();
  queue->Submit(gate);
  started.Wait();  // gate is now in-flight

  std::atomic<bool> ran{false};
  auto* watchdog = agent.NewPacket(&waiter_cmd, 1, {&gate_cmd});
  bool has_edge = !watchdog->deps.empty();
  watchdog->work = [&ran](int) { ran = true; };
  Signal* wd_done = watchdog->done;
  wd_done->retain();
  queue->Submit(watchdog);

  Check(!ran.load(), "dependent parked while dependency in-flight");

  release.Open();
  gate_done->Wait();
  gate_done->release();
  wd_done->Wait();
  wd_done->release();

  Check(has_edge, "pending dependency created an edge");
  Check(ran.load(), "dependent ran after dependency completed");
  Pass("pending dependency creates edge");
}

// ---- a completed dependency creates no edge ----
void TestCompletedDepCreatesNoEdge() {
  TestCmd finished;
  TestCmd later_cmd;
  Agent agent(2);
  Queue* queue = agent.NewQueue();

  auto* first = agent.NewPacket(&finished, 1, {});
  first->work = [](int) {};
  SubmitAndWait(queue, first);

  std::atomic<bool> ran{false};
  auto* later = agent.NewPacket(&later_cmd, 1, {&finished});
  bool no_edge = later->deps.empty();
  later->work = [&ran](int) { ran = true; };
  SubmitAndWait(queue, later);

  Check(no_edge, "waiting on a completed event added no edge");
  Check(ran.load(), "the later packet ran");
  Pass("completed dependency creates no edge");
}

// ---- a zero-ticket packet (marker) completes ----
void TestMarker() {
  TestCmd cmd;
  Agent agent(2);
  Queue* queue = agent.NewQueue();

  auto* p = agent.NewPacket(&cmd, 0, {});
  Signal* done = p->done;
  done->retain();
  queue->Submit(p);
  done->Wait();
  Check(done->IsDone(), "marker's done fired");
  done->release();
  Pass("marker");
}

// ---- concurrent submission from several threads all completes ----
void TestConcurrentSubmit() {
  constexpr int kThreads = 4, kPerThread = 25, kTotal = kThreads * kPerThread;
  std::vector<TestCmd> cmds(kTotal);
  Agent agent(4);
  std::vector<Queue*> queues;
  for (int i = 0; i < 4; ++i) queues.push_back(agent.NewQueue());

  std::atomic<int> completed{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t] {
      for (int i = 0; i < kPerThread; ++i) {
        int idx = t * kPerThread + i;
        auto* p = agent.NewPacket(&cmds[idx], 1, {});
        p->work = [&completed](int) { completed.fetch_add(1); };
        queues[idx % 4]->Submit(p);
      }
    });
  }
  for (auto& th : threads) th.join();

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (completed.load() != kTotal && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  Check(completed.load() == kTotal, "all concurrent packets completed");
  Pass("concurrent submit");
}

// ---- get_ticket CAS hands out each ticket exactly once under contention ----
void TestGetTicketRace() {
  constexpr int kIters = 2000;
  constexpr int kThreads = 8;
  for (int iter = 0; iter < kIters; ++iter) {
    Packet p;
    p.num_tickets = 1;
    std::atomic<int> valid{0};
    std::atomic<int> ready{0};
    std::atomic<bool> go{false};
    std::vector<std::thread> th;
    for (int t = 0; t < kThreads; ++t) {
      th.emplace_back([&] {
        ready.fetch_add(1);
        while (!go.load(std::memory_order_acquire)) {}
        if (p.get_ticket().has_value()) valid.fetch_add(1);
      });
    }
    while (ready.load() != kThreads) {}
    go.store(true, std::memory_order_release);
    for (auto& t : th) t.join();
    if (valid.load() != 1) {
      Check(false, "get_ticket handed out duplicate/missing ticket under contention");
      return;
    }
  }
  Pass("get_ticket race");
}

// ---- a continuation packet runs its work exactly once ----
void TestContinuationRunsOnce() {
  TestCmd first;
  TestCmd second;
  Agent agent(2);
  Queue* queue = agent.NewQueue();

  std::atomic<int> first_count{0};
  std::atomic<int> second_count{0};

  auto* p1 = agent.NewPacket(&first, 1, {});
  p1->work = [&first_count](int) { first_count.fetch_add(1); };
  queue->Submit(p1);

  auto* p2 = agent.NewPacket(&second, 1, {});
  p2->work = [&second_count](int) { second_count.fetch_add(1); };
  Signal* done = p2->done;
  done->retain();
  queue->Submit(p2);
  done->Wait();
  done->release();

  Check(first_count.load() == 1 && second_count.load() == 1,
        "each single-ticket packet ran exactly once");
  Pass("continuation runs once");
}

}  // namespace

int main() {
  TestSingleTicketFifo();
  TestMultiTicketFanout();
  TestKernelThenCommandTicket();
  TestPendingDepCreatesEdge();
  TestCompletedDepCreatesNoEdge();
  TestMarker();
  TestConcurrentSubmit();
  TestGetTicketRace();
  TestContinuationRunsOnce();

  if (g_failures.load() != 0) {
    printf("\n%d check(s) failed\n", g_failures.load());
    return 1;
  }
  printf("\nall tests passed\n");
  return 0;
}
