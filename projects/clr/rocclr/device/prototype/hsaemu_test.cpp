// PROTOTYPE tests: the emulated HSA agent, and the completion-entry protocol
// that maps rocclr commands onto it.
//
// Only the externally visible surface is used: NewQueue, NewPacket, Submit,
// WaitIdle. A packet is a single-owner chain -- Submit hands it to the agent and
// DonePacket deletes it -- so a test reads a packet's fields before submitting
// and never after. Dependencies are commands (the thing event_signals_ is keyed
// by), not signals: NewPacket looks up the wait list in the table and wires the
// signal only if the command's entry is still there.
#include "device/prototype/hsaemu.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <memory>
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

void Sleep(int micros) { std::this_thread::sleep_for(std::chrono::microseconds(micros)); }

//! A one-shot gate: Wait blocks until Open, Open is idempotent.
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

//! Stands in for amd::Command: reference counted from 1, destroyed by the last
//! release. NewPacket only needs the base class.
struct TestCmd : amd::ReferenceCountedObject {
 public:
  ~TestCmd() override = default;
};

//! A command whose packet stays in flight until Release. While in flight its
//! entry is in the table, so another packet can depend on cmd(). The gate
//! occupies one worker while armed, so the agent needs that much headroom.
class Gate {
 public:
  explicit Gate(Agent& agent) : agent_(agent) {
    queue_ = agent_.NewQueue();
    auto* packet = agent_.NewPacket(&cmd_, {});
    signal_ = packet->done;
    signal_->retain();
    packet->work = [this] {
      opened_.Wait();
      closed_.Open();
    };
    queue_->Submit(packet);
  }

  ~Gate() {
    Release();
    signal_->Wait();
    signal_->release();
  }

  Gate(const Gate&) = delete;
  Gate& operator=(const Gate&) = delete;

  //! The command other packets depend on. Its entry lives in the table until
  /// Release() lets the gate's work finish.
  Key* cmd() { return &cmd_; }

  //! Lets the gate finish and waits until its work returned, so the latches are
  //! not destroyed underneath a worker.
  void Release() {
    opened_.Open();
    closed_.Wait();
  }

 private:
  Agent& agent_;
  Queue* queue_ = nullptr;
  TestCmd cmd_;
  Signal* signal_ = nullptr;
  Latch opened_;
  Latch closed_;
};

// ---- A queue runs its packets in submission order ----
void TestFifo() {
  Agent agent(4);
  Queue* queue = agent.NewQueue();
  std::atomic<int> seen{0};
  std::atomic<bool> ordered{true};
  constexpr int kPackets = 500;

  std::vector<TestCmd> commands(kPackets);
  for (int i = 0; i < kPackets; ++i) {
    auto* packet = agent.NewPacket(&commands[i], {});
    packet->work = [&seen, &ordered, i] {
      if (seen.load() != i) ordered = false;
      seen = i + 1;
    };
    queue->Submit(packet);
  }
  agent.WaitIdle();

  Check(ordered.load() && seen.load() == kPackets, "500 packets ran in submission order");
  Pass("fifo");
}

// ---- An in-flight dependency creates an edge, and the waiter honours it ----
void TestPendingDepCreatesEdge() {
  Agent agent(3);
  Queue* second = agent.NewQueue();
  Gate gate(agent);
  TestCmd waiter_cmd;
  std::atomic<bool> ran{false};

  auto* watchdog = agent.NewPacket(&waiter_cmd, {gate.cmd()});
  const bool has_edge = !watchdog->deps.empty();
  watchdog->work = [&ran] { ran = true; };
  second->Submit(watchdog);

  Sleep(20000);
  bool ran_early = ran.load();

  gate.Release();
  agent.WaitIdle();

  Check(has_edge, "an in-flight dependency created an edge");
  Check(!ran_early, "the dependent packet did not run before its dependency");
  Check(ran.load(), "the dependent packet ran after its dependency");
  Pass("pending dependency creates an edge");
}

// ---- The core invariant: a completed dependency creates no edge ----
// The entry disappears only after the packet ran, so a missing entry means the
// event already completed. A later packet must add no dependency, and must not
// mint a signal nobody would ever Notify.
void TestCompletedDepCreatesNoEdge() {
  Agent agent(2);
  Queue* queue = agent.NewQueue();
  TestCmd finished;

  auto* first = agent.NewPacket(&finished, {});
  first->work = [] {};
  queue->Submit(first);
  agent.WaitIdle();  // `finished` has completed and its entry is gone

  TestCmd later_cmd;
  auto* later = agent.NewPacket(&later_cmd, {&finished});
  const bool no_edge = later->deps.empty();
  std::atomic<bool> ran{false};
  later->work = [&ran] { ran = true; };
  queue->Submit(later);
  agent.WaitIdle();

  Check(no_edge, "waiting on a completed event added no dependency");
  Check(ran.load(), "the later packet ran instead of parking forever");
  Pass("completed dependency creates no edge");
}

// ---- A parked queue does not occupy a worker ----
void TestParkedQueueHoldsNoWorker() {
  Agent agent(2);  // one worker held by the gate, one left for the others
  Queue* parked = agent.NewQueue();
  Queue* other = agent.NewQueue();
  Gate gate(agent);
  TestCmd parked_cmd;
  TestCmd other_cmd;
  std::atomic<bool> parked_ran{false};
  std::atomic<bool> other_ran{false};

  auto* parked_packet = agent.NewPacket(&parked_cmd, {gate.cmd()});
  parked_packet->work = [&parked_ran] { parked_ran = true; };
  parked->Submit(parked_packet);

  auto* other_packet = agent.NewPacket(&other_cmd, {});
  other_packet->work = [&other_ran] { other_ran = true; };
  other->Submit(other_packet);

  Sleep(30000);
  bool other_finished_while_parked = other_ran.load();
  bool parked_early = parked_ran.load();

  gate.Release();
  agent.WaitIdle();

  Check(other_finished_while_parked, "another queue ran while this one was parked");
  Check(!parked_early, "the parked packet did not run before its dependency");
  Check(parked_ran.load(), "the parked packet ran once its dependency completed");
  Pass("parked queue releases its worker");
}

// ---- Registering against a signal that completes at the same moment ----
// OnDone runs inline when the signal is already done, so either the packet is
// woken or it never parks. Neither may lose it.
void TestNotifySubscribeRace() {
  for (int iter = 0; iter < 200; ++iter) {
    Agent agent(3);
    Queue* queue = agent.NewQueue();
    Gate gate(agent);
    TestCmd cmd;
    std::atomic<bool> ran{false};

    auto* packet = agent.NewPacket(&cmd, {gate.cmd()});
    packet->work = [&ran] { ran = true; };
    queue->Submit(packet);

    std::thread releaser([&gate] {
      Sleep(50);
      gate.Release();
    });
    releaser.join();
    agent.WaitIdle();

    if (!ran) {
      Check(false, "packet lost a wakeup");
      return;
    }
  }
  Pass("subscribe/completion race (200 iterations)");
}

// ---- Several dependencies, each fired separately, dispatch once ----
void TestManyDepsDispatchOnce() {
  constexpr int kDeps = 5;
  for (int iter = 0; iter < 50; ++iter) {
    Agent agent(kDeps + 2);
    Queue* queue = agent.NewQueue();
    std::vector<std::unique_ptr<Gate>> gates;

    KeyList wait;
    for (int i = 0; i < kDeps; ++i) {
      gates.emplace_back(new Gate(agent));
      wait.push_back(gates.back()->cmd());
    }

    TestCmd cmd;
    auto* packet = agent.NewPacket(&cmd, wait);
    std::atomic<int> dispatches{0};
    packet->work = [&dispatches] {
      ++dispatches;
      Sleep(5000);
    };
    queue->Submit(packet);

    for (int i = 0; i + 1 < kDeps; ++i) {
      gates[i]->Release();
      Sleep(200);
    }
    bool ran_early = dispatches.load() != 0;

    gates[kDeps - 1]->Release();
    agent.WaitIdle();

    if (ran_early || dispatches.load() != 1) {
      Check(false, "packet did not wait for every dependency, or ran twice");
      return;
    }
  }
  Pass("many deps (5 released one by one, one dispatch)");
}

// ---- Concurrent submission from several threads all completes ----
void TestConcurrentSubmit() {
  for (int iter = 0; iter < 30; ++iter) {
    Agent agent(4);
    constexpr int kQueues = 8, kPerThread = 40, kThreads = 4;
    constexpr int kTotalCmds = kThreads * kPerThread;

    std::vector<Queue*> queues;
    for (int i = 0; i < kQueues; ++i) queues.push_back(agent.NewQueue());

    std::vector<TestCmd> commands(kTotalCmds);
    std::atomic<int> ran{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
      threads.emplace_back([&, t] {
        for (int i = 0; i < kPerThread; ++i) {
          int idx = t * kPerThread + i;
          auto* packet = agent.NewPacket(&commands[idx], {});
          packet->work = [&ran] { ++ran; };
          queues[idx % kQueues]->Submit(packet);
        }
      });
    }
    for (auto& thread : threads) thread.join();
    agent.WaitIdle();

    if (ran.load() != kTotalCmds) {
      Check(false, "lost packets under concurrent submission");
      return;
    }
  }
  Pass("concurrent submit (30 iterations x 4 threads)");
}

// ---- Two queues chained through completion entries run in order ----
void TestChainedQueues() {
  for (int iter = 0; iter < 100; ++iter) {
    Agent agent(3);
    Queue* queue = agent.NewQueue();
    Gate gate(agent);
    TestCmd follower_cmd;
    std::atomic<bool> ran{false};

    auto* follower = agent.NewPacket(&follower_cmd, {gate.cmd()});
    follower->work = [&ran] { ran = true; };
    queue->Submit(follower);

    Sleep(20000);
    bool ran_early = ran.load();

    gate.Release();
    agent.WaitIdle();

    if (ran_early || !ran.load()) {
      Check(false, "the chained packet did not wait for its dependency");
      return;
    }
  }
  Pass("cross-queue chain (100 iterations)");
}

// ---- A command submitted twice gets a fresh live signal the second time ----
void TestReusedCommandGetsFreshSignal() {
  Agent agent(2);
  Queue* queue = agent.NewQueue();
  TestCmd cmd;

  auto* first = agent.NewPacket(&cmd, {});
  Signal* first_done = first->done;
  first_done->retain();  // outlives the packet, which is deleted on completion
  first->work = [] {};
  queue->Submit(first);
  agent.WaitIdle();
  Check(first_done->IsDone(), "the first submission's signal completed");

  auto* second = agent.NewPacket(&cmd, {});
  Signal* second_done = second->done;
  second_done->retain();
  Check(second_done != first_done, "the second submission got a different signal");
  Check(!second_done->IsDone(), "the second submission's signal is armed");

  second->work = [] {};
  queue->Submit(second);
  agent.WaitIdle();
  Check(second_done->IsDone(), "the second submission's signal completed");

  second_done->release();
  first_done->release();
  Pass("a reused command gets a fresh signal");
}

// ---- Independent commands never share a completion signal ----
void TestCommandsHaveDistinctSignals() {
  Agent agent(2);
  Queue* queue = agent.NewQueue();
  TestCmd one;
  TestCmd two;

  auto* a = agent.NewPacket(&one, {});
  auto* b = agent.NewPacket(&two, {});
  Check(a->done != b->done, "two commands got two signals");

  a->work = [] {};
  b->work = [] {};
  queue->Submit(a);
  queue->Submit(b);
  agent.WaitIdle();
  Pass("commands have distinct signals");
}

// ---- Submitting against a command that is completing right now ----
// The dependency lookup and the entry's removal are serialized by the agent's
// lock, so either the edge is taken while the entry exists and the waiter is
// woken, or the entry is already gone and the waiter never parks.
void TestEdgeVersusCompletionRace() {
  for (int iter = 0; iter < 200; ++iter) {
    Agent agent(3);
    Queue* queue = agent.NewQueue();
    Gate gate(agent);
    TestCmd cmd;
    std::atomic<bool> ran{false};

    std::thread racer([&] {
      auto* packet = agent.NewPacket(&cmd, {gate.cmd()});
      packet->work = [&ran] { ran = true; };
      queue->Submit(packet);
    });
    racer.join();

    gate.Release();
    agent.WaitIdle();

    if (!ran) {
      Check(false, "the racing packet was neither woken nor run immediately");
      return;
    }
  }
  Pass("dependency-vs-completion race (200 iterations)");
}

}  // namespace

int main() {
  TestFifo();
  TestPendingDepCreatesEdge();
  TestCompletedDepCreatesNoEdge();
  TestParkedQueueHoldsNoWorker();
  TestNotifySubscribeRace();
  TestManyDepsDispatchOnce();
  TestConcurrentSubmit();
  TestChainedQueues();
  TestReusedCommandGetsFreshSignal();
  TestCommandsHaveDistinctSignals();
  TestEdgeVersusCompletionRace();

  if (g_failures.load() != 0) {
    printf("\n%d check(s) failed\n", g_failures.load());
    return 1;
  }
  printf("\nall tests passed\n");
  return 0;
}
