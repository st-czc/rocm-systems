// PROTOTYPE tests: the emulated HSA agent, and the completion-entry protocol
// that maps rocclr commands onto it.
//
// Every object here is reference counted, so the tests never touch retain and
// release by hand -- Ref below holds one reference for a scope. What is being
// tested is ordering, parking and the lifetime rule for the (command -> signal)
// table, not reference bookkeeping.
#include "device/prototype/hsaemu.hpp"

#include <atomic>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

using namespace amd::device::prototype;

namespace {

int g_failures = 0;

void Check(bool ok, const char* what) {
  if (!ok) {
    printf("  FAIL %s\n", what);
    ++g_failures;
  }
}

void Pass(const char* what) { printf("PASS %s\n", what); }

//! Holds exactly one reference for as long as it is in scope. NewSignal() and
//! `new` both hand out one reference, so this is the counterpart for both.
template <typename T>
class Ref {
 public:
  explicit Ref(T* object) : object_(object) {}
  ~Ref() {
    if (object_ != nullptr) {
      object_->release();
    }
  }
  Ref(const Ref&) = delete;
  Ref& operator=(const Ref&) = delete;

  T* get() const { return object_; }
  T& operator*() const { return *object_; }
  T* operator->() const { return object_; }

 private:
  T* object_;
};

//! Stands in for amd::Command: reference counted from 1, destroyed by the last
//! release. PreparePacket only needs the base class, so this is the whole thing.
struct TestCmd : amd::ReferenceCountedObject {
 public:
  ~TestCmd() override = default;
};

//! Sleeps a little, the way a real dispatch would occupy its queue.
void Work(int micros = 500) { std::this_thread::sleep_for(std::chrono::microseconds(micros)); }

// ---- A queue runs its packets in submission order ----
void TestFifo() {
  Agent agent(4);
  Queue* queue = agent.NewQueue();
  std::atomic<int> next{0};
  bool ordered = true;

  for (int i = 0; i < 500; ++i) {
    auto packet = std::make_shared<Packet>();
    packet->work = [&next, &ordered, i] {
      if (next.load() != i) ordered = false;
      next = i + 1;
    };
    queue->Submit(packet);
  }
  agent.WaitIdle();

  Check(ordered && next.load() == 500, "500 packets ran in submission order");
  Pass("fifo");
}

// ---- An unmet dependency parks the queue without occupying a worker ----
void TestParkedQueueHoldsNoWorker() {
  Agent agent(1);  // one worker: if the parked queue held it, nothing else runs
  Queue* park = agent.NewQueue();
  Queue* other = agent.NewQueue();
  Ref<Signal> gate(agent.NewSignal());
  std::atomic<bool> other_ran{false};

  auto blocked = std::make_shared<Packet>();
  blocked->AddDep(gate.get());
  blocked->work = [] {};
  park->Submit(blocked);

  auto free = std::make_shared<Packet>();
  free->work = [&other_ran] { other_ran = true; };
  other->Submit(free);

  Work(50000);
  bool ran_while_parked = other_ran.load();
  gate->Notify();
  agent.WaitIdle();

  Check(ran_while_parked, "another queue still ran with the only worker");
  Check(other_ran.load(), "the parked packet ran after its gate");
  Pass("parked queue releases its worker");
}

// ---- Subscribing to a signal that already fired runs inline, no lost wakeup ----
void TestNotifySubscribeRace() {
  for (int iter = 0; iter < 300; ++iter) {
    Agent agent(2);
    Queue* queue = agent.NewQueue();
    Ref<Signal> gate(agent.NewSignal());
    std::atomic<bool> ran{false};

    auto packet = std::make_shared<Packet>();
    packet->AddDep(gate.get());
    packet->work = [&ran] { ran = true; };
    queue->Submit(packet);

    std::thread notifier([&gate] {
      Work(50);
      gate->Notify();
    });
    notifier.join();
    agent.WaitIdle();

    if (!ran) {
      Check(false, "packet lost a wakeup");
      return;
    }
  }
  Pass("notify/subscribe race (300 iterations)");
}

// ---- A packet with several dependencies dispatches exactly once ----
void TestManyDepsDispatchOnce() {
  for (int iter = 0; iter < 100; ++iter) {
    Agent agent(2);
    Queue* queue = agent.NewQueue();
    constexpr int kDeps = 5;

    std::vector<std::unique_ptr<Ref<Signal>>> deps;
    auto packet = std::make_shared<Packet>();
    for (int i = 0; i < kDeps; ++i) {
      deps.emplace_back(new Ref<Signal>(agent.NewSignal()));
      packet->AddDep(deps.back()->get());
    }

    std::atomic<int> dispatches{0};
    packet->work = [&dispatches] {
      ++dispatches;
      Work(5000);
    };
    queue->Submit(packet);

    // Fire every dependency but the last; the packet must stay parked.
    for (int i = 0; i + 1 < kDeps; ++i) {
      deps[i]->get()->Notify();
      Work(100);
    }
    bool ran_early = dispatches.load() != 0;

    deps[kDeps - 1]->get()->Notify();
    agent.WaitIdle();

    if (ran_early || dispatches.load() != 1) {
      Check(false, "packet did not wait for all dependencies, or ran twice");
      return;
    }
  }
  Pass("many deps (5 fired one by one, one dispatch)");
}

// ---- Concurrent submit from several threads all completes ----
void TestConcurrentSubmit() {
  for (int iter = 0; iter < 50; ++iter) {
    Agent agent(4);
    constexpr int kQueues = 8, kPerThread = 40, kThreads = 4;

    std::vector<Queue*> queues;
    for (int i = 0; i < kQueues; ++i) queues.push_back(agent.NewQueue());

    std::atomic<int> ran{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
      threads.emplace_back([&, t] {
        for (int i = 0; i < kPerThread; ++i) {
          auto packet = std::make_shared<Packet>();
          packet->work = [&ran] { ++ran; };
          queues[(t * kPerThread + i) % kQueues]->Submit(packet);
        }
      });
    }
    for (auto& thread : threads) thread.join();
    agent.WaitIdle();

    if (ran.load() != kThreads * kPerThread) {
      Check(false, "lost packets under concurrent submission");
      return;
    }
  }
  Pass("concurrent submit (50 iterations x 4 threads)");
}

// ---- PreparePacket leaves an armed completion signal in the packet ----
void TestPrepareLeavesArmedSignal() {
  Agent agent(1);
  Ref<TestCmd> cmd(new TestCmd());

  auto packet = std::make_shared<Packet>();
  agent.PreparePacket(*packet, *cmd, {});

  Check(packet->done != nullptr, "prepare set a completion signal");
  Check(!packet->done->IsDone(), "a fresh packet's completion signal starts armed");
  Pass("prepare leaves armed signal in packet");
}

// ---- The entry holds the command, and gives it back exactly once ----
void TestEntryOwnsCommand() {
  Agent agent(2);
  Queue* queue = agent.NewQueue();
  Ref<TestCmd> cmd(new TestCmd());
  const uint32_t caller_refs = cmd.get()->referenceCount();

  auto packet = std::make_shared<Packet>();
  agent.PreparePacket(*packet, *cmd, {});
  Check(cmd.get()->referenceCount() == caller_refs + 1,
        "the entry retained the command");

  packet->work = [] {};
  queue->Submit(packet);
  agent.WaitIdle();

  Check(cmd.get()->referenceCount() == caller_refs,
        "completing the packet released exactly one reference");
  packet.reset();
  Pass("entry owns the command for its whole life");
}

// ---- The core invariant: a completed dependency creates no edge ----
// The entry disappears only after the packet ran, so a missing entry means the
// event already completed. A later packet must therefore add no dependency and
// must not mint a signal that nobody would ever Notify().
void TestCompletedDepCreatesNoEdge() {
  Agent agent(2);
  Queue* queue = agent.NewQueue();
  Ref<TestCmd> first(new TestCmd());
  Ref<TestCmd> second(new TestCmd());

  auto a = std::make_shared<Packet>();
  agent.PreparePacket(*a, *first, {});
  a->work = [] {};
  queue->Submit(a);
  agent.WaitIdle();  // `first` has completed; its entry is gone

  std::atomic<bool> ran{false};
  auto b = std::make_shared<Packet>();
  agent.PreparePacket(*b, *second, {first.get()});
  b->work = [&ran] { ran = true; };
  queue->Submit(b);
  agent.WaitIdle();

  Check(b->deps.empty(), "waiting on a completed event added no dependency");
  Check(ran.load(), "the later packet ran instead of parking forever");
  Pass("completed dependency creates no edge");
}

// ---- A dependency that is still in flight does create an edge ----
void TestPendingDepCreatesEdge() {
  Agent agent(2);
  Queue* first_queue = agent.NewQueue();
  Queue* second_queue = agent.NewQueue();
  Ref<TestCmd> first(new TestCmd());
  Ref<TestCmd> second(new TestCmd());
  std::atomic<int> step{0};

  auto a = std::make_shared<Packet>();
  agent.PreparePacket(*a, *first, {});
  a->work = [&step] {
    Work(20000);  // keep `a` in flight while `b` is prepared
    step = 1;
  };
  first_queue->Submit(a);

  auto b = std::make_shared<Packet>();
  agent.PreparePacket(*b, *second, {first.get()});
  b->work = [&step] {
    if (step.load() == 1) step = 2;
  };
  second_queue->Submit(b);

  agent.WaitIdle();
  Check(step.load() == 2, "the dependent packet waited for its dependency");
  Pass("pending dependency creates an edge");
}

// ---- A packet that never runs still retires its entry ----
// Nothing calls Execute for a packet dropped before dispatch, so the entry has
// to be dropped by the packet's own destructor. Otherwise the table keeps a
// reference on a dead command forever.
void TestUnrunPacketRetires() {
  Agent agent(2);
  Ref<TestCmd> cmd(new TestCmd());
  const uint32_t caller_refs = cmd.get()->referenceCount();

  {
    auto packet = std::make_shared<Packet>();
    agent.PreparePacket(*packet, *cmd, {});
    Check(cmd.get()->referenceCount() == caller_refs + 1, "the entry retained the command");
    // Dropped without ever being submitted.
  }

  Check(cmd.get()->referenceCount() == caller_refs,
        "destroying an unrun packet released the entry's reference");
  Pass("unrun packet retires its entry");
}

// ---- Two queues chained through entries complete in order ----
void TestChainedQueues() {
  for (int iter = 0; iter < 100; ++iter) {
    Agent agent(4);
    Queue* first_queue = agent.NewQueue();
    Queue* second_queue = agent.NewQueue();
    Ref<TestCmd> first(new TestCmd());
    Ref<TestCmd> second(new TestCmd());
    std::atomic<int> step{0};

    auto a = std::make_shared<Packet>();
    agent.PreparePacket(*a, *first, {});
    a->work = [&step] {
      Check(step.load() == 0, "the second packet started before the first");
      step = 1;
    };
    first_queue->Submit(a);

    auto b = std::make_shared<Packet>();
    agent.PreparePacket(*b, *second, {first.get()});
    b->work = [&step] {
      Check(step.load() == 1, "the second packet ran before the first finished");
      step = 2;
    };
    second_queue->Submit(b);

    agent.WaitIdle();
    if (step.load() != 2) {
      Check(false, "the chain did not complete in order");
      return;
    }
  }
  Pass("cross-queue chain through PreparePacket (100 iterations)");
}

// ---- Submitting against an event that is completing right now ----
// The dependency lookup and the entry's removal are serialized by the agent's
// lock, so either the edge is taken while the entry exists (and the waiter is
// woken) or the entry is already gone (and the waiter never parks).
void TestEdgeVersusRetireRace() {
  for (int iter = 0; iter < 300; ++iter) {
    Agent agent(2);
    Queue* first_queue = agent.NewQueue();
    Queue* second_queue = agent.NewQueue();
    Ref<TestCmd> first(new TestCmd());
    Ref<TestCmd> second(new TestCmd());
    std::atomic<bool> ran{false};

    auto a = std::make_shared<Packet>();
    agent.PreparePacket(*a, *first, {});
    a->work = [] {};
    first_queue->Submit(a);

    std::thread racer([&] {
      auto b = std::make_shared<Packet>();
      agent.PreparePacket(*b, *second, {first.get()});
      b->work = [&ran] { ran = true; };
      second_queue->Submit(b);
      agent.WaitIdle();
    });
    racer.join();

    if (!ran) {
      Check(false, "the racing packet was neither woken nor run immediately");
      return;
    }
  }
  Pass("dependency-vs-retire race (300 iterations)");
}

// ---- A command submitted twice gets a live signal the second time ----
// The entry is removed on completion, so re-preparing the same address must
// allocate a fresh armed signal rather than hand back the finished one.
void TestCommandReuseGetsFreshSignal() {
  Agent agent(2);
  Queue* queue = agent.NewQueue();
  Ref<TestCmd> cmd(new TestCmd());

  auto first = std::make_shared<Packet>();
  agent.PreparePacket(*first, *cmd, {});
  // Retain before submitting: the packet owns the signal and dies with it, but
  // this test needs to look at the old signal after the packet is gone.
  Signal* first_done = first->done;
  first_done->retain();
  first->work = [] {};
  queue->Submit(first);
  agent.WaitIdle();
  Check(first_done->IsDone(), "the first submission's signal completed");

  auto second = std::make_shared<Packet>();
  agent.PreparePacket(*second, *cmd, {});
  Check(second->done != first_done, "the second submission got a different signal");
  Check(!second->done->IsDone(), "the second submission's signal is armed");

  second->work = [] {};
  queue->Submit(second);
  agent.WaitIdle();
  Check(second->done->IsDone(), "the second submission's signal completed");
  first_done->release();
  Pass("a reused command gets a fresh signal");
}

// ---- Independent commands never share a completion signal ----
void TestCommandsHaveDistinctSignals() {
  Agent agent(2);
  Ref<TestCmd> one(new TestCmd());
  Ref<TestCmd> two(new TestCmd());

  auto a = std::make_shared<Packet>();
  auto b = std::make_shared<Packet>();
  agent.PreparePacket(*a, *one, {});
  agent.PreparePacket(*b, *two, {});

  Check(a->done != b->done, "two commands got two signals");
  Pass("commands have distinct signals");
}

}  // namespace

int main() {
  TestFifo();
  TestParkedQueueHoldsNoWorker();
  TestNotifySubscribeRace();
  TestManyDepsDispatchOnce();
  TestConcurrentSubmit();
  TestPrepareLeavesArmedSignal();
  TestEntryOwnsCommand();
  TestCompletedDepCreatesNoEdge();
  TestPendingDepCreatesEdge();
  TestUnrunPacketRetires();
  TestChainedQueues();
  TestEdgeVersusRetireRace();
  TestCommandReuseGetsFreshSignal();
  TestCommandsHaveDistinctSignals();

  if (g_failures != 0) {
    printf("\n%d check(s) failed\n", g_failures);
    return 1;
  }
  printf("\nall tests passed\n");
  return 0;
}
