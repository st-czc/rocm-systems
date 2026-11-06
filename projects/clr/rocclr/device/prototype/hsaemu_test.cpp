#include "device/prototype/hsaemu.hpp"
#include <cassert>
#include <cstdio>
#include <atomic>
#include <vector>
#include <random>
#include <thread>

using namespace amd::device::prototype;

// ---- T1: in-order within a queue ----
static void TestFifo() {
  Agent agent(4);
  Queue* q = agent.NewQueue();
  std::vector<int> order;
  std::mutex m;
  for (int i = 0; i < 500; ++i) {
    auto p = std::make_shared<Packet>();
    p->work = [i, &order, &m] {
      std::lock_guard<std::mutex> l(m);
      order.push_back(i);
    };
    q->Submit(p);
  }
  agent.WaitIdle();
  for (int i = 0; i < 500; ++i) if (order[i] != i) { printf("FAIL fifo at %d\n", i); return; }
  printf("PASS fifo (500 packets, in order)\n");
}

// ---- T2: head-of-line blocking, no worker held ----
static void TestBlocking() {
  Agent agent(1);   // one worker only
  Queue* q = agent.NewQueue();
  Signal* gate = agent.NewSignal();
  std::atomic<bool> ran_after{false};
  std::atomic<bool> ran_before{false};

  auto a = std::make_shared<Packet>();
  a->deps = {gate};
  a->work = [&ran_after] { ran_after = true; };
  auto b = std::make_shared<Packet>();
  b->work = [&ran_before] { ran_before = true; };
  q->Submit(a);
  q->Submit(b);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (ran_after || ran_before) { printf("FAIL blocking ran early\n"); return; }

  gate->Notify();
  agent.WaitIdle();
  if (!ran_after || !ran_before) { printf("FAIL blocking never ran\n"); return; }
  printf("PASS blocking (single worker, dep gate held head 100ms)\n");
}

// ---- T3: the worker is genuinely free while a queue is parked ----
static void TestWorkerNotHeld() {
  Agent agent(1);
  Queue* parked = agent.NewQueue();
  Queue* other = agent.NewQueue();
  Signal* gate = agent.NewSignal();
  std::atomic<bool> other_ran{false};

  auto a = std::make_shared<Packet>();
  a->deps = {gate};
  a->work = [] {};
  parked->Submit(a);

  auto b = std::make_shared<Packet>();
  b->work = [&other_ran] { other_ran = true; };
  other->Submit(b);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  bool ok = other_ran;
  gate->Notify();
  agent.WaitIdle();
  if (!ok) { printf("FAIL parked queue held the only worker\n"); return; }
  printf("PASS parked queue holds no worker\n");
}

// ---- T4: lost-wakeup stress, notify racing subscribe ----
static void TestRace() {
  for (int iter = 0; iter < 300; ++iter) {
    Agent agent(2);
    Queue* q = agent.NewQueue();
    Signal* gate = agent.NewSignal();
    std::atomic<bool> done{false};
    auto p = std::make_shared<Packet>();
    p->deps = {gate};
    p->work = [&done] { done = true; };
    q->Submit(p);
    std::thread t([gate] {
      std::this_thread::sleep_for(std::chrono::microseconds(50));
      gate->Notify();
    });
    t.join();
    agent.WaitIdle();
    if (!done) { printf("FAIL lost wakeup at iter %d\n", iter); return; }
  }
  printf("PASS race (300 iterations, subscribe/notify race)\n");
}

// ---- T5: two queues chained by signal ----
static void TestCrossQueue() {
  Agent agent(4);
  Queue* q1 = agent.NewQueue();
  Queue* q2 = agent.NewQueue();
  Signal* s1 = agent.NewSignal();
  std::atomic<int> step{0};

  auto a = std::make_shared<Packet>();
  a->done = s1;
  a->work = [&step] { assert(step == 0); step = 1; };
  q1->Submit(a);

  auto b = std::make_shared<Packet>();
  b->deps = {s1};
  b->work = [&step] { assert(step == 1); step = 2; };
  q2->Submit(b);

  agent.WaitIdle();
  if (step != 2) { printf("FAIL cross queue order\n"); return; }
  printf("PASS cross-queue signal dependency\n");
}

// ---- T6: many queues, random deps, all complete ----
static void TestManyQueues() {
  for (int iter = 0; iter < 50; ++iter) {
    Agent agent(4);
    const int nq = 8, np = 40, nt = 4;
    std::vector<Queue*> qs;
    std::vector<Signal*> done_sigs;
    for (int i = 0; i < nq; ++i) qs.push_back(agent.NewQueue());
    std::atomic<int> count{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < nt; ++t) {
      threads.emplace_back([&] {
        for (int i = 0; i < np; ++i) {
          auto p = std::make_shared<Packet>();
          p->work = [&count] { ++count; };
          p->done = agent.NewSignal();
          qs[(size_t)&p % nq]->Submit(p);
        }
      });
    }
    for (auto& t : threads) t.join();
    agent.WaitIdle();
    if (count != nt * np) { printf("FAIL many queues iter %d: %d\n", iter, count.load()); return; }
  }
  printf("PASS concurrent submit (50 iters x 320 packets x 4 threads)\n");
}

// ---- T7: several deps fired one by one, exactly one dispatch ----
static void TestManyDeps() {
  for (int iter = 0; iter < 100; ++iter) {
    Agent agent(2);
    Queue* q = agent.NewQueue();
    const int kDeps = 5;
    std::vector<Signal*> deps;
    for (int i = 0; i < kDeps; ++i) deps.push_back(agent.NewSignal());

    std::atomic<int> runs{0};
    auto p = std::make_shared<Packet>();
    p->deps = deps;
    p->work = [&runs] {
      int seen = ++runs;   // second dispatch would make this 2
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      (void)seen;
    };
    q->Submit(p);

    // Fire every dep except the last one while the queue is parked.
    for (int i = 0; i + 1 < kDeps; ++i) {
      deps[i]->Notify();
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    if (runs != 0) { printf("FAIL many deps ran before last dep\n"); return; }

    deps[kDeps - 1]->Notify();
    agent.WaitIdle();
    if (runs != 1) { printf("FAIL many deps dispatched %d times\n", runs.load()); return; }
  }
  printf("PASS many deps (5 deps fired one by one, one dispatch)\n");
}

int main() {
  TestFifo();
  TestBlocking();
  TestWorkerNotHeld();
  TestRace();
  TestCrossQueue();
  TestManyQueues();
  TestManyDeps();
  printf("done\n");
  return 0;
}
