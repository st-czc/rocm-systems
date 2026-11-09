# Prototype: a software HSA for a CPU device

A self-contained stand-in for the slice of HSA that rocclr's `VirtualDevice`
implementations rely on. It gives a CPU backend the same split the ROCm backend
gets from the real HSA runtime: queue ordering, dependency waits, and
asynchronous completion live below the submit path, not inside it.

It builds and tests without any rocclr header, so the scheduling core can be
exercised on its own.

| File | Role |
|---|---|
| `hsaemu.hpp` / `hsaemu.cpp` | The runtime: `Pool`, `Queue`, `Packet`, `Signal`, `AsyncHandler` |
| `cpuvirtual.hpp` / `cpuvirtual.cpp` | Reference usage: a `VirtualDevice` mapping rocclr commands onto packets |
| `hsaemu_test.cpp` | Standalone tests of the runtime |

---

## Thread model

This is the design's organizing constraint, and the thing most likely to be
broken by a careless change.

| Thread | Count | Does |
|---|---|---|
| App | caller's | Enqueue; block on a CPU-wait marker; batch bookkeeping for that marker |
| Worker | N | **Only** `packet->task` |
| Async | 1 | Completion bookkeeping, and any user callback it triggers |

The worker pool is for compute. Nothing else may occupy a worker, because a
worker that blocks is a worker removed from the pool — and once concurrent
blockers outnumber workers, the pool deadlocks.

Completion bookkeeping is the realistic source of such blocking: advancing a
command to complete runs user callbacks inline, and a user callback may do
anything, including wait on another stream. So it runs on the async thread
instead, which executes no work and whose only hazard is delaying other
completions.

The real HSA runtime draws the same line: completion handlers run on a
dedicated async-events thread, one per process, serial, never on a thread that
executes kernels. `AsyncHandler` is that thread. Handlers run in post order and
never concurrently, so bookkeeping for two batches cannot interleave.

A consequence worth knowing: a user callback that blocks stalls every pending
completion, since the async thread is serial. The real runtime has the same
property.

---

## Objects

**`Pool`** owns the workers, the async thread, the queues, and the in-flight
event table. One per device. Its lifetime is the program's.

```cpp
explicit Pool(uint32_t workers);

Queue* newQueue(const void* owner);          // owner: opaque, the stream
void deleteQueue(const void* owner);

Packet* newPacket(Event* event, uint32_t num_tickets,
                  const EventWaitList& event_wait_list);

void postAsync(AsyncHandler::Handler handler);   // run on the async thread
```

**`Queue`** is an in-order queue, one per stream. Exactly one packet runs at a
time, in submission order.

```cpp
void submit(Packet* packet);
bool isDrained();        // nothing running, queued, or parked
```

**`Packet`** is one unit of work, divisible into `num_tickets` parts.

```cpp
std::function<void(int)> task;    // argument is the ticket; set before submit
Signal* signal;                   // fires when the packet completes
SignalWaitList signal_wait_list;  // filled in by newPacket
Event* event;                     // retained by the packet
```

**`Signal`** is one-shot: armed (1) drops to done (0) once, never back.

```cpp
void notify();              // 1 -> 0, then runs every registered callback
bool isDone() const;
void wait();                // app thread only
void onDone(Callback cb);   // runs cb when done; inline if already done
```

Reference-counted. A subscriber registered after the drop runs immediately, so
parking on a signal and then subscribing to it cannot lose a wakeup.

---

## Tickets: dividing one packet across workers

`num_tickets = N` means the work splits into N independent parts. Each worker
that picks up the packet claims a ticket and calls `task(ticket)`. Every value
in `[0, N)` is handed out exactly once, in **descending** order (N-1 first), so
partitioning logic must not assume ascending.

- `N = 1` — an indivisible command. Always runs with ticket 0.
- `N > 1` — a data-parallel kernel. Treat the ticket as a chunk index into the
  divided grid. Multiple workers run it concurrently while the queue stays
  ordered with respect to other packets.
- `N = 0` — a marker. No task runs; the packet retires as soon as it reaches the
  head of its queue. This is how a batch boundary is expressed.

Being handed out first is not the same as running first: a worker can claim
ticket N-1 and be preempted before another worker runs N-2.

---

## Dependencies

`Pool` keeps a table mapping each in-flight event to its completion signal. An
entry exists **exactly while** its command has not completed.

`newPacket` looks up every event in the wait list. An event still in the table
contributes a dependency edge; an event already gone contributes nothing,
because its dependency is already satisfied. This is what keeps a completed
dependency from producing a signal that will never fire.

When a queue's head has unmet dependencies, the **queue** parks — the worker is
returned to the pool. K dependencies cost K callbacks and zero threads. The last
dependency to complete releases the head for dispatch. The real runtime achieves
this with a hardware barrier packet; here it is bookkeeping, but the externally
visible behavior is the same.

---

## Lifetimes

**A packet is single-owner.** `submit` hands it over; the runtime deletes it on
completion. Read its fields before submitting, never after.

**To wait on a packet's signal, retain it before submitting.** The packet's own
reference goes away when the packet is deleted, which can happen the instant
`submit` returns.

```cpp
Signal* done = packet->signal;
done->retain();
queue->submit(packet);
done->wait();
done->release();
```

**`newPacket` must be followed by `submit`.** The event's table entry is created
by `newPacket`, and only completion removes it. A packet that is created and
dropped leaves an entry behind, and anything depending on that event parks
forever.

**Set `task` before submitting** whenever `num_tickets >= 1`.

**`wait()` is for the app thread.** A worker that blocks there is a worker lost.

**A queue outlives its stream when it must.** `deleteQueue` destroys the queue
if it has drained, and otherwise keeps it alive until it does — a parked head's
callbacks hold a pointer to the queue, so destroying it early would leave them
pointing at freed memory.

---

## Integrating a virtual device

`cpuvirtual.cpp` is the worked example. The shape, independent of any particular
backend:

- **Construction** takes a `Pool&` and calls `newQueue(this)`; **destruction**
  calls `deleteQueue(this)`. The owner key is opaque to the runtime, so the
  stream identity never leaks downward.
- **An ordinary command** becomes a 1-ticket packet carrying the command and its
  `eventWaitList()`.
- **A kernel** becomes an N-ticket packet, N from the launch geometry.
- **A marker** becomes a 0-ticket packet, and splits two ways:
  - *CPU wait requested* — retain the signal, submit, wait, then do the batch
    bookkeeping on the app thread.
  - *otherwise* — subscribe to the signal and have the subscription **post** the
    bookkeeping to the async thread. The subscription itself may fire on any
    thread, so it must do nothing but post.
- **`flush` is a no-op.** Submission happens at enqueue time; nothing is
  batched in the virtual device.

Command status is advanced in bulk from the batch head when its marker
completes, matching how the ROCm backend does it — each command in the batch is
walked, advanced, and released once.

---

## State

Working and exercised by the standalone tests: in-order dispatch per queue,
ticket fan-out and uniqueness, dependency edges and parking, marker retirement,
concurrent submission across queues, and the completion/subscription races.

Structurally in place: queue lifetime tied to stream lifetime; completion
bookkeeping off the worker pool.

Not done:

- **Real command execution.** `simulateWork` is a sleeping placeholder, and its
  per-ticket `printf` is debug scaffolding. This is the main remaining work and
  it belongs in the backend, not here.
- **Ticket count from launch geometry.** The example hardcodes 32 for kernels.
- **Profiling timestamps.** Every command in a batch currently receives one
  shared timestamp for both running and complete, so durations read as zero.
  Deferred deliberately. The intended fix is to record times per packet as it
  runs and apply them during the batch walk.

Loose ends: `newQueue` gained its `owner` parameter after the tests were
written, so the test file's call sites need updating; `AsyncHandler::inHandler()`
exists with no caller, awaiting a deferral path that needs it.

---

## Deliberate choices

Easy to mistake for bugs:

- **Shutdown drops queued packets.** Once the pool is shutting down, unstarted
  work is work no longer wanted. Pending async handlers, by contrast, are
  drained — the work they describe already happened, and skipping their
  bookkeeping would leave commands unreleased.
- **`Signal` has no condition variable.** Only a few signals are ever waited on,
  so `wait()` builds its own notification out of a shared promise rather than
  every signal carrying a member for the rare case. The promise is owned by the
  callback, which is what keeps it alive across `set_value`.
- **`notify()` is not idempotent.** A packet's signal is notified exactly once,
  by its own completion. The guard was removed as dead weight.
- **Queue destruction can be deferred rather than immediate.** See Lifetimes.

---

## Building the tests

The standalone build substitutes a local reference-counted base for rocclr's,
so no rocclr headers are needed. Run from the rocclr source root:

```bash
g++ -std=c++17 -DPROTOTYPE_STANDALONE -I. -pthread -fsanitize=thread \
    device/prototype/hsaemu.cpp device/prototype/hsaemu_test.cpp -o /tmp/hsaemu_test
/tmp/hsaemu_test
```

Swap `-fsanitize=thread` for `-fsanitize=address` to check allocation and
lifetime instead. Both are worth running: the invariants here are about
concurrency and ownership, and each sanitizer sees only one of those.

`cpuvirtual.cpp` is not part of this build — it needs rocclr headers and only
compiles inside the real tree.
