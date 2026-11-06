# Prototype Handoff

## What this directory is

A self-contained software-HSA emulation that backs a prototype `VirtualCPU` for
the rocclr `AMD_DIRECT_DISPATCH` path. It deliberately avoids touching
`device/cpu` and is tested independently of the full ROCm build.

Files:

| File | Role |
|---|---|
| `hsaemu.hpp` / `hsaemu.cpp` | `Signal`, `Packet`, `Queue`, `Agent`, `AgentTask` |
| `cpuvirtual.hpp` / `cpuvirtual.cpp` | `VirtualCPU` — maps rocclr commands onto packets |
| `hsaemu_test.cpp` | 11 standalone unit tests (no rocclr headers needed) |

---

## Status

All 11 tests pass under both TSan and ASan/LSan. No data races, no leaks.

Tests:

- fifo
- pending dependency creates an edge
- completed dependency creates no edge
- parked queue releases its worker
- subscribe/completion race (200 iterations)
- many deps (5 released one by one, one dispatch)
- concurrent submit (30 iterations × 4 threads)
- cross-queue chain (100 iterations)
- a reused command gets a fresh signal
- commands have distinct signals
- dependency-vs-completion race (200 iterations)

---

## Key design decisions

### `event_signals_` protocol

`Agent::event_signals_` maps `const amd::ReferenceCountedObject* → Signal*`.
An entry is present **if and only if** the command has not yet completed.

- `NewPacket(cmd, wait_list)`: retains `cmd` (packet holds that reference);
  creates a fresh `Signal* done`; for each entry in `wait_list` that is still in
  `event_signals_`, retains its signal and adds it to `packet->deps`; inserts
  `cmd → done` into the map. All under `lock_`.
- `DonePacket(packet)`: erases `cmd` from the map (under `lock_`), then
  `done->Notify()`, then `cmd->release()`, then `done->release()`, then
  `delete packet`.  Order matters: erase before Notify so a concurrent
  `NewPacket` for a reuse of the same command address sees no entry and creates a
  fresh signal (never inherits a stale one).

Consequence: if `cmd2` depends on `cmd1` but `cmd1` has already completed and
been erased from the map, `cmd2` gets **no dep** for that entry — which is
correct because the dependency is already satisfied.

### `Queue::Pump` and the `num_waiting_` / `waiting_ready_` state machine

`Queue` is in-order. `Pump` is the only place that moves the head forward.

- `busy_` — a packet is currently executing on a worker.
- `num_waiting_` — number of dep subscriptions that have not yet fired.  
  Non-zero means the queue is parked.
- `waiting_ready_` — all subscriptions fired; the head can proceed without
  re-scanning `IsDone()`.

Invariant: `num_waiting_ != 0` and `waiting_ready_` are mutually exclusive.

`Pump` flow:

1. Under `lock_`: bail if `busy_`, empty, or `num_waiting_ != 0`.
2. If `waiting_ready_`: release all remaining dep references, clear `deps`,
   clear `waiting_ready_`.
3. Else: scan `deps`, release those that are `IsDone()`, compact survivors.
4. If `deps` is now empty: pop packet, set `busy_ = true`, `dispatch = true`.
5. Else: set `num_waiting_ = deps.size()`, then (outside lock) register
   `OnDone` callbacks — each decrements `num_waiting_`; the last one sets
   `waiting_ready_ = true` and calls `Pump()`.

The callback loop uses an **index-based** iteration with a pre-captured
`num_waiting` because the last callback fires `Pump()` which might call
`deps.clear()` under the queue lock, invalidating a range-for iterator.

### `AgentTask` continuation

`AgentTask` wraps `std::function<AgentTask(bool isAgent)>`.

When a worker calls `task(true)`, it may receive the next ready task back
instead of re-posting to `agent_.Submit`. `Agent::Loop` drives this as
`while (task) { task = task(true); }`, avoiding a redundant lock + notify for
sequential ready packets.

`isAgent == false` paths (called from the submitting thread, not a worker):
- If `packet->work` is non-null, post to `agent_.Submit`.
- If `packet->work` is null (marker, zero-dep), execute inline — may recurse
  for a chain of consecutive ready markers.

### `submitMarker` paths

```
CpuWaitRequested == true:
    done->retain()         // extra ref so packet→done stays alive past DonePacket
    queue_->Submit(packet)
    done->Wait()           // submitting thread blocks
    done->release()
    UpdateBatchState(head) // called on submitting thread, after all deps done

CpuWaitRequested == false:
    done->OnDone([head] { UpdateBatchState(head); })  // async, no block
    queue_->Submit(packet)
```

`done->retain()` must happen **before** `Submit`, not after, because the worker
can call `DonePacket` (and thus `done->release()`) the moment `Submit` returns.

### `Signal::Notify` ordering

```
lock → store 0 → swap waiters_ out → unlock → cv_.notify_all() → run waiters
```

Waiters run outside the lock to avoid deadlock: an `OnDone` callback often calls
`Pump()` which acquires `queue_->lock_`. This is safe because `waiters_` has
already been swapped out under the signal's lock; no other thread can see or
mutate the local copy.

### Lifetime rules

- `Signal` starts at refcount 1 (from `new`). `NewPacket` retains it for
  each `deps` slot. `DonePacket` releases the `done` slot. Each dep slot is
  released in the `waiting_ready_` branch of `Pump` (not in callbacks).
- `cmd` starts at refcount 1 (from whoever created the command). `NewPacket`
  retains it once; `DonePacket` releases that one retain.
- `~Agent` / `~Queue` have no cleanup for in-flight packets. This is acceptable
  because `Agent`'s lifetime is the whole program.

---

## How to build and run the tests

The test file defines `PROTOTYPE_STANDALONE` to get the local stub for
`amd::ReferenceCountedObject` (no rocclr headers needed).

`hsaemu.cpp` includes `"device/prototype/hsaemu.hpp"`, so `-I` must be the
rocclr source root (the directory containing `device/`). Run from that root:

```bash
# TSan
g++ -std=c++17 -DPROTOTYPE_STANDALONE -I. -fsanitize=thread \
    -o hsaemu_test device/prototype/hsaemu.cpp device/prototype/hsaemu_test.cpp \
    -pthread && ./hsaemu_test

# ASan + LSan
g++ -std=c++17 -DPROTOTYPE_STANDALONE -I. -fsanitize=address \
    -o hsaemu_test device/prototype/hsaemu.cpp device/prototype/hsaemu_test.cpp \
    -pthread && ./hsaemu_test
```

`cpuvirtual.cpp` is not part of the standalone test build — it needs rocclr
headers (`device/device.hpp`, `platform/command.hpp`) and only compiles inside
the real build tree.

---

## Known limitations / not-yet-done

- `~Agent` and `~Queue` do not drain in-flight work. Safe for program-lifetime
  agents; would need a drain loop for earlier teardown.
- `VirtualCPU::Submit` uses `SimulateWork()` (random sleep). Real memory/kernel
  operations are not wired up yet.
- No CMakeLists integration for the prototype tests; they are built by hand.

---

## Next natural step

Integrate `VirtualCPU` + `Agent` into the actual `device/cpu` build path,
replacing or sitting alongside the existing `cpu` implementation. The prototype
confirms the ordering and lifetime model; the next session should wire up real
command execution in place of `SimulateWork()`.

---

## Suggested skills for the next agent

Call the `Skill` tool for these when they match the work:

| Skill | When |
|---|---|
| `mattpocock-skills:code-review` | Before committing the integration into `device/cpu` |
| `mattpocock-skills:diagnosing-bugs` | If the emulation misbehaves under TSan after integration |
| `mattpocock-skills:domain-modeling` | Before changing the `Signal`/`Packet`/`Queue`/`Agent` vocabulary |
| `mattpocock-skills:prototype` | If a new slice of the design needs the same throwaway treatment |

---

## Conventions this work followed

- `device/rocm` is the golden reference for the contract `device/cpu` must
  satisfy; rocclr's own APIs are fixed and not modifiable.
- Discussion before code. Design changes are proposed, reviewed, then applied.
- Tests never bind to non-public implementation details; they use only
  `NewQueue`, `NewPacket`, `Submit`, `WaitIdle`.
- Build with both TSan and ASan before declaring done.
