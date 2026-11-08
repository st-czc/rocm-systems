# Prototype HSA 软件仿真 — 接口与实现

> 供另一台电脑上的开发使用。描述 `device/prototype/` 的对外接口与内部实现，
> 目标是把它作为 `device/cpu` 的线程池来使用。

## 1. 概述

`device/prototype/` 是 rocclr ROCm 后端所依赖的 HSA 子集的软件替身。它自包含
（不依赖 rocclr 头文件即可独立测试），提供四样东西：

| 组件     | 对应概念        | 作用 |
|----------|-----------------|------|
| `Pool`   | worker 线程池   | 固定 N 个 worker 线程，执行 packet |
| `Queue`  | `hsa_queue_t`   | 有序队列，同一时刻只运行一个 packet |
| `Packet` | AQL packet      | 一个可拆分的工作单元 |
| `Signal` | `hsa_signal_t`  | 一次性完成信号 |

核心设计：一个逻辑任务（如 kernel）可被**拆成多份**（ticket 机制）交给多个 worker
并行执行，同时保持队列内的先后顺序与依赖等待。

## 2. 文件

- `hsaemu.hpp` / `hsaemu.cpp` — 核心实现（Pool / Queue / Packet / Signal）。
- `cpuvirtual.hpp` / `cpuvirtual.cpp` — rocclr 命令到 packet 的映射（`VirtualCPU`）。
- `hsaemu_test.cpp` — 独立测试（11 个用例）。

## 3. 类型与公共接口

### 3.1 类型别名

```cpp
// 独立构建（PROTOTYPE_STANDALONE）：
using Event = amd::ReferenceCountedObject;
using EventWaitList = std::vector<Event*>;
// 实际构建（接入 rocclr，hsaemu.hpp 包含 platform/command.hpp）：
//   Event 通过名字查找解析为 amd::Event，
//   using EventWaitList = Event::EventWaitList;

using SignalWaitList = std::vector<Signal*>;
using Ticket = std::optional<uint32_t>;   // nullopt 表示票已发完
```

### 3.2 Pool — 线程池

```cpp
class Pool {
 public:
  explicit Pool(uint32_t workers);   // 创建 workers 个线程
  ~Pool();                           // 置 shutdown 并 join 所有 worker

  void submit(Packet* packet);       // 把 packet 投入就绪队列（非阻塞）
  Queue* newQueue();                 // 新建一个有序队列（由 Pool 持有）
  Packet* newPacket(Event* event, uint32_t num_tickets,
                    const EventWaitList& event_wait_list);
  void donePacket(Packet* packet);   // 完成一个 packet（会 delete packet）
};
```

### 3.3 Queue — 有序队列

```cpp
class Queue {
 public:
  void submit(Packet* packet);  // 入队；若队列空闲则立即调度
};
```

### 3.4 Packet — 可拆分工作单元

```cpp
struct Packet {
  std::function<void(int)> task;   // 工作函数，参数是 ticket 编号（submit 前必须设置）
  Signal* signal;                  // 完成信号（packet 完成时被 notify）
  SignalWaitList signal_wait_list; // 依赖的完成信号（由 newPacket 填充）
  Event* event;                    // 关联的命令/事件（packet 持有其引用）
  std::atomic<uint32_t> num_owners;
  std::atomic<uint32_t> num_tickets;
  Ticket getTicket();              // CAS 发一张票
};
```

### 3.5 Signal — 一次性完成信号

```cpp
class Signal : public amd::ReferenceCountedObject {
 public:
  void notify();               // 1 -> 0，并执行所有已注册的回调
  bool isDone() const;         // value_ == 0
  void wait();                 // 阻塞直到 done（worker 禁止调用）
  void onDone(Callback cb);    // done 时执行 cb（已 done 则立即执行）
};
```

## 4. 实现细节

### 4.1 Ticket 模型（可拆分执行）

`Packet::num_tickets = N` 表示该任务可拆成 N 份。`getTicket()` 用 CAS 把
`num_tickets` 依次减 1，返回 `N-1, N-2, ..., 0`（每个值恰好一次），发完后返回
`nullopt`。于是 `task` 会被调用 N 次，参数依次是这些 ticket 编号。N=1 时只有
ticket 0，等价于不可拆分任务。

### 4.2 Owners 模型（所有权倒计数）

`num_owners` 初值 1（代表 packet 在 `ready_packets_` 里的"基引用"）。一个 worker
抢到非零票时 `num_owners += 1` 且不把 packet 从就绪队列弹出；抢到 0 号票（或发现已
发完、返回 nullopt）的 worker 负责弹出（接管基引用）。总的 `fetch_sub(1)` 次数 =
1 + 非零票增量，所以恰好有一次返回 1，那一次执行 `donePacket`。

### 4.3 Queue 语义（有序 + 依赖 park）

- `busy_` 表示当前有一个 packet 在跑。只有 busy_ 为假且队头依赖就绪时才派发。
- 队头有未完成的依赖（`signal_wait_list` 非空）时，队列 park（`num_waiting_` 记录
  未就绪的依赖数），worker 被释放去做别的事。
- 每个依赖 signal 注册 `onDone` 回调；最后一个依赖就绪时置 `waiting_ready_` 并重新
  提交，派发队头。

### 4.4 Signal 语义（一次性 + park-then-subscribe）

`value_` 从 armed(1) 变 done(0) 只发生一次。`onDone` 用双重检查（快路径 + 锁内重查）
保证"已 done 立即执行、未 done 注册回调"，无丢失唤醒。`notify` 先在锁内置 0，再在
锁外遍历 `callbacks_` 执行（置 0 之后 `onDone` 必走内联、不再 push，所以遍历安全），
最后 `clear()`。

### 4.5 锁拆分

Pool 有三把锁，互不嵌套、无死锁：

| 锁 | 保护对象 |
|----|----------|
| `queue_lock_` | `queues_` |
| `event_lock_` | `event_signals_` |
| `packet_lock_` | `ready_packets_` + `shutdown_`（与 `cv_` 配对） |

Queue 自身还有一把 `lock_` 保护 `packets_` / `busy_` / `num_waiting_` /
`waiting_ready_`。

### 4.6 续延（continuation）

一个 worker 完成当前 packet（成为最后一个 owner）后，直接 `donePacket` 然后
`queue->nextTaskPacket(true)` 拿下一个 packet 继续执行，避免重新提交的往返。

### 4.7 Packet 生命周期

- `Packet` 构造函数 `retain(event)` 并 `new Signal`；析构函数 `release(signal)` 和
  `release(event)`。所以 packet 持有 event 和 signal 各一个引用。
- `donePacket` 顺序：`event_signals_.erase(event)` → `signal->notify()` →
  `delete packet`（析构释放 signal 和 event）。

## 5. 作为线程池使用

```cpp
using namespace amd::device::prototype;

Pool pool(4);                       // 4 个 worker 线程
Queue* queue = pool.newQueue();

// 不可拆分任务（num_tickets = 1，ticket 只有 0）
Packet* p = pool.newPacket(event, 1, {});
p->task = [](int ticket) { /* 干活 */ };
queue->submit(p);

// 可拆分任务（num_tickets = 32，并行 32 份，ticket = 31..0）
Packet* k = pool.newPacket(event2, 32, {});
k->task = [](int ticket) { /* 用 ticket 干活 */ };
queue->submit(k);

// 等待完成：先 retain，因为 packet 完成后会被 delete
Signal* done = k->signal;
done->retain();
done->wait();
done->release();
```

## 6. 与 device/cpu 集成

- **Event 抽象**：独立构建时 `Event = ReferenceCountedObject`；接入 rocclr 时
  `Event` 解析为 `amd::Event`（`platform/command.hpp`）。`newPacket` 的第一个参数是
  要跟踪的命令/事件，packet 会 `retain` 它直到完成。
- **依赖**：`event_signals_` 是 `event -> Signal*` 的映射（每在飞命令一条）。提交
  packet 时，`newPacket` 查 wait_list 里每个 event 的 signal，retain 并放进
  `signal_wait_list`；若 event 已完成（不在 map 里）则无依赖边。
- **参考实现**：`cpuvirtual.cpp` 里的 `submit` / `submitKernel` / `submitMarker`
  展示了如何把 rocclr 命令映射成 packet（kernel 用 32 票，普通命令 1 票，marker
  0 票）。

## 7. 注意点

1. `num_tickets >= 1` 的 packet **必须**在 submit 前设置 `task`，否则抛
   `bad_function_call`。
2. 等 signal 前**必须 retain**（`donePacket` 会 delete packet）。
3. `wait()` 只能由非 worker 线程调用（worker 阻塞会导致死锁）。
4. `nextPacket` / `nextTaskPacket` / `donePacket` 是内部接口；外部只用
   `newQueue` / `newPacket` / `submit` / `wait` / `onDone`。

## 8. 测试

独立构建（`-DPROTOTYPE_STANDALONE`）：

```bash
g++ -std=c++17 -DPROTOTYPE_STANDALONE -I. -pthread -fsanitize=thread \
  device/prototype/hsaemu_test.cpp device/prototype/hsaemu.cpp -o /tmp/t
```

11 个用例覆盖：单票 FIFO、多票 fanout、跨 packet 票连续性、依赖边、marker、
并发提交、getTicket CAS 竞争、续延只跑一次、多票并行度、多依赖 park。

---

*如果在新机器上用 Claude Code 继续开发，可参考 `code-review`（审查改动）与
`diagnosing-bugs`（定位问题）两个 skill。*
