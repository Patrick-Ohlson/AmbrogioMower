# Active Object / HSM Framework — ao

## Module Summary

Clean-room reimplementation of the L200's custom Active Object / Hierarchical
State Machine framework, originally from `lib/active/`. This is **not** QP/C —
it is an independent framework with unique design choices:

- **Signal = function pointer**: Events carry the ao_post_* function address as signal
- **Single-byte pool handles**: Events are referenced by a handle byte, not raw pointers
- **Sentinel addresses for ENTRY/INIT/EXIT**: Three adjacent 4-byte variables, all
  0xDEADBEEF, distinguished by address
- **State descriptor tables**: Const structs with handler, parent pointer, state_data_size
- **Cooperative priority scheduler**: 16-level priority bitmask, immediate dispatch

| Property | Value |
|----------|-------|
| Framework | Custom AO/HSM (not QP/C) |
| Pool system | SmallHeap — up to 8 pools × 32 blocks, 1-byte handles |
| Event min size | 6 bytes (signal + next + flags) |
| Scheduler | 16 priority levels, cooperative |
| Timer | Doubly-linked list, one-shot + repeating |
| Source doc | `STATES/state_ActiveObject.md` |

## Files

| File | Description |
|------|-------------|
| `ao.h` | Umbrella header — includes all framework headers |
| `ao.c` | Framework init convenience (`ao_FrameworkInit`) |
| `smallheap.h` | Pool allocator API (handles, descriptors) |
| `smallheap.c` | Pool allocator implementation |
| `queue.h` | Event queue API (handle-based FIFO) |
| `queue.c` | Event queue implementation |
| `hsm.h` | HSM engine, AO struct, timer, event types + API |
| `hsm.c` | HSM dispatch, transitions, LCA, enter/exit chains |
| `schedulerrt.h` | Priority-based cooperative scheduler API |
| `schedulerrt.c` | Scheduler implementation (priority table + ready set) |
| `error.h` | Error reporting API with ASSERT macro |
| `error.c` | Error buffer and UART output (TestFirmware mode) |
| `ao_doc.md` | This file |
| `tests/test_ao.c` | Interactive test (menu key I) |

## How to Include

```c
#include "active_object/ao.h"     /* includes all framework headers */
```

```makefile
OBJS += active_object/smallheap.o active_object/queue.o \
        active_object/hsm.o active_object/schedulerrt.o \
        active_object/error.o active_object/ao.o
```

### Dependencies

| Dependency | Required For |
|------------|-------------|
| `<stdint.h>` | Fixed-width types |
| `<stdbool.h>` | Timer repeat flag |
| `<string.h>` | memset in ao.c |
| `sci.h` | SendString — UART error output (TestFirmware mode) |

## Architecture

```
Pool (SmallHeap) → Event Allocation (evt_Alloc)
                       ↓
                   Queue (FIFO)
                       ↓
              Active Object (ao_t)
                       ↓
                  HSM Engine (hsm_Dispatch)
                       ↓
                  State Handlers
                       ↓
              Scheduler (ao_Scheduler)
                       ↓
              Timer (timer_Tick → timer_Fire)
```

### Pool System (SmallHeap)

Fixed-block allocator with up to 8 pools. Each pool has uniform block size.
Blocks are referenced by single-byte handles:

```
handle = ~(pool_index << 5 | block_index)
pool_index  = (~handle) >> 5        (0-7)
block_index = (~handle) & 0x1F      (0-31)
handle 0x00 = POOL_HANDLE_NULL
```

Free blocks form a singly-linked list through each block's first byte.
Pools must be registered in ascending order of block_size.

Firmware pool config (from OldMain):
- Pool 0: 24-byte blocks × 16 — small events (6–22 bytes)
- Pool 1: 48-byte blocks × 8 — large events / config data

### Event System

Events have a 6-byte header:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| +0x00 | 4 | signal | Function pointer (event constructor address) |
| +0x04 | 1 | next | Queue chain link (handle) |
| +0x05 | 1 | flags | Reserved |
| +0x06 | N | payload | Event-specific data |

The signal field stores the address of the `ao_post_*` function that created
the event. State handlers compare `evt->signal == (signal_t)&ao_post_motor_run`
to identify the event type.

### HSM Engine

Hierarchical State Machine with:
- **State descriptors**: Const structs with `handler`, `parent`, `state_data_size`
- **Handler returns**: `HSM_HANDLED` (consumed), `HSM_UNHANDLED` (pass to parent),
  `HSM_TRAN(&target)` (transition)
- **Sentinel signals**: `HSM_SIG_ENTRY`, `HSM_SIG_INIT`, `HSM_SIG_EXIT` —
  three static uint32_t variables (all 0xDEADBEEF), distinguished by address
- **LCA transitions**: Compute Least Common Ancestor, exit up, enter down, init chain

### Scheduler

16-level cooperative scheduler using a `ready_set` bitmask:
- `ao_SignalReady(priority)` sets bit and calls `ao_Scheduler()` if idle
- `ao_Scheduler()` iterates priorities (lowest number first = highest priority)
- Each priority level has a linked list of AOs

### Timer

Doubly-linked list of `ao_timer_t` structures:
- `ao_timer_Set()` — add, modify, or remove a timer
- `ao_timer_Tick()` — called from ISR, decrements all active timers
- `ao_timer_Fire()` — posts signal to target AO when timeout reaches 0
- One-shot (reload=0) or repeating (reload=N)

## API Reference

### Pool System (smallheap.h)

| Function | Firmware Addr | Description |
|----------|--------------|-------------|
| `pool_Init(desc_array, count)` | 0x16BC0 | Initialize pool descriptor array |
| `pool_Register(mem, block_size, num_blocks)` | 0x16C30 | Register a fixed-block pool |
| `pool_Alloc(size)` → `handle_t` | 0x16DC2 | Allocate from smallest suitable pool |
| `pool_Resolve(handle)` → `void*` | 0x170AA | Decode handle to memory pointer |
| `pool_Free(handle)` | 0x16F2C | Return block to free-list |

### Queue System (queue.h)

| Function | Firmware Addr | Description |
|----------|--------------|-------------|
| `queue_Init(q)` | 0x1808A | Initialize queue to empty |
| `queue_Enqueue(q, handle)` | 0x18094 | Append event to queue tail |
| `queue_Dequeue(q)` → `handle_t` | 0x180C4 | Pop event from queue head |

### Event Allocation (hsm.h)

| Function | Firmware Addr | Description |
|----------|--------------|-------------|
| `evt_Alloc(signal, size)` → `handle_t` | 0x16304 | Allocate + init event |
| `ao_PostTransition(ao, signal)` | 0x16858 | Post minimal 6-byte state event |

### HSM Engine (hsm.h)

| Function | Firmware Addr | Description |
|----------|--------------|-------------|
| `hsm_Constructor(ao, initial_state)` | 0x16628 | Create + enter HSM initial state |
| `hsm_Dispatch(ao, event)` | 0x16652 | Dispatch event through state hierarchy |
| `hsm_EnterState(ao, target)` | — | Enter a state (recursive parent-first) |
| `hsm_InitChain(ao)` | — | Process INIT transition chain |
| `hsm_Transition(ao, target)` | — | Full LCA state transition |
| `hsm_ExitChain(ao, target)` | — | Exit states up to ancestor |
| `hsm_CalcDepth(state)` → `int16_t` | — | Calculate state nesting depth |

### Active Object (hsm.h)

| Function | Firmware Addr | Description |
|----------|--------------|-------------|
| `ao_Constructor(ao, priority)` | 0x16740 | Initialize AO instance |
| `ao_ProcessQueue(ao)` | 0x167EE | Dequeue + dispatch all pending events |
| `ao_PostEvent(ao, handle)` | 0x1683E | Route event to AO queue |
| `ao_EnqueueAndSignal(ao, handle)` | 0x16772 | Enqueue + set scheduler ready bit |

### Scheduler (schedulerrt.h)

| Function | Firmware Addr | Description |
|----------|--------------|-------------|
| `sched_Init()` | — | Initialize priority table + ready set |
| `ao_Start(priority, ao, initial_state)` | 0x16984 | Register AO + construct HSM |
| `ao_Scheduler()` | 0x1688A | Run cooperative priority scheduler |
| `ao_SignalReady(priority)` | 0x16A18 | Signal scheduler that AO has events |

### Timer (hsm.h)

| Function | Firmware Addr | Description |
|----------|--------------|-------------|
| `ao_timer_Set(tmr, ao, callback, timeout, repeat)` | 0x16AC0 | Add/modify/remove timer |
| `ao_timer_Fire(tmr)` | 0x16A5E | Called on timer expiry |
| `ao_timer_Tick()` | — | Advance all timers by one tick |

> **Note**: Named `ao_timer_*` to avoid conflict with the standalone `timer/timer.h`
> module which provides a simpler `timer_Set()` for non-AO use.

### Error (error.h)

| Function | Firmware Addr | Description |
|----------|--------------|-------------|
| `error_set(msg)` | — | Record error, print via UART |
| `error_clear()` | — | Clear error flag |
| `error_active()` → `int` | — | Check if error is latched |
| `ASSERT(cond, msg)` | — | Conditional error_set macro |

### Convenience (ao.h)

| Function | Description |
|----------|-------------|
| `ao_FrameworkInit()` | error_clear + pool_Init(2 pools) + sched_Init |

## Firmware Traceability

### OldMain Initialization Sequence

The `ao_FrameworkInit()` function replicates the OldMain Phase 2+3 sequence:

| Step | Firmware | Reimplementation |
|------|----------|-----------------|
| Clear error flag | RealBoot → unk_FFFF00 = 0 | `error_clear()` |
| Init pool system | pool_Init(2) | `pool_Init(g_pool_descs, 2)` |
| Register pool 0 | 24B × 16 blocks | `pool_Register(g_pool0_mem, 24, 16)` |
| Register pool 1 | 48B × 8 blocks | `pool_Register(g_pool1_mem, 48, 8)` |
| Init scheduler | sched_Init() | `sched_Init()` |

### Firmware Function Addresses

| Function | Address | Source File |
|----------|---------|-------------|
| pool_Init | 0x16BC0 | smallheap.c |
| pool_Register | 0x16C30 | smallheap.c |
| pool_Alloc | 0x16DC2 | smallheap.c |
| pool_Resolve | 0x170AA | smallheap.c |
| pool_Free | 0x16F2C | smallheap.c |
| queue_Init | 0x1808A | queue.c |
| queue_Enqueue | 0x18094 | queue.c |
| queue_Dequeue | 0x180C4 | queue.c |
| evt_Alloc | 0x16304 | hsm.c |
| hsm_Constructor | 0x16628 | hsm.c |
| hsm_Dispatch | 0x16652 | hsm.c |
| ao_Constructor | 0x16740 | hsm.c |
| ao_ProcessQueue | 0x167EE | hsm.c |
| ao_PostEvent | 0x1683E | hsm.c |
| ao_EnqueueAndSignal | 0x16772 | hsm.c |
| ao_PostTransition | 0x16858 | hsm.c |
| ao_Start | 0x16984 | schedulerrt.c |
| ao_Scheduler | 0x1688A | schedulerrt.c |
| ao_SignalReady | 0x16A18 | schedulerrt.c |
| ao_timer_Set | 0x16AC0 | hsm.c |
| ao_timer_Fire | 0x16A5E | hsm.c |

### Sentinel Signal Addresses

| Signal | Firmware Address | Value |
|--------|-----------------|-------|
| HSM_SIG_ENTRY | 0x1E512 | 0xDEADBEEF |
| HSM_SIG_INIT | 0x1E516 | 0xDEADBEEF |
| HSM_SIG_EXIT | 0x1E51A | 0xDEADBEEF |

### Error Convention

| Item | Firmware | Reimplementation |
|------|----------|-----------------|
| Error buffer | unk_FFFF00 (64 bytes) | `g_error_str[64]` (RAM) |
| Error check | CheckWatchdog: strlen(0xFFFF00) | `error_active()` |
| Error display | LCD line 1: "HALT:", line 2: scroll message | UART: "ASSERT: msg" |

## Usage Example

```c
#include "active_object/ao.h"

/* Define signal functions (their addresses are the signal IDs) */
static void sig_motor_run(void)  {}
static void sig_motor_idle(void) {}

/* Define state descriptors */
static const state_desc_t motor_top  = { motor_top_handler, NULL, 0 };
static const state_desc_t motor_idle = { motor_idle_handler, &motor_top, 0 };

/* AO instance + timer */
static ao_t       ao_motor;
static ao_timer_t motor_timer;

/* State handler */
static void *motor_idle_handler(ao_t *ao, void *event)
{
    if (event == HSM_SIG_ENTRY) return HSM_HANDLED;
    if (event == HSM_SIG_EXIT)  return HSM_HANDLED;
    if (event == HSM_SIG_INIT)  return HSM_HANDLED;

    evt_header_t *evt = (evt_header_t *)event;
    if (evt->signal == (signal_t)&sig_motor_run)
        return HSM_TRAN(&motor_running);

    return HSM_UNHANDLED;  /* pass to parent */
}

/* Initialization */
void app_init(void)
{
    ao_FrameworkInit();
    ao_Start(7, &ao_motor, &motor_top);
}

/* Post an event */
ao_PostTransition(&ao_motor, (signal_t)&sig_motor_run);

/* Tick from ISR */
void periodic_isr(void)
{
    ao_timer_Tick();
    ao_Scheduler();
}
```

## Not Included

The following firmware AO instances are **not** part of this framework module.
They use the framework API but are application-level:

| AO | Priority | Description |
|----|----------|-------------|
| aoMotor | 7 | Motor control HSM (drives, blade) |
| aoWire | 5 | Boundary wire following HSM |
| aoKeyboard | 3 | Keyboard/UI HSM |
| aoBattery | 4 | Battery management HSM |
| aoTimer | 8 | Periodic timer events |

These will be implemented as separate TestFirmware modules as the
reverse-engineering effort continues.
