# Industrial Joystick Product Line — Firmware Architecture Design (v6)

## I. Problem Analysis

An industrial joystick product line typically faces variation across three dimensions:

```
Dimension 1: Product Model (hardware differences)
├── JS-100: Single-axis joystick, 2 buttons, CAN output
├── JS-200: Dual-axis joystick, 4 buttons, CAN output
├── JS-300: Three-axis joystick + knob, 8 buttons, CAN + safety function
└── JS-400: Dual-axis + force feedback, 12 buttons, CAN + safety function

Dimension 2: Communication Protocol (customer requirement differences) — CAN bus family only
├── CAN 2.0B (custom protocol)
├── CANopen (DS401 / DS406)
├── SAE J1939
└── CANopen Safety (EN 50325-5)

Dimension 3: Runtime Platform (chip differences)
├── STM32F103
├── STM32G431
├── S32K144
└── Future platforms
```

By combinatorial enumeration, 4×4×3 = 48 configurations. The traditional approach is one project per configuration — a maintenance nightmare.
The correct approach is to use **composition over inheritance**, allowing the three dimensions to be orthogonal and independent through architectural design.

---

## II. Overall Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                       Configuration Layer                       │
│  config/products/js*.c (one file per product) / build system (CMake) │
├────────────────────────────────────────────────────────────────┤
│                        Event Bus (Observer)                     │
│  Event publish/subscribe bus — decouples inter-module comms (ISR-deferred dispatch supported) │
├────────────────────────────────────────────────────────────────┤
│              RTOS Task Layer (task scheduling)                  │
│  safety_task(highest) / main_task / proto_task / diag_task(lowest) │
├────────────────────────────────────────────────────────────────┤
│                       Application Layer                         │
│  device_state (state machine) / app_compose (assembly) / app_tasks (task entry) │
├────────────┬───────────────────────┬──────────────────────────┤
│ Joystick   │  Safety Architecture  │  Diagnostics (STL/WDG)   │
│ Signal     │  (E-Gas 3-layer)      │  Diagnostic service      │
│ Pipeline   │  ┌─────────────────┐  │  (low priority)          │
│ (Level 1)  │  │ L2 Independent  │  │                          │
│            │  │ Compute Channel │  │                          │
│            │  │ L2 Comparator   │  │                          │
│            │  │ L3 Conditional  │  │                          │
│            │  │ Watchdog Feed   │  │                          │
│            │  │ L3 Independent  │  │                          │
│            │  │ Shutdown Path   │  │                          │
│            │  └─────────────────┘  │                          │
├────────────┴───────────────────────┴──────────────────────────┤
│                        Service Layer                            │
│  calibration / nv_storage / logging                            │
├────────────────────────────────────────────────────────────────┤
│                  Output Adapter Layer (new in v4)               │
│  output_adapter — range conversion (pure function, pipeline→self-describing output_data_t) │
├──────────┬─────────────────────────────────────────────────────┤
│ Protocol │  CANopen │ J1939 │ CAN_Raw │                        │
│ Engines  │  (DS401) │       │         │  Profile-driven        │
│ (packing │          │       │         │  Receives output_data_t│
│ only)    │          │       │         │                        │
├──────────┴─────────────────────────────────────────────────────┤
│              Signal Processing Pipeline (explicit signal pipeline) │
│  axis_pipeline: fault_detect → calibrate → filter → deadzone   │
│  (uint16_t domain)              (int32_t domain)               │
│  debounce (button handling, independent)                        │
├────────────────────────────────────────────────────────────────┤
│              Input Acquisition Layer (v5 three-layer model)     │
│  input_acq_t → scan_buffer_t ← buf_axis_source / buf_btn      │
├────────────────────────────────────────────────────────────────┤
│                    Driver Interface Layer                        │
│  adc_reader.h / adc_voltage_reader.h / adc_continuous.h        │
│  can_port.h (v5) / gpio_reader.h / spi_driver.h               │
├────────────────────────────────────────────────────────────────┤
│              Async Primitives Layer (new in v5, ISR→task boundary) │
│  async_notify_t / async_queue_t / async_barrier / frame_pub    │
├────────────────────────────────────────────────────────────────┤
│              RTOS Abstraction Layer (CMSIS-RTOS v2)             │
│  osThreadNew / osMutexAcquire / osDelay (task↔task sync)       │
├────────────────────────────────────────────────────────────────┤
│                    Platform Registry                            │
│  platform_registry.h — platform_init() explicit registration (MISRA compliant) │
├──────────┬──────────┬──────────────────────────────────────────┤
│ STM32G4  │ S32K144  │  PC Mock                                 │
│ Platform │ Platform │  Platform                                │
├──────────┴──────────┴──────────────────────────────────────────┤
│                    BSP (Board Support)                           │
│  Pin mapping / clock config / interrupt priority / CubeMX generated code │
├────────────────────────────────────────────────────────────────┤
│                    Hardware Safety (new in v4)                   │
│  External watchdog IC / safety relay (STO) / MPU memory partition │
└────────────────────────────────────────────────────────────────┘
```

### Architecture Evolution History

| Change Point | v1 Design | v2 Improvement | v3 Improvement | v4 Improvement | v5 Improvement | Corresponding Principle |
|--------|---------|---------|---------|---------|---------|----------|
| Joystick core | Single function, five responsibilities | Signal processing chain (pipeline) | Same as v2 | Removed generic chain, unified as explicit `axis_pipeline` (`uint16_t→int32_t` type boundary) | Same as v4 | SRP |
| Input acquisition | Direct ADC call | Same as v1 | `scan_buffer_t` + `board_scanner_t` scan-buffer decoupling | Same as v3 | Three-layer model: sensor backend → `input_acq_t` → `scan_buffer_t`; DMA double-buffer + sequence number + barrier | SRP+OCP |
| Button acquisition | GPIO only | Same as v1 | `buf_btn_source` (MUX channel threshold detection, supports simultaneous multi-button press) | Same as v3 | Same as v3 (consumer unchanged, producer changed to `input_acq_t`) | ISP |
| Product configuration | `#elif` chain | Independent `.c` per product | Same as v2 (full-scale derived from `resolution_bits`) | Added `protocol_bundle` field + per-axis `max_deflection` | Same as v4 | OCP |
| Protocol interface | 7 methods fully implemented | Capability bitmask | Same as v2 | Protocol engine only handles packing, range conversion moved to `output_adapter` | Same as v4 | SRP+Adapter |
| Protocol output | Value conversion embedded in engine | Same as v1 | Profile-driven | Added `output_adapter` layer, protocol engine receives self-describing `output_data_t` | Same as v4 | Anti-Corruption |
| ADC interface | Fat interface | ISP split | Added SPI interface | Same as v3 | `adc_continuous_t` adds completion callback, incorporated into `frame_publisher_t` double-buffer | ISP |
| CAN interface | — | — | `can_port_t` transitional design | Same as v3 | `can_port_t` (`try_recv`/`wait_recv`, callback kept internal to platform layer) | DIP |
| Platform registration | `#if defined` | `__attribute__((constructor))` | Explicit `platform_init()` (MISRA compliant) | Same as v3 | Added `async_*` primitive registration + `event_queue_backend` registration | DIP |
| Async primitives | None | None | None | None | `async_notify_t` / `async_queue_t` / `async_barrier` (covers ISR→task boundary only) | ISP |
| Normalized range | ±10000 fixed | Same as v1 | Internal unsigned raw range, protocol layer converts as needed | No normalization; `output_adapter` scales once before protocol packing | Same as v4 | — |
| Task architecture | Bare loop | Bare loop + `os_delay` | RTOS multi-task (CMSIS-RTOS v2) | safety_task upgraded to L2 independent compute channel | main_task uses `input_acq_t`, proto_task uses `can_port_t.wait_recv` | SRP+Safety |
| Event bus | None | Synchronous dispatch | Added ISR-safe deferred dispatch | Same as v3 | Control plane/data plane separation + `event_queue_backend_t` injection (removes ARM IPSR dependency) | Observer |
| Protocol output | CAN + Analog mixed | Same as v1 | CAN family only (CANopen/J1939/CAN_Raw) | Same as v3 | Same as v3 | Simplification |
| Deadzone strategy | Hardcoded linear deadzone | Pluggable strategy (linear/S-curve/exponential) | Same as v3 | Added | Same as v4 | — |
| Functional safety | None | None | safety_task passive monitoring | E-Gas three-layer architecture (L1 functional / L2 independent monitoring / L3 conditional watchdog + hardware shutdown) | Same as v4 | IEC 61508 SIL 2 |
| Safety shutdown | None | None | Relies on event_bus → state machine | Independent hardware shutdown path (GPIO directly drives safety relay, bypasses RTOS) | Same as v4 | PL d Cat.3 |
| Watchdog | None | None | Unconditional watchdog feeding | Conditional watchdog (feed only when L2 comparison passes + program flow marker is correct) | Same as v4 | IEC 61508-7 |

---

## III. Driver Interface Layer Design (ISP Improvement: Split by Usage Scenario)

### 3.1 ADC Interface — Isolated by Usage Scenario

The original design placed all ADC functions in a single `adc_driver_t`. The joystick core only needs `read()`, safety monitoring needs `read_mv()`, and DMA acquisition needs `start_continuous()`. Following the Interface Segregation Principle, split into three independent interfaces:

```c
// drivers/adc_reader.h — Basic read interface; most modules only need this
#pragma once
#include "drv_common.h"

typedef struct {
    drv_status_t (*init)(void);
    drv_status_t (*read)(adc_channel_id_t ch, uint16_t *raw);
    drv_status_t (*deinit)(void);
} adc_reader_t;
```

```c
// drivers/adc_voltage_reader.h — Voltage read interface; needed by safety monitor/diagnostics
#pragma once
#include "drv_common.h"

typedef struct {
    drv_status_t (*read_mv)(adc_channel_id_t ch, uint32_t *mv);
} adc_voltage_reader_t;
```

```c
// drivers/adc_continuous.h — DMA continuous acquisition interface; needed by high-speed acquisition modules
#pragma once
#include "drv_common.h"

typedef struct {
    drv_status_t (*start_continuous)(adc_channel_id_t *chs, uint8_t cnt,
                                     uint16_t *buf, uint16_t buf_len);
    drv_status_t (*stop_continuous)(void);
} adc_continuous_t;
```

```c
// drivers/drv_common.h — Common type definitions
#pragma once
#include <stdint.h>

typedef enum {
    DRV_OK = 0,
    DRV_ERROR,
    DRV_TIMEOUT,
    DRV_BUSY,
    DRV_INVALID_ARG,
    DRV_NOT_SUPPORTED,
} drv_status_t;

typedef enum {
    ADC_CH_AXIS_X,
    ADC_CH_AXIS_Y,
    ADC_CH_AXIS_Z,
    ADC_CH_KNOB,
    ADC_CH_VREF,
    ADC_CH_TEMP,
    ADC_CH_SUPPLY,
    ADC_CH_MAX
} adc_channel_id_t;
```

Platform implementations can satisfy multiple interfaces simultaneously:

```c
// platform/stm32g4/adc_stm32g4.c
const adc_reader_t          adc_reader_stm32g4 = { .init = ..., .read = ..., .deinit = ... };
const adc_voltage_reader_t  adc_voltage_stm32g4 = { .read_mv = ... };
const adc_continuous_t      adc_continuous_stm32g4 = { .start_continuous = ..., .stop_continuous = ... };
```

Each module depends only on the minimal interface it requires:

```c
// Joystick core depends only on basic read
void joystick_core_init(joystick_core_t *js,
                         const adc_reader_t *adc,
                         const gpio_reader_t *gpio);

// Safety monitor depends on voltage read interface
void safety_monitor_init(safety_monitor_t *sm,
                          const adc_voltage_reader_t *adc,
                          const gpio_reader_t *gpio);
```

### 3.2 CAN Port Interface (v5 Improvement: Callback Contained Within Platform Layer)

The original v4 design exposed ISR callbacks to upper layers via `register_rx_cb(can_rx_callback_t cb)`. This caused:
- Upper layers must handle ISR semantics in callbacks (re-entrance constraints, stack size limits)
- Differences between STM32 weak-symbol callbacks, NXP interrupt dispatch handles, and PC threads leak to the application layer
- Callback registration patterns differ completely across platforms

v5 adopts a **port model**: callbacks exist only inside the platform adaptation layer; upper layers consume frames through `try_recv` / `wait_recv`.

```c
// drivers/drv_can.h
#pragma once
#include "drv_common.h"

typedef struct {
    uint32_t id;
    uint8_t  data[64];   // CAN-FD support
    uint8_t  dlc;
    uint8_t  is_extended; // 0=standard frame 1=extended frame
    uint8_t  is_fd;       // 0=CAN2.0 1=CAN-FD
} can_frame_t;

typedef struct {
    drv_status_t (*init)(void *ctx, uint32_t baudrate);

    // Send (called from task context)
    drv_status_t (*send)(void *ctx, const can_frame_t *frame);

    // Non-blocking receive: returns immediately
    // DRV_OK=frame received, DRV_BUSY=queue empty
    drv_status_t (*try_recv)(void *ctx, can_frame_t *frame);

    // Blocking receive: yields CPU waiting for frame arrival
    // RTOS platform → osMessageQueueGet(timeout)
    // Bare-metal platform → ring buffer + WFI
    // PC mock  → condition variable
    drv_status_t (*wait_recv)(void *ctx, can_frame_t *frame, uint32_t timeout_ms);

    drv_status_t (*set_filter)(void *ctx, uint32_t id, uint32_t mask);
    drv_status_t (*get_bus_state)(void *ctx, uint8_t *tx_err, uint8_t *rx_err);
    drv_status_t (*deinit)(void *ctx);

    void *ctx;  // Platform context (contains internal ring buffer / async_queue, etc.)
} can_port_t;
```

Platform adaptation layer internal structure example:

```c
// platform/stm32g4/can_stm32g4.c (internal, not exposed to upper layers)
#include "async_queue.h"

typedef struct {
    async_queue_t rx_queue;           // ISR → task frame queue
    can_frame_t   rx_buf[16];         // Queue storage
    // ... HAL handles and other platform details
} can_stm32g4_ctx_t;

// STM32 HAL CAN RX interrupt callback (platform internal, not exposed)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    can_frame_t frame;
    // ... read frame from HAL ...
    async_queue_put_from_isr(&g_can_ctx.rx_queue, &frame);  // ISR-safe enqueue
}

// External interface implementation
static drv_status_t stm32_try_recv(void *ctx, can_frame_t *frame) {
    can_stm32g4_ctx_t *c = (can_stm32g4_ctx_t *)ctx;
    return async_queue_get(&c->rx_queue, frame, 0) ? DRV_OK : DRV_BUSY;
}

static drv_status_t stm32_wait_recv(void *ctx, can_frame_t *frame, uint32_t timeout_ms) {
    can_stm32g4_ctx_t *c = (can_stm32g4_ctx_t *)ctx;
    return async_queue_get(&c->rx_queue, frame, timeout_ms) ? DRV_OK : DRV_TIMEOUT;
}
```

> **Core principle**: Callbacks exist only within the platform layer. Upper-layer code never touches ISR callbacks; it only consumes frames through `try_recv` / `wait_recv`.

### 3.3 GPIO Driver Interface

```c
// drivers/drv_gpio.h
typedef enum {
    GPIO_PIN_BTN_1, GPIO_PIN_BTN_2, GPIO_PIN_BTN_3, GPIO_PIN_BTN_4,
    GPIO_PIN_BTN_5, GPIO_PIN_BTN_6, GPIO_PIN_BTN_7, GPIO_PIN_BTN_8,
    GPIO_PIN_LED_STATUS, GPIO_PIN_LED_ERROR,
    GPIO_PIN_ENABLE_OUT,
    GPIO_PIN_MAX
} gpio_pin_id_t;

// Split per ISP: read and write are separated
typedef struct {
    drv_status_t (*init)(void);
    drv_status_t (*read)(gpio_pin_id_t pin, uint8_t *state);
} gpio_reader_t;

typedef struct {
    drv_status_t (*write)(gpio_pin_id_t pin, uint8_t state);
    drv_status_t (*toggle)(gpio_pin_id_t pin);
} gpio_writer_t;
```

### 3.4 Platform Registration Mechanism (DIP Improvement: Fully Inverted Dependency Direction)

In the original design, `app_config.c` directly referenced platform symbols via `#if defined(PLATFORM_STM32G4)`, making compile-time dependency flow top-down. After the improvement, the platform layer actively registers upward; upper-layer code has no knowledge of the specific platform.

```
Before (dependency direction downward):
  app_config.c ──→ platform/stm32g4/adc_stm32g4.c

After (dependency direction upward):
  app_config.c ──→ drivers/adc_reader.h ←── platform/stm32g4/adc_stm32g4.c
```

```c
// platform/platform_registry.h — upper layers depend only on this abstract interface
#pragma once
#include "adc_reader.h"
#include "adc_voltage_reader.h"
#include "drv_can.h"
#include "drv_gpio.h"
#include "drv_spi.h"

void platform_register_adc(const adc_reader_t *drv);
void platform_register_adc_voltage(const adc_voltage_reader_t *drv);
void platform_register_can(const can_port_t *port);
void platform_register_gpio_reader(const gpio_reader_t *drv);
void platform_register_gpio_writer(const gpio_writer_t *drv);
void platform_register_spi(const spi_driver_t *drv);
void platform_register_stl(const safety_stl_t *stl);  // v4: safety self-test library registration

const adc_reader_t*          platform_get_adc(void);
const adc_voltage_reader_t*  platform_get_adc_voltage(void);
const can_port_t*            platform_get_can(void);
const gpio_reader_t*         platform_get_gpio_reader(void);
const gpio_writer_t*         platform_get_gpio_writer(void);
const spi_driver_t*          platform_get_spi(void);
const safety_stl_t*          platform_get_stl(void);   // v4

// Explicit initialization entry — each platform implements one
// Replaces __attribute__((constructor)), satisfies MISRA C Rule 1.2
void platform_init(void);
```

```c
// platform/stm32g4/platform_init.c
// Explicit call; initialization order is deterministic and controllable; debuggable with breakpoints
// No GCC extensions required; satisfies functional safety requirements

#include "platform_registry.h"
#include "adc_stm32g4.h"
#include "can_stm32g4.h"
#include "gpio_stm32g4.h"
#include "spi_stm32g4.h"

void platform_init(void) {
    platform_register_adc(&adc_reader_stm32g4);
    platform_register_adc_voltage(&adc_voltage_stm32g4);
    platform_register_can(&can_port_stm32g4);
    platform_register_gpio_reader(&gpio_reader_stm32g4);
    platform_register_gpio_writer(&gpio_writer_stm32g4);
    platform_register_spi(&spi_driver_stm32g4);
    #if SAFETY_ENABLED
    platform_register_stl(&stl_stm32g4);   // v4: register X-CUBE-STL certified safety library
    #endif
}
```

> **Why not use `__attribute__((constructor))`?**
>
> | Comparison | `__attribute__((constructor))` | Explicit `platform_init()` |
> |--------|-------------------------------|----------------------|
> | MISRA C compliance | Violates Rule 1.2 (implementation-defined behavior) | Compliant |
> | Initialization order | Indeterminate; cannot be guaranteed across translation units | Explicitly controlled in code |
> | Debuggability | Executes before `main()`, breakpoints difficult | Ordinary function call, breakpoints at any time |
> | Functional safety certification | Fails IEC 61508 / ISO 13849 audit | Auditable and traceable |
> | Portability | GCC-exclusive; not supported by IAR/ARMCC/Green Hills | Pure C standard; works with all compilers |

```c
// main.c — startup sequence is clear (using CMSIS-RTOS v2 API)
#include "cmsis_os2.h"

int main(void) {
    // 1. Hardware initialization (BSP / CubeMX generated)
    HAL_Init();
    SystemClock_Config();

    // 2. Platform driver registration (explicit call, replaces constructor)
    platform_init();

    // 3. RTOS kernel initialization
    osKernelInitialize();

    // 4. Create application tasks
    app_tasks_create();

    // 5. Start scheduler (does not return)
    osKernelStart();
}
```

The linker determines which platform's `platform_init.c` is compiled in; upper-layer code requires zero modification. Each platform only needs to provide its own `platform_init()` implementation.

### 3.6 SPI Driver Interface (Hall Angle Sensors, etc.)

```c
// drivers/drv_spi.h — for Hall angle sensors, external ADC, etc.
#pragma once
#include "drv_common.h"

typedef struct {
    drv_status_t (*init)(uint32_t clock_hz);
    drv_status_t (*transfer)(uint8_t cs_pin,
                              const uint8_t *tx, uint8_t *rx,
                              uint16_t len);
    drv_status_t (*deinit)(void);
} spi_driver_t;
```

### 3.7 Async Primitives Layer (New in v5: ISR Boundary Decoupling)

#### 3.7.1 Design Motivation

The architecture has three types of async interaction patterns whose platform differences are greater than RTOS differences:

| Scenario | ISR-side behavior | Task-side behavior | Platform differences |
|------|-----------|-----------|---------|
| ADC DMA complete | Update double-buffer index + set notify | Retrieve latest completed frame | STM32 HAL callback vs NXP eDMA callback vs PC timer |
| CAN RX | Frame into ring buffer + wake consumer | Block/poll for frame | STM32 weak-symbol callback vs NXP interrupt dispatch handle vs PC thread |
| Event bus ISR publish | Event enqueued | Deferred dispatch | CMSIS-RTOS auto-detects ARM IPSR register; unavailable on non-ARM platforms |

**CMSIS-RTOS v2 is an OS abstraction, not an I/O semantic abstraction.** It solves portability of task scheduling/mutex/delay, but does not unify the semantics of ISR→task data transfer. A very thin async primitives layer is needed, specifically covering the **ISR→task boundary**.

#### 3.7.2 Responsibility Boundary (Hard Constraint)

```
async_* only resolves the "ISR → task" boundary; does not handle task → task synchronization.

┌───────────────────────────────────┐
│        async primitives           │  ← this layer
│  async_notify_t: ISR wakes task   │
│  async_queue_t:  ISR delivers data│
│  async_barrier:  memory visibility│
└───────────────────────────────────┘
          ↑ does not replace ↓
┌───────────────────────────────────┐
│        CMSIS-RTOS v2              │  ← OS layer
│  task scheduling / mutex /        │
│  semaphore / timer / abs. delay   │
└───────────────────────────────────┘
```

Rules:
- Task scheduling, inter-task mutex, timers → continue using CMSIS-RTOS v2
- ISR data delivery / waking consumer → use `async_notify_t` / `async_queue_t`
- Memory visibility guarantees → use `async_write_barrier()` / `async_read_barrier()`

#### 3.7.3 Memory Barrier Primitives

`volatile` only constrains compiler reordering; it is insufficient to express visibility across cores/buses/peripheral DMA. Memory barriers must be explicitly inserted in double-buffer patterns.

```c
// async/async_barrier.h — platform-agnostic memory barrier abstraction
#pragma once

// Write barrier: ensures all preceding writes are visible to other observers before subsequent writes
// Usage: called by producer after updating double-buffer data, before publishing ready_idx
void async_write_barrier(void);

// Read barrier: ensures all preceding reads complete before subsequent reads
// Usage: called by consumer after reading ready_idx, before accessing buffer data
void async_read_barrier(void);
```

Platform implementations:

```c
// platform/stm32g4/async_barrier_stm32.c
#include "async_barrier.h"
void async_write_barrier(void) { __DMB(); }  // ARM Data Memory Barrier
void async_read_barrier(void)  { __DMB(); }

// platform/riscv/async_barrier_riscv.c (future platform)
void async_write_barrier(void) { __asm volatile("fence w,w" ::: "memory"); }
void async_read_barrier(void)  { __asm volatile("fence r,r" ::: "memory"); }

// platform/test/async_barrier_mock.c
// On PC, volatile + compiler barrier is sufficient (single-core simulation)
void async_write_barrier(void) { __asm volatile("" ::: "memory"); }
void async_read_barrier(void)  { __asm volatile("" ::: "memory"); }
```

#### 3.7.4 Notify Primitive

ISR-side signals to wake a task; task-side blocks or polls.

```c
// async/async_notify.h — one-way notification from ISR → task
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct async_notify async_notify_t;

// Initialize (called during platform startup)
void async_notify_init(async_notify_t *n);

// ISR context: send notification (must be ISR-safe, must not block)
void async_notify_give_from_isr(async_notify_t *n);

// Task context: wait for notification
// timeout_ms = 0 → non-blocking poll
// timeout_ms = UINT32_MAX → wait forever
bool async_notify_take(async_notify_t *n, uint32_t timeout_ms);
```

Platform implementations:

```c
// CMSIS-RTOS v2 platform → mapped to task notification or binary semaphore
// Bare-metal platform → volatile flag + WFI
// PC mock → condition variable
```

#### 3.7.5 Queue Primitive

ISR-side delivers fixed-length data; task-side retrieves blocking or non-blocking.

```c
// async/async_queue.h — one-way data queue from ISR → task
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct async_queue async_queue_t;

// Initialize; item_size and capacity determined at compile time
void async_queue_init(async_queue_t *q, void *buf, uint32_t item_size, uint32_t capacity);

// ISR context: enqueue (ISR-safe, non-blocking)
// Returns true on success, false if queue is full
bool async_queue_put_from_isr(async_queue_t *q, const void *item);

// Task context: dequeue
// timeout_ms = 0 → non-blocking (returns immediately)
// timeout_ms = UINT32_MAX → wait forever
bool async_queue_get(async_queue_t *q, void *item, uint32_t timeout_ms);
```

#### 3.7.6 Frame Publisher Encapsulation

Double-buffer + sequence number publish/consume operations are easy to get wrong. Encapsulated as dedicated functions to ensure correct barrier ordering:

```c
// async/frame_publisher.h — double-buffer frame publish/consume
#pragma once
#include "async_barrier.h"
#include "async_notify.h"
#include <stdint.h>

// Frame buffer descriptor (shared between producer and consumer)
typedef struct {
    void         *buffers[2];       // Double buffer pointers
    uint32_t      frame_size;       // Single frame size in bytes
    volatile uint8_t  ready_idx;    // Index of currently readable buffer
    volatile uint32_t seq;          // Monotonically increasing sequence number
    async_notify_t    notify;       // Frame-ready notification
} frame_publisher_t;

// Called in ISR/DMA completion callback: publish new frame
static inline void frame_publish(frame_publisher_t *fp, uint8_t completed_idx) {
    async_write_barrier();          // Ensure buffer data is visible first
    fp->ready_idx = completed_idx;
    fp->seq++;
    async_notify_give_from_isr(&fp->notify);
}

// Called from task context: consume latest frame
// Returns DRV_OK=new frame, DRV_BUSY=no update
static inline drv_status_t frame_consume(frame_publisher_t *fp,
                                          void *out,
                                          uint32_t *out_seq) {
    uint32_t s = fp->seq;
    uint8_t  idx = fp->ready_idx;
    async_read_barrier();           // Ensure index is read before data
    memcpy(out, fp->buffers[idx], fp->frame_size);
    *out_seq = s;
    return DRV_OK;
}
```

#### 3.7.7 Platform Registration

Async primitives are registered along with platform drivers:

```c
// platform/platform_registry.h — new additions
void platform_register_async_notify_ops(const async_notify_ops_t *ops);
void platform_register_async_queue_ops(const async_queue_ops_t *ops);
```

> **Design constraint**: The total API count of the `async_*` layer is strictly limited to 10 functions. If more functions are needed, the responsibilities have crossed into the RTOS layer.

---

## IV. Input Acquisition Layer (v5 Improvement: Three-Layer Input Model + Async Decoupling)

### 4.1 Design Motivation

The original design assumed "axis = ADC channel"; `joystick_update()` called `adc->read(ch, &raw)` directly. However, the actual input sources of industrial joysticks are far more complex:

```
Actual hardware topology (JS-300 example):
         5V ──┬── R1 ──┬── GND     5V ──┬── R1 ──┬── GND
              │ Button │                │ Button │
              2.5V     │                2.5V     │
                ↓      ↓                  ↓      ↓
        ┌──────────────────────────────────────────┐
        │       Analog MUX (CD74HC4067)            │
        │  CH0: Axis-X pot    CH4: Button-0 (2.5V) │
        │  CH1: Axis-Y pot    CH5: Button-1 (2.5V) │
        │  CH2: Axis-Z pot    CH6: Button-2 (2.5V) │
        │  CH3: (reserved)    CH7: Button-3 (2.5V) │
        └──────────────┬───────────────────────────┘
                       │ COM (shared output)
                       ↓
                  ┌─────────┐
                  │   ADC   │ ← same ADC channel
                  └─────────┘
```

Key constraints:
1. **Axes and buttons share MUX+ADC resources** — cannot be read independently; scan order must be coordinated
2. **Buttons use per-channel 5V→2.5V voltage divider** — each button occupies an independent MUX channel; reads ≥2.5V when pressed; supports simultaneous multi-button press
3. **Large hardware differences across products** — some use MUX, some use direct multi-channel ADC, some use DMA continuous acquisition
4. **Mixed pull and push modes** — MUX/direct ADC is synchronous pull; DMA/SPI DMA is asynchronous push

Encapsulating MUX+ADC as a monolithic resource would couple axis source and button source to the same scanner, lacking platform-ability. The v4 `board_scanner_t.scan()` synchronous model masked async DMA as synchronous scanning, hiding the semantic difference. v5 formally splits the input side into three layers.

### 4.2 Three-Layer Input Model (v5 Core Improvement)

The v4 `board_scanner_t` mixed "synchronous topology orchestration" and "async acquisition mechanism" into a single `scan()` interface. v5 splits this into three layers:

```
  ┌─────────────────────────────────────────────────────────────┐
  │  Layer 1: Sensor Backend (internal to platform layer;       │
  │           not exposed to upper layers)                       │
  │  Handles ADC/SPI/GPIO/MUX/DMA/HAL callbacks                 │
  │  ┌────────────┐ ┌──────────────┐ ┌──────────────┐           │
  │  │mux_adc_    │ │direct_adc_   │ │dma_adc_      │           │
  │  │backend     │ │backend       │ │backend       │           │
  │  │(sync pull) │ │(sync pull)   │ │(async push)  │           │
  │  └─────┬──────┘ └──────┬───────┘ └──────┬───────┘           │
  │        │               │                │                    │
  │  ┌─────┴───────┐ ┌─────┴──────┐  ┌──────┴──────────────┐   │
  │  │adc_reader_t │ │adc_reader_t│  │adc_continuous_t     │   │
  │  │gpio_writer_t│ │            │  │frame_publisher_t    │   │
  │  └─────────────┘ └────────────┘  │async_notify_t       │   │
  │                                  └─────────────────────┘   │
  └──────────────────────────┬──────────────────────────────────┘
                             │ unified interface
                             ▼
  ┌─────────────────────────────────────────────────────────────┐
  │  Layer 2: input_acq_t (acquisition backend interface;       │
  │           sole entry point for upper layers)                 │
  │  acquire_latest(ctx, &buf, &seq) → DRV_OK / DRV_BUSY       │
  │  Provides "latest consistent snapshot + sequence number"    │
  └──────────────────────────┬──────────────────────────────────┘
                             │ writes to
                             ▼
  ┌─────────────────────────────────────────────────────────────┐
  │  Layer 3: scan_buffer_t (generic sample carrier; zero deps) │
  │  Consumed by buf_axis_source / buf_btn_source               │
  └─────────┬───────────────────┬───────────────────────────────┘
            │ read-only          │ read-only
            ↓                    ↓
  ┌──────────────────┐ ┌──────────────────────┐
  │  buf_axis_source │ │  buf_btn_source      │
  │  Read by channel │ │  Threshold detect    │
  │  index           │ │  → bitmask           │
  └──────────────────┘ └──────────────────────┘
```

### 4.3 input_acq_t — Unified Acquisition Interface (New in v5)

Replaces the v4 `board_scanner_t`. No longer distinguishes "scan" from "frame retrieval"; unified as a single atomic acquisition:

```c
// input/input_acq.h — unified acquisition backend interface
#pragma once
#include "drv_common.h"
#include "scan_buffer.h"

typedef struct {
    // Start acquisition backend (initialize DMA / configure MUX / start SPI, etc.)
    drv_status_t (*start)(void *ctx);

    // Stop acquisition backend
    drv_status_t (*stop)(void *ctx);

    // Get latest consistent snapshot (single atomic operation; replaces has_fresh_sample + acquire)
    // DRV_OK    = successfully acquired consistent snapshot; seq returns current sequence number
    // DRV_BUSY  = no new sample available (caller reuses last snapshot or waits for notification)
    // DRV_ERROR = underlying acquisition error
    drv_status_t (*acquire_latest)(void *ctx, scan_buffer_t *out, uint32_t *seq);

    void *ctx;  // Points to the context of the specific backend
} input_acq_t;
```

Three uses of `seq`:
1. **Detect updates**: `curr_seq == last_seq` → no new data; reuse last snapshot
2. **Detect dropped frames**: `curr_seq - last_seq > 1` → acquisition rate faster than consumption rate
3. **Safety channel timeliness monitoring**: L2 can check whether L1 input `seq` has stalled

### 4.4 scan_buffer_t — Generic Sample Carrier

```c
// input/scan_buffer.h — pure data structure; zero external dependencies
#pragma once
#include <stdint.h>

#define SCAN_BUF_MAX_CHANNELS  16

typedef struct {
    uint16_t values[SCAN_BUF_MAX_CHANNELS];  // Raw ADC values for each channel
    uint8_t  count;                           // Number of valid channels
    uint32_t seq;                             // Monotonically increasing sample sequence number
    uint32_t timestamp_ms;                    // Snapshot generation time
} scan_buffer_t;
```

`scan_buffer_t` is the **decoupling core** of the entire input acquisition layer:
- Upstream (`input_acq_t`) only writes to it
- Downstream (input sources) only reads from it
- It has no hardware interface dependencies; can be populated directly with test data on PC Mock
- v5 clarifies: it is a **snapshot object**, not a DMA working buffer
- `input_acq_t.acquire_latest()` must fill both `seq` and `timestamp_ms`
- `main_task` verifies `now_ms - timestamp_ms <= INPUT_MAX_AGE_MS` before consuming

### 4.5 Sensor Backend — Synchronous Pull Implementation

**MUX + ADC Backend** (for products using MUX, such as JS-300/JS-400):

```c
// input/backend/mux_adc_acq.c — MUX channel selection + ADC per-channel acquisition
#include "input_acq.h"
#include "adc_reader.h"
#include "drv_gpio.h"

typedef struct {
    const adc_reader_t  *adc;
    const gpio_writer_t *gpio;
    adc_channel_id_t     adc_channel;
    gpio_pin_id_t        sel_pins[4];
    uint8_t              sel_pin_count;
    uint8_t              total_channels;
    uint16_t             settle_us;
    uint32_t             seq;           // Monotonically increasing sequence number
} mux_adc_acq_ctx_t;

static drv_status_t mux_adc_acquire(void *ctx, scan_buffer_t *out, uint32_t *seq) {
    mux_adc_acq_ctx_t *c = (mux_adc_acq_ctx_t *)ctx;
    out->count = c->total_channels;

    for (uint8_t ch = 0; ch < c->total_channels; ch++) {
        for (uint8_t s = 0; s < c->sel_pin_count; s++) {
            const gpio_level_t level = (ch >> s) & 1u ? GPIO_HIGH : GPIO_LOW;
            c->gpio->write(c->sel_pins[s], level);
        }
        drv_delay_us(c->settle_us);
        uint16_t raw = 0;
        c->adc->read(c->adc_channel, &raw);
        out->values[ch] = raw;
    }

    out->seq = ++c->seq;
    out->timestamp_ms = drv_get_time_ms();
    *seq = out->seq;
    return DRV_OK;  // Synchronous pull always succeeds (unless hardware error)
}
```

**Direct Multi-Channel ADC Backend**

```c
// input/backend/direct_adc_acq.c
#include "input_acq.h"
#include "adc_reader.h"

typedef struct {
    const adc_reader_t *adc;
    adc_channel_id_t    channels[SCAN_BUF_MAX_CHANNELS];
    uint8_t             channel_count;
    uint32_t            seq;
} direct_adc_acq_ctx_t;

static drv_status_t direct_adc_acquire(void *ctx, scan_buffer_t *out, uint32_t *seq) {
    direct_adc_acq_ctx_t *c = (direct_adc_acq_ctx_t *)ctx;
    out->count = c->channel_count;

    for (uint8_t i = 0; i < c->channel_count; i++) {
        uint16_t raw = 0;
        c->adc->read(c->channels[i], &raw);
        out->values[i] = raw;
    }

    out->seq = ++c->seq;
    out->timestamp_ms = drv_get_time_ms();
    *seq = out->seq;
    return DRV_OK;
}
```

**SPI Hall Sensor Backend** (AS5048A and other magnetic encoders, synchronous mode)

```c
// input/backend/spi_hall_acq.c — SPI Hall angle sensor
#include "input_acq.h"
#include "drv_spi.h"

typedef struct {
    const spi_driver_t *spi;
    uint8_t             cs_pin;
    uint8_t             channel_index;  // Position in scan_buffer
    uint32_t            seq;
} spi_hall_acq_ctx_t;

static drv_status_t spi_hall_acquire(void *ctx, scan_buffer_t *out, uint32_t *seq) {
    spi_hall_acq_ctx_t *c = (spi_hall_acq_ctx_t *)ctx;

    uint8_t tx[2] = {0xFF, 0xFF};  // AS5048A: NOP command reads angle
    uint8_t rx[2] = {0};
    drv_status_t st = c->spi->transfer(c->cs_pin, tx, rx, 2);
    if (st != DRV_OK) return DRV_ERROR;

    uint16_t angle = ((rx[0] & 0x3F) << 8) | rx[1];  // 14-bit angle
    out->values[c->channel_index] = angle;
    out->count = c->channel_index + 1;

    out->seq = ++c->seq;
    out->timestamp_ms = drv_get_time_ms();
    *seq = out->seq;
    return DRV_OK;
}
```

### 4.6 Sensor Backend — Asynchronous Push Implementation (DMA / SPI DMA)

DMA is not a scan strategy; it is an **acquisition mechanism**. v5 uses double-buffering + sequence numbers + memory barriers. Upper layers never touch ISR callbacks.

```c
// input/backend/dma_adc_acq.c — DMA continuous acquisition backend
#include "input_acq.h"
#include "adc_continuous.h"
#include "frame_publisher.h"  // Double-buffer frame publish (see 3.7.6)

typedef struct {
    const adc_continuous_t *adc_cont;
    frame_publisher_t       fp;                              // Double-buffer + sequence number
    uint16_t                buf_a[SCAN_BUF_MAX_CHANNELS];    // ping buffer
    uint16_t                buf_b[SCAN_BUF_MAX_CHANNELS];    // pong buffer
    uint8_t                 channel_count;
    uint32_t                last_consumed_seq;               // Sequence number last read by consumer
} dma_adc_acq_ctx_t;

// Called from platform-layer DMA completion ISR (callback not exposed to upper layers)
static void dma_complete_isr(void *ctx) {
    dma_adc_acq_ctx_t *c = (dma_adc_acq_ctx_t *)ctx;
    uint8_t completed_idx = /* platform HAL reports the completed buffer index */ 0;
    frame_publish(&c->fp, completed_idx);  // Contains async_write_barrier()
}

static drv_status_t dma_adc_start(void *ctx) {
    dma_adc_acq_ctx_t *c = (dma_adc_acq_ctx_t *)ctx;
    // Initialize frame_publisher
    c->fp.buffers[0] = c->buf_a;
    c->fp.buffers[1] = c->buf_b;
    c->fp.frame_size = c->channel_count * sizeof(uint16_t);
    async_notify_init(&c->fp.notify);
    // Start DMA continuous acquisition
    adc_channel_id_t chs[SCAN_BUF_MAX_CHANNELS];
    // ... populate channel list ...
    return c->adc_cont->start_continuous(chs, c->channel_count,
                                          c->buf_a, c->channel_count);
}

static drv_status_t dma_adc_acquire(void *ctx, scan_buffer_t *out, uint32_t *seq) {
    dma_adc_acq_ctx_t *c = (dma_adc_acq_ctx_t *)ctx;
    uint32_t new_seq;

    // frame_consume contains async_read_barrier()
    drv_status_t st = frame_consume(&c->fp, out->values, &new_seq);
    if (st != DRV_OK) return st;

    if (new_seq == c->last_consumed_seq) {
        return DRV_BUSY;  // No new frame
    }

    out->count = c->channel_count;
    out->seq = new_seq;
    out->timestamp_ms = drv_get_time_ms();
    *seq = new_seq;
    c->last_consumed_seq = new_seq;
    return DRV_OK;
}
```

> **Key design**: The DMA ISR does only three things: ① record the index of the completed buffer ② update the sequence number ③ send notification. No data copying or business logic. Upper layers retrieve a stable snapshot from the inactive buffer via `acquire_latest()`.

### 4.7 buf_axis_source — Buffer-Based Axis Input Source

```c
// input/buf_axis_source.h
#pragma once
#include "drv_common.h"
#include "scan_buffer.h"

// Unified axis input source interface (signal processing chain depends on this)
typedef struct {
    drv_status_t (*read_raw)(void *ctx, int32_t *raw_value);
    void *ctx;
    uint8_t resolution_bits;    // 12 for 12-bit ADC, 14 for AS5048A, etc.
} axis_source_t;

// Axis input source context based on scan_buffer
typedef struct {
    const scan_buffer_t *buf;         // Read-only reference; does not own
    uint8_t              channel_index; // Channel index of this axis in scan_buffer
} buf_axis_ctx_t;

static inline drv_status_t buf_axis_read(void *ctx, int32_t *raw) {
    const buf_axis_ctx_t *c = (const buf_axis_ctx_t *)ctx;
    if (c->channel_index >= c->buf->count) {
        return DRV_INVALID_ARG;
    }
    *raw = (int32_t)c->buf->values[c->channel_index];
    return DRV_OK;
}
```

### 4.8 buf_btn_source — Buffer-Based Button Input Source

```c
// input/buf_btn_source.h
#pragma once
#include "drv_common.h"
#include "scan_buffer.h"

// Unified button input source interface
typedef struct {
    drv_status_t (*read_buttons)(void *ctx, uint16_t *button_mask);
    void *ctx;
    uint8_t button_count;
} button_source_t;

// Button input source context based on scan_buffer
// Each button occupies an independent MUX channel; 5V→2.5V voltage divider;
// reads >= threshold when pressed
typedef struct {
    const scan_buffer_t *buf;
    uint8_t    channel_indices[12];  // scan_buffer channel indices for each button
    uint16_t   threshold;            // ADC threshold to determine button pressed (~2.5V)
    uint8_t    count;                // Number of buttons
} buf_btn_ctx_t;

static inline drv_status_t buf_btn_read(void *ctx, uint16_t *mask) {
    const buf_btn_ctx_t *c = (const buf_btn_ctx_t *)ctx;
    *mask = 0;
    for (uint8_t i = 0; i < c->count; i++) {
        if (c->buf->values[c->channel_indices[i]] >= c->threshold) {
            *mask |= (1u << i);
        }
    }
    return DRV_OK;
}
```

The threshold detection approach naturally supports **simultaneous multi-button press**: each button has its own independent voltage divider circuit on an independent MUX channel, with no mutual interference.

### 4.9 Assembling Input Sources in Product Configuration

The product descriptor declares acquisition backend type and channel mapping. During the `app_compose` phase, the corresponding `input_acq_t` instance is constructed based on the configuration. Different products select different backend strategies:

| Product | Acquisition Backend | Mode |
|------|---------|------|
| JS-100 | `direct_adc_acq` | Synchronous pull |
| JS-200 | `mux_adc_acq` | Synchronous pull |
| JS-300 | `mux_adc_acq` + `spi_hall_acq` | Synchronous pull (mixed ADC + SPI Hall) |
| JS-400 | `dma_adc_acq` | Asynchronous push (DMA double-buffer) |

### 4.10 Updated Input Data Flow (v5)

```
Hardware sensors ──→ ┌─────────────────────────────────────────────────┐
                     │   Sensor Backend (internal to platform layer)    │
                     │   ADC/SPI/GPIO/MUX/DMA/HAL callbacks             │
                     │   DMA backend: double-buffer + seq + write_barrier│
                     └──────────────────┬──────────────────────────────┘
                                        │ unified interface
                                        ▼
                     ┌─────────────────────────────────────────────────┐
                     │   input_acq_t.acquire_latest(ctx, &buf, &seq)  │
                     │   DRV_OK=consistent snapshot  DRV_BUSY=no update│
                     └──────────────────┬──────────────────────────────┘
                                        │ writes to
                                        ▼
                     ┌─────────────────────────────────────────────────┐
                     │   scan_buffer_t (snapshot object; pure data      │
                     │   decoupling point)                              │
                     │   values[0..N] — raw ADC values for all channels │
                     └─────────┬───────────────────┬───────────────────┘
                               │ read-only          │ read-only
                               ↓                    ↓
                     ┌──────────────────┐ ┌──────────────────────┐
                     │  buf_axis_source │ │  buf_btn_source      │
                     │  Read by channel │ │  Threshold detect    │
                     │  index           │ │  → bitmask           │
                     └────────┬─────────┘ └──────────┬───────────┘
                              │ int32_t raw           │ uint16_t mask
                              ↓                       ↓
                     ┌──────────────────┐    ┌──────────────┐
                     │  axis_pipeline   │    │  Debounce    │
                     │ fault→cal→flt→dz │    │              │
                     └────────┬─────────┘    └──────┬───────┘
                              │ int32_t              │ uint16_t
                              └──────────┬───────────┘
                                         ↓
                     ┌──────────────────────────────────────────┐
                     │      joystick_data_t                     │
                     │  axes[] + buttons + safety_state         │
                     └──────────────────────────────────────────┘
```

---

## V. Signal Processing Pipeline Architecture (v4 Improvement: Unified to Explicit Pipeline, Removed Generic Chain)

### 5.1 Internal Signal Representation: Unsigned Raw Range, No Normalization

The signal processing pipeline **does not normalize**; it maintains the sensor's native raw range throughout. Range conversion is performed once by the independent `output_adapter` layer before protocol packing (see Chapter VI).

| Sensor | Raw Range | Center Value | Data Type |
|--------|---------|--------|---------|
| 12-bit ADC | 0–4095 | ~2048 (calibrated) | uint16_t |
| 14-bit Hall | 0–16383 | ~8192 (calibrated) | uint16_t |

**Why not normalize within the pipeline:**

- **Precision loss is real** — a 14-bit sensor dithers ±2~3 LSB; a single integer division mapping may bury the signal in quantization noise
- **Non-linearity introduced by asymmetric calibration** — when positive and negative travel differ, normalization produces a gain step near the zero point
- **Pipeline stages do not require a common unit** — fault detection operates in the `uint16_t` domain; filtering/deadzone operate in the `int32_t` domain around zero, naturally independent of absolute range

**v4 type domain boundary:**

```
              uint16_t domain                int32_t domain
         ┌──────────────────┐    ┌──────────────────────────┐
  raw ──→│ fault_detect     │───→│ calibrate (boundary)      │
         │ threshold compare│    │ linear: raw - center     │
         └──────────────────┘    │ circular: wrap_subtract  │
                                 ├──────────────────────────┤
                                 │ lowpass_filter           │
                                 ├──────────────────────────┤
                                 │ deadzone                 │
                                 ├──────────────────────────┤
                                 │ output (signed offset)   │
                                 └──────────────────────────┘
                                          │
                                 output_adapter scales as needed (sole mapping point)
```

### 5.2 Design Motivation

In the original design, the `process_axis()` function in `joystick_core.c` was responsible for five duties simultaneously: ADC acquisition, fault detection, calibration calculation, deadzone processing, and button debouncing. Any change in requirements (replace debounce algorithm, replace deadzone curve, add digital filter) required modifying this file.

Improvement: split into a signal processing pipeline, where each stage is an independent module with only one reason to change.

> **v4 change:** v3 had both a generic `signal_chain_t` (unified `int16_t` signature) and an explicit `axis_pipeline_t` (`uint16_t→int32_t` boundary) coexisting. v4 removes the generic chain and retains only the explicit pipeline. Rationale: pipeline stage order is fixed; what actually changes are parameters and strategies. The flexibility of the generic chain is not needed and introduces type mismatches and conceptual confusion.

### 5.3 Independent Stage Implementations (v4 Unified Signatures)

Each stage is independently implemented in its own file with only one reason to change. In v4, all stages use the pipeline's actual internal types rather than a generic `int16_t` signature:

```c
// signal/fault_detector.h — handles only open/short-circuit detection (uint16_t domain)
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t threshold_low;   // e.g. 50
    uint16_t threshold_high;  // e.g. 4046 / 16330
    uint8_t  fault_flag;      // output: whether fault exists
} fault_detect_ctx_t;

// Returns true if fault detected
bool fault_detect(fault_detect_ctx_t *ctx, uint16_t raw);
```

```c
// signal/fault_detector.c
bool fault_detect(fault_detect_ctx_t *ctx, uint16_t raw) {
    if (raw < ctx->threshold_low || raw > ctx->threshold_high) {
        ctx->fault_flag = 1;
        return true;
    }
    ctx->fault_flag = 0;
    return false;
}
```

```c
// signal/calibrator.h — calibration strategy interface (uint16_t → int32_t boundary)
#pragma once
#include <stdint.h>

typedef struct {
    uint16_t min;
    uint16_t center;
    uint16_t max;
} calibrator_ctx_t;

// Calibration strategy function pointer: input uint16_t raw value, output int32_t signed offset
typedef int32_t (*calibrate_fn_t)(uint16_t raw, const calibrator_ctx_t *ctx);

// Built-in strategies
int32_t calibrate_linear(uint16_t raw, const calibrator_ctx_t *ctx);
int32_t calibrate_circular(uint16_t raw, const calibrator_ctx_t *ctx);
```

```c
// signal/calibrator.c
int32_t calibrate_linear(uint16_t raw, const calibrator_ctx_t *ctx) {
    // No normalization; returns signed offset directly
    return (int32_t)raw - (int32_t)ctx->center;
}

int32_t calibrate_circular(uint16_t raw, const calibrator_ctx_t *ctx) {
    // Circular sensor (Hall angle): handles wrap-around
    int32_t diff = (int32_t)raw - (int32_t)ctx->center;
    int32_t half_range = (int32_t)(ctx->max - ctx->min) / 2;
    if (diff > half_range)  diff -= (int32_t)(ctx->max - ctx->min);
    if (diff < -half_range) diff += (int32_t)(ctx->max - ctx->min);
    return diff;
}
```

```c
// signal/filter.h — handles only digital filtering (int32_t domain)
#pragma once
#include <stdint.h>

typedef struct {
    int32_t prev_output;
    uint8_t alpha;  // 0-255; larger = smoother; 0 = disabled
} lowpass_ctx_t;

int32_t lowpass_filter(lowpass_ctx_t *ctx, int32_t input);
```

```c
// signal/filter.c
int32_t lowpass_filter(lowpass_ctx_t *ctx, int32_t input) {
    int32_t num = (int32_t)ctx->alpha * ctx->prev_output
                + (256 - (int32_t)ctx->alpha) * input;
    num += (num >= 0) ? 128 : -128;  // Round to avoid long-term negative bias
    int32_t out = num / 256;
    ctx->prev_output = out;
    return out;
}
```

```c
// signal/deadzone.h — handles only deadzone curve; supports strategy injection (int32_t domain)
#pragma once
#include <stdint.h>

typedef int32_t (*deadzone_fn_t)(int32_t input, int32_t threshold);

// Built-in strategies
int32_t deadzone_linear(int32_t input, int32_t threshold);
int32_t deadzone_scurve(int32_t input, int32_t threshold);
int32_t deadzone_exponential(int32_t input, int32_t threshold);
```

```c
// signal/debounce.h — handles only button debouncing
// Independent of axis signal processing chain; used for button handling
#pragma once
#include <stdint.h>

typedef struct {
    uint8_t  history[12];  // Sampling history for each button
    uint8_t  count;
    uint16_t debounced;
} debounce_ctx_t;

void debounce_update(debounce_ctx_t *ctx, uint8_t btn_idx, uint8_t state);
uint16_t debounce_get_buttons(const debounce_ctx_t *ctx);
```

### 5.4 Explicit Signal Pipeline (v4 Sole Signal Chain Interface)

Pipeline stage order is essentially fixed across products; what actually changes are the **parameters and strategies** of each stage. Implemented with explicit functions and clear type boundaries:

```c
// signal/axis_pipeline.h
#pragma once
#include <stdint.h>
#include "fault_detector.h"
#include "calibrator.h"
#include "filter.h"
#include "deadzone.h"

typedef struct {
    uint16_t       fault_threshold_low;
    uint16_t       fault_threshold_high;
    uint8_t        filter_alpha;       // 0 = filtering disabled
    int32_t        deadzone_threshold;
    deadzone_fn_t  deadzone_fn;        // Deadzone strategy (linear / scurve / exponential)
} axis_pipeline_config_t;

typedef struct {
    // Fault detection context
    fault_detect_ctx_t fault_ctx;
    // Calibration context
    calibrator_ctx_t   cal_ctx;
    calibrate_fn_t     calibrate;     // Linear / circular (injected by product config)
    // Filter context
    lowpass_ctx_t      filter_ctx;
    // Deadzone
    int32_t            dz_threshold;
    deadzone_fn_t      dz_fn;
    // Output
    int32_t            output;        // Signed offset (not normalized)
    uint16_t           max_deflection; // Maximum deflection for this axis (range/2); used by output_adapter
} axis_pipeline_t;

void axis_pipeline_init(axis_pipeline_t *p,
                        const axis_pipeline_config_t *cfg,
                        calibrate_fn_t calibrate,
                        uint8_t resolution_bits);
void axis_pipeline_process(axis_pipeline_t *p, uint16_t raw);
```

```c
// signal/axis_pipeline.c
void axis_pipeline_init(axis_pipeline_t *p,
                        const axis_pipeline_config_t *cfg,
                        calibrate_fn_t calibrate,
                        uint8_t resolution_bits) {
    p->fault_ctx.threshold_low  = cfg->fault_threshold_low;
    p->fault_ctx.threshold_high = cfg->fault_threshold_high;
    p->calibrate    = calibrate;
    p->filter_ctx.alpha = cfg->filter_alpha;
    p->filter_ctx.prev_output = 0;
    p->dz_threshold = cfg->deadzone_threshold;
    p->dz_fn        = cfg->deadzone_fn;
    p->output       = 0;
    p->max_deflection = (uint16_t)(1u << (resolution_bits - 1));  // range/2
}

void axis_pipeline_process(axis_pipeline_t *p, uint16_t raw) {
    // 1. Fault detection (uint16_t domain)
    if (fault_detect(&p->fault_ctx, raw)) {
        p->output = 0;
        return;
    }

    // 2. Calibration (uint16_t → int32_t, type domain boundary)
    //    Linear sensor: (int32_t)raw - center
    //    Digital angle:  circular_subtract(raw, center)
    int32_t val = p->calibrate(raw, &p->cal_ctx);

    // 3. Low-pass filter (int32_t domain)
    if (p->filter_ctx.alpha > 0) {
        val = lowpass_filter(&p->filter_ctx, val);
    }

    // 4. Deadzone (int32_t domain, around zero)
    val = p->dz_fn(val, p->dz_threshold);

    p->output = val;
}
```

```c
// joystick initialization and execution
void joystick_init(joystick_t *js, const product_descriptor_t *product) {
    for (uint8_t i = 0; i < product->hw->axis_map_count; i++) {
        axis_pipeline_init(&js->pipelines[i],
                           &product->signal->pipeline,
                           product->hw->axis_map[i].calibrate,
                           product->hw->resolution_bits);
    }
}

void joystick_update(joystick_t *js) {
    for (uint8_t i = 0; i < js->axes_count; i++) {
        int32_t raw_val = 0;
        js->axis_sources[i].read_raw(js->axis_sources[i].ctx, &raw_val);
        axis_pipeline_process(&js->pipelines[i], (uint16_t)raw_val);
        js->data.axes[i] = js->pipelines[i].output;
        js->data.max_deflection[i] = js->pipelines[i].max_deflection;
    }
    js->btn_source.read_buttons(js->btn_source.ctx, &js->data.buttons);
}
```

---

## VI. Protocol Layer Architecture (v4 Improvement: Output Adapter + Profile-Driven + Protocol Bundle)

### 6.1 Design Motivation

Issues with the v3 protocol layer:

1. **`void *config`** — `init(const void *config)` has no type checking; passing the wrong config compiles successfully but crashes at runtime → resolved in v3 with Protocol Bundle
2. **Hardcoded protocol behavior** — OD indices, PGNs, byte packing, value ranges are all hardcoded → resolved in v3 with profile-driven design
3. **Range conversion embedded in protocol engine** — `canopen_engine_send()` does both value scaling and protocol packing, violating SRP → **resolved in v4 with new Output Adapter layer**
4. **`joystick_data_t` is non-self-describing** — `max_deflection` is a single value, assuming all axes have the same resolution; but JS-300 mixes 12-bit/14-bit → **resolved in v4 by making it per-axis**

### 6.2 Common Data Model (v4 Improvement: Per-Axis max_deflection)

```c
// app/joystick_data.h
#pragma once
#include <stdint.h>

#define MAX_AXES    4
#define MAX_BUTTONS 12

typedef struct {
    int32_t  axes[MAX_AXES];              // Signal pipeline output (calibrated signed offset; not normalized)
    uint16_t max_deflection[MAX_AXES];    // v4: per-axis maximum deflection (resolves mixed resolution)
    uint8_t  axes_count;
    uint8_t  axes_valid;                  // Bitmask: which axes have valid data
    uint16_t buttons;                     // Bitmask
    uint8_t  buttons_count;
    uint32_t timestamp_ms;
    uint8_t  safety_state;                // 0=normal 1=degraded 2=safe stop
    uint8_t  error_flags;                 // Sensor faults, etc.
} joystick_data_t;

typedef struct {
    uint8_t  comm_ok;
    uint8_t  bus_off;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;
} protocol_status_t;
```

### 6.3 Axis Output Mapping Descriptor (Universal)

Shared axis value conversion descriptor across all protocols; defines target range and direction:

```c
// output/axis_output_map.h — v4: moved from protocols/ to output/; used by output_adapter
#pragma once
#include <stdint.h>

typedef struct {
    int32_t output_min;     // Target minimum value (e.g. -10000 / 0)
    int32_t output_max;     // Target maximum value (e.g. +10000 / 65535)
    int8_t  direction;      // +1 normal, -1 reversed
} axis_output_map_t;

// Universal conversion: pipeline output → protocol target range (pure function)
static inline int32_t axis_map_output(int32_t val, int32_t max_deflection,
                                       const axis_output_map_t *map) {
    if ((map == NULL) || (max_deflection <= 0)) {
        return 0;
    }

    const int32_t center = (map->output_min + map->output_max) / 2;
    const int32_t half   = (map->output_max - map->output_min) / 2;
    int64_t scaled = (int64_t)val * (int64_t)map->direction * (int64_t)half;
    int64_t out = (int64_t)center + scaled / (int64_t)max_deflection;

    if (out > map->output_max) out = map->output_max;
    if (out < map->output_min) out = map->output_min;
    return (int32_t)out;
}
```

### 6.4 Output Adapter Layer (New in v4)

**Design motivation:** In v3, range conversion was embedded inside protocol engines, causing:
- Protocol engines violated SRP (conversion + packing are two responsibilities)
- HMI display/data logging/remote debug wanting scaled values could not reuse the code
- Testing value conversion required mocking CAN drivers
- `joystick_data_t` was non-self-describing (`axes[]` meaning depends on external context)

v4 introduces the `output_adapter` layer with a single responsibility: convert pipeline internal offsets to self-describing output values.

```c
// output/output_adapter.h — new in v4: range conversion layer
#pragma once
#include "joystick_data.h"
#include "axis_output_map.h"

// Per-axis output descriptor: self-describing; does not depend on external context
typedef struct {
    int32_t  value;          // Converted value (within target range)
    int32_t  output_min;     // Valid lower bound for this value
    int32_t  output_max;     // Valid upper bound for this value
    uint8_t  valid;          // Whether data is valid
} axis_output_t;

typedef struct {
    axis_output_t axes[MAX_AXES];
    uint8_t       axes_count;
    uint16_t      buttons;
    uint8_t       buttons_count;
    uint32_t      timestamp_ms;
    uint8_t       safety_state;
    uint8_t       error_flags;
} output_data_t;

// Pure function: joystick_data_t → output_data_t
void output_adapt(const joystick_data_t *raw,
                  const axis_output_map_t *maps,
                  uint8_t map_count,
                  output_data_t *out);
```

```c
// output/output_adapter.c
#include "output_adapter.h"

void output_adapt(const joystick_data_t *raw,
                  const axis_output_map_t *maps,
                  uint8_t map_count,
                  output_data_t *out) {
    out->axes_count    = raw->axes_count;
    out->buttons       = raw->buttons;
    out->buttons_count = raw->buttons_count;
    out->timestamp_ms  = raw->timestamp_ms;
    out->safety_state  = raw->safety_state;
    out->error_flags   = raw->error_flags;

    for (uint8_t i = 0; i < map_count && i < raw->axes_count; i++) {
        const axis_output_map_t *m = &maps[i];
        out->axes[i].output_min = m->output_min;
        out->axes[i].output_max = m->output_max;
        out->axes[i].valid      = (raw->axes_valid >> i) & 1u;
        out->axes[i].value = out->axes[i].valid
                           ? axis_map_output(raw->axes[i],
                                             (int32_t)raw->max_deflection[i], m)
                           : ((m->output_min + m->output_max) / 2);
    }
}
```

**Layer responsibility comparison (v4):**

```
Layer                 Input type       Output type      Responsibility
─────────────────────────────────────────────────────────────
axis_pipeline         uint16_t raw     int32_t offset   Signal processing (fault/calibrate/filter/deadzone)
joystick_data_t       —                —                Pipeline internal data contract (raw offsets)
output_adapter        joystick_data_t  output_data_t    Range conversion (pure function; reusable)
protocol_engine       output_data_t    CAN frame        Protocol packing (OD/PGN/raw frame)
```

**Design pattern comparison:**

| Pattern | v3 | v4 |
|------|------|--------|
| **SRP** | Protocol layer does both conversion and packing | Conversion and packing separated |
| **Adapter** | Conversion logic embedded inside protocol engine | `output_adapter` is a standalone Adapter |
| **Anti-Corruption Layer** | Protocol layer directly interprets pipeline internal representation | `output_data_t` is a self-describing boundary object |
| **Reusability** | HMI/logging/debug wanting scaled values requires code duplication | `output_adapt()` is a pure function; any consumer can call it |
| **Testability** | Testing value conversion requires mocking CAN driver | `output_adapt()` pure function; direct unit testing |

### 6.5 CANopen Profile — OD + PDO + Value Mapping

All brand-specific differences in CANopen (OD layout, PDO mapping, value ranges, timing, EMCY codes) are abstracted into a descriptor table:

```c
// protocols/canopen/canopen_profile.h
#pragma once
#include "axis_output_map.h"

// ---- Object Dictionary entry descriptor ----
typedef enum { OD_UINT8, OD_UINT16, OD_UINT32, OD_INT16, OD_INT32 } od_data_type_t;
typedef enum { OD_RO, OD_WO, OD_RW, OD_CONST } od_access_t;

typedef struct {
    uint16_t       index;
    uint8_t        subindex;
    od_data_type_t type;
    od_access_t    access;
    uint16_t       data_offset;   // Offset in od_runtime_t.data[]
    uint32_t       default_val;
} od_entry_t;

typedef struct {
    uint8_t data[128];            // Runtime values for all OD entries
} od_runtime_t;

typedef struct {
    const od_entry_t *entries;
    uint16_t          entry_count;
    od_runtime_t     *runtime;
} od_table_t;

// ---- TPDO mapping descriptor ----
typedef struct {
    uint16_t od_index;
    uint8_t  od_subindex;
    uint8_t  bit_length;
} tpdo_mapping_entry_t;

typedef struct {
    tpdo_mapping_entry_t entries[8];
    uint8_t count;
    uint16_t event_timer_ms;
} tpdo_config_t;

// ---- CANopen Profile (complete brand descriptor) ----
typedef struct {
    const char              *profile_name;
    const od_table_t        *od;
    const tpdo_config_t      tpdos[4];
    uint8_t                  tpdo_count;
    const axis_output_map_t *axis_maps;
    uint8_t                  axis_map_count;
    uint16_t                 heartbeat_ms;
    uint16_t                 emcy_code_sensor_fault;
    bool                     has_sdo_params;   // Whether master can configure params online via SDO
} canopen_profile_t;
```

Standard DS401 profile example:

```c
// config/canopen_profiles/ds401_standard.c
static const od_entry_t od_ds401_entries[] = {
    { 0x1000, 0, OD_UINT32, OD_RO, 0,  0x00020192 },  // Device type
    { 0x1001, 0, OD_UINT8,  OD_RO, 4,  0          },  // Error register
    { 0x6401, 1, OD_INT16,  OD_RO, 5,  0          },  // Analog input 1
    { 0x6401, 2, OD_INT16,  OD_RO, 7,  0          },  // Analog input 2
    { 0x6000, 1, OD_UINT16, OD_RO, 9,  0          },  // Digital inputs
};
static od_runtime_t od_ram_ds401;
static const od_table_t od_ds401 = {
    .entries = od_ds401_entries, .entry_count = 5, .runtime = &od_ram_ds401,
};

static const axis_output_map_t ds401_axes[] = {
    { .output_min = -10000, .output_max = 10000, .direction = 1 },
    { .output_min = -10000, .output_max = 10000, .direction = 1 },
};

const canopen_profile_t canopen_ds401 = {
    .profile_name  = "DS401 Standard",
    .od            = &od_ds401,
    .tpdo_count    = 2,
    .tpdos = {
        [0] = { .entries = {{ 0x6401, 1, 16 }, { 0x6401, 2, 16 }},
                .count = 2, .event_timer_ms = 20 },
        [1] = { .entries = {{ 0x6000, 1, 16 }},
                .count = 1, .event_timer_ms = 50 },
    },
    .axis_maps     = ds401_axes,
    .axis_map_count = 2,
    .heartbeat_ms  = 1000,
    .emcy_code_sensor_fault = 0x5000,
    .has_sdo_params = false,
};
```

Custom brand profile (private OD indices + SDO-configurable parameters):

```c
// config/canopen_profiles/brand_b_compat.c
static const od_entry_t od_brand_b_entries[] = {
    { 0x1000, 0, OD_UINT32, OD_RO, 0,  0x00020192 },
    { 0x1001, 0, OD_UINT8,  OD_RO, 4,  0          },
    // Private axis data area (unsigned)
    { 0x2100, 1, OD_UINT16, OD_RO, 5,  0          },
    { 0x2100, 2, OD_UINT16, OD_RO, 7,  0          },
    { 0x2200, 1, OD_UINT16, OD_RO, 9,  0          },
    // Custom: SDO-configurable parameters
    { 0x2300, 1, OD_UINT16, OD_RW, 11, 300        },  // Deadzone (master-adjustable)
    { 0x2300, 2, OD_UINT8,  OD_RW, 13, 200        },  // Filter coefficient
    { 0x2300, 3, OD_UINT8,  OD_RW, 14, 0          },  // Calibration command
};
static od_runtime_t od_ram_brand_b;
static const od_table_t od_brand_b = {
    .entries = od_brand_b_entries, .entry_count = 8, .runtime = &od_ram_brand_b,
};

static const axis_output_map_t brand_b_axes[] = {
    { .output_min = 0, .output_max = 65535, .direction =  1 },
    { .output_min = 0, .output_max = 65535, .direction = -1 },  // Y reversed
};

const canopen_profile_t canopen_brand_b = {
    .profile_name  = "Brand-B Compatible",
    .od            = &od_brand_b,
    .tpdo_count    = 1,
    .tpdos = {
        [0] = { .entries = {{ 0x2100, 1, 16 }, { 0x2100, 2, 16 }, { 0x2200, 1, 16 }},
                .count = 3, .event_timer_ms = 10 },
    },
    .axis_maps     = brand_b_axes,
    .axis_map_count = 2,
    .heartbeat_ms  = 500,
    .emcy_code_sensor_fault = 0x8000,
    .has_sdo_params = true,
};
```

### 6.6 CANopen Universal Engine (v4 Improvement: Packing Only, No Value Conversion)

```c
// protocols/canopen/canopen_engine.c
#include "canopen_profile.h"
#include "canopen_stack.h"
#include "output_adapter.h"     // v4: receives output_data_t

static const canopen_profile_t *profile;
static const can_port_t *can;
static uint8_t node_id;

drv_status_t canopen_engine_init(const can_port_t *port,
                                 const canopen_profile_t *cfg_profile,
                                 uint8_t cfg_node_id) {
    can = port;
    node_id = cfg_node_id;
    profile = cfg_profile;

    co_node_init(node_id);
    co_nmt_set_heartbeat(profile->heartbeat_ms);

    // Initialize OD default values from profile
    od_load_defaults(profile->od);

    // Configure TPDO mappings from profile
    for (uint8_t t = 0; t < profile->tpdo_count; t++) {
        const tpdo_config_t *tp = &profile->tpdos[t];
        for (uint8_t e = 0; e < tp->count; e++) {
            co_tpdo_map(t + 1, tp->entries[e].od_index,
                        tp->entries[e].od_subindex,
                        tp->entries[e].bit_length);
        }
        co_tpdo_set_event_timer(t + 1, tp->event_timer_ms);
    }

    event_bus_subscribe(EVT_SAFETY_STATE_CHANGED, on_safety_changed);
    return DRV_OK;
}

// v4: receives output_data_t (already converted by output_adapter; values are in target range)
drv_status_t canopen_engine_send(const output_data_t *data) {
    // Write already-converted values to OD; no longer calls axis_map_output()
    for (uint8_t i = 0; i < profile->axis_map_count && i < data->axes_count; i++) {
        od_update_value(profile->od,
                        profile->tpdos[0].entries[i].od_index,
                        profile->tpdos[0].entries[i].od_subindex,
                        data->axes[i].value);
    }
    // Write buttons to OD
    od_update_value(profile->od, profile->digital_input_index,
                    profile->digital_input_subindex, data->buttons);
    return DRV_OK;
}

drv_status_t canopen_engine_on_rx_frame(const can_frame_t *frame) {
    return co_node_on_frame(frame);
}

drv_status_t canopen_engine_process_periodic(uint32_t now_ms) {
    (void)now_ms;
    co_node_process();
    return DRV_OK;
}

static void on_safety_changed(const event_t *evt) {
    if (evt->data.safety_state == SAFETY_STOP) {
        co_emcy_send(profile->emcy_code_sensor_fault);
    }
}
```

### 6.7 J1939 Profile — PGN + SPN Descriptors

J1939 has no OD; variation points are PGN selection and byte packing within frames. Defined using SPN descriptors:

```c
// protocols/j1939/j1939_profile.h
#pragma once
#include <stdint.h>

typedef enum {
    SPN_SOURCE_AXIS,       // Source: axis data
    SPN_SOURCE_BUTTON,     // Source: button bitmask
    SPN_SOURCE_SAFETY,     // Source: safety state
    SPN_SOURCE_CONST,      // Source: fixed value (e.g. 0xFF fill)
} spn_source_t;

typedef struct {
    spn_source_t source;
    uint8_t  source_index;    // Axis/button number (for CONST: the fixed value)
    uint8_t  byte_offset;     // Starting byte in frame (0-7)
    uint8_t  bit_offset;      // Starting bit in byte (0-7)
    uint8_t  bit_length;      // Bit width
    int32_t  scale_num;       // Scale numerator
    int32_t  scale_den;       // Scale denominator (physical = raw * num / den + offset)
    int32_t  add_offset;      // Additive offset
    int8_t   direction;       // +1 normal, -1 reversed
} spn_descriptor_t;

typedef struct {
    uint32_t pgn;
    uint8_t  priority;
    uint16_t tx_interval_ms;
    const spn_descriptor_t *spns;
    uint8_t  spn_count;
} pgn_descriptor_t;

typedef struct {
    const char             *profile_name;
    uint8_t                 source_address;
    const pgn_descriptor_t *pgns;
    uint8_t                 pgn_count;
} j1939_profile_t;
```

The difference between a standard profile and a custom brand profile is only in the data definition:

```c
// config/j1939_profiles/j1939_standard.c
static const spn_descriptor_t std_joy_spns[] = {
    { SPN_SOURCE_AXIS,   0, 0, 0, 16, 65535, 4096, 0,  1 },  // X: bytes 0-1
    { SPN_SOURCE_AXIS,   1, 2, 0, 16, 65535, 4096, 0,  1 },  // Y: bytes 2-3
    { SPN_SOURCE_BUTTON, 0, 4, 0, 16, 1,     1,    0,  1 },  // Buttons: bytes 4-5
    { SPN_SOURCE_SAFETY, 0, 6, 0, 8,  1,     1,    0,  1 },  // Safety: byte 6
    { SPN_SOURCE_CONST,  0xFF, 7, 0, 8, 1,   1,    0,  1 },  // Fill: byte 7
};
static const pgn_descriptor_t std_pgns[] = {
    { .pgn = 0xEF00, .priority = 6, .tx_interval_ms = 20,
      .spns = std_joy_spns, .spn_count = 5 },
};
const j1939_profile_t j1939_standard = {
    .profile_name = "J1939 Standard", .source_address = 0x27,
    .pgns = std_pgns, .pgn_count = 1,
};

// config/j1939_profiles/j1939_brand_x.c — custom: Y first, reversed, 10-bit
static const spn_descriptor_t bx_joy_spns[] = {
    { SPN_SOURCE_AXIS,   1, 0, 0, 10, 1023, 4096, 0, -1 },  // Y first, reversed
    { SPN_SOURCE_AXIS,   0, 1, 2, 10, 1023, 4096, 0,  1 },  // X second, cross-byte
    { SPN_SOURCE_BUTTON, 0, 3, 0, 8,  1,    1,    0,  1 },  // Buttons: 8-bit only
};
static const pgn_descriptor_t bx_pgns[] = {
    { .pgn = 0xFF21, .priority = 3, .tx_interval_ms = 10,
      .spns = bx_joy_spns, .spn_count = 3 },
};
const j1939_profile_t j1939_brand_x = {
    .profile_name = "Brand-X Compatible", .source_address = 0x30,
    .pgns = bx_pgns, .pgn_count = 1,
};
```

### 6.8 J1939 Universal Engine (v4 Improvement: Receives output_data_t)

```c
// protocols/j1939/j1939_engine.c
#include "j1939_profile.h"
#include "output_adapter.h"     // v4: receives output_data_t

static const j1939_profile_t *profile;
static const can_port_t *can;

drv_status_t j1939_engine_init(const can_port_t *port,
                               const j1939_profile_t *cfg_profile) {
    can = port;
    profile = cfg_profile;
    return DRV_OK;
}

// v4: receives output_data_t; axis values already converted by output_adapter
drv_status_t j1939_engine_send(const output_data_t *data) {
    for (uint8_t p = 0; p < profile->pgn_count; p++) {
        const pgn_descriptor_t *pgn = &profile->pgns[p];
        can_frame_t frame = {0};
        frame.id = ((uint32_t)pgn->priority << 26)
                 | (pgn->pgn << 8)
                 | profile->source_address;
        frame.is_extended = 1;
        frame.dlc = 8;
        memset(frame.data, 0xFF, 8);

        for (uint8_t s = 0; s < pgn->spn_count; s++) {
            const spn_descriptor_t *spn = &pgn->spns[s];
            int32_t val = spn_get_source(data, spn);
            // J1939 SPNs still require their own scale/offset packing (protocol-specific bit encoding)
            uint32_t packed = (uint32_t)(val * spn->direction
                              * spn->scale_num / spn->scale_den
                              + spn->add_offset);
            spn_write_bits(frame.data, packed,
                           spn->byte_offset, spn->bit_offset, spn->bit_length);
        }
        can->send(can->ctx, &frame);
    }
    return DRV_OK;
}

static int32_t spn_get_source(const output_data_t *data,
                              const spn_descriptor_t *spn) {
    switch (spn->source) {
        case SPN_SOURCE_AXIS:
            if ((spn->source_index >= data->axes_count) ||
                !data->axes[spn->source_index].valid) {
                return 0;
            }
            return data->axes[spn->source_index].value;
        case SPN_SOURCE_BUTTON:  return data->buttons;
        case SPN_SOURCE_SAFETY:  return data->safety_state;
        case SPN_SOURCE_CONST:   return spn->source_index;
        default:                 return 0;
    }
}

drv_status_t j1939_engine_on_rx_frame(const can_frame_t *frame) {
    (void)frame;
    return DRV_NOT_SUPPORTED;
}

drv_status_t j1939_engine_process_periodic(uint32_t now_ms) {
    (void)now_ms;
    return DRV_OK;
}
```

### 6.9 Protocol Bundle (Type-Safe Protocol Configuration Package)

Replace `void *config` with a tagged union to ensure at compile time that the protocol matches its configuration:

```c
// protocols/protocol_bundle.h
#pragma once
#include "canopen_profile.h"
#include "j1939_profile.h"

typedef enum {
    PROTOCOL_TYPE_CANOPEN,
    PROTOCOL_TYPE_J1939,
    PROTOCOL_TYPE_CAN_RAW,
} protocol_type_t;

typedef struct {
    const can_port_t        *can;
    uint8_t                  node_id;
    const canopen_profile_t *profile;
} canopen_config_t;

typedef struct {
    const can_port_t        *can;
    const j1939_profile_t   *profile;
} j1939_config_t;

typedef struct {
    const can_port_t        *can;
    uint32_t                 tx_id;
    uint8_t                  dlc;
} can_raw_config_t;

typedef struct {
    protocol_type_t type;
    union {
        canopen_config_t canopen;
        j1939_config_t   j1939;
        can_raw_config_t can_raw;
    };
} protocol_bundle_t;
```

### 6.10 Protocol Capabilities and Runtime Interface (v5 Improvement: send / on_rx_frame / process_periodic Three-Part Interface)

Capability bitmask is retained — runtime query of "does this support diagnostics" is still useful:

```c
// protocols/protocol_runtime.h
#include "output_adapter.h"     // v4: send receives output_data_t

typedef enum {
    PROTO_CAP_RECEIVABLE    = (1 << 0),
    PROTO_CAP_CONFIGURABLE  = (1 << 1),
    PROTO_CAP_DIAGNOSABLE   = (1 << 2),
    PROTO_CAP_SAFETY        = (1 << 3),
} protocol_capability_t;

typedef struct {
    protocol_type_t type;
    uint32_t capabilities;

    drv_status_t (*send)(const output_data_t *data);
    drv_status_t (*on_rx_frame)(const can_frame_t *frame);
    drv_status_t (*process_periodic)(uint32_t now_ms);
    drv_status_t (*get_bus_diag)(protocol_status_t *status);
} protocol_runtime_t;
```

Build the runtime interface from the bundle type at initialization (the sole switch; executed once at startup):

```c
// protocols/protocol_init.c
static protocol_runtime_t runtime;

drv_status_t protocol_init(const protocol_bundle_t *bundle) {
    runtime.type = bundle->type;

    switch (bundle->type) {
        case PROTOCOL_TYPE_CANOPEN:
            canopen_engine_init(bundle->canopen.can,
                                bundle->canopen.profile,
                                bundle->canopen.node_id);
            runtime.send         = canopen_engine_send;
            runtime.on_rx_frame  = canopen_engine_on_rx_frame;
            runtime.process_periodic = canopen_engine_process_periodic;
            runtime.get_bus_diag = canopen_engine_get_diag;
            runtime.capabilities = PROTO_CAP_RECEIVABLE
                                 | PROTO_CAP_CONFIGURABLE
                                 | PROTO_CAP_DIAGNOSABLE;
            break;

        case PROTOCOL_TYPE_J1939:
            j1939_engine_init(bundle->j1939.can, bundle->j1939.profile);
            runtime.send         = j1939_engine_send;
            runtime.on_rx_frame  = j1939_engine_on_rx_frame;
            runtime.process_periodic = j1939_engine_process_periodic;
            runtime.get_bus_diag = NULL;
            runtime.capabilities = PROTO_CAP_RECEIVABLE;
            break;

        case PROTOCOL_TYPE_CAN_RAW:
            can_raw_engine_init(&bundle->can_raw);
            runtime.send         = can_raw_engine_send;
            runtime.on_rx_frame  = can_raw_engine_on_rx_frame;
            runtime.process_periodic = can_raw_engine_process_periodic;
            runtime.get_bus_diag = NULL;
            runtime.capabilities = PROTO_CAP_RECEIVABLE;
            break;

        default:
            return DRV_INVALID_ARG;
    }
    return DRV_OK;
}

const protocol_runtime_t* protocol_get_runtime(void) {
    return &runtime;
}
```

Runtime usage (v4: in main_task, convert via output_adapter first, then pass to protocol engine):

```c
void app_run_cycle(const joystick_data_t *data,
                   const axis_output_map_t *maps, uint8_t map_count) {
    const protocol_runtime_t *proto = protocol_get_runtime();

    // v4: output_adapter conversion (pure function; reusable by HMI/logging, etc.)
    output_data_t out;
    output_adapt(data, maps, map_count, &out);

    proto->send(&out);
    proto->process_periodic(data->timestamp_ms);
    if (proto->capabilities & PROTO_CAP_DIAGNOSABLE) {
        protocol_status_t status;
        proto->get_bus_diag(&status);
    }
}
```

---

## VII. Product Configuration System (v5 Improvement: Split hardware/signal/protocol/safety profiles)

### 7.1 Product Descriptor Interface

```c
// config/product_descriptor.h
#pragma once
#include <stdint.h>
#include "axis_pipeline.h"
#include "protocol_bundle.h"
#include "axis_output_map.h"

// ---- Channel remapping: physical channel → logical function ----

typedef struct {
    uint8_t        physical_channel;  // Physical channel number in scan_buffer
    uint8_t        logical_index;     // Logical axis number
    calibrate_fn_t calibrate;         // Calibration strategy (calibrate_linear / calibrate_circular)
} axis_channel_map_t;

typedef struct {
    uint8_t physical_channel;         // Physical channel number in scan_buffer
    uint8_t logical_index;            // Logical button number
} btn_channel_map_t;

// ---- Safety channel configuration (new in v4, SIL 2 / PL d) ----

typedef struct {
    uint8_t  l2_adc_channels[MAX_AXES];   // Level 2 independent ADC channels
    uint16_t l2_center[MAX_AXES];         // Level 2 independent calibration center points
    int32_t  l2_tolerance[MAX_AXES];      // L1/L2 allowable deviation
    uint8_t  l2_fault_threshold;          // Consecutive deviation threshold count
} safety_channel_config_t;

// ---- New in v5: Safety Profile (single source of truth; replaces has_safety boolean) ----

typedef enum {
    PRODUCT_SAFETY_NONE = 0,
    PRODUCT_SAFETY_SIL2
} product_safety_level_t;

typedef struct {
    product_safety_level_t         level;
    const safety_channel_config_t *channel_cfg;
    const safety_channel_hw_profile_t *hw_profile;  // See 12.4.1
} safety_profile_t;

// ---- Profile split ----

typedef struct {
    uint8_t    resolution_bits;         // Sensor resolution (12 / 14)
    uint8_t    has_knob;
    uint8_t    has_force_feedback;
    const axis_channel_map_t *axis_map;
    uint8_t                   axis_map_count;
    const btn_channel_map_t  *btn_map;
    uint8_t                   btn_map_count;
} hardware_profile_t;

typedef struct {
    axis_pipeline_config_t pipeline;
} signal_profile_t;

typedef struct {
    const protocol_bundle_t *bundle;
    const axis_output_map_t *output_maps;
    uint8_t                  output_map_count;
} protocol_profile_t;

// ---- Product descriptor ----

typedef struct {
    const char *model_name;
    const hardware_profile_t *hw;
    const signal_profile_t   *signal;
    const protocol_profile_t *protocol;
    const safety_profile_t   *safety;
} product_descriptor_t;

// Sole external declaration — defined by the specific product configuration file
extern const product_descriptor_t product_config;
```

### 7.2 Individual Product Configuration Files

Adding a new product requires only adding a new file, without modifying any existing code:

```c
// config/products/js100.c — single-axis joystick, CAN output
#include "product_descriptor.h"
#include "calibrator.h"

static const axis_channel_map_t js100_axes[] = {
    { .physical_channel = 0, .logical_index = 0, .calibrate = calibrate_linear },
};

static const btn_channel_map_t js100_btns[] = {
    { .physical_channel = 2, .logical_index = 0 },
    { .physical_channel = 3, .logical_index = 1 },
};

static const hardware_profile_t js100_hw = {
    .resolution_bits = 12,
    .axis_map = js100_axes,
    .axis_map_count = 1,
    .btn_map = js100_btns,
    .btn_map_count = 2,
};

static const signal_profile_t js100_signal = {
    .pipeline = {
        .fault_threshold_low  = 50,
        .fault_threshold_high = 4046,
        .filter_alpha         = 0,
        .deadzone_threshold   = 300,
        .deadzone_fn          = deadzone_linear,
    },
};

const product_descriptor_t product_config = {
    .model_name = "JS-100",
    .hw         = &js100_hw,
    .signal     = &js100_signal,
};
```

```c
// config/products/js200.c — dual-axis joystick, CAN output
#include "product_descriptor.h"
#include "calibrator.h"

static const axis_channel_map_t js200_axes[] = {
    { .physical_channel = 0, .logical_index = 0, .calibrate = calibrate_linear },
    { .physical_channel = 1, .logical_index = 1, .calibrate = calibrate_linear },
};

static const btn_channel_map_t js200_btns[] = {
    { .physical_channel = 2, .logical_index = 0 },
    { .physical_channel = 3, .logical_index = 1 },
    { .physical_channel = 4, .logical_index = 2 },
    { .physical_channel = 5, .logical_index = 3 },
};

static const hardware_profile_t js200_hw = {
    .resolution_bits = 12,
    .axis_map = js200_axes,
    .axis_map_count = 2,
    .btn_map = js200_btns,
    .btn_map_count = 4,
};

static const signal_profile_t js200_signal = {
    .pipeline = {
        .fault_threshold_low  = 50,
        .fault_threshold_high = 4046,
        .filter_alpha         = 200,
        .deadzone_threshold   = 400,
        .deadzone_fn          = deadzone_linear,
    },
};

const product_descriptor_t product_config = {
    .model_name = "JS-200",
    .hw         = &js200_hw,
    .signal     = &js200_signal,
};
```

```c
// config/products/js300.c — three-axis Hall joystick + knob, safety type (v4: includes safety channel config)
#include "product_descriptor.h"
#include "calibrator.h"
#include "canopen_profile.h"

static const axis_channel_map_t js300_axes[] = {
    { .physical_channel = 0, .logical_index = 0, .calibrate = calibrate_circular },  // X Hall
    { .physical_channel = 1, .logical_index = 1, .calibrate = calibrate_circular },  // Y Hall
    { .physical_channel = 2, .logical_index = 2, .calibrate = calibrate_linear },    // Knob (potentiometer)
};

static const btn_channel_map_t js300_btns[] = {
    { .physical_channel = 3, .logical_index = 0 },
    { .physical_channel = 4, .logical_index = 1 },
    { .physical_channel = 5, .logical_index = 2 },
    { .physical_channel = 6, .logical_index = 3 },
    { .physical_channel = 7, .logical_index = 4 },
    { .physical_channel = 8, .logical_index = 5 },
    { .physical_channel = 9, .logical_index = 6 },
    { .physical_channel = 10, .logical_index = 7 },
};

// v4: output mappings (CANopen DS401 standard range)
static const axis_output_map_t js300_output_maps[] = {
    { .output_min = -10000, .output_max = 10000, .direction =  1 },  // X
    { .output_min = -10000, .output_max = 10000, .direction =  1 },  // Y
    { .output_min = -10000, .output_max = 10000, .direction =  1 },  // Knob
};

// v4: safety channel configuration (SIL 2 / PL d)
static const safety_channel_config_t js300_safety_ch = {
    .l2_adc_channels   = { ADC_CH_AXIS_X, ADC_CH_AXIS_Y, ADC_CH_KNOB },
    .l2_center         = { 8192, 8192, 2048 },  // Level 2 independent calibration center points
    .l2_tolerance      = { 200, 200, 100 },      // L1/L2 allowable deviation
    .l2_fault_threshold = 5,                      // Trigger after 5 consecutive deviations
};

// v5: hardware isolation descriptor
static const safety_channel_hw_profile_t js300_hw_profile = {
    .l1_adc_instance         = 1,    // L1 → ADC1
    .l2_adc_instance         = 2,    // L2 → ADC2 (independent instance)
    .l2_adc_channels         = { ADC_CH_AXIS_X, ADC_CH_AXIS_Y, ADC_CH_KNOB },
    .l2_vref_monitor_channel = ADC_CH_VREF,
    .has_independent_vref    = 1,
    .vref_tolerance_mv       = 30,
};

// v5: Safety Profile (single declaration; replaces has_safety boolean)
static const safety_profile_t js300_safety = {
    .level      = PRODUCT_SAFETY_SIL2,
    .channel_cfg = &js300_safety_ch,
    .hw_profile  = &js300_hw_profile,
};

// v4: Protocol Bundle
static const protocol_bundle_t js300_protocol = {
    .type = PROTOCOL_TYPE_CANOPEN,
    .canopen = {
        .can     = NULL,     // Filled at runtime by platform_get_can()
        .node_id = 0x10,
        .profile = &canopen_ds401,
    },
};

static const hardware_profile_t js300_hw = {
    .resolution_bits = 14,
    .axis_map = js300_axes,
    .axis_map_count = 3,
    .btn_map = js300_btns,
    .btn_map_count = 8,
};

static const signal_profile_t js300_signal = {
    .pipeline = {
        .fault_threshold_low  = 50,
        .fault_threshold_high = 16330,
        .filter_alpha         = 220,
        .deadzone_threshold   = 500,
        .deadzone_fn          = deadzone_scurve,
    },
};

static const protocol_profile_t js300_proto = {
    .bundle = &js300_protocol,
    .output_maps = js300_output_maps,
    .output_map_count = 3,
};

const product_descriptor_t product_config = {
    .model_name = "JS-300",
    .hw         = &js300_hw,
    .signal     = &js300_signal,
    .protocol   = &js300_proto,
    .safety     = &js300_safety,
};
```

```c
// config/products/js400.c — dual-axis Hall + force feedback; same hardware, different channel mapping
#include "product_descriptor.h"
#include "calibrator.h"

// Shares the same PCB as JS-300, but different channel mapping:
// Physical channel 0 → logical axis Y (reversed), physical channel 1 → logical axis X
static const axis_channel_map_t js400_axes[] = {
    { .physical_channel = 1, .logical_index = 0, .calibrate = calibrate_circular },  // CH1 → X
    { .physical_channel = 0, .logical_index = 1, .calibrate = calibrate_circular },  // CH0 → Y
};

static const btn_channel_map_t js400_btns[] = {
    { .physical_channel = 3,  .logical_index = 0 },
    { .physical_channel = 4,  .logical_index = 1 },
    { .physical_channel = 5,  .logical_index = 2 },
    { .physical_channel = 6,  .logical_index = 3 },
    { .physical_channel = 7,  .logical_index = 4 },
    { .physical_channel = 8,  .logical_index = 5 },
    { .physical_channel = 9,  .logical_index = 6 },
    { .physical_channel = 10, .logical_index = 7 },
    { .physical_channel = 11, .logical_index = 8 },
    { .physical_channel = 12, .logical_index = 9 },
    { .physical_channel = 13, .logical_index = 10 },
    { .physical_channel = 14, .logical_index = 11 },
};

static const hardware_profile_t js400_hw = {
    .resolution_bits = 14,
    .has_force_feedback = 1,
    .axis_map = js400_axes,
    .axis_map_count = 2,
    .btn_map = js400_btns,
    .btn_map_count = 12,
};

static const signal_profile_t js400_signal = {
    .pipeline = {
        .fault_threshold_low  = 50,
        .fault_threshold_high = 16330,
        .filter_alpha         = 240,
        .deadzone_threshold   = 400,
        .deadzone_fn          = deadzone_exponential,
    },
};

const product_descriptor_t product_config = {
    .model_name = "JS-400",
    .hw         = &js400_hw,
    .signal     = &js400_signal,
};
```

---

## VIII. Event Bus (v5 Rewrite: Async Dispatch + Dual-Priority Queue + Backpressure Strategy)

### 8.1 Design Motivation

In the original design, there was no communication mechanism between protocol plugins and safety monitoring. Actual requirements:
- When CAN Bus-Off occurs → safety monitor must be notified
- When safety state changes → protocol layer must send an emergency message
- When sensor fault occurs → diagnostics module must log the event

The v4 `event_bus_publish()` **synchronously** called all handlers, with the following risks:
- Slow handlers (e.g. `co_emcy_send()` waiting for CAN buffer) block the publisher
- Safety events and diagnostic events receive equal treatment; no priority distinction when queue is full
- `event_bus_init()` had two signatures in Chapters VIII and XI (no-arg vs. with backend), causing document drift

v5 adopts a "**publish-to-queue, ctrl_task dispatches**" model, eliminating synchronous blocking entirely.

### 8.2 Event Definitions and Priorities

```c
// event/event_bus.h — v5: async dispatch + dual-priority queue + backpressure strategy
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    EVT_SENSOR_FAULT,
    EVT_CAN_BUS_OFF,
    EVT_CAN_RECOVERED,
    EVT_SAFETY_STATE_CHANGED,
    EVT_CALIBRATION_COMPLETE,
    EVT_WATCHDOG_WARNING,
    EVT_DEVICE_STATE_CHANGED,
    EVT_MUTEX_TIMEOUT,
    EVT_MAX
} event_type_t;

typedef struct {
    event_type_t type;
    uint32_t     timestamp;
    union {
        uint8_t  sensor_id;
        uint8_t  safety_state;  // 0=normal, 1=degraded, 2=safe-stop
        uint32_t error_code;
        uint8_t  new_device_state;
    } data;
} event_t;

// ---- New in v5: event priority ----
typedef enum {
    EVT_PRIO_SAFETY = 0,   // Safety/fault events → critical_queue
    EVT_PRIO_FAULT  = 1,   // Fault events → critical_queue
    EVT_PRIO_STATE  = 2,   // State transitions → normal_queue
    EVT_PRIO_DIAG   = 3    // Diagnostic events → normal_queue
} event_priority_t;

// ---- New in v5: publish return status (backpressure feedback) ----
typedef enum {
    EVT_POST_OK,            // Successfully enqueued
    EVT_POST_DROPPED,       // Queue full; event dropped
    EVT_POST_OVERWROTE_OLD  // Queue full; overwrote oldest same-priority event
} event_post_status_t;

typedef void (*event_handler_t)(const event_t *evt);

// ---- Queue backend (injected by platform layer; supports cross-platform) ----
typedef struct {
    bool (*enqueue)(const event_t *evt, event_priority_t prio);
    bool (*dequeue)(event_t *evt);
} event_queue_backend_t;

#define EVENT_BUS_MAX_SUBSCRIBERS 4  // Max 4 subscribers per event type

// v5: unified initialization signature (removes old no-arg event_bus_init(void))
void event_bus_init(const event_queue_backend_t *backend);

void event_bus_subscribe(event_type_t type, event_handler_t handler);

// v5: async publish — writes to queue; does not call handler directly
event_post_status_t event_bus_post(const event_t *evt, event_priority_t prio);

// v5: called periodically by ctrl_task; dequeues and dispatches to subscribers
void event_bus_dispatch(void);
```

### 8.3 Dual-Priority Queue and Backpressure Strategy

```
┌──────────────────────────────────────────────────────────────────┐
│                Event Bus v5 Architecture                          │
│                                                                    │
│  Publishers (any task/ISR)                                         │
│       │                                                            │
│       │ event_bus_post(evt, prio)                                  │
│       ▼                                                            │
│  ┌─────────────────┐  ┌─────────────────┐                         │
│  │ critical_queue   │  │ normal_queue     │                         │
│  │ (SAFETY/FAULT)   │  │ (STATE/DIAG)     │                         │
│  │ depth: 8         │  │ depth: 16         │                         │
│  └────────┬────────┘  └────────┬────────┘                         │
│           │ priority dequeue   │ secondary dequeue                  │
│           └──────┬─────────────┘                                   │
│                  ▼                                                  │
│           ctrl_task: event_bus_dispatch()                           │
│                  │                                                  │
│                  ▼                                                  │
│           subscribers[evt->type].handlers[]                         │
└──────────────────────────────────────────────────────────────────┘
```

**Full-queue policy**:

| Queue | Behavior when full | Rationale |
|------|---------|------|
| `critical_queue` | Overwrite oldest non-safety event; if all are safety events, call `safe_shutdown_activate()` directly | Safety events must never be silently dropped |
| `normal_queue` | Drop the newest diagnostic event; increment `drop_counter` | Diagnostic events tolerate loss |

`diag_task` periodically reports queue watermark and `drop_counter` to monitor event system health.

### 8.4 Event Bus Implementation

```c
// event/event_bus.c — v5: async dispatch implementation
#include "event_bus.h"

static const event_queue_backend_t *g_backend;
static uint32_t g_drop_counter;

static struct {
    event_handler_t handlers[EVENT_BUS_MAX_SUBSCRIBERS];
    uint8_t count;
} subscribers[EVT_MAX];

void event_bus_init(const event_queue_backend_t *backend) {
    g_backend = backend;
    g_drop_counter = 0;
    for (uint8_t i = 0; i < EVT_MAX; i++) {
        subscribers[i].count = 0;
    }
}

void event_bus_subscribe(event_type_t type, event_handler_t handler) {
    if (type < EVT_MAX && subscribers[type].count < EVENT_BUS_MAX_SUBSCRIBERS) {
        subscribers[type].handlers[subscribers[type].count++] = handler;
    }
}

event_post_status_t event_bus_post(const event_t *evt, event_priority_t prio) {
    if (g_backend && g_backend->enqueue(evt, prio)) {
        return EVT_POST_OK;
    }
    // Handle full queue
    if (prio <= EVT_PRIO_FAULT && evt->type == EVT_SAFETY_STATE_CHANGED) {
        // Safety event failed to enqueue = last line of defense
        safe_shutdown_activate();
    }
    g_drop_counter++;
    return EVT_POST_DROPPED;
}

void event_bus_dispatch(void) {
    event_t evt;
    // Dequeue and dispatch (ctrl_task context, not ISR)
    while (g_backend && g_backend->dequeue(&evt)) {
        if (evt.type < EVT_MAX) {
            for (uint8_t i = 0; i < subscribers[evt.type].count; i++) {
                subscribers[evt.type].handlers[i](&evt);
            }
        }
    }
}
```

### 8.5 Handler Constraints and Async Deferral Pattern

**Hard constraint**: All event handlers must return within **50μs**. Operations that violate this constraint must be converted to the flag-and-defer pattern.

```c
// Example: flag-and-defer pattern for CANopen protocol
// The handler does NOT send CAN directly; it only sets a flag
static volatile uint8_t pending_emcy = 0;

static void on_safety_changed(const event_t *evt) {
    if (evt->data.safety_state == 2) {  // SAFE_STOP
        pending_emcy = 1;               // Set flag, do not block
    }
}

// proto_task sends in its own context
drv_status_t canopen_engine_process_periodic(uint32_t now_ms) {
    (void)now_ms;
    if (pending_emcy) {
        pending_emcy = 0;
        co_emcy_send(profile->emcy_code_sensor_fault);  // Send in proto_task context
    }
    co_node_process();
    return DRV_OK;
}
```

### 8.6 Event Subscriptions by Module

```c
// Safety monitor subscribes to CAN events and sensor events
void safety_monitor_init(...) {
    event_bus_subscribe(EVT_CAN_BUS_OFF, on_can_bus_off);
    event_bus_subscribe(EVT_SENSOR_FAULT, on_sensor_fault);
}

static void on_can_bus_off(const event_t *evt) {
    // CAN bus fault → enter degraded mode (only set flag inside handler)
    safety_enter_degraded_mode();
}

// Diagnostics module subscribes to all fault events
void diagnostics_init(void) {
    event_bus_subscribe(EVT_SENSOR_FAULT, diag_log_fault);
    event_bus_subscribe(EVT_CAN_BUS_OFF, diag_log_fault);
    event_bus_subscribe(EVT_WATCHDOG_WARNING, diag_log_fault);
}
```

---

## 9. Device State Machine

### 9.1 Design Motivation

The original design lacks explicit device lifecycle state management. Real embedded products must manage states such as boot self-test, pre-operational, normal operation, safe stop, calibration, and firmware update.

### 9.2 State Machine Definition

```c
// app/device_state.h
#pragma once
#include "event_bus.h"

typedef enum {
    STATE_BOOT,            // Boot self-test
    STATE_PRE_OPERATIONAL, // Initialization complete, waiting for enable
    STATE_OPERATIONAL,     // Normal operation
    STATE_SAFE_STOP,       // Safe stop (fault-triggered)
    STATE_CALIBRATION,     // Calibration mode
    STATE_FIRMWARE_UPDATE, // Firmware update
    STATE_MAX
} device_state_t;

typedef struct {
    device_state_t current;
    drv_status_t (*on_enter)(void);
    drv_status_t (*on_execute)(void);
    drv_status_t (*on_exit)(void);

    // State transition table
    struct {
        event_type_t   trigger;
        device_state_t next;
        bool (*guard)(void);  // Transition guard condition
    } transitions[8];
    uint8_t transition_count;
} state_handler_t;

void device_state_init(void);
void device_state_process(void);
device_state_t device_state_get_current(void);
```

```c
// app/device_state.c

static state_handler_t state_table[STATE_MAX];
static device_state_t  current_state = STATE_BOOT;

void device_state_init(void) {
    // Register state handlers
    state_table[STATE_BOOT] = (state_handler_t){
        .on_enter   = boot_self_test,
        .on_execute = boot_check_complete,
        .transitions = {
            { .trigger = EVT_DEVICE_STATE_CHANGED, .next = STATE_PRE_OPERATIONAL,
              .guard = boot_test_passed },
            { .trigger = EVT_SENSOR_FAULT, .next = STATE_SAFE_STOP },
        },
        .transition_count = 2,
    };

    state_table[STATE_OPERATIONAL] = (state_handler_t){
        .on_enter   = operational_enter,
        .on_execute = operational_run,
        .on_exit    = operational_exit,
        .transitions = {
            { .trigger = EVT_SAFETY_STATE_CHANGED, .next = STATE_SAFE_STOP },
            { .trigger = EVT_CALIBRATION_COMPLETE, .next = STATE_CALIBRATION,
              .guard = calibration_requested },
        },
        .transition_count = 2,
    };

    // ... other state configurations

    // Subscribe to all events that may trigger transitions
    event_bus_subscribe(EVT_SAFETY_STATE_CHANGED, on_state_event);
    event_bus_subscribe(EVT_SENSOR_FAULT, on_state_event);
    event_bus_subscribe(EVT_DEVICE_STATE_CHANGED, on_state_event);
}

static void on_state_event(const event_t *evt) {
    state_handler_t *handler = &state_table[current_state];
    for (uint8_t i = 0; i < handler->transition_count; i++) {
        if (handler->transitions[i].trigger == evt->type) {
            if (!handler->transitions[i].guard
                || handler->transitions[i].guard()) {
                // Execute state transition
                if (handler->on_exit) handler->on_exit();
                current_state = handler->transitions[i].next;
                state_handler_t *next = &state_table[current_state];
                if (next->on_enter) next->on_enter();
                break;
            }
        }
    }
}

void device_state_process(void) {
    if (state_table[current_state].on_execute) {
        state_table[current_state].on_execute();
    }
}
```

---

## 10. Composition Root (v5 Improvement: input_acq_t + can_port_t + Event Backend Injection)

### 10.1 Assembly Entry Point

```c
// app/app_compose.c — Responsible only for dependency assembly, no business logic

#include "product_descriptor.h"
#include "platform_registry.h"
#include "protocol_bundle.h"
#include "protocol_init.h"
#include "output_adapter.h"
#include "event_bus.h"
#include "device_state.h"
#include "input_acq.h"             // v5: replaces board_scan.h
#include "safety_channel.h"
#include "safety_comparator.h"
#include "safety_watchdog.h"

static joystick_t            joystick;
static scan_buffer_t         scan_buf;
static input_acq_t           input_acq;        // v5: replaces board_scanner_t
static safety_channel_ctx_t  l2_channel;
static safety_comparator_t   l2_comparator;
static safety_wdg_ctx_t      l2_wdg;

void app_compose(void) {
    // 1. v5: Initialize event bus (inject platform queue backend)
    event_bus_init(platform_get_event_queue_backend());

    // 2. Initialize device state machine
    device_state_init();

    // 3. v5: Initialize acquisition backend (replaces board_scan_init)
    input_acq_init(&input_acq, &product_config);
    input_acq.start(input_acq.ctx);  // Start (DMA backend starts continuous acquisition here)

    // 4. Assemble input sources from channel mapping table (physical channel → logical function)
    for (uint8_t i = 0; i < product_config.hw->axis_map_count; i++) {
        const axis_channel_map_t *m = &product_config.hw->axis_map[i];
        joystick.axis_sources[m->logical_index] =
            create_buf_axis_source(&scan_buf, m->physical_channel);
        joystick.axis_calibrate[m->logical_index] = m->calibrate;
    }
    for (uint8_t i = 0; i < product_config.hw->btn_map_count; i++) {
        const btn_channel_map_t *m = &product_config.hw->btn_map[i];
        joystick.btn_physical_map[m->logical_index] = m->physical_channel;
    }
    joystick.axes_count = product_config.hw->axis_map_count;
    joystick.btns_count = product_config.hw->btn_map_count;

    // 5. Initialize joystick signal pipeline
    joystick_init(&joystick, &product_config);

    // 6. Initialize protocol (Protocol Bundle, type-safe, no void*)
    protocol_init(product_config.protocol->bundle);

    // 7. v5: Compile-time consistency check (eliminates dual-source configuration risk)
    #if defined(SAFETY_ENABLED) && (SAFETY_ENABLED == 1)
        BUILD_ASSERT(product_config.safety != NULL,
            "Safety build requires safety profile in product_config");
    #else
        BUILD_ASSERT(product_config.safety == NULL,
            "Non-safety build must not include safety profile");
    #endif

    // 8. If safety model, initialize E-Gas three-layer safety architecture
    if (product_config.safety != NULL) {
        const safety_channel_config_t *sc = product_config.safety->channel_cfg;

        safety_channel_init(&l2_channel, &(safety_channel_init_cfg_t){
            .adc             = platform_get_adc_voltage(),
            .channel_indices = sc->l2_adc_channels,
            .axes_count      = product_config.hw->axis_map_count,
            .center          = sc->l2_center,
        });

        safety_comparator_init(&l2_comparator, &(safety_comparator_cfg_t){
            .tolerance       = sc->l2_tolerance,
            .fault_threshold = sc->l2_fault_threshold,
        });

        safety_wdg_init(&l2_wdg);
    }

    // 9. Initialize diagnostics
    diagnostics_init();
}

joystick_t*            app_get_joystick(void)      { return &joystick; }
input_acq_t*           app_get_input_acq(void)      { return &input_acq; }  // v5
scan_buffer_t*         app_get_scan_buf(void)       { return &scan_buf; }
const can_port_t*      app_get_can_port(void)       { return platform_get_can(); }  // v5
safety_channel_ctx_t*  app_get_l2_channel(void)     { return &l2_channel; }
safety_comparator_t*   app_get_l2_comparator(void)  { return &l2_comparator; }
safety_wdg_ctx_t*      app_get_l2_wdg(void)         { return &l2_wdg; }
```

---

## 11. RTOS Task Architecture

### 11.1 Design Motivation

Industrial joystick firmware must use an RTOS for the following reasons:

| Requirement | Bare-loop Problem | RTOS Solution |
|-------------|-------------------|---------------|
| Safety monitoring 2ms response | Blocked by protocol processing or signal chain | High-priority task preemption |
| CANopen heartbeat 1ms precision | High jitter in main loop | Dedicated task + hardware timer |
| Watchdog feeding timing | Long processing chain causes timeout | Dedicated high-priority task |
| CAN reception real-time | Polling loses frames | ISR + queue + task |

### 11.2 RTOS Abstraction Layer — Using CMSIS-RTOS v2

No custom abstraction layer is developed; instead, the official ARM **CMSIS-RTOS v2** standard API is used directly. Rationale:

| Comparison Item | Custom os_abstraction.h | CMSIS-RTOS v2 |
|-----------------|-------------------------|---------------|
| API coverage | Must implement each RTOS adapter manually | FreeRTOS / ThreadX / RTX5 all have official adapters |
| ISR safety | Must manually distinguish `_from_isr` variants | Auto-detects IPSR register; same API for both contexts |
| Absolute delay | Undefined (causes period drift) | `osDelayUntil()` built-in |
| Priority inheritance | Requires extra convention | `osMutexNew()` with `osMutexPrioInherit` flag |
| Platform compatibility | STM32 / S32K require separate implementations | STM32 CubeMX generates by default; S32K FreeRTOS supported directly |
| Maintenance cost | Self-maintained | ARM officially maintained, zero cost |
| MISRA compliance | Must ensure manually | CMSIS headers have already passed MISRA scan |

Core APIs used in this project:

```c
#include "cmsis_os2.h"

// ---- Task management ----
osThreadId_t osThreadNew(osThreadFunc_t func, void *argument,
                          const osThreadAttr_t *attr);

// ---- Mutex (with priority inheritance) ----
osMutexId_t osMutexNew(const osMutexAttr_t *attr);  // attr includes osMutexPrioInherit
osStatus_t  osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout);
osStatus_t  osMutexRelease(osMutexId_t mutex_id);

// ---- Message queue (unified API for ISR/task) ----
osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size,
                                      const osMessageQueueAttr_t *attr);
osStatus_t osMessageQueuePut(osMessageQueueId_t mq_id, const void *msg_ptr,
                              uint8_t msg_prio, uint32_t timeout);
osStatus_t osMessageQueueGet(osMessageQueueId_t mq_id, void *msg_ptr,
                              uint8_t *msg_prio, uint32_t timeout);

// ---- Timing and delay ----
osStatus_t osDelay(uint32_t ticks);
osStatus_t osDelayUntil(uint32_t ticks);  // Absolute delay, eliminates period drift
uint32_t   osKernelGetTickCount(void);

// ---- Kernel control ----
osStatus_t osKernelInitialize(void);
osStatus_t osKernelStart(void);  // Does not return
```

The FreeRTOS adaptation layer is provided by the `CMSIS/RTOS2/FreeRTOS/` directory (automatically included by CubeMX); no manual implementation required.

### 11.3 Task Decomposition (v5 Improvement: safety_task Streamlined + New ctrl_task)

```
┌──────────────────────────────────────────────────────────────────────┐
│                     RTOS Task Architecture (v5)                      │
├────────────┬──────────┬────────┬─────────────────────────────────────┤
│ Task Name  │ Priority │ Period │ Responsibilities                    │
├────────────┼──────────┼────────┼─────────────────────────────────────┤
│ safety_task│ Realtime │ 2 ms   │ L2 acquisition+comparison+          │
│            │          │        │ conditional watchdog feed+           │
│            │          │        │ sliced diagnostics                   │
│ ctrl_task  │ AboveNorm│ 5 ms   │ device_state_process+event dispatch │
│ main_task  │ Normal   │ 10 ms  │ L1 joystick acq→signal chain→       │
│            │          │        │ output_adapt→post to TX queue        │
│ proto_task │ Normal+1 │ 1 ms   │ Protocol stack single-thread        │
│            │          │        │ processing + CAN TX/RX              │
│ diag_task  │ Low      │ 100 ms │ Diagnostic logging, NV storage      │
│            │          │        │ flush, task health check            │
├────────────┼──────────┼────────┼─────────────────────────────────────┤
│ ISR: CAN RX│ HW IRQ   │ Event  │ CAN receive → enqueue               │
│ ISR: ADC   │ HW IRQ   │ DMA    │ ADC DMA complete → notify main_task │
│ ISR: Timer │ HW IRQ   │ 1 ms   │ RTOS system tick                    │
├────────────┼──────────┼────────┼─────────────────────────────────────┤
│ HW: WWDG   │ Indep HW │ ~50ms  │ L3: conditional feed timeout →      │
│            │          │        │ hardware reset                       │
│ HW: STO    │ Indep HW │ Immed. │ L3: safe_shutdown pulls GPIO low    │
└────────────┴──────────┴────────┴─────────────────────────────────────┘
```

> **v5 Change**: `device_state_process()` and `event_bus_dispatch()` have been moved from `safety_task` to the newly added `ctrl_task`, making safety_task a pure safety closed loop with precisely measurable WCET.

### 11.4 Task Creation and Startup

```c
// app/app_tasks.c — RTOS task creation (CMSIS-RTOS v2)

#include "cmsis_os2.h"
#include "app_compose.h"

// Task attribute definitions (stack size in bytes)
static const osThreadAttr_t safety_attr = {
    .name = "safety",  .priority = osPriorityRealtime,  .stack_size = 384 * 4,
};
static const osThreadAttr_t main_attr = {
    .name = "main",    .priority = osPriorityNormal,    .stack_size = 512 * 4,
};
static const osThreadAttr_t proto_attr = {
    .name = "proto",   .priority = osPriorityAboveNormal, .stack_size = 384 * 4,
};
static const osThreadAttr_t diag_attr = {
    .name = "diag",    .priority = osPriorityLow,       .stack_size = 256 * 4,
};

static osMessageQueueId_t g_proto_tx_queue;

// v5 new: ctrl_task — control-plane operations (device state machine + event dispatch)
static const osThreadAttr_t ctrl_attr = {
    .name = "ctrl",    .priority = osPriorityAboveNormal, .stack_size = 256 * 4,
};

void app_tasks_create(void) {
    // Assemble dependencies
    app_compose();

    // Create protocol TX queue
    g_proto_tx_queue = osMessageQueueNew(8, sizeof(output_data_t), NULL);

    // Create tasks
    osThreadNew(safety_task_entry, NULL, &safety_attr);
    osThreadNew(ctrl_task_entry,   NULL, &ctrl_attr);   // v5 new
    osThreadNew(main_task_entry,   NULL, &main_attr);
    osThreadNew(proto_task_entry,  NULL, &proto_attr);
    osThreadNew(diag_task_entry,   NULL, &diag_attr);
}
```

### 11.5 Task Responsibilities (v5 Refactor: Control Plane Split + Protocol Stack Single-Threading)

```c
// safety_task — v5: Minimal safety closed loop (L2 acquisition + comparison + watchdog + sliced diagnostics only)
// Rule: mutex is prohibited; shared data accessed only via atomic/volatile
// WCET budget: L2 sample+compute 200μs, comparator 50μs, WDG 20μs, diag slice 300μs, margin >50%
static void safety_task_entry(void *param) {
    safety_channel_ctx_t *l2_ch  = app_get_l2_channel();
    safety_comparator_t  *l2_cmp = app_get_l2_comparator();
    safety_wdg_ctx_t     *l2_wdg = app_get_l2_wdg();
    uint32_t prev_tick = osKernelGetTickCount();

    for (;;) {
        // ── Level 2: Independent sensor acquisition + simplified computation ──
        safety_channel_output_t l2_out;
        safety_channel_compute(l2_ch, &l2_out);

        // ── Level 2: Read Level 1 output and compare ──
        int32_t l1_axes[MAX_AXES];
        safety_shared_read_l1_output(l1_axes);  // atomic read

        compare_result_t cmp = safety_compare(
            l2_cmp, l1_axes, &l2_out, l2_ch->axes_count);

        switch (cmp) {
            case COMPARE_OK:
                safety_wdg_update_token(l2_wdg);   // Allow watchdog feed
                break;
            case COMPARE_DEGRADED:
                event_bus_post(&(event_t){
                    .type = EVT_SAFETY_STATE_CHANGED,
                    .data.safety_state = 1 }, EVT_PRIO_SAFETY);  // Async enqueue
                break;
            case COMPARE_FAIL:
                safe_shutdown_activate();             // L3: Direct hardware shutdown
                break;
        }

        // ── Level 3: Conditional watchdog feed ──
        safety_wdg_service(l2_wdg);

        // ── Additional diagnostics (sliced execution, budget-constrained) ──
        safety_diagnostics_step(g_diag_step++, 300 /*budget_us*/);

        // v5: device_state_process() and event_bus_dispatch() moved to ctrl_task
        // safety_task no longer executes any control-plane logic

        osDelayUntil(prev_tick += 2);  // Absolute delay, ensures constant 2ms period
    }
}

// ctrl_task — v5 new: Control-plane operations (AboveNormal priority, 5ms period)
// Takes over device_state_process() and event dispatch from former safety_task
static void ctrl_task_entry(void *param) {
    uint32_t prev_tick = osKernelGetTickCount();
    for (;;) {
        device_state_process();
        event_bus_dispatch();   // Dequeue from dual-priority queue and dispatch
        osDelayUntil(prev_tick += 5);
    }
}

// main_task — Level 1 functional channel (v5: input_acq_t replaces board_scanner_t)
static void main_task_entry(void *param) {
    joystick_t *js         = app_get_joystick();
    input_acq_t *acq       = app_get_input_acq();     // v5: replaces board_scanner_t
    scan_buffer_t *buf     = app_get_scan_buf();
    joystick_data_t data;
    output_data_t   out;
    uint32_t acq_seq = 0;
    uint32_t prev_tick = osKernelGetTickCount();

    for (;;) {
        cf_marker_checkpoint(CF_MAIN_START);

        if (device_state_get_current() == STATE_OPERATIONAL) {
            // 1. v5: Obtain consistent snapshot via input_acq_t (atomic + sequence number)
            uint32_t new_seq;
            drv_status_t st = acq->acquire_latest(acq->ctx, buf, &new_seq);

            if (st == DRV_OK) {
                // Detect frame drops (optional: for diagnostics)
                if (new_seq - acq_seq > 1) {
                    // Log frame drop event (non-blocking)
                }
                acq_seq = new_seq;
            }
            // DRV_BUSY: Continue with last snapshot (synchronous backends won't return this)
            uint32_t now_ms = osKernelGetTickCount();
            if ((buf->timestamp_ms != 0u) &&
                ((now_ms - buf->timestamp_ms) > INPUT_MAX_AGE_MS)) {
                event_bus_post(&(event_t){ .type = EVT_INPUT_STALE }, EVT_PRIO_FAULT);
                osDelayUntil(prev_tick += 10);
                continue;
            }
            cf_marker_checkpoint(CF_MAIN_SCAN_DONE);

            // 2. Joystick update: buf_axis_source / buf_btn_source read from buffer
            joystick_update(js);
            joystick_get_data(js, &data);
            cf_marker_checkpoint(CF_MAIN_SIGNAL_DONE);

            // 3. Write to shared comparison region (for Level 2 comparison)
            safety_shared_write_l1_output(data.axes, data.axes_count,
                                          buf->seq, buf->timestamp_ms);

            // 4. output_adapter conversion (pure function)
            output_adapt(&data, product_config.protocol->output_maps,
                         product_config.protocol->output_map_count, &out);

            // 5. v5: main_task does NOT touch protocol internal state directly;
            //    only posts to proto_tx_queue
            if (osMessageQueuePut(g_proto_tx_queue, &out, 0, 0) != osOK) {
                event_bus_post(&(event_t){ .type = EVT_PROTO_TX_DROPPED }, EVT_PRIO_DIAG);
            }
        }

        cf_marker_checkpoint(CF_MAIN_END);
        osDelayUntil(prev_tick += 10);  // Absolute delay, ensures constant 10ms period
    }
}

// proto_task — Protocol stack real-time processing (v5: can_port_t.wait_recv replaces callback pattern)
static void proto_task_entry(void *param) {
    const protocol_runtime_t *proto = protocol_get_runtime();
    const can_port_t *can = app_get_can_port();     // v5: replaces direct callback
    can_frame_t rx_frame;
    output_data_t tx_out;

    for (;;) {
        while (osMessageQueueGet(g_proto_tx_queue, &tx_out, NULL, 0) == osOK) {
            proto->send(&tx_out);
        }

        // v5: Block waiting for CAN frame (platform layer handles ISR→queue conversion internally)
        // 1ms timeout ensures CANopen heartbeat/NMT timer processing
        if (can->wait_recv(can->ctx, &rx_frame, 1) == DRV_OK) {
            proto->on_rx_frame(&rx_frame);
        }
        proto->process_periodic(osKernelGetTickCount());
    }
}

// diag_task — Low-priority background service (unchanged from v4)
static void diag_task_entry(void *param) {
    uint32_t prev_tick = osKernelGetTickCount();

    for (;;) {
        diagnostics_update();
        nv_storage_flush();   // Write cached data to Flash/EEPROM
        logging_flush();
        task_health_check();  // Check each task's alive flag
        osDelayUntil(prev_tick += 100);  // Absolute delay, ensures constant 100ms period
    }
}
```

### 11.6 Shared Data Protection (v4 Update)

```
┌──────────────────────────────────────────────────────────────────┐
│               Shared Resource Protection Strategy (v4)           │
├──────────────────────┬──────────────────┬────────────────────────┤
│ Shared Data          │ Writer           │ Protection Mechanism    │
├──────────────────────┼──────────────────┼────────────────────────┤
│ proto_tx_queue       │ main_task        │ CMSIS message queue    │
│                      │ (reader:         │ Single consumer;       │
│                      │  proto_task)     │ protocol stack serial  │
├──────────────────────┼──────────────────┼────────────────────────┤
│ safety_shared_t (v4) │ main_task (L1)   │ _Atomic (C11)          │
│                      │ (reader:         │ L1 writes, L2 reads;   │
│                      │  safety_task)    │ lock-free              │
├──────────────────────┼──────────────────┼────────────────────────┤
│ protocol_status      │ proto_task       │ atomic / single writer  │
│                      │ (reader:         │ safety task must not   │
│                      │  safety_task)    │ use mutex              │
├──────────────────────┼──────────────────┼────────────────────────┤
│ device_state         │ safety_task      │ atomic / single writer  │
│                      │ (reader: all)    │                        │
├──────────────────────┼──────────────────┼────────────────────────┤
│ CAN RX frames        │ ISR              │ async_queue_t           │
│                      │ (reader:         │ v5: managed by         │
│                      │  proto_task)     │ can_port_t internally  │
├──────────────────────┼──────────────────┼────────────────────────┤
│ event_bus events     │ Any task/ISR     │ See 11.7               │
├──────────────────────┼──────────────────┼────────────────────────┤
│ task_alive_flags     │ Each task sets   │ atomic bit operation    │
│                      │ (reader:         │ Timeout = alarm        │
│                      │  diag_task)      │                        │
├──────────────────────┼──────────────────┼────────────────────────┤
│ cf_actual (v4)       │ main_task (L1)   │ Single writer          │
│                      │ (reader:         │ (main_task only)       │
│                      │  safety_task)    │ safety_task checks and │
│                      │                  │ resets during cond.    │
│                      │                  │ watchdog feed          │
└──────────────────────┴──────────────────┴────────────────────────┘
```

### 11.7 Event Bus Architecture (v5 Improvement: Control Plane/Data Plane Separation + Queue Backend Injection)

#### 11.7.1 Control Plane vs. Data Plane Separation Principle

The v4 event_bus carried both state events and high-frequency data. v5 clearly separates them:

| Plane | Content | Transport Mechanism | Frequency |
|-------|---------|---------------------|-----------|
| **Control plane** | Fault events, state transitions, mode changes, CAN Bus-Off | `event_bus` (observer pattern) | Low-frequency (event-driven) |
| **Data plane** | ADC samples, CAN RX frames, SPI sensor values | `input_acq_t` / `can_port_t` (snapshot + queue) | High-frequency (periodic) |

**Hard constraint**: The event_bus **does NOT carry high-frequency raw data**. Periodic samples from ADC/CAN/SPI go through `input_acq_t.acquire_latest()` or `can_port_t.wait_recv()`.

#### 11.7.2 ISR Safety: Queue Backend Injection

The v4 `event_bus_publish_from_isr()` relied on CMSIS-RTOS v2's automatic IPSR register detection (ARM Cortex-M exclusive). v5 changes the queue backend to an injectable strategy, supporting cross-platform use:

```c
// event/event_bus.h — v5: Queue backend injection

// Event queue backend (injected by platform layer, supports cross-platform;
// signature identical to Chapter 8 event_bus_init)
// Note: v5 removes the old event_bus_init(void) and event_bus_publish() synchronous interface.
// All code uses event_bus_post() for async enqueue + event_bus_dispatch() in ctrl_task.
typedef struct {
    bool (*enqueue)(const event_t *evt, event_priority_t prio);  // ISR/task-safe enqueue
    bool (*dequeue)(event_t *evt);                                // Task-context dequeue
} event_queue_backend_t;

// v5 unified signature (see Chapter 8, section 8.2)
void event_bus_init(const event_queue_backend_t *backend);

// Async publish: write to dual-priority queue (callable from both ISR and task context)
event_post_status_t event_bus_post(const event_t *evt, event_priority_t prio);

// Called periodically by ctrl_task; dequeues and dispatches to subscribers
void event_bus_dispatch(void);
```

Platform-specific implementations:

```c
// platform/stm32g4/event_queue_cmsis.c — ARM platform (v5: dual queue)
#include "cmsis_os2.h"
static osMessageQueueId_t critical_queue;  // Depth 8
static osMessageQueueId_t normal_queue;    // Depth 16

static bool cmsis_enqueue(const event_t *evt, event_priority_t prio) {
    osMessageQueueId_t q = (prio <= EVT_PRIO_FAULT) ? critical_queue : normal_queue;
    return osMessageQueuePut(q, evt, 0, 0) == osOK;
}
static bool cmsis_dequeue(event_t *evt) {
    // Dequeue critical_queue first
    if (osMessageQueueGet(critical_queue, evt, NULL, 0) == osOK) return true;
    return osMessageQueueGet(normal_queue, evt, NULL, 0) == osOK;
}
const event_queue_backend_t event_queue_cmsis = {
    .enqueue = cmsis_enqueue,
    .dequeue = cmsis_dequeue,
};

// platform/test/event_queue_mock.c — PC test platform (v5: simplified dual-queue mock)
static event_t crit_buf[8], norm_buf[16];
static uint8_t crit_head, crit_tail, norm_head, norm_tail;
static bool mock_enqueue(const event_t *evt, event_priority_t prio) {
    if (prio <= EVT_PRIO_FAULT) {
        crit_buf[crit_head++ & 7] = *evt;
    } else {
        norm_buf[norm_head++ & 15] = *evt;
    }
    return true;
}
static bool mock_dequeue(event_t *evt) {
    if (crit_head != crit_tail) { *evt = crit_buf[crit_tail++ & 7]; return true; }
    if (norm_head != norm_tail) { *evt = norm_buf[norm_tail++ & 15]; return true; }
    return false;
}
const event_queue_backend_t event_queue_mock = {
    .enqueue = mock_enqueue,
    .dequeue = mock_dequeue,
};
```

> **Safety constraint unchanged**: The L2 safety channel does not depend on event_bus; at most it reads the shared snapshot or samples independently. The L3 hardware shutdown path (GPIO directly driving safety relay) does not go through event_bus/RTOS at all.

### 11.8 Timing Requirements and Priority Inversion Prevention

```
Timeline (worst case):

0ms ─── safety_task begins execution (2ms period)
         Safety monitoring + watchdog feed
0.3ms ── safety_task completes, yields CPU
         proto_task gets CPU (1ms period)
         CANopen protocol stack processing
0.8ms ── proto_task completes
         main_task gets CPU (10ms period)
         Joystick acquisition + signal chain + protocol send
2ms ──── safety_task preempts main_task
         Ensures safety monitoring is never blocked
```

Key design rules:
- **Mutex is prohibited** in `safety_task` (prevents priority inversion from causing safety timeout)
- Safety task only reads data, using atomic or volatile access
- `protocol_status` changed to atomic single-writer (proto_task writes, safety_task reads lock-free)
- CAN RX interrupt handling time < 5μs (only performs `osMessageQueuePut` enqueue)
- Protocol stack shared state is no longer protected by mutex; instead converged to single-thread access via `proto_tx_queue`
- All periodic tasks use `osDelayUntil()` for absolute delay (prevents period drift)

### 11.9 Interrupt Priority Matrix

v5 explicitly freezes interrupt priorities for input sampling, protocol reception, and safety shutdown, preventing `NVIC` configuration drift during porting:

| Interrupt Source | Preempt Priority | Sub-priority | Allowed Actions | Prohibited | Notes |
|-----------------|------------------|--------------|-----------------|------------|-------|
| `EXTI/SAFE_SHUTDOWN` | 0 | 0 | Directly pull STO low / set fault latch | RTOS API prohibited | Highest priority; ensures shutdown path is never delayed |
| `WWDG/EWI` | 1 | 0 | Log watchdog pre-warning, trigger fault latch | No blocking | Last-resort logging before failure |
| `CAN_RX` | 2 | 0 | Read hardware mailbox, enqueue to `can_port_t` | Protocol parsing prohibited | ISR only transfers frames, never touches protocol state |
| `ADC_DMA_TC` | 3 | 0 | Publish sample-complete sequence number/notify | Signal processing prohibited | Only publishes new snapshot |
| `SPI_DMA_TC` | 3 | 1 | Publish new sensor frame | Value conversion prohibited | Same level as ADC, lower sub-priority |
| `SysTick` | 7 | 0 | RTOS Tick | No business logic | Must remain below real-time peripheral ISRs |

Constraints:
- `CAN_RX` and `ADC_DMA_TC` must be higher than `SysTick`; otherwise, 1ms period tasks would inversely block hardware transfer paths.
- All ISRs that may call CMSIS-RTOS API must satisfy the `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` constraint.
- Safety-shutdown-related ISRs do not enter `event_bus` and do not depend on any queues or locks.

---

## 12. Functional Safety Architecture (v4 New: E-Gas Three-Layer Model, SIL 2 / PL d)

### 12.1 Safety Requirements vs. Standards

Industrial joysticks are used in safety-critical applications such as construction machinery and agricultural equipment, requiring compliance with:

| Standard | Level | Core Requirements |
|----------|-------|-------------------|
| IEC 61508 | SIL 2 | DC ≥ 90%; safety function independent from control function; program flow monitoring |
| ISO 13849 | PL d, Cat.3 | Dual-channel structure; no single fault causes loss of safety function |

### 12.2 Why v3 Was Non-Compliant

The v3 `safety_task` was a "bystander" — executing at high priority but sharing the RTOS, memory space, ADC driver, and event bus with `main_task`. **Priority isolation ≠ independence**. RTOS kernel bugs, stack overflow, and system tick loss are all common-cause failure points.

### 12.3 E-Gas Three-Layer Architecture Adaptation

```
┌──────────────────────────────────────────────────────────────────┐
│                            MCU                                    │
│                                                                   │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │  Level 1: Functional Channel (main_task, 10ms)           │    │
│  │  Scanner → axis_pipeline → output_adapter                │    │
│  │  → protocol_engine → CAN TX                              │    │
│  │                        │                                 │    │
│  │                        │ atomic write to shared compare  │    │
│  └────────────────────────┼─────────────────────────────────┘    │
│                           ↓                                       │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Level 2: Safety Monitoring Channel (safety_task, 2ms)       │ │
│  │                                                              │ │
│  │  ┌───────────────┐  ┌──────────────────────────────────┐   │ │
│  │  │ Independent   │  │ Comparator                        │   │ │
│  │  │ Sensor Acq.   │  │ |L1_output - L2_output| < ε ?    │   │ │
│  │  │ ADC indep.    │  │ YES → alive_token++               │   │ │
│  │  │ sampling      │  │ NO  → safe_shutdown_activate()    │   │ │
│  │  └──────┬────────┘  └──────────────────┬───────────────┘   │ │
│  │         ↓                               │                   │ │
│  │  ┌───────────────┐                      │                   │ │
│  │  │ Simplified    │──────────────────────┘                   │ │
│  │  │ signal compute│                                          │ │
│  │  │ (independent  │                                          │ │
│  │  │  code impl.)  │                                          │ │
│  │  └───────────────┘                                          │ │
│  │  Extra diagnostics: RAM CRC / clock cross-check /           │ │
│  │  stack check / program flow monitoring                      │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                           │ alive_token                            │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │  Level 3: Hardware Monitoring Layer                          │ │
│  │  Window WDG: Feed only when token correct + flow flags OK    │ │
│  │  Independent shutdown: GPIO directly drives safety relay/STO │ │
│  │  (does not go through RTOS/event_bus)                        │ │
│  └──────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────────┘
```

### 12.4 Level 2 — Independent Safety Computation Channel

**Core principle: Code independence, data independence, execution independence.**

The Level 2 signal computation **must be independently implemented** and must not reuse Level 1's `axis_pipeline` code. IEC 61508-3 clause 7.4.2.2 requires software diversity — even if the algorithms are identical, they must be independently implemented to avoid systematic failures.

#### 12.4.1 L1/L2 Hardware Independence Requirements (v5 New)

If L1/L2 share the same ADC instance and reference voltage, a Vref drift will cause both channels to err simultaneously — this is a **Common Cause Failure (CCF)** that directly undermines the E-Gas dual-channel independence assumption.

**Hardware isolation levels** (in order of recommendation):

| Level | Approach | Independence | Applicable Scenario |
|-------|----------|--------------|---------------------|
| Optimal | L2 uses an independent ADC instance (e.g., L1→ADC1, L2→ADC2) | High | STM32G4 and other multi-ADC chips |
| Sub-optimal | Same MCU, different ADC + independent sampling trigger + independent Vref monitoring | Medium | Dual ADC but shared partial power supply |
| Minimum | Same ADC, different sampling sequence (must declare residual risk) | Low | Single-ADC chips (e.g., STM32F103); requires external compensation |

```c
// config/product_descriptor.h — v5 new: L2 hardware isolation descriptor
typedef struct {
    uint8_t  l2_adc_instance;              // ADC instance used by L2 (e.g., ADC2)
    uint8_t  l1_adc_instance;              // ADC instance used by L1 (e.g., ADC1)
    uint8_t  l2_adc_channels[MAX_AXES];    // L2 independent ADC channels
    uint8_t  l2_vref_monitor_channel;      // Vref monitoring channel (VREFINT)
    uint8_t  has_independent_vref;         // Whether independent Vref monitoring is available
    uint16_t vref_tolerance_mv;            // Vref tolerance (typical ±30mV)
} safety_channel_hw_profile_t;
```

#### 12.4.2 L2 Channel Interface

```c
// safety/safety_channel.h — Level 2 independent computation channel
#pragma once
#include <stdint.h>
#include "drv_common.h"
#include "adc_voltage_reader.h"

#define SAFETY_MV_LOW   100    // Open-circuit detection lower limit (mV)
#define SAFETY_MV_HIGH  3200   // Short-circuit detection upper limit (mV)
#define VREF_NOMINAL_MV 1200   // STM32 internal VREFINT typical value

typedef struct {
    const adc_voltage_reader_t *l2_adc;    // v5: L2 dedicated ADC instance
    uint8_t   channel_indices[MAX_AXES];
    uint8_t   axes_count;
    uint16_t  center[MAX_AXES];            // L2 independent calibration parameters (separate storage)
    uint8_t   vref_channel;                // Vref monitoring channel
    uint16_t  vref_tol;                    // Vref tolerance (mV)
} safety_channel_ctx_t;

typedef struct {
    int32_t axes[MAX_AXES];
    uint8_t axes_count;
    uint8_t sensor_ok;             // Independent sensor diagnostics
} safety_channel_output_t;

void safety_channel_init(safety_channel_ctx_t *ctx,
                          const safety_channel_init_cfg_t *cfg);
void safety_channel_compute(safety_channel_ctx_t *ctx,
                             safety_channel_output_t *out);
```

#### 12.4.3 L2 Channel Implementation (Including VREFINT Monitoring)

```c
// safety/safety_channel.c — Independent implementation; does not call any signal chain functions
#include "safety_channel.h"

void safety_channel_compute(safety_channel_ctx_t *ctx,
                             safety_channel_output_t *out) {
    out->axes_count = ctx->axes_count;
    out->sensor_ok = 1;

    // v5 new: Check L2 ADC Vref is within tolerance first
    uint32_t vref_mv = 0;
    ctx->l2_adc->read_mv(ctx->vref_channel, &vref_mv);
    if (vref_mv < VREF_NOMINAL_MV - ctx->vref_tol ||
        vref_mv > VREF_NOMINAL_MV + ctx->vref_tol) {
        out->sensor_ok = 0;
        return;  // Vref abnormal; L2 data is untrustworthy
    }

    for (uint8_t i = 0; i < ctx->axes_count; i++) {
        uint32_t mv = 0;
        drv_status_t ret = ctx->l2_adc->read_mv(ctx->channel_indices[i], &mv);

        // Independent open-circuit/short-circuit detection (does not reuse fault_detector.c)
        if (ret != DRV_OK || mv < SAFETY_MV_LOW || mv > SAFETY_MV_HIGH) {
            out->sensor_ok = 0;
            out->axes[i] = 0;
            continue;
        }

        // Independent simplified calibration (does not reuse calibrator.c)
        // No filtering/deadzone needed — used for comparison only; precision less critical than Level 1
        uint16_t raw_approx = (uint16_t)(mv * 4095u / 3300u);
        out->axes[i] = (int32_t)raw_approx - (int32_t)ctx->center[i];
    }
}
```

**Hardware level**: In CubeMX configuration, L1 is assigned ADC1 and L2 is assigned ADC2, each with an independent reference voltage channel. If the chip has only one ADC (e.g., STM32F103), L2 must use an external SPI ADC (e.g., MCP3208), and the residual risk must be declared in `safety_channel_hw_profile_t`.

### 12.5 Level 2 — Comparator

```c
// safety/safety_comparator.h
#pragma once
#include <stdint.h>
#include "safety_channel.h"

typedef enum {
    COMPARE_OK,
    COMPARE_DEGRADED,     // Out of tolerance but below threshold
    COMPARE_FAIL,         // Persistently out of tolerance → safe stop
} compare_result_t;

typedef struct {
    int32_t  tolerance[MAX_AXES];    // Allowed L1-L2 deviation (per axis)
    uint8_t  fault_count[MAX_AXES];  // Consecutive out-of-tolerance count
    uint8_t  fault_threshold;        // Allowed consecutive out-of-tolerance count
} safety_comparator_t;

compare_result_t safety_compare(
    safety_comparator_t *cmp,
    const int32_t *l1_axes,
    const safety_channel_output_t *l2,
    uint8_t axes_count);
```

```c
// safety/safety_comparator.c
#include "safety_comparator.h"

compare_result_t safety_compare(
    safety_comparator_t *cmp,
    const int32_t *l1_axes,
    const safety_channel_output_t *l2,
    uint8_t axes_count)
{
    if (!l2->sensor_ok) {
        return COMPARE_FAIL;
    }

    compare_result_t worst = COMPARE_OK;

    for (uint8_t i = 0; i < axes_count; i++) {
        int32_t diff = l1_axes[i] - l2->axes[i];
        if (diff < 0) { diff = -diff; }

        if (diff > cmp->tolerance[i]) {
            cmp->fault_count[i]++;
            if (cmp->fault_count[i] >= cmp->fault_threshold) {
                worst = COMPARE_FAIL;
            } else if (worst < COMPARE_DEGRADED) {
                worst = COMPARE_DEGRADED;
            }
        } else {
            cmp->fault_count[i] = 0;
        }
    }
    return worst;
}
```

### 12.6 Level 3 — Conditional Watchdog Feed + Program Flow Monitoring

The v3 `watchdog_feed()` was **unconditional** — it fed the watchdog as long as safety_task was running. This does not satisfy SIL 2 requirements. v4 changes to conditional feeding:

```c
// safety/safety_watchdog.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

// alive_token: Updated only when Level 2 comparison passes
// Feed condition: token correct + program flow flags complete
typedef struct {
    uint32_t expected_token;
    uint32_t actual_token;
    uint32_t cf_expected;       // Program flow expected signature
    uint32_t cf_actual;         // Program flow actual signature
} safety_wdg_ctx_t;

void safety_wdg_init(safety_wdg_ctx_t *ctx);
void safety_wdg_update_token(safety_wdg_ctx_t *ctx);   // Called when L2 comparison passes
void safety_wdg_service(safety_wdg_ctx_t *ctx);         // Conditional watchdog feed
```

```c
// safety/safety_watchdog.c
#include "safety_watchdog.h"

// Simple pseudo-random sequence to prevent accidental jumps to update_token passing
static uint32_t next_token(uint32_t seed) {
    return seed * 1103515245u + 12345u;
}

void safety_wdg_init(safety_wdg_ctx_t *ctx) {
    ctx->expected_token = 0xA5A5A5A5u;
    ctx->actual_token   = 0;
    ctx->cf_expected    = CF_MAIN_EXPECTED_SIGNATURE;
    ctx->cf_actual      = CF_SEED;
}

void safety_wdg_update_token(safety_wdg_ctx_t *ctx) {
    ctx->actual_token = ctx->expected_token;
    ctx->expected_token = next_token(ctx->expected_token);
}

void safety_wdg_service(safety_wdg_ctx_t *ctx) {
    bool token_ok = (ctx->actual_token != 0) &&
                    (next_token(ctx->actual_token) == ctx->expected_token);
    bool cf_ok    = (ctx->cf_actual == ctx->cf_expected);

    if (token_ok && cf_ok) {
        IWDG_feed();            // Internal watchdog
        ext_wdg_feed();         // External watchdog (e.g., MAX6369)
    }
    // else: Do not feed watchdog → watchdog timeout → hardware reset

    // Reset program flow flag, wait for next cycle
    ctx->cf_actual = CF_SEED;
}
```

### 12.7 Level 3 — Independent Hardware Shutdown Path

**Last line of defense — does not go through RTOS, event_bus, state machine, or driver abstraction layer.**

v5 improvement: `safe_shutdown_activate()` is refactored to a platform-abstraction injection pattern, preventing the safety generic header from directly `#include "stm32g4xx.h"` and breaking the platform-independence boundary. The implementation still allows direct register writes, but platform differences are confined to the platform directory.

```c
// safety/safe_shutdown.h — v5: Platform abstraction injection (no longer includes platform headers)
#pragma once
#include <stdint.h>

// Platform-provided hardware shutdown operation
typedef struct {
    void (*activate)(void);     // Hardware direct shutdown implementation (direct GPIO register write)
} safe_shutdown_ops_t;

// Registered by platform_init() early in startup
void safe_shutdown_register(const safe_shutdown_ops_t *ops);

// Safety shutdown entry — internally calls registered ops->activate()
void safe_shutdown_activate(void);
```

```c
// safety/safe_shutdown.c
#include "safe_shutdown.h"

static const safe_shutdown_ops_t *g_ops;
static volatile uint32_t g_safety_shutdown_active;

void safe_shutdown_register(const safe_shutdown_ops_t *ops) {
    g_ops = ops;
}

void safe_shutdown_activate(void) {
    if (g_ops && g_ops->activate) {
        g_ops->activate();
    }
    __atomic_store_n(&g_safety_shutdown_active, 1, __ATOMIC_RELEASE);
    // Best-effort CAN emergency frame (non-blocking, no waiting)
    // Even if CAN send fails, hardware shutdown has already taken effect
}
```

```c
// platform/stm32g4/shutdown_stm32g4.c — STM32G4 implementation
#include "stm32g4xx.h"
#include "safe_shutdown.h"

static void stm32g4_shutdown(void) {
    // Directly operate GPIO register; does not go through gpio_writer_t abstraction layer
    GPIOB->BRR = GPIO_PIN_ENABLE_OUT_Msk;  // Pull enable output low
}

const safe_shutdown_ops_t shutdown_stm32g4 = { .activate = stm32g4_shutdown };
// Called in platform_init(): safe_shutdown_register(&shutdown_stm32g4);
```

```c
// platform/s32k144/shutdown_s32k144.c — S32K144 implementation
#include "S32K144.h"
#include "safe_shutdown.h"

static void s32k144_shutdown(void) {
    PTB->PCOR = (1u << ENABLE_OUT_PIN);  // Pull enable output low
}

const safe_shutdown_ops_t shutdown_s32k144 = { .activate = s32k144_shutdown };
```

> **Trade-off**: Adding one level of indirection (function pointer) theoretically adds a failure point to the shutdown path. However, the function pointer is stored in `.rodata` (Flash), covered by CRC verification, and the risk is acceptable. If certification audit rejects this, it can be changed to `#if defined(PLATFORM_STM32G4)` conditional compilation — breaking platform independence only at this single point.

### 12.8 L1/L2 Shared Data Region

Level 1 and Level 2 communicate via an atomically-protected shared data region:

```c
// safety/safety_shared.h — L1 writes, L2 reads, protected by atomic operations
#pragma once
#include <stdint.h>
#include <stdatomic.h>

#define MAX_AXES 4

typedef struct {
    _Atomic int32_t axes[MAX_AXES];
    _Atomic uint8_t axes_count;
    _Atomic uint32_t sequence;        // Monotonically increasing sequence number for update detection
} safety_shared_t;

void safety_shared_write_l1_output(const int32_t *axes, uint8_t count);
void safety_shared_read_l1_output(int32_t *axes_out);
```

### 12.9 Program Flow Monitoring

```c
// safety/control_flow.h — Detects whether code executes in the expected order
#pragma once
#include <stdint.h>

#define CF_SEED                     0x5A5A5A5Au
#define CF_MAIN_START               0x01u
#define CF_MAIN_SCAN_DONE           0x02u
#define CF_MAIN_SIGNAL_DONE         0x04u
#define CF_MAIN_END                 0x08u
#define CF_MAIN_EXPECTED_SIGNATURE  (CF_SEED ^ CF_MAIN_START ^ CF_MAIN_SCAN_DONE \
                                    ^ CF_MAIN_SIGNAL_DONE ^ CF_MAIN_END)

// Called at each checkpoint in main_task
void cf_marker_checkpoint(uint32_t marker);
```

```c
// safety/control_flow.c
#include "control_flow.h"
#include "safety_watchdog.h"

extern safety_wdg_ctx_t *g_l2_wdg;  // Set by app_compose

void cf_marker_checkpoint(uint32_t marker) {
    g_l2_wdg->cf_actual ^= marker;
}
```

### 12.10 Additional Diagnostics — Platform-Certified Safety Library Integration (DC ≥ 90%)

Low-level hardware diagnostics are **not self-developed**; instead, platform-vendor-provided certified safety libraries are used:

| Platform | Safety Library | Certification Level | Coverage |
|----------|---------------|---------------------|----------|
| STM32G4 | **X-CUBE-STL** (ST official) | IEC 61508 SIL 2/3 | CPU/RAM/Flash/Clock self-test |
| S32K144 | **SafeAssure Safety SW** (NXP) | IEC 61508 SIL 2, ISO 26262 ASIL B | CST(CPU)/ECC(RAM)/CRC(Flash)/CMU(Clock) |

> Benefit of using pre-certified libraries: Reduces certification documentation effort by 80%+; vendor provides FMEDA and safety manual, which can be directly referenced during audit.

#### 12.10.1 Safety Self-Test Abstraction Interface

Upper layers do not call STL/NXP API directly; instead, a unified abstract interface is used to maintain platform independence:

```c
// safety/safety_stl.h — Safety self-test interface (upper layers depend only on this)
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    STL_RESULT_PASS,
    STL_RESULT_FAIL,
    STL_RESULT_IN_PROGRESS,
} stl_result_t;

typedef struct {
    stl_result_t (*cpu_test_partial)(void);     // CPU instruction set test (sliced execution)
    stl_result_t (*ram_test_partial)(void);     // RAM March C- test (sliced execution)
    stl_result_t (*flash_crc_check)(void);      // Flash CRC integrity
    stl_result_t (*clock_cross_check)(void);    // Clock cross-check
    void         (*stack_overflow_init)(void);  // MPU/Sentinel initialization
} safety_stl_t;

// Registered by platform layer
void platform_register_stl(const safety_stl_t *stl);
const safety_stl_t* platform_get_stl(void);
```

#### 12.10.2 STM32G4 Implementation (X-CUBE-STL)

```c
// platform/stm32g4/stl_stm32g4.c
#include "safety_stl.h"
#include "stm32_stl_api.h"   // X-CUBE-STL official header

static stl_result_t stm32_cpu_test(void) {
    STL_Status_t res = STL_SCH_RunCpuTMx();
    return (res == STL_PASSED) ? STL_RESULT_PASS : STL_RESULT_FAIL;
}

static stl_result_t stm32_ram_test(void) {
    STL_Status_t res = STL_SCH_RunRamTMx();
    switch (res) {
        case STL_PASSED:      return STL_RESULT_PASS;
        case STL_PARTIAL_PASS: return STL_RESULT_IN_PROGRESS;
        default:              return STL_RESULT_FAIL;
    }
}

static stl_result_t stm32_flash_crc(void) {
    STL_Status_t res = STL_SCH_RunFlashTMx();
    return (res == STL_PASSED) ? STL_RESULT_PASS : STL_RESULT_FAIL;
}

static stl_result_t stm32_clock_check(void) {
    STL_Status_t res = STL_SCH_RunClockTMx();
    return (res == STL_PASSED) ? STL_RESULT_PASS : STL_RESULT_FAIL;
}

static void stm32_stack_init(void) {
    // MPU configuration generated by CubeMX; activate Stack Sentinel region here
    HAL_MPU_ConfigRegion(&mpu_stack_sentinel_config);
}

const safety_stl_t stl_stm32g4 = {
    .cpu_test_partial  = stm32_cpu_test,
    .ram_test_partial  = stm32_ram_test,
    .flash_crc_check   = stm32_flash_crc,
    .clock_cross_check = stm32_clock_check,
    .stack_overflow_init = stm32_stack_init,
};
```

#### 12.10.3 S32K144 Implementation (NXP Safety SW)

```c
// platform/s32k144/stl_s32k144.c
#include "safety_stl.h"
#include "safety_test.h"      // NXP SafeAssure API

static stl_result_t s32k_cpu_test(void) {
    // S32K144 Core Self-Test (CST)
    uint32_t result = FS_CM4_CST_CPU();
    return (result == FS_PASS) ? STL_RESULT_PASS : STL_RESULT_FAIL;
}

static stl_result_t s32k_ram_test(void) {
    // S32K144 ECC continuously monitored by EIM/ERM hardware modules
    // Check ERM error register here
    if (ERM->SR0 & ERM_SR0_NCE0_MASK) {
        ERM->SR0 = ERM_SR0_NCE0_MASK;  // Clear flag
        return STL_RESULT_FAIL;          // Non-correctable error
    }
    return STL_RESULT_PASS;
}

static stl_result_t s32k_flash_crc(void) {
    uint32_t result = FS_CM4_FLASH_CRC(flash_start, flash_end, expected_crc);
    return (result == FS_PASS) ? STL_RESULT_PASS : STL_RESULT_FAIL;
}

static stl_result_t s32k_clock_check(void) {
    // S32K144 CMU (Clock Monitor Unit) monitors automatically in hardware
    // Check CMU status register here
    if (CMU_FC_0->SR & CMU_FC_SR_FLL_MASK) {
        return STL_RESULT_FAIL;   // Frequency loss
    }
    return STL_RESULT_PASS;
}

const safety_stl_t stl_s32k144 = {
    .cpu_test_partial  = s32k_cpu_test,
    .ram_test_partial  = s32k_ram_test,
    .flash_crc_check   = s32k_flash_crc,
    .clock_cross_check = s32k_clock_check,
    .stack_overflow_init = s32k_stack_init,
};
```

#### 12.10.4 Invocation in safety_task (v5: Sliced Execution + Supply Voltage Monitoring)

v5 refactors diagnostics to budgeted sliced execution: each safety_task period executes only one diagnostic step, keeping WCET predictable:

```c
// safety/safety_diagnostics.h — v5: Sliced diagnostic step enumeration
typedef enum {
    SAFETY_DIAG_CPU_REG,
    SAFETY_DIAG_RAM_SLICE,
    SAFETY_DIAG_FLASH_CRC_SLICE,
    SAFETY_DIAG_CLOCK,
    SAFETY_DIAG_SUPPLY_VOLTAGE,    // v5 new: supply voltage monitoring
    SAFETY_DIAG_STEP_COUNT
} safety_diag_step_t;

// Execute one diagnostic step; budget_us is the maximum allowed execution time for this call
bool safety_diagnostics_step(safety_diag_step_t step, uint32_t budget_us);
```

```c
// Sliced diagnostic invocation in safety_task
static safety_diag_step_t g_diag_step = 0;

static bool safety_diagnostics_step(safety_diag_step_t step, uint32_t budget_us) {
    const safety_stl_t *stl = platform_get_stl();
    (void)budget_us;  // Reserved for future dynamic budget control

    switch (step % SAFETY_DIAG_STEP_COUNT) {
        case SAFETY_DIAG_CPU_REG:
            if (stl->cpu_test_partial() == STL_RESULT_FAIL) {
                safe_shutdown_activate();
                return false;
            }
            break;

        case SAFETY_DIAG_RAM_SLICE:
            if (stl->ram_test_partial() == STL_RESULT_FAIL) {
                safe_shutdown_activate();
                return false;
            }
            break;

        case SAFETY_DIAG_CLOCK:
            if (stl->clock_cross_check() == STL_RESULT_FAIL) {
                safe_shutdown_activate();
                return false;
            }
            break;

        case SAFETY_DIAG_FLASH_CRC_SLICE: {
            // Flash CRC sliced execution; one slice checked every SAFETY_DIAG_STEP_COUNT rounds
            static uint16_t flash_round = 0;
            if (++flash_round >= 100) {  // Approximately one full pass per ~1s
                flash_round = 0;
                if (stl->flash_crc_check() == STL_RESULT_FAIL) {
                    safe_shutdown_activate();
                    return false;
                }
            }
            break;
        }

        case SAFETY_DIAG_SUPPLY_VOLTAGE: {
            // v5 new: Supply voltage monitoring
            // Abnormal sensor supply (e.g., 5V drops to 4V) causes systematic offset in L1/L2
            uint32_t supply_mv = 0;
            safety_channel_ctx_t *l2_ch = app_get_l2_channel();
            l2_ch->l2_adc->read_mv(ADC_CH_SUPPLY, &supply_mv);
            if (supply_mv < SUPPLY_MIN_MV || supply_mv > SUPPLY_MAX_MV) {
                event_bus_post(&(event_t){
                    .type = EVT_SAFETY_STATE_CHANGED,
                    .data.safety_state = 2 }, EVT_PRIO_SAFETY);
            }
            break;
        }
    }
    return true;
}
```

> **Supply voltage thresholds**: `SUPPLY_MIN_MV` / `SUPPLY_MAX_MV` are defined by product configuration (typical 5V ±10%: 4500mV ~ 5500mV). `ADC_CH_SUPPLY` is defined in `adc_channel_id_t` but was previously unused.

#### 12.10.5 Safety Variable Complementary Storage (Application Layer — Still Self-Developed)

Certified libraries do not cover application-layer safety-critical variable protection; this part must still be self-developed:

```c
// safety/safe_var.h — Safety-critical variable with complementary storage
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int32_t  value;
    int32_t  complement;   // Must equal ~value
} safe_var_t;

static inline bool safe_var_check(const safe_var_t *v) {
    return v->value == (int32_t)(~(uint32_t)v->complement);
}

static inline void safe_var_write(safe_var_t *v, int32_t val) {
    v->value      = val;
    v->complement = (int32_t)(~(uint32_t)val);
}
```

### 12.11 Self-Developed vs. Certified Library Responsibility Division

```
┌──────────────────────────────────────────────────────────────┐
│              Safety Function Responsibility Division         │
├──────────────────────────┬───────────────────────────────────┤
│  Use Certified Library   │  Self-Developed                   │
│  (Platform-provided)     │  (Application-layer safety logic) │
├──────────────────────────┼───────────────────────────────────┤
│  CPU instruction set     │  L2 independent sensor acq +      │
│  self-test               │  computation                      │
│  RAM March C- test       │  L1/L2 output comparator          │
│  Flash CRC integrity     │  Conditional watchdog strategy     │
│  Clock cross-check       │  (token + program flow)           │
│  MPU memory partitioning │  Program flow monitoring           │
│  ECC hardware correction │  (checkpoint signature chain)     │
│  (S32K)                  │  Independent hardware shutdown    │
│  WDG hardware driver     │  path                             │
│                          │  Safety variable complementary    │
│                          │  storage                          │
│                          │  Safety state management          │
├──────────────────────────┴───────────────────────────────────┤
│  Principle: Use certified library for hardware diagnostics;  │
│  self-develop application safety logic.                      │
│  During certification: Certified library cites vendor safety │
│  manual directly; self-developed parts submit V&V report.    │
└──────────────────────────────────────────────────────────────┘
```

### 12.12 Necessary Measures for Single-MCU SIL 2 Compliance

| Measure | Purpose | Implementation |
|---------|---------|----------------|
| CPU/RAM/Flash/Clock self-test | Hardware fault diagnostics | **X-CUBE-STL / NXP Safety SW** (pre-certified) |
| MPU memory partitioning | L1/L2 memory isolation | CubeMX/SDK configuration |
| Independent code implementation | Software diversity | L2 signal computation does not reuse L1 `axis_pipeline` code |
| External watchdog | Independent from CPU | External WDG IC (e.g., MAX6369) |
| Independent shutdown path | Direct hardware | GPIO directly drives safety relay; does not go through software abstraction |
| Program flow monitoring | Detects jump errors | Checkpoint signature chain (self-developed) |
| Complementary storage | Application-layer safety variables | `safe_var_t` (self-developed) |

> **SIL 3 / PL e** only mandates independent MCU (dual processor) or lockstep core (e.g., S32K3 Cortex-M7 lockstep).

### 12.13 Power-On Self-Test (POST) (v5 New)

IEC 61508 requires hardware integrity verification before power-on. v4 only had runtime periodic self-tests, lacking a safety gate at startup. v5 places POST as the state machine entry, completing it before RTOS starts.

**Boot sequence**: `HAL_Init` → `platform_init` → `safe_shutdown_register()` → **`safety_post_run()`** → On POST pass: `osKernelStart`

```c
// safety/safety_post.h — Power-on full self-test
#pragma once
#include "safety_stl.h"
#include <stdbool.h>

typedef enum {
    POST_STAGE_CPU,
    POST_STAGE_RAM,
    POST_STAGE_FLASH,
    POST_STAGE_CLOCK,
    POST_STAGE_STACK_INIT,
    POST_STAGE_ADC_SELF_TEST,
    POST_STAGE_WDG_VERIFY,
    POST_STAGE_RELAY_TEST,
    POST_STAGE_COMPLETE
} post_stage_t;

typedef struct {
    post_stage_t current_stage;
    uint32_t     fail_mask;       // Bitmask recording failed items
} post_ctx_t;

// Returns true = all passed, false = at least one failure
bool safety_post_run(post_ctx_t *ctx, const safety_stl_t *stl);
```

```c
// safety/safety_post.c
#include "safety_post.h"
#include "safe_shutdown.h"

bool safety_post_run(post_ctx_t *ctx, const safety_stl_t *stl) {
    ctx->fail_mask = 0;

    // CPU full self-test (POST phase: not sliced, run to completion)
    if (stl->cpu_test_partial() == STL_RESULT_FAIL) {
        ctx->fail_mask |= (1u << POST_STAGE_CPU);
    }

    // RAM March C- (POST phase: run full, not sliced)
    if (stl->ram_test_partial() == STL_RESULT_FAIL) {
        ctx->fail_mask |= (1u << POST_STAGE_RAM);
    }

    // Flash CRC full verification
    if (stl->flash_crc_check() == STL_RESULT_FAIL) {
        ctx->fail_mask |= (1u << POST_STAGE_FLASH);
    }

    // Clock cross-check
    if (stl->clock_cross_check() == STL_RESULT_FAIL) {
        ctx->fail_mask |= (1u << POST_STAGE_CLOCK);
    }

    // MPU / Stack Sentinel initialization
    stl->stack_overflow_init();

    // ADC self-test: Read known reference channel (internal Vrefint), verify within tolerance
    // WDG verification: Confirm WDG can timeout correctly
    // Safety relay test: Briefly activate-release to confirm relay responds

    return ctx->fail_mask == 0;
}
```

```c
// app/device_state.c — STATE_BOOT integrates POST
static drv_status_t boot_self_test(void) {
    static post_ctx_t post_ctx;
    const safety_stl_t *stl = platform_get_stl();

    if (stl == NULL) {
        // Non-safety model; skip POST
        return DRV_OK;
    }

    if (!safety_post_run(&post_ctx, stl)) {
        // POST failed; do not allow entry into PRE_OPERATIONAL
        nv_storage_write_post_result(post_ctx.fail_mask);
        safe_shutdown_activate();
        return DRV_ERROR;
    }
    return DRV_OK;
}
```

> **Key**: The `safety_stl_t` interface must distinguish POST mode (full, blocking for hundreds of ms allowed) from runtime mode (sliced, < 300μs each). It is recommended to add `cpu_test_full()` / `ram_test_full()` interfaces or pass a mode parameter.

### 12.14 Safety Certification Materials Framework (v5 New)

The architecture designs an E-Gas three-layer structure, but lacks supporting documentation mapping it to IEC 61508 audit requirements. The following provides a framework definition; actual certification requires filling in concrete data based on FMEDA tools (e.g., exSILentia).

#### 12.14.1 Failure Mode and Diagnostic Coverage Matrix (FMEDA Framework)

```
┌────────────────────┬──────────────┬────────────────────┬────────────┬────────┐
│ Failure Mode       │ Effect        │ Diagnostic Measure  │ Detection  │ DC%    │
├────────────────────┼──────────────┼────────────────────┼────────────┼────────┤
│ Sensor open circuit│ Abnormal out │ fault_detect thresh │ 10ms (L1)  │ 99%    │
│ Sensor shorted to  │ Output satur.│ fault_detect thresh │ 10ms (L1)  │ 99%    │
│  supply            │              │                    │            │        │
│ Sensor drift       │ Output offset│ L1/L2 comparator   │ 2ms×5 (L2) │ 90%    │
│ ADC module fault   │ L1/L2 both  │ Independent ADC     │ 2ms (L2)   │ 90%    │
│                    │  wrong       │  instances         │            │        │
│ Vref drift         │ Systematic   │ Vref cross-check   │ 2ms (L2)   │ 90%    │
│                    │  offset      │                    │            │        │
│ DMA stall          │ Stale data   │ seq number detect  │ 10ms (L1)  │ 99%    │
│ CPU instr. fault   │ Compute error│ X-CUBE-STL CST     │ 2ms (slice)│ Per    │
│                    │              │                    │            │ vendor │
│ RAM bit flip       │ Data corrupt │ STL March C-/ECC   │ Sliced     │ Per    │
│                    │              │                    │            │ vendor │
│ Flash data corrupt │ Program/conf │ CRC verification   │ 1s (Flash) │ 99%    │
│                    │  corrupt     │                    │            │        │
│ Clock drift/loss   │ Period error │ Clock cross-check/ │ 2ms        │ 99%    │
│                    │              │  CMU               │            │        │
│ Task hang/deadlock │ Loss of func.│ Conditional WDG +  │ WDG window │ 99%    │
│                    │              │  program flow      │            │        │
│ CAN comm. interrup.│ No output    │ Bus-Off detect +   │ 100ms      │ 60%    │
│                    │              │  timeout           │            │        │
│ Stack overflow     │ Unpredictable│ MPU Sentinel       │ Immediate  │ 99%    │
│ Supply voltage     │ Global effect│ ADC_CH_SUPPLY      │ 2ms (slice)│ 90%    │
│  abnormal          │              │  monitoring        │            │        │
└────────────────────┴──────────────┴────────────────────┴────────────┴────────┘
```

#### 12.14.2 Safety Reaction Time Budget (FHTT)

```c
// safety/safety_timing.h — Safety reaction time budget definitions
#pragma once

// Fault detection time budget (unit: ms)
#define FHTT_L1_SAMPLE_PERIOD       10    // L1 sampling period
#define FHTT_L2_SAMPLE_PERIOD        2    // L2 sampling period
#define FHTT_L2_FAULT_FILTER       (FHTT_L2_SAMPLE_PERIOD * 5)  // 5 consecutive → 10ms
#define FHTT_WDG_WINDOW            50    // Watchdog window
#define FHTT_RELAY_RELEASE          15    // Safety relay release time (mechanical)
#define FHTT_TOTAL_WORST_CASE      (FHTT_L1_SAMPLE_PERIOD + FHTT_L2_FAULT_FILTER \
                                    + FHTT_WDG_WINDOW + FHTT_RELAY_RELEASE)
// Total: 10 + 10 + 50 + 15 = 85ms (must be < application safety requirement PFHD reaction time)

_Static_assert(FHTT_TOTAL_WORST_CASE < 100,
    "Safety reaction time exceeds 100ms budget");
```

```
Safety Reaction Time Chain:

Sensor fault → L1 sample detection → L2 independent sample → Consecutive fault filter → WDG feed stops → WDG timeout → Relay releases
   0ms              10ms                   2ms              10ms (5×2ms)                   0ms              50ms            15ms
                                                                                                     Total: ≤ 85ms
```

#### 12.14.3 Common Cause Failure Analysis (CCF — IEC 61508-6 Annex D)

```
┌──────────────────────────────┬────────────────────────────────────────┬──────┐
│ IEC 61508-6 Annex D Measure  │ Corresponding Implementation           │ Score│
├──────────────────────────────┼────────────────────────────────────────┼──────┤
│ Physical separation / signal │ L1/L2 use independent ADC instances    │ ✓    │
│  isolation                   │                                        │      │
│ Software diversity           │ L2 independent code; no reuse of       │ ✓    │
│                              │  axis_pipeline                         │      │
│ Inter-channel independence   │ Atomic shared region; L2 forbids mutex │ ✓    │
│ Independent shutdown path    │ GPIO directly drives relay; no         │ ✓    │
│                              │  RTOS/event_bus                        │      │
│ Environmental factors        │ TBD: PCBA layout isolation             │ △    │
│  (temp/EMC)                  │  requirements                          │      │
│ Maintenance/modification     │ L1/L2 code in independent compilation  │ ✓    │
│  independence                │  units                                 │      │
└──────────────────────────────┴────────────────────────────────────────┴──────┘
```

> The β-factor score requires a safety engineer to fill in after hardware design is complete. The table above is an architecture-level self-assessment; final score is determined by the certification body's audit.

---

## 13. Build System (v5 Improvement: Safety Configuration Single Source + Auto-Derive)

### 13.1 CMake Configuration (v5 Improvement: Safety Config Auto-Derive, No External Override)

```cmake
# CMakeLists.txt (top level)

cmake_minimum_required(VERSION 3.20)

# ---- Product and protocol selection ----
set(PRODUCT  "JS200"    CACHE STRING "Product model")
set(PROTOCOL "CANOPEN"  CACHE STRING "Communication protocol")
set(PLATFORM "STM32G4"  CACHE STRING "Target platform")

string(TOLOWER ${PRODUCT} PRODUCT_LOWER)

project(joystick_${PRODUCT}_${PROTOCOL} C ASM)

# ---- v5: Safety feature auto-derived from product matrix; external override forbidden ----
# Eliminates dual-source configuration risk: product_config.safety != NULL
# must be consistent with CMake SAFETY_ENABLED
include(cmake/product_matrix.cmake)
# product_matrix.cmake example content:
#   set(PRODUCT_JS300_SAFETY ON)
#   set(PRODUCT_JS400_SAFETY ON)
#   set(PRODUCT_JS100_SAFETY OFF)
#   set(PRODUCT_JS200_SAFETY OFF)
string(TOUPPER ${PRODUCT} PRODUCT_UPPER)
if(DEFINED PRODUCT_${PRODUCT_UPPER}_SAFETY)
    set(SAFETY_ENABLED ${PRODUCT_${PRODUCT_UPPER}_SAFETY})
else()
    set(SAFETY_ENABLED OFF)
endif()

# Forbid external override: if user manually passed -DSAFETY_ENABLED inconsistent with matrix, error
if(DEFINED CACHE{SAFETY_ENABLED})
    if(NOT "${SAFETY_ENABLED}" STREQUAL "${PRODUCT_${PRODUCT_UPPER}_SAFETY}")
        message(FATAL_ERROR
            "SAFETY_ENABLED is auto-derived from product_matrix.cmake. "
            "Do not pass -DSAFETY_ENABLED manually. "
            "Product ${PRODUCT} safety=${PRODUCT_${PRODUCT_UPPER}_SAFETY}")
    endif()
endif()

# ---- Compile definitions ----
add_compile_definitions(
    PROTOCOL_${PROTOCOL}
    PLATFORM_${PLATFORM}
)
if(SAFETY_ENABLED)
    add_compile_definitions(SAFETY_ENABLED=1)
endif()

# ---- OCP: Each product has a dedicated configuration file ----
set(PRODUCT_CONFIG_SRC "config/products/${PRODUCT_LOWER}.c")
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/${PRODUCT_CONFIG_SRC}")
    message(FATAL_ERROR "Unknown product: ${PRODUCT}")
endif()

# ---- Common source files (v4: removed signal_chain.c, added output_adapter.c) ----
set(APP_SOURCES
    app/app_compose.c
    app/app_run.c
    app/app_tasks.c
    app/device_state.c
    event/event_bus.c
    input/buf_axis_source.c
    input/buf_btn_source.c
    input/backend/mux_adc_acq.c
    input/backend/direct_adc_acq.c
    input/backend/spi_hall_acq.c
    input/backend/dma_adc_acq.c
    signal/axis_pipeline.c
    signal/fault_detector.c
    signal/calibrator.c
    signal/filter.c
    signal/deadzone.c
    signal/debounce.c
    output/output_adapter.c
    service/calibration.c
    service/diagnostics.c
    service/nv_storage.c
    platform/platform_registry.c
    ${PRODUCT_CONFIG_SRC}
)

# ---- Select protocol plugin based on protocol choice ----
if(PROTOCOL STREQUAL "CANOPEN")
    list(APPEND APP_SOURCES protocols/canopen/canopen_engine.c)
    list(APPEND APP_SOURCES protocols/canopen/canopen_od.c)
elseif(PROTOCOL STREQUAL "J1939")
    list(APPEND APP_SOURCES protocols/j1939/j1939_engine.c)
elseif(PROTOCOL STREQUAL "CAN_RAW")
    list(APPEND APP_SOURCES protocols/can_raw/can_raw_engine.c)
endif()

# ---- v4: Safety modules driven by SAFETY_ENABLED (OCP improvement: no hardcoded product names) ----
if(SAFETY_ENABLED)
    list(APPEND APP_SOURCES
        safety/safety_channel.c
        safety/safety_comparator.c
        safety/safety_watchdog.c
        safety/safe_shutdown.c
        safety/safety_shared.c
        safety/control_flow.c
        safety/ram_check.c
    )
endif()

# ---- Select driver based on platform (DIP: linker decides platform) ----
if(PLATFORM STREQUAL "STM32G4")
    include(cmake/stm32g4.cmake)
elseif(PLATFORM STREQUAL "S32K144")
    include(cmake/s32k144.cmake)
endif()

add_executable(${PROJECT_NAME} ${APP_SOURCES} ${PLATFORM_SOURCES})
```

### 13.2 Batch Build (v5: SAFETY_ENABLED Auto-Derived by product_matrix.cmake)

```bash
#!/bin/bash
# build_all.sh

# v5: SAFETY_ENABLED is no longer passed manually; auto-derived by product_matrix.cmake
# Format: PRODUCT PROTOCOL PLATFORM
CONFIGS=(
    "JS100 CAN_RAW  STM32G4"
    "JS200 CANOPEN  STM32G4"
    "JS200 J1939    STM32G4"
    "JS300 CANOPEN  STM32G4"
    "JS300 J1939    STM32G4"
    "JS400 CANOPEN  STM32G4"
    "JS200 CANOPEN  S32K144"
)

for cfg in "${CONFIGS[@]}"; do
    read -r PROD PROTO PLAT <<< "$cfg"
    BUILD_DIR="build/${PROD}_${PROTO}_${PLAT}"
    echo "=== Building ${PROD} + ${PROTO} on ${PLAT} ==="
    cmake -B "$BUILD_DIR" \
          -DPRODUCT="$PROD" \
          -DPROTOCOL="$PROTO" \
          -DPLATFORM="$PLAT"
    cmake --build "$BUILD_DIR" -j$(nproc)
done
```

> **v5 Change**: `SAFETY_ENABLED` is no longer passed manually by the user; it is auto-derived by `cmake/product_matrix.cmake` from the product name. The `BUILD_ASSERT` in `app_compose()` verifies consistency between the `product_config.safety` pointer and the `SAFETY_ENABLED` macro at compile time, fully eliminating dual-source configuration risk.

---

## 14. Project Directory Structure Overview (v5 Update)

```
joystick-firmware/
│
├── CMakeLists.txt
├── build_all.sh
│
├── drivers/                    # Hardware abstraction interfaces (header-only after ISP split)
│   ├── drv_common.h            # Common types (drv_status_t, adc_channel_id_t, etc.)
│   ├── adc_reader.h            # Basic ADC read interface
│   ├── adc_voltage_reader.h    # Voltage read interface
│   ├── adc_continuous.h        # DMA continuous acquisition interface
│   ├── drv_can.h
│   ├── drv_gpio.h              # Split into gpio_reader_t / gpio_writer_t
│   ├── drv_timer.h
│   ├── drv_spi.h               # SPI interface (Hall sensors, etc.)
│   └── drv_nv.h
│
├── input/                      # Input acquisition layer (scan_buffer decoupling pattern)
│   ├── scan_buffer.h           # Pure data buffer (decoupling core; zero external dependencies)
│   ├── input_acq.h/c           # Acquisition unified interface + factory
│   ├── backend/
│   │   ├── mux_adc_acq.c       # MUX + ADC per-channel acquisition
│   │   ├── direct_adc_acq.c    # Multiple independent ADC channels direct read
│   │   ├── spi_hall_acq.c      # SPI Hall / magnetic encoder acquisition
│   │   └── dma_adc_acq.c       # DMA continuous acquisition
│   ├── buf_axis_source.h/c     # scan_buffer-based axis input source
│   └── buf_btn_source.h/c      # scan_buffer-based button input source (threshold detection)
│
├── platform/                   # Driver implementations per platform (DIP: explicit registration)
│   ├── platform_registry.h/c   # Platform registry
│   ├── stm32g4/
│   │   ├── cubemx/
│   │   ├── adc_stm32g4.c
│   │   ├── can_stm32g4.c
│   │   ├── gpio_stm32g4.c
│   │   ├── spi_stm32g4.c
│   │   ├── stl_stm32g4.c      # v4: X-CUBE-STL adapter (IEC 61508 SIL 2/3 certified)
│   │   └── platform_init.c
│   ├── s32k144/
│   │   └── ...
│   └── test/                   # PC simulation drivers
│       └── ...
│
├── signal/                     # v4: Signal processing pipeline (generic chain removed)
│   ├── axis_pipeline.h/c       # Explicit signal pipeline (sole signal chain interface)
│   ├── fault_detector.h/c      # Open-circuit/short-circuit detection (uint16_t domain)
│   ├── calibrator.h/c          # Calibration strategies: linear / circular (uint16_t → int32_t)
│   ├── filter.h/c              # Digital low-pass filter (int32_t domain)
│   ├── deadzone.h/c            # Deadzone processing, strategy injection (int32_t domain)
│   └── debounce.h/c            # Button debounce
│
├── output/                     # v4 new: Output adaptation layer
│   ├── axis_output_map.h       # Axis output mapping descriptor (target range + direction)
│   ├── output_adapter.h/c      # Value domain conversion (pure function; joystick_data→output_data)
│   └── output_data.h           # Self-describing output data structure (includes axis_output_t)
│
├── protocols/                  # Protocol engines (v4: packaging only; no value conversion)
│   ├── protocol_bundle.h       # Protocol Bundle (tagged union, type-safe)
│   ├── protocol_runtime.h      # Runtime interface (send receives output_data_t)
│   ├── protocol_init.c         # Builds runtime from bundle type
│   ├── canopen/
│   │   ├── canopen_engine.c    # Profile-driven generic engine
│   │   ├── canopen_profile.h   # OD + PDO + value mapping descriptors
│   │   └── canopen_od.c        # OD runtime operations
│   ├── j1939/
│   │   ├── j1939_engine.c      # SPN descriptor-driven packing
│   │   └── j1939_profile.h     # PGN + SPN descriptors
│   └── can_raw/
│       └── can_raw_engine.c
│
├── event/                      # Event bus (observer pattern)
│   ├── event_bus.h             # Includes ISR-safe publish_from_isr()
│   └── event_bus.c
│
├── CMSIS/RTOS2/                # CMSIS-RTOS v2 (generated by CubeMX)
│   └── FreeRTOS/
│
├── app/                        # Application core
│   ├── app_compose.c/h         # Composition Root (v4: includes safety channel assembly)
│   ├── app_tasks.c/h           # RTOS tasks (v4: safety_task is the L2 channel)
│   ├── device_state.c/h        # Device state machine
│   └── joystick_data.h         # Common data model (v4: per-axis max_deflection)
│
├── service/                    # Common services
│   ├── calibration.c/h
│   ├── diagnostics.c/h
│   ├── nv_storage.c/h
│   └── logging.c/h
│
├── safety/                     # v4 refactored: E-Gas three-layer safety architecture
│   │                           # (conditional compilation with SAFETY_ENABLED)
│   ├── safety_stl.h            # Safety self-test abstraction interface (platform cert. lib adapter)
│   ├── safety_channel.h/c      # L2: Independent sensor acq + simplified compute (no signal chain reuse)
│   ├── safety_comparator.h/c   # L2: L1/L2 output comparator
│   ├── safety_shared.h/c       # L1/L2 shared data region (atomic protection)
│   ├── safety_watchdog.h/c     # L3: Conditional watchdog feed (token + program flow check)
│   ├── safe_shutdown.h/c       # L3: Independent hardware shutdown path (direct GPIO; no RTOS)
│   ├── control_flow.h/c        # Program flow monitoring (checkpoint signature chain; self-developed)
│   └── safe_var.h              # Safety variable complementary storage (self-developed)
│
├── config/                     # Product configuration (OCP: one file per product)
│   ├── product_descriptor.h    # Product descriptor (v4: includes protocol_bundle + safety_config)
│   ├── products/
│   │   ├── js100.c
│   │   ├── js200.c
│   │   ├── js300.c             # v4: Includes safety channel configuration
│   │   └── js400.c             # v4: Includes safety channel configuration
│   ├── canopen_profiles/       # CANopen profile data files
│   │   ├── ds401_standard.c
│   │   └── brand_b_compat.c
│   └── j1939_profiles/         # J1939 profile data files
│       ├── j1939_standard.c
│       └── j1939_brand_x.c
│
└── test/                       # Unit tests (run on PC)
    ├── test_axis_pipeline.c    # v4: Replaces test_signal_chain.c
    ├── test_output_adapter.c   # v4 new
    ├── test_fault_detector.c
    ├── test_calibrator.c
    ├── test_deadzone.c
    ├── test_event_bus.c
    ├── test_device_state.c
    ├── test_axis_source.c
    ├── test_canopen_mapping.c
    ├── test_j1939_packing.c
    ├── test_safety_channel.c   # v4 new
    ├── test_safety_comparator.c # v4 new
    └── test_safety_watchdog.c  # v4 new
```

---

## 15.5 Resource Budget and Implementation Margins

The resource budget is frozen against the `JS-300 + CANopen + SAFETY_ENABLED` baseline configuration, used to constrain WCET, stack, and storage upper bounds during implementation.

### 15.5.1 Task-Level WCET / Stack Budget

| Execution Unit | Period | WCET Budget | Stack Budget | Notes |
|----------------|--------|-------------|--------------|-------|
| `safety_task` | 2 ms | 0.60 ms | 1536 B | L2 acquisition, comparison, conditional watchdog, single-step diagnostics |
| `proto_task` | 1 ms | 0.35 ms | 1536 B | `proto_tx_queue` dequeue, `on_rx_frame`, `process_periodic` |
| `ctrl_task` | 5 ms | 0.25 ms | 1024 B | State machine advance, event dispatch |
| `main_task` | 10 ms | 1.20 ms | 2048 B | Sample snapshot, signal chain, `output_adapt`, post TX |
| `diag_task` | 100 ms | 2.00 ms | 1024 B | Logging, NV flush, health check |
| `CAN_RX ISR` | Event-driven | 5 μs | 96 B | Transfer one frame to queue only |
| `ADC_DMA_TC ISR` | Event-driven | 4 μs | 96 B | Publish seq and notification only |

### 15.5.2 Flash / RAM Budget

| Module | Flash Budget | RAM Budget | Notes |
|--------|-------------|------------|-------|
| Platform drivers + HAL adaptation | 48 KB | 6 KB | ADC/CAN/GPIO/SPI/STL adapters |
| Input acquisition + signal chain | 18 KB | 3 KB | `input_acq`, `scan_buffer_t`, pipeline |
| Protocol stack | 28 KB | 4 KB | CANopen/J1939 engine and queues |
| Safety architecture | 24 KB | 4 KB | L2/L3, POST, diagnostic slicing |
| Event/service/app assembly | 14 KB | 3 KB | event bus, NV, logging, app |
| Margin | 28 KB | 12 KB | For new profiles, diagnostic extensions, certification instrumentation |
| **Total budget** | **160 KB** | **32 KB** | Frozen based on mid-range single-MCU configuration |

### 15.5.3 Budget Constraints

- When any task's measured WCET exceeds 80% of the table budget, the budget table must be updated and scheduling margin re-evaluated.
- The protocol internal state of `proto_task` allows single-thread access only; introducing a second TX path immediately invalidates the budget.
- RAM consumption of `scan_buffer_t`, `proto_tx_queue`, and event queues must be accounted for in the product variant difference table; implicit growth is not permitted.

## 15. Complete Dependency Graph (v4 Update)

```
                         ┌──────────────┐
                         │  event_bus   │ ←── Decoupled communication backbone
                         └──────┬───────┘
                                │ subscribe/publish
              ┌─────────────────┼──────────────────┐
              ↓                 ↓                  ↓
     ┌──────────────┐  ┌───────────────┐  ┌──────────────────────┐
     │ device_state │  │  app_tasks    │  │ Safety Architecture   │
     │ (state mach.)│  │ (RTOS tasks)  │  │ (E-Gas L1/L2/L3)     │
     └──────┬───────┘  └──────┬────────┘  │ ┌──────────────────┐ │
            │                 │           │ │ safety_channel    │ │
            │     ┌───────────┼───────┐   │ │ safety_comparator │ │
            ↓     ↓           ↓       ↓   │ │ safety_watchdog   │ │
       ┌─────────────┐ ┌──────────┐   │   │ │ safe_shutdown     │ │
       │ joystick    │ │ output   │   │   │ │ control_flow      │ │
       │ axis_pipeline│ │ adapter  │   │   │ └──────────────────┘ │
       └──────┬──────┘ └────┬─────┘   │   └──────────┬───────────┘
              │              │         │              │
              │              ↓         │              │
              │       ┌────────────┐   │              │
              │       │  protocol  │   │              │
              │       │  engines   │   │              │
              │       └─────┬──────┘   │              │
              ↓             │          ↓              │
  ┌──────────────────────┐  │   ┌──────────────┐     │
  │  Input Acquisition   │  │   │  diagnostics │     │
  │  scan_buffer_t       │  │   └──────┬───────┘     │
  │  buf_axis / buf_btn  │  │          │              │
  └──────────┬───────────┘  │          │              │
             │              │          │              │
             ↓              ↓          ↓              ↓
  ┌──────────────────────────────────────────────────────────┐
  │         drivers/*.h (interface definitions)              │ ←── Dependency inversion boundary
  │   adc_reader / adc_voltage_reader / can / gpio / spi     │
  └──────────────────────────┬───────────────────────────────┘
                             ↑ Implementation (dependency direction upward)
  ┌──────────────────────────┴───────────────────────────────┐
  │         platform_registry.h                              │
  │         platform_init() ← explicit call (MISRA compliant)│
  └──────────────────────────┬───────────────────────────────┘
                             ↑ Registration
              ┌──────────────┼──────────────┐
              │              │              │
       ┌──────┴───┐   ┌─────┴─────┐  ┌─────┴──────┐
       │ STM32G4  │   │ S32K144   │  │ PC Mock    │
       │ platform │   │ platform  │  │ platform   │
       └──────────┘   └───────────┘  └────────────┘
                             │
                    ┌────────┴─────────┐
                    │ Hardware Safety   │ ←── v4 new
                    │ External WDG IC   │
                    │ Safety relay/STO  │
                    │ MPU partitioning  │
                    └──────────────────┘
```

Key changes (v4 vs. v3):
- Added Output Adapter layer (`output/`); value domain conversion separated from protocol packaging
- Protocol engines receive `output_data_t` (self-describing); no longer directly interpret pipeline internal values
- Removed generic `signal_chain_t`; unified as explicit `axis_pipeline_t`
- Safety module refactored to E-Gas three-layer architecture (L2 independent computation + L2 comparator + L3 conditional watchdog)
- Added independent hardware shutdown path (does not go through RTOS/event_bus)
- `joystick_data_t.max_deflection` changed to per-axis (supports mixed-resolution sensors)

---

## 16. Design Principles Compliance Summary

### SOLID Principles Compliance

| Principle | Original Design | Improvement | After Improvement |
|-----------|----------------|-------------|-------------------|
| **S** — Single Responsibility | `joystick_core.c` combined five responsibilities; `main_app.c` mixed assembly with execution; protocol engine did both value conversion and packaging | Signal pipeline split; assembly and execution separated; v4 adds output_adapter to strip value conversion from protocol engine | Each module has only one reason to change |
| **O** — Open/Closed | `product_config.h` `#elif` chain; CMake hardcoded safety product names | Each product has dedicated `.c`; v4 changed to `SAFETY_ENABLED` parameter-driven | Adding products/safety models requires no modification of existing code |
| **L** — Liskov Substitution | Protocol interface forces all methods; some protocols need empty stub functions | Core interface + capability bitmask + optional extension methods | All CAN protocols can be safely substituted |
| **I** — Interface Segregation | `adc_driver_t` contained all methods | Split into `adc_reader` / `adc_voltage_reader` / `adc_continuous` | Each module depends only on the minimal interface it needs |
| **D** — Dependency Inversion | `#if defined(PLATFORM_STM32G4)` hardcoded platform | Platform registry + explicit dependency injection | Upper layers are completely unaware of lower-layer concrete implementations |

### Design Patterns Used

| Pattern | Purpose | Location |
|---------|---------|----------|
| **Observer** | Loose-coupled inter-module event communication | `event/event_bus.h` |
| **Strategy** | Pluggable deadzone/calibration algorithms | `deadzone_fn_t` / `calibrate_fn_t` |
| **State** | Device lifecycle management | `app/device_state.h` |
| **Scan-Buffer** | Decouple hardware acquisition from data consumption | `input/scan_buffer.h` + `input/input_acq.h` |
| **Adapter** | v4: Pipeline internal value → self-describing output | `output/output_adapter.h` |
| **Anti-Corruption Layer** | v4: Protocol layer does not interpret pipeline internal representation | `output_data_t` self-describing boundary object |
| **Tagged Union** | v4: Compile-time type-safe protocol configuration | `protocol_bundle_t` |
| **Dual-Channel Comparison** | v4: E-Gas L2 independent computation + comparison | `safety/safety_comparator.h` |

### Key Improvements: v4 vs. v3

| Improvement | v3 Design | v4 Improvement | Rationale |
|-------------|----------|----------------|-----------|
| Signal chain interface | Generic chain (`int16_t`) + explicit pipeline (`int32_t`) coexisting | Remove generic chain; unify as explicit `axis_pipeline` (`uint16_t→int32_t` boundary) | Eliminate type mismatch and conceptual confusion |
| Value domain conversion | Embedded inside protocol engine | Dedicated `output_adapter` layer (pure function) | SRP + reusable (HMI/logging/debug can all call it) |
| Protocol engine input | `joystick_data_t` (non-self-describing) | `output_data_t` (self-describing, includes value range) | Anti-Corruption Layer |
| max_deflection | Single value (assumes all axes same resolution) | Per-axis (`uint16_t max_deflection[MAX_AXES]`) | Supports JS-300 mixed 12-bit/14-bit sensors |
| Product descriptor | Missing protocol field | Added `protocol_bundle` + `output_maps` + `safety_config` | Complete product self-description |
| Safety architecture | safety_task bystander-style monitoring (reads L1 data to detect anomalies) | E-Gas three layers: L2 independent acquisition+comparison, L3 conditional watchdog+hardware shutdown | SIL 2 / PL d compliance |
| Watchdog | Unconditional feed | Conditional feed (L2 token + program flow flag dual verification) | IEC 61508-7 requirement |
| Safety shutdown | Depends on event_bus → state machine (common cause failure) | GPIO directly drives safety relay (does not go through RTOS/event_bus) | Independent shutdown path |
| CMake safety module | Hardcoded `JS300 OR JS400` | `SAFETY_ENABLED` parameter-driven | OCP: Adding safety models requires no modification of CMakeLists.txt |

### Key Constraints

All improvements maintain practical embedded C usability and functional safety compliance:
- Pure C99 standard; no GCC/compiler-specific extensions (`_Atomic` from C11; safety modules require `-std=c11`)
- No virtual function tables / RTTI
- No dynamic memory allocation (`malloc`/`free`)
- No C++ exceptions
- All buffers statically allocated
- Event bus subscriber count determined at compile time
- All initialization order explicitly controlled (no constructor / implicit initialization)
- MISRA C:2012 achievable
- IEC 61508 SIL 2 / ISO 13849 PL d architecture compliant (E-Gas three layers + independent shutdown path)
