## 控制运行时（ControlRuntime）

概述
----

Control Runtime 提供了一个小而专注的执行环境，用于在固定的时间步长下
对多个控制器实例进行步进（step），并将它们的输出路由到整数输出槽（通常
用于电机的电流/命令值）。它是控制器逻辑（例如 PID、带状态的控制器）与
底层电机发送代码（CAN 电机电流消息）之间的粘合层。

关键概念
--------

- Controller：可插拔的控制对象，通过其虚函数表暴露 `get_output(ctx)` 函数，
  并由 `controller_base` 管理。
- Binding（绑定）：将一个 Controller 映射到一个目标 `volatile int16_t *` 槽，
  运行时时会把饱和后的输出写入该槽。
- Registry（注册表）：运行时用于管理已注册控制器并一次性步进所有控制器的
  内部容器。

公共 API
--------

运行时的 API 在 `Tasks/control_runtime.h` 中声明。

- `void control_runtime_init(void)`
  - 初始化运行时内部结构。必须在绑定控制器或启动运行时任务之前调用一次。

- `bool control_runtime_bind(Controller *c, volatile int16_t *slot, float out_limit)`
  - 将控制器绑定到输出槽。
  - 参数 `c`：指向已创建的 Controller 实例。
  - 参数 `slot`：指向运行时写入饱和后的控制器输出的 `int16_t` 位置，通常
    与电机发送代码共享，不能为空指针。
  - 参数 `out_limit`：用于在转换为整数之前对控制器浮点输出进行限制的正值。
  - 返回 `true` 表示绑定成功，返回 `false` 表示参数无效、绑定表已满或控制器注册失败。

- `void control_runtime_task(void *args)`
  - 作为周期性 RTOS 任务运行的无限循环入口函数。其行为：
    1) 对所有已注册控制器执行步进（step）。
    2) 将受限并饱和后的输出写入各自绑定的槽位。
    3) 触发 CAN 电机电流的发送。
    4) 以 `CTRL_TS_MS` 毫秒为周期休眠。

行为与保证
-----------

- 输出饱和：在写入整数槽之前，运行时会将控制器的浮点输出限定在
  `[-out_limit, +out_limit]` 之内，然后以带饱和的方式转换为 int16，避免
  在打包电机命令时发生整数溢出。
- 安全模式：当全局标志 `manual_online` 为 false 时，`control_runtime_flush()`
  会重置所有控制器并将输出槽清零；这是在系统不在线时停止电机的局部安全行为。
- 线程模型：`control_runtime_task()` 为单一专用线程（或任务），负责调用
  `controller_step_all()` 并向 `volatile` 槽写入数据。其他任务（例如 CAN 发送）
  应当读取这些槽，但必须容忍并发读取。

依赖项与副作用
---------------

- 使用 `controller_base.h` 中的控制器注册/管理函数（`controller_register`、
  `controller_step_all`、`controller_reset_all`）来管理控制器。
- `control_runtime_flush()` 调用 `CanMotor_SendCurrent(...)` 来触发 CAN 电机电流的
  发送，这些发送函数会读取运行时写入的整数槽。
- 运行时依赖于全局变量 `manual_online` 来决定是否在刷新时将输出清零并重置控制器。

使用示例
--------

1. 在系统启动时调用 `control_runtime_init()` 一次。
2. 创建指定类型的控制器 (如`pid`)，调用对应的初始化函数 (如`pid_init`) 配置控制器参数。
3. 创建指定类型的测量提供器上下文实例 (如`DJIMotorProvCtx`)，调用对应的初始化函数 (如`djimotor_ctx_init`) 绑定测量数据源，以及配置测量数据选项。
4. 创建测量提供器实例，调用对应的生成函数 (如`make_djimotor_provider`) 将上下文实例变成统一的测量提供器实例。
5. 调用对应的绑定函数 (如`pid_bind_provider`) 将测量提供器实例绑定到指定类型的控制器上。
6. 创建控制器实例，调用对应的生成函数 (如`make_pid`) 将指定类型的控制器变成统一的控制器实例。
7. 对每个控制器实例，调用 `control_runtime_bind(controller, slot, out_limit)` 将其
   注册到运行时并指定输出写入位置。
8. 创建并启动一个 FreeRTOS 任务来运行 `control_runtime_task(NULL)`，或以其他方式
   周期性地调用 `control_runtime_step()` 与 `control_runtime_flush()`，以满足期望的控制频率。

- 代码示例

```c
// 以拨盘电机绑定为例
PID pid_feeder;
Controller ctrl_feeder;
DJIMotorProvCtx feeder_ctx;
MeasProvider feeder_prov;

// 初始化指定控制器
pid_init(&pid_feeder, 120.0f, 30.0f, 0.0f, CTRL_TS, 10000.0f, 3000.0f, PID_PLANT_VELOCITY);
// 初始化测量提供器上下文
djimotor_ctx_init(&feeder_ctx, &CAN2_Motors[FEEDER_ID], (1.0f/M2006_REDUCTION_RATIO), 0.0f, 1, 0);
// 生成统一的测量提供器实例
feeder_prov = make_djimotor_provider(&feeder_ctx);
// 将提供器示例绑定至指定控制器
pid_bind_provider(&pid_feeder, &feeder_prov, 1);
// 生成统一的控制器实例
ctrl_feeder = make_pid(&pid_feeder);
// 启动控制器（默认关闭）
controller_enable(&ctrl_feeder, true);
// 将控制器绑定至运行时
control_runtime_bind(&ctrl_feeder, &CAN2_MotorCurrents[FEEDER_ID], 10000.0f);
```

设计说明
--------

- 运行时采用固定大小的静态绑定表以保持实现简单并保证确定性的内存使用（上限为
  `CTRL_BINDING_MAX`）。
- 绑定仅保存指针——调用方负责控制器对象的生命周期管理。
- 运行时将步进（`control_runtime_step`）与发送/刷新（`control_runtime_flush`）分离，
  便于测试并允许在需要时采用不同的调度策略。

后续改进建议
--------------

如需更严格的保证，可以考虑：

- 如果槽位的写入/读取比简单的 int16 写入更复杂，则在绑定与槽写入处加入原子操作或
  互斥锁。
- 提供解绑（unbind）和查询当前绑定的 API。
- 添加测试桩（test harness）以在可控循环中运行 `control_runtime_step()`，便于单元测试。

文件位置
--------

- 源文件：`Tasks/control_runtime.c`
- 头文件：`Tasks/control_runtime.h`

相关文件说明
------------

下面是与 Control Runtime 密切相关的四个文件的简要中文说明（头文件采用 Doxygen 风格注释，源码文件使用普通英文注释）：

- `Components/Inc/controller_base.h`：
  - 定义了通用的 Controller 接口和虚表（ControllerVTable）、引用/测量数据结构（`ctrl_ref_t`, `ctrl_meas_t`）以及控制器注册表 `ControllerRegistry`。
  - 提供了一组便捷内联函数（例如 `controller_set_vref`, `controller_ingest_xmeas`）用于向控制器传入参考/测量、使能/复位等。头文件中的注释采用 Doxygen 风格，描述每个类型字段与 API 的用途和参数。

- `Components/Src/controller_base.c`：
  - 实现了注册表的初始化、控制器注册、对所有已注册控制器的逐步调用（`controller_step_all`）以及对所有控制器的复位（`controller_reset_all`）。
  - 源码文件使用了简明的英文注释，解释了每个函数的行为和跳过空/未启用控制器的理由。

- `Components/Inc/controller_pid.h`：
  - 定义了 PID 控制器的配置/运行态结构 `PID`，包括增益、时间步 Ts、输出与积分限制、当前参考与测量缓存，以及可选的测量提供者（MeasProvider）。
  - 声明了用于初始化 PID 的 `pid_init`、将 PID 封装为 `Controller` 的 `make_pid` 以及绑定测量提供者的 `pid_bind_provider`。头文件已用 Doxygen 风格注释说明参数与返回值语义。

- `Components/Src/controller_pid.c`：
  - PID 的具体实现：包含 `set_reference`、`ingest_measurement`、`step`、`get_output`、`reset`、`enable` 等函数并在 `PID_VT` 中注册为 Controller 的 vtable。
  - `pid_step` 实现了常见的 PID 算法步骤：可选的从 provider 拉取测量、按模式（位置/速度）计算误差、更新积分并作抗积分饱和（anti-windup）、计算导数（有限差分）、合成输出并限制到指定范围。源码文件中以英文注释解释了关键逻辑块。

这些注释和文档一起可以帮助开发者快速理解控制器与运行时之间的接口契约、线程模型和安全保证。如果需要，我可以把这些中文说明拆分为单独的文档文件或把控制器的使用示例添加到文档中。

深入理解：`controller_base` 通用函数接口
---------------------------------

下面以较高可读性的方式说明 `Components/Inc/controller_base.h` 中的核心函数与辅助内联函数，帮助你理解运行时如何与控制器交互：

- `void controller_registry_init(ControllerRegistry *r)`
  - 作用：初始化一个 `ControllerRegistry`，将 `count` 置零并把内部数组清空。
  - 场景：在系统启动或运行时重置时调用，确保注册表处于已知的空状态。

- `bool controller_register(ControllerRegistry *r, Controller *c)`
  - 作用：将一个 `Controller` 指针加入注册表。返回 `true` 表示成功，`false` 表示注册表已满。
  - 注意：该函数只存储指针，不会复制控制器对象；调用方必须保证 `Controller` 的生命周期足够长。

- `void controller_step_all(ControllerRegistry *r)`
  - 作用：遍历注册表，对每个非空且 `enabled` 的控制器调用其 vtable 中的 `step` 函数。
  - 语义：这是驱动控制器逻辑（例如 PID 运算）前进的统一入口，通常在控制周期（control tick）中被调用一次。

- `void controller_reset_all(ControllerRegistry *r)`
  - 作用：遍历注册表并调用每个控制器的 `reset` 函数（如果实现了的话），用于清理内部状态（例如积分器）。
  - 场景：进入安全/离线模式或手动复位时使用。

辅助内联函数（便捷接口）

- `clamp(float x, float lo, float hi)`：简单的边界限制器，许多实现直接使用它对浮点量做上下界约束。
- `controller_enable(Controller *c, bool en)`：设置 `c->enabled` 并通知控制器的 `enable` 回调（如果存在）。用于启动/停止单个控制器。
- `controller_set_vref`、`controller_set_xref`、`controller_set_pvref`：以统一的 `ctrl_ref_t` 结构把参考输入传递到控制器；这些都是方便调用的包装函数。
- `controller_ingest_vmeas`、`controller_ingest_xmeas`、`controller_ingest_pvmeas`：将测量值（位置/速度）封装为 `ctrl_meas_t` 并调用控制器的 `ingest_measurement` 回调。

设计要点（简洁）

- 注册表采用固定大小数组（`CONTROLLER_MAX`），保证内存可预测与实时性，适合嵌入式环境。
- 控制器接口通过虚表（`ControllerVTable`）抽象，支持不同类型控制器（PID、状态空间、跟踪控制器等）统一管理。
- 注册和绑定都以指针为主，运行时不负责控制器对象的内存分配或回收，使用者需管理生命周期。

创建并使用 `controller_pid`（PID 控制器）的接口说明
-----------------------------------------

`controller_pid` 模块在 `Components/Inc/controller_pid.h` / `Components/Src/controller_pid.c` 中实现。下面说明如何正确创建、初始化并将 PID 用在 Control Runtime 中：

1) 配置并初始化 PID 上下文结构体 `PID`：

   - 使用 `pid_init(PID *p, float Kp, float Ki, float Kd, float Ts, float out_limit, float integral_limit, pid_mode_t mode)` 初始化结构体。该函数会清零内部状态并设置增益、时间步、输出/积分限值以及工作模式（位置或速度型）。

2) 将 PID 包装为通用 `Controller`：

   - 调用 `Controller c = make_pid(&my_pid);` 来得到一个 `Controller` 值（结构体），该值的 `vt` 指向 PID 的 vtable (`PID_VT`) 且 `ctx` 指向你的 `PID` 实例。

3) 选择测量提供方式（push/pull）：

   - `PID` 支持两种测量输入方式：
     - push：外部代码直接调用 `controller_ingest_*` / PID 的 `ingest_measurement` 回调将测量推送到 PID。
     - pull：配置一个 `MeasProvider`，调用 `pid_bind_provider(PID *p, MeasProvider *prov, uint8_t prefer_pull)` 并设置 `prefer_pull=1`，在 `pid_step()` 中会尝试从 provider 拉取最新样本。

4) 将 Controller 注册并绑定到 Control Runtime：

   - 先通过 `control_runtime_bind(&c, slot_ptr, out_limit)` 把控制器与一个 `volatile int16_t` 输出槽绑定（`out_limit` 在运行时会再次用于限制输出）。注意：`control_runtime_bind` 内部也会把该控制器注册到其内部 `ControllerRegistry`。

5) 启用控制器并运行：

   - 可调用 `controller_enable(&c, true);` 启动 PID（或者在 runtime 调用前设置 `c.enabled = true`）。
   - 确保 `control_runtime_task()` 正在周期性运行或手动按周期调用 `control_runtime_step()` + `control_runtime_flush()`。

简短示例（伪代码）

```c
// 1) 初始化
PID my_pid;
pid_init(&my_pid, 1.0f, 0.1f, 0.01f, CTRL_TS, 1000.0f, 100.0f, PID_PLANT_VELOCITY);

// 2) 包装为 Controller
Controller c = make_pid(&my_pid);

// 3) 准备输出槽并绑定到 runtime
volatile int16_t motor_slot = 0;
control_runtime_init();
control_runtime_bind(&c, &motor_slot, 1000.0f);

// 4) 启用并启动运行时任务（例：FreeRTOS）
controller_enable(&c, true);
// start FreeRTOS task calling control_runtime_task(NULL)
```

生命周期与注意事项

- 绑定后不要释放 `PID` 实例或 `Controller`，直到确保运行时不再访问它们。
- `out_limit` 在两处生效：PID 内部 `p->out_limit`（在 `pid_init` 时设置）和 `control_runtime_bind` 时传入的绑定级别 `out_limit`；两者都用于限制输出，但运行时的绑定级别是最终写入整数槽前的最后一道保护。
- 如果需要在线修改 PID 参数（例如增益），应避免在控制器正在被 step 的同时修改共享内存，或使用简单的同步（在嵌入式中可禁用中断或在非实时配置时进行）。

如果你愿意，我可以把上面的伪代码改成能编译的真实示例（加上头文件包含、任务创建示例和通信槽说明），并把它加入 `docs/control_runtime.md` 或单独的示例文件。
