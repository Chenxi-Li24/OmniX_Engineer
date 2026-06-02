# 项目算法报告

## 1. 概览

本报告只整理当前项目主运行链路中**实际参与控制、估计、标定、映射和门控**的算法实现，不展开纯库能力或未接入主流程的保留实现。

当前算法主链路可以概括为：

- 遥控/外部输入 -> 模式解析 -> 目标生成/映射 -> 执行器控制律 -> 电机输出
- IMU 原始采样 -> 温控与标定 -> 姿态解算/INS -> 对外状态输出
- 底盘速度指令 -> 4 轮 swerve 运动学 -> 舵向/轮速级联 PID -> 电流输出

主要代码入口：

- 云台/机械臂执行：`Tasks/Src/Gimbal_Task.cpp`
- LK 目标层与补偿执行：`Tasks/Src/Gimbal_behavior_Task.cpp`
- 底盘控制：`Tasks/Src/Chassis_Task.cpp`
- IMU 任务：`Frameworks/tskptt_imu/Src/tskptt_imu.c`
- IMU/INS：`Frameworks/lib_imu/Src/lib_imu.cpp`、`Frameworks/lib_imu/Src/lib_ins.c`
- 通用算法库：`Frameworks/lib_algos`

配置主要集中在：

- 云台/机械臂：`Tasks/Inc/CONF_Gimbal_Task.h`、`Tasks/Inc/CONF_Gimbal_Zero.h`
- 底盘：`Tasks/Inc/CONF_Chassis_Task.h`

---

## 2. 云台/机械臂算法

### 2.1 DM 关节控制（J4/J5/J6/J7）

代码位置：`Tasks/Src/Gimbal_Task.cpp`

#### 作用目标

- 为 DM4310/DM4340 类关节提供统一的角度控制外壳。
- 同时支持角度模式、速率模式、MIT 发送、绝对/相对参考及上电回目标逻辑。

#### 输入 / 输出

- 输入：遥控器输入、外部控制输入、当前电机反馈、零点与角度限位配置、重力补偿前馈。
- 输出：每个关节的目标角、MIT 控制命令或电流/力矩命令。

#### 核心逻辑

- `GimbalAxis` 类封装了关节状态、参考系、PID、MIT 参数和发送模式。
- 角度控制本质是：
  - 位置误差 -> 角度 PID -> 目标速度
  - 目标速度/位置 -> MIT 命令下发
- 不同关节通过 `GimbalMotorPosRefMode` 决定使用多圈相对、单圈绝对或有限单圈窗口。
- `clamp_fp32`、`wrap_symmetric_period`、`nearest_equivalent` 用于限幅、最近等价角选择和单圈目标归一化。

#### 调用链与所属任务

- `Start_Gimbal_Task()` 周期更新 DM 关节反馈、模式、目标、PID 和 CAN 发送。
- DM 关节算法与 LK 关节并行运行，但由不同任务文件负责。

#### 关键配置参数

- 单轴 PID、MIT 参数、零点、角度限位、power-on home、方向配置都在 `Tasks/Inc/CONF_Gimbal_Task.h`。
- 原始零点在 `Tasks/Inc/CONF_Gimbal_Zero.h`。

#### 当前问题 / 注意事项

- 大量动态效果依赖实机调参，尤其是 J7 与补偿链相关轴。
- DM 关节的“控制器映射”和“执行器控制律”仍耦合在同一任务文件中，文档化后适合继续拆层。

### 2.2 外部输入映射与保持逻辑

代码位置：`Tasks/Inc/CONF_Gimbal_Task.h`、`Tasks/Src/Gimbal_Task.cpp`、`Tasks/Src/Gimbal_behavior_Task.cpp`

#### 作用目标

- 把控制器原始值、RC、Link、自定义外部输入统一映射为关节目标。
- 在回中、死区、单边有效侧等场景下生成 hold / inactive / absolute 语义。

#### 输入 / 输出

- 输入：控制器原始值、死区参数、映射端点、零位、激活侧配置。
- 输出：
  - DM 侧相对角目标
  - LK 侧 `LkTargetRequest`

#### 核心逻辑

- `Gimbal_MapExternalRawToBidirRelAngle(...)`：统一的 7 轴双边映射，输入是控制器 raw，输出是目标相对角；中心死区内返回 hold。
- `Gimbal_MapRelAngleToExternalRaw(...)`：把机器人当前相对角反算回等效控制器 raw，供 `0309.raw_u16[]` 回传同域反馈。
- `Gimbal_NormalizeRawCenteredNoWrap(...)` 先把控制器值归一化到 `[-1, 1]`，再映射到目标角。
- 映射只负责把输入转成目标语义，不直接下发电机命令。

#### 调用链与所属任务

- DM J4~J7 外部输入映射在 `Start_Gimbal_Task()` 中消费。
- LK J1~J3 外部输入映射在 `Start_Gimbal_behave()` 中进一步封装为目标层请求。

#### 关键配置参数

- 映射端点、零位、单侧有效方向、外部死区均在 `Tasks/Inc/CONF_Gimbal_Task.h`。

#### 当前问题 / 注意事项

- 映射配置和标定强相关，最近已对 J2/J3/J7 多次重标定，文档应作为后续复测基线。
- 当前配置体现了“工程调通优先”，不是完全抽象的标定框架。

### 2.3 LK 统一目标层（J1/J2/J3）

代码位置：`Tasks/Src/Gimbal_behavior_Task.cpp`

#### 作用目标

- 将 J1/J2/J3 统一为“上层封装目标、下层执行不变”的结构。
- 屏蔽 RC、Link、外部输入之间的差异，让执行层只消费 `target_rel_deg`。

#### 输入 / 输出

- 输入：本地 RC/Link 摇杆、外部原始控制器值、pause/mapping reset 覆盖、safe/disable 门控。
- 输出：每轴 `LkTargetState.target_rel_deg`。

#### 核心逻辑

- `LkTargetRequest` 只表达请求语义：inactive / hold / delta / absolute。
- `LkTargetState` 持久保存每轴目标缓存与来源状态。
- `lk_resolve_target(...)` 只做目标合成：
  - inactive：不改缓存目标
  - hold：保留当前目标
  - delta：在缓存上积分
  - absolute：直接覆盖目标
- 未初始化目标统一从“相对配置零点的 0deg”开始，不再抓当前姿态做会话零点。

#### 调用链与所属任务

- `Start_Gimbal_behave()` 每个周期先读取输入，再为 J1/J2/J3 生成 request，最后合成为统一目标缓存。

#### 关键配置参数

- 各轴配置零点、手柄比例、死区、最小步进、限位都在 `Tasks/Inc/CONF_Gimbal_Task.h`。

#### 当前问题 / 注意事项

- 该层已经比旧版分支式下发更清晰，但仍是文件内私有 lambda 组织，后续可以提取成独立模块。
- 当前目标层与日志层绑定较紧，后续若要复用到更多关节，需要再做接口收敛。

### 2.4 LK torque-pos 执行层（J1/J2/J3）

代码位置：`Tasks/Src/Gimbal_behavior_Task.cpp`

#### 作用目标

- 将 LK 关节统一为基于目标角的力矩模式闭环执行。
- J1 也已统一到该链路，不再单独走纯位置模式。

#### 输入 / 输出

- 输入：`target_rel_deg`、当前编码器角度、当前转速、各轴 PID/限流/死区配置。
- 输出：`set_iq_A(...)` 电流命令。

#### 核心逻辑

- `lk_apply_torque_pos_target(...)` 逻辑为：
  - `raw_err = target_rel - current_rel`
  - `p_out = kp * err`
  - `i_out += ki * err * dt`，并做积分限幅
  - `d_out = -kd * speed`
  - `cmd = p + i + d`，再做电流限幅
  - 误差和速度同时落入死区时，清零积分并输出 0
- 当前相对角一律由“编码器角 - 配置零点”得到，配置零点是唯一参考系。

#### 调用链与所属任务

- 目标层完成后，`Start_Gimbal_behave()` 直接调用 `lk_apply_torque_pos_target(...)` 分别执行 J1/J2/J3。

#### 关键配置参数

- `GIMBAL_LK_Jx_TORQUE_POS_KP/KI/KD/MAX_A/I_MAX_A/DB_DEG/DB_DPS`
- `GIMBAL_LK_Jx_POS_DIR_INVERT`

#### 当前问题 / 注意事项

- J1/J2/J3 的“手感”和稳定性高度依赖实机调参，近期已经针对 J1 抖动、J2/J3 过冲做过软化。
- 执行器内部使用固定 `0.002s` 采样周期，默认依赖任务周期稳定。

### 2.5 补偿电机 J8/J9 与门控逻辑

代码位置：`Tasks/Src/Gimbal_behavior_Task.cpp`

#### 作用目标

- 使用 DM 补偿电机分担 J2/J3 的力矩输出。
- 在 pause、power-on home、safe/zero-force 下维持一致的门控行为。

#### 输入 / 输出

- 输入：`lk_j2_cmd_A`、`lk_j3_cmd_A`、补偿比例、方向符号、限幅、slew 参数。
- 输出：J8/J9 的 MIT 力矩命令。

#### 核心逻辑

- 目标力矩由 `主关节电流命令 * 扭矩换算系数 * share * sign` 得到。
- 再经过：
  - `clamp_nm(...)` 扭矩限幅
  - `slew_nm(...)` 斜率限制
- pause 和 power-on home 时，不走扭矩补偿，而是直接走 MIT 位置保持命令。
- safe 模式下，J8/J9 会直接 disable。

#### 调用链与所属任务

- 与 LK 执行层同周期在 `Start_Gimbal_behave()` 中运行。

#### 关键配置参数

- `GIMBAL_DM_J8_TORQUE_*`、`GIMBAL_DM_J9_TORQUE_*`
- `GIMBAL_PAUSE_DM_J8_MIT_*`、`GIMBAL_PAUSE_DM_J9_MIT_*`

#### 当前问题 / 注意事项

- J9 方向近期发生过修正，说明补偿链对机构方向非常敏感。
- 异响和打齿通常不仅是方向问题，也可能来自补偿比例过高或斜率过陡。

### 2.6 云台模式解析与安全门控

代码位置：`Tasks/Src/VT03_Gimbal_Mode.cpp`、`Tasks/Src/Gimbal_Task.cpp`、`Tasks/Src/Gimbal_behavior_Task.cpp`

#### 作用目标

- 把 DR16/VT03 遥控状态解析为统一模式，再据此控制是否允许各算法链输出。

#### 输入 / 输出

- 输入：RC 快照、VT03 按钮、主模式拨杆、云台模式拨杆。
- 输出：`RcControlMode`、各任务内部 enable/disable/hold/zero-force 判定。

#### 核心逻辑

- `VT03_Gimbal_ResolveControlMode(...)` 在 VT03 模式下通过按键沿触发锁存 DM/LK/Link 模式。
- `ZeroForce` / safe 模式被统一提升为高优先级门控：所有电机下力、外部输入失效。
- pause 和 mapping reset 属于更外层的目标覆盖机制，但 safe 仍能直接截断输出。

#### 调用链与所属任务

- 云台 DM 和 LK 两个任务都依赖统一的控制模式解析结果。

#### 关键配置参数

- RC 通道、按键位、门控开关分散在 `Tasks/Inc/RC_Control_Mode.h`、`Tasks/Inc/CONF_Gimbal_Task.h`。

#### 当前问题 / 注意事项

- 当前门控规则分布在多个任务文件中，语义已经逐步统一，但仍不完全集中。

---

## 3. 底盘算法

### 3.1 模式决策与 safe hold

代码位置：`Tasks/Src/Chassis_Task.cpp`

#### 作用目标

- 根据遥控器主模式、在线状态、阻塞状态切换底盘控制模式，并在必要时直接输出零力。

#### 输入 / 输出

- 输入：`RcControlMode`、RC 在线状态、gimbal 快照、导航输入。
- 输出：`ChassisMode`、底盘零力状态、各电机输出命令。

#### 核心逻辑

- `decide_mode_from_control(...)` 把主模式解析为 `NoForce / Follow / Auto / Hold`。
- `safe_hold` 条件下直接：
  - 清零底盘目标速度
  - 清零电机输出
  - 保持日志与状态输出
- safe hold 是底盘控制最外层的安全策略。

#### 调用链与所属任务

- `Start_Chassis_Task` 内的控制器周期调用 `update_mode()`、`run_pid()`、`can_send()`。

#### 关键配置参数

- safe hold、驱动限流、模式相关参数主要在 `Tasks/Inc/CONF_Chassis_Task.h`。

#### 当前问题 / 注意事项

- 底盘安全策略和云台安全策略风格接近，但尚未抽象成项目统一安全状态机。

### 3.2 四轮 swerve 运动学与 anti 翻转

代码位置：`Frameworks/lib_algos/Src/swerve_kinematics.cpp`、`Tasks/Src/Chassis_Task.cpp`

#### 作用目标

- 将车体速度 `(vx, vy, wz)` 分解成四个模块的舵向角和轮速。
- 通过 anti 逻辑尽量减少舵轮转角，允许轮速反向替代 180 度大角度转向。

#### 输入 / 输出

- 输入：车体速度、轮子安装位置、当前舵角、上一周期 anti 状态。
- 输出：每个模块的 `theta_set_deg`、`wheel_mps`、`anti`。

#### 核心逻辑

- 对第 `i` 个轮，先计算：
  - `vxi = vx - wz * y_i`
  - `vyi = vy + wz * x_i`
- 再计算目标角 `atan2(vyi, vxi)` 和对向角 `angle - 180deg`。
- 比较两者相对当前角的误差，结合 `anti_hyst_deg` 滞回决定是否翻转。
- 若翻转，则角度走对向角，轮速取负。

#### 调用链与所属任务

- `Chassis_Task` 的 `update_target_from_cmd()` 中调用 `solve_swerve_4wheel(...)`。

#### 关键配置参数

- 轮子几何位置、停转 45 度姿态、anti 滞回、速度阈值都在 `Tasks/Inc/CONF_Chassis_Task.h`。

#### 当前问题 / 注意事项

- 该算法结构清晰，但与底层轮速/舵角控制没有彻底解耦，调试时仍需同时观察 PID 输出。

### 3.3 舵向/轮速级联 PID

代码位置：`Tasks/Src/Chassis_Task.cpp`

#### 作用目标

- 将舵角误差和轮速误差转成 6020/C620 的电流命令。

#### 输入 / 输出

- 输入：当前舵角、目标舵角、当前轮速、目标轮速。
- 输出：舵向电流命令、驱动电流命令。

#### 核心逻辑

- 舵向链路：
  - `delta_angle = current - target`
  - 角度 PID 输出期望角速度
  - 速度 PID 再输出舵向电流
- 驱动链路：
  - 轮速 PID 直接输出驱动电流
- 所有输出经过限幅，并记录饱和次数用于诊断。

#### 调用链与所属任务

- `run_pid()` 中完成全部级联 PID 计算。
- `can_send()` 把 PID 输出按方向符号映射到各电机。

#### 关键配置参数

- 舵向角度/速度 PID、轮速 PID、最大输出、方向符号在 `Tasks/Inc/CONF_Chassis_Task.h`。

#### 当前问题 / 注意事项

- 当前 PID 完全依赖工程调参，没有更上层的自整定或模型补偿。
- 轮速饱和诊断已实现，但还没有进一步自动降额或防饱和策略。

### 3.4 底盘输入滤波

代码位置：`Tasks/Src/Chassis_Task.cpp`、`Frameworks/lib_algos/Src/filters.cpp`

#### 作用目标

- 预留对 `vx / vy / wz` 指令做一阶低通滤波，降低操控突变。

#### 输入 / 输出

- 输入：原始速度指令。
- 输出：滤波后速度指令。

#### 核心逻辑

- 一阶滤波公式为：
  - `out = T / (T + dt) * input + dt / (T + dt) * last_out`
- 底盘任务中已初始化 `filt_vx_ / filt_vy_ / filt_wz_`，但实际滤波调用目前被注释掉。

#### 调用链与所属任务

- 滤波器由底盘控制器构造函数初始化。
- 当前主流程中未真正生效。

#### 关键配置参数

- 时间常数在 `Tasks/Inc/CONF_Chassis_Task.h`。

#### 当前问题 / 注意事项

- 这是“已接入初始化、但当前关闭”的算法，不应误判为在线有效链路。

---

## 4. IMU / 姿态算法

### 4.1 IMU 温控 PID

代码位置：`Frameworks/lib_imu/Src/lib_imu.cpp`、`Frameworks/tskptt_imu/Src/tskptt_imu.c`

#### 作用目标

- 将 BMI088 工作温度稳定在目标温度附近，提升陀螺和加速度计一致性。

#### 输入 / 输出

- 输入：当前温度、目标温度、稳定判定窗口、PID 参数。
- 输出：PWM 加热占空比。

#### 核心逻辑

- 温控是分段控制：
  - 温差大时全功率加热
  - 接近目标温度后切换到 PID 精调
  - 超温则 fault，立即关闭加热
- 近目标时使用 `PID_DELTA` 形式的 PID。
- 只有温度在误差带内持续一段时间后，才判定为 `STABLE`。

#### 调用链与所属任务

- `tskptt_imu` 周期采样后调用 `IMU_TempCtrl_StepWithTemp(...)`。
- 温控稳定是进入后续标定/INS 的前置条件。

#### 关键配置参数

- 目标温度、稳定误差、稳定时间、PID 参数在 `Frameworks/tskptt_imu/Src/tskptt_imu.c` 顶部宏和初始化参数中给出。

#### 当前问题 / 注意事项

- PID 参数直接写在任务文件内，适合后续迁移到配置头统一管理。
- 当前没有更细粒度热模型，属于工程实用型温控。

### 4.2 IMU 静态标定与 EEPROM 持久化

代码位置：`Frameworks/lib_imu/Src/lib_imu.cpp`、`Frameworks/tskptt_imu/Src/tskptt_imu.c`

#### 作用目标

- 在温度稳定后获得陀螺零偏和加速度尺度因子，并根据模式决定是否持久化到 EEPROM。

#### 输入 / 输出

- 输入：IMU 样本、温度状态、EEPROM 中历史结果。
- 输出：`imu_calib_result_t`。

#### 核心逻辑

- 模式 0：不重新标定，直接从 EEPROM 读取；若无效则用默认值。
- 模式 1：静态标定，累计固定样本数：
  - 陀螺偏置取均值
  - 加速度模长均值用于估计缩放系数，使其回归 9.81
- 模式 2：转台标定预留，当前未实现。
- 标定成功后可写回 EEPROM 持久化。

#### 调用链与所属任务

- `tskptt_imu` 中温控稳定后进入标定状态机，再切换到 INS 状态机。

#### 关键配置参数

- `IMU_CALIBRATION_MODE`、样本数、超时、EEPROM 地址均在 `Frameworks/tskptt_imu/Src/tskptt_imu.c`。

#### 当前问题 / 注意事项

- 当前默认模式是 0，意味着工程更依赖历史标定而不是每次上电自动重标定。
- 标定方法是工程静态标定，不包含多姿态六面体标定或在线偏置估计。

### 4.3 Mahony 姿态解算与 INS 包装

代码位置：`Frameworks/lib_imu/Src/lib_ins.c`、`Frameworks/lib_algos/Src/mahony_filter.c`

#### 作用目标

- 使用陀螺 + 加速度计输出姿态四元数、欧拉角、去重力加速度和简化导航量。

#### 输入 / 输出

- 输入：校正后的 `gyro`、`accel`、采样周期 `dt`。
- 输出：
  - 四元数 `q`
  - 原始 `roll/pitch/yaw`
  - `roll_out/pitch_out/yaw_out`
  - `motion_accel_b / motion_accel_n`
  - `yaw_total`、`v_n`、`x_n`

#### 核心逻辑

- Mahony 部分：
  - 加速度归一化
  - 由估计重力方向与实测重力方向求误差
  - 用 `Kp/Ki` 修正角速度
  - 四元数积分并归一化
- INS 包装部分：
  - 用四元数完成体坐标系与地坐标系变换
  - 从加速度中减去重力，得到运动加速度
  - 对运动加速度做一阶低通
  - `yaw` 做多圈跟踪
  - 在 `RUN` 状态下积分得到简单的 `v_n / x_n`

#### 调用链与所属任务

- `tskptt_imu` 在标定完成后调用 `LIB_INS_Update(...)`。
- 外部通过 `LIB_INS_GetState()` 和同步后的 `INS` 公共结构访问结果。

#### 关键配置参数

- Mahony 初值和 warmup 时间在 `Frameworks/lib_imu/Src/lib_ins.c`。
- 当前 `mahony_init(&s_mahony, 1.0f, 0.0f, 0.001f)`，默认无积分项。
- `s_accel_lpf`、warmup 时间也在该文件内定义。

#### 当前问题 / 注意事项

- `CONV -> RUN` 的收敛判据目前主要依赖时间，不是完整的姿态稳定性判据。
- 速度/位移积分很轻量，更接近调试辅助量，不应当成高可信导航解。

### 4.4 IMU 任务状态机

代码位置：`Frameworks/tskptt_imu/Src/tskptt_imu.c`

#### 作用目标

- 把采样、温控、标定、INS 更新、外部 OMX IMU 接入统一串起来。

#### 输入 / 输出

- 输入：BMI088 样本、OMX CAN IMU 数据、EEPROM、温控状态。
- 输出：
  - `g_imu_app_state`
  - `INS` 公共结构
  - 对外快照与调试量

#### 核心逻辑

- 运行流程是：
  - 初始化 BMI088、INS、温控、EEPROM、OMX IMU
  - 采样成功时先温控
  - 温控稳定后进入标定/读取标定
  - 标定完成后更新 INS
  - 同时维护 internal / external 快照

#### 调用链与所属任务

- `tskptt_imu` 是 IMU 算法主任务入口。

#### 关键配置参数

- 任务周期、日志分频、温控参数、标定模式、EEPROM 地址在 `Frameworks/tskptt_imu/Src/tskptt_imu.c` 顶部宏区。

#### 当前问题 / 注意事项

- 任务承载内容较多：采样、温控、标定、INS、持久化、外部 IMU 汇聚都在一个任务内。
- 如果后续状态机继续增大，建议拆分温控/标定/姿态处理职责。

---

## 5. 通用算法基座

### 5.1 PID 基础库

代码位置：`Frameworks/lib_algos/Src/pid.cpp`

#### 作用目标

- 为云台、底盘、IMU 温控提供统一 PID 计算能力。

#### 输入 / 输出

- 输入：反馈值、设定值、PID 参数、积分限幅、输出限幅。
- 输出：PID 输出值及内部 `Pout/Iout/Dout` 状态。

#### 核心逻辑

- 支持两种模式：
  - `PID_POSITION`
  - `PID_DELTA`
- 两种模式均带：
  - 输出限幅
  - 积分限幅
  - 内部历史误差缓存

#### 调用链与所属任务

- 底盘舵向/轮速 PID
- DM 关节角度 PID
- IMU 温控 PID

#### 关键配置参数

- 参数由各任务或配置头传入，库本身不持有项目级参数。

#### 当前问题 / 注意事项

- 该 PID 库通用性强，但目前缺少抗积分饱和、前馈融合、采样周期显式参数等更高级特性。

### 5.2 数学工具与滤波工具

代码位置：`Frameworks/lib_algos/Inc/mathutil.h`、`Frameworks/lib_algos/Src/mathutil.cpp`、`Frameworks/lib_algos/Src/filters.cpp`

#### 作用目标

- 提供控制链路反复使用的基础数学运算、角度包络、死区、一阶滤波、OLS 工具。

#### 输入 / 输出

- 输入：标量、角度、编码器值、采样输入。
- 输出：限幅值、归一化值、滤波输出、OLS 结果。

#### 核心逻辑

- `clampf / wrap_pi / wrap_deg / deadzone / norm660_to_unit` 是控制与映射的基础工具。
- `first_order_filter_*` 提供简单低通滤波。
- `OLS_*` 提供平滑和导数估计接口。

#### 调用链与所属任务

- 当前主链路实用部分主要是 PID 与角度/限幅工具。
- 一阶滤波在底盘中已初始化但未实际使能。
- OLS 当前未进入主流程。

#### 关键配置参数

- 一阶滤波时间常数由调用方配置。

#### 当前问题 / 注意事项

- 工具库能力大于当前实际使用范围，应区分“库能力”和“在线链路”。

---

## 6. 库内存在但当前未接入主流程

以下实现位于仓库内，但从当前主任务调用链看，不属于正文中的在线算法：

- `Frameworks/lib_algos/Src/QuaternionEKF.c`
  - 基于通用 KalmanFilter 的四元数 EKF 姿态解算。
  - 当前主流程实际使用的是 Mahony + INS 包装，不是 QuaternionEKF。
- `Frameworks/lib_algos/Src/kalman_filter.c`
  - 通用 Kalman 框架，作为库能力存在。
  - 当前项目主链没有直接调用它。
- `Frameworks/lib_algos/Src/mathutil.cpp` 中的 `OLS_*`
  - 当前未在主任务链路中发现实际消费。
- `Frameworks/lib_power_control`
  - 当前未在主流程中发现明确调用关系，不纳入本报告正文。

---

## 7. 当前风险与现状

### 7.1 参数强依赖实机

- 云台 LK torque-pos、DM 角度环、J8/J9 补偿、底盘舵向/轮速 PID 都高度依赖实际机构、减速比、摩擦和负载。
- 当前工程更像“参数驱动的实机控制系统”，算法正确性与参数质量同等重要。

### 7.2 算法层与任务层耦合较重

- 云台目标层、执行层、门控、日志仍集中在少数大文件中。
- IMU 任务同时承担温控、标定、姿态、持久化和外部 IMU 汇聚。
- 这对调试方便，但对复用和单元测试不友好。

### 7.3 安全门控已成体系，但尚未完全统一抽象

- 云台、底盘都形成了 safe/zero-force/pause 的安全策略。
- 目前属于“多任务内一致实现”，还不是“单一安全状态机模块”。

### 7.4 库能力与实际在线链路需要继续分离表达

- 仓库中的算法库比在线系统使用的算法更多。
- 后续若继续整理技术资产，建议把“在线使用算法”和“可选备用算法”拆成两份文档。

---

## 8. 结论

当前项目的算法体系已经具备较完整的工程控制闭环：

- 云台/机械臂侧：已形成“输入映射 -> 统一目标层 -> torque-pos/DM 执行 -> 补偿电机”的分层结构。
- 底盘侧：已形成“模式决策 -> swerve 运动学 -> 级联 PID -> 电流输出”的标准链路。
- IMU 侧：已形成“温控 -> 标定 -> Mahony/INS -> 对外状态”的稳定工作流。

从工程角度看，当前最值得继续优化的不是新增算法，而是：

- 继续解耦任务文件中的控制逻辑
- 将关键参数和标定流程进一步制度化
- 明确哪些算法是在线主链，哪些只是库储备能力
