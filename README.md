# RobotMatrix — Unity 工业机械臂运动学引擎

[![Unity](https://img.shields.io/badge/Unity-2021.3+-black.svg)](https://unity.com)
[![C#](https://img.shields.io/badge/C%23-9.0-blue.svg)](https://docs.microsoft.com/dotnet/csharp/)
[![.NET Standard](https://img.shields.io/badge/.NET_Standard-2.1-purple.svg)]()

> **高精度 N-DOF 机械臂运动学与控制系统** — 纯 C# 实现，零第三方依赖，四阶段并行 IK 求解架构

---

## 功能特性

**运动学核心**
- **正运动学 (FK)** — DH 参数链式矩阵变换，O(N) 复杂度
- **逆运动学 (IK)** — 四阶段并行求解：加权 DLS + 零空间优化 + 锁定关节补偿 + 多解搜索
- **N-DOF 通用** — 不限关节数量，支持旋转 (Revolute) / 移动 (Prismatic) 关节

**工业运动指令**
- **MoveJ** — 关节空间插值 (PTP)，各关节独立匀速，同步到达
- **MoveL** — 笛卡尔直线插值，TCP 沿直线匀速运动，路径精确可控
- **IK 失败策略** — StopAndReport / FallbackToPTP 自动降级

**高级特性**
- **四级梯队评分** — 缓动系数 + 梯队衰减系数，智能选择最优解
- **软限位动态权重** — 关节靠近限位时自动抑制，梯度惩罚可配置
- **零空间中位回归** — 冗余自由度利用，关节自动趋向行程中位
- **拖动保持 TCP** — 锁定指定关节，其他关节自动补偿维持末端位置
- **异步 IK 计算** — 后台线程求解，不阻塞主线程
- **热参数更新** — 运行时动态调整所有 IK 参数，无需重启

**辅助套件**
- **RMCollision** — 关节碰撞检测，自动分段 + 碰撞体生成 + 静态干涉校准
- **RMRecorder** — 关节运动录制与回放
- **RMPersistence** — 异步批量数据持久化

**其他**
- 多坐标系增量控制 (World / Base / TCP)
- Scene 视图 Gizmo 可视化 (坐标轴、连杆、限位热力图、多解幽灵臂)
- 键盘 FK/IK 交互控制
- 引擎无关数学库，可移植到其他引擎
- 许可证验证系统 (硬件绑定 + 在线激活)

---

## 快速开始

### 1. 许可证激活

插件首次使用需要激活许可证：

1. 在 Unity 菜单栏点击 **RobotMatrix > License > Activate**
2. 在弹出窗口中输入你的邮箱地址，点击 **Request Access**
3. 系统自动采集设备信息并提交申请
4. 管理员审批后，你会收到一封包含 License Key 的邮件
5. 回到激活窗口，点击 "I already have a license key"，粘贴 Key 并点击 **Activate**

激活成功后本机离线即可使用，无需反复联网。

### 2. 安装插件

将整个仓库内容复制到 Unity 项目的 `Assets/` 目录下，确保以下结构完整：

```
Assets/
├── RobotMatrix/          # 核心插件
├── IEngine/              # 引擎抽象层
├── RMCollision/          # 碰撞检测套件
├── RMPersistence/        # 数据持久化
├── RMRecorder/           # 运动录制
├── Plugins/RMLicense/    # 许可证 DLL
├── FBX/                  # 示例机器人模型
└── Scenes/               # 示例场景
```

### 3. 挂载组件

1. 在场景中构建机械臂关节层级（父子结构或独立节点均可）
2. 在根物体上添加 **RobotArmBehaviour** 组件（菜单：Add Component > RobotMatrix）
3. 将关节 Transform 按运动学链顺序拖入 `Joint Transforms` 数组
4. 将末端执行器 Transform 拖入 `End Effector Transform`（可选）

### 4. Inspector 配置

#### 关节配置 (Joint Configs)

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `Name` | 关节名称 | "Joint" |
| `Type` | Revolute (旋转) / Prismatic (移动) | Revolute |
| `Axis Direction` | 局部旋转轴方向 | Z (Forward) |
| `Has Limits` | 是否启用关节限位 | true |
| `Min Limit (deg)` | 最小角度限制 | -90 |
| `Max Limit (deg)` | 最大角度限制 | 90 |
| `Initial Angle (deg)` | 初始角度 | 0 |

#### IK 求解器配置 (IK Config)

**基础参数**

| 参数 | 说明 | 默认值 | 范围 |
|------|------|--------|------|
| `Max Iterations` | 最大迭代次数 | 150 | 10~300 |
| `Position Threshold` | 位置收敛阈值 (m) | 0.001 | — |
| `Rotation Threshold` | 旋转收敛阈值 (rad) | 0.001 | — |
| `Damping Factor` | 阻尼因子 λ | 0.5 | 0.01~5 |
| `Position Weight` | 位置误差权重 | 1.0 | 0.01~5 |
| `Rotation Weight` | 姿态误差权重 | 0.3 | 0.01~5 |

**加权 DLS (Phase 1)**

| 参数 | 说明 | 默认值 | 范围 |
|------|------|--------|------|
| `Enable Weighted DLS` | 启用关节权重 | true | — |
| `Soft Limit Activation Zone` | 软限位激活区域比例 | 0.1 | 0.01~0.5 |
| `Soft Limit Max Penalty` | 接近限位时的最大惩罚倍数 | 50 | 1~200 |

**零空间 (Phase 2)**

| 参数 | 说明 | 默认值 | 范围 |
|------|------|--------|------|
| `Null Space Mid Range Gain` | 零空间中位回归增益 (0=禁用) | 0.5 | 0~2 |

**并行多解 (Phase 4)**

| 参数 | 说明 | 默认值 | 范围 |
|------|------|--------|------|
| `Multi Solution Trials` | 并行搜索试验数 | 8 | 1~32 |
| `Good Threshold` | 安全阈值 (limitFactor) | 0.15 | 0.05~0.4 |
| `Easing Priority Coefficient` | 缓动优先系数 | 0.5 | 0~1 |
| `Tier Decay Coefficient` | 梯队衰减系数 | 0.8 | 0.1~1 |

#### 运动配置

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `Motion Interpolation Mode` | MoveJ / MoveL | MoveJ |
| `Max Joint Interpolation Speed` | MoveJ 最大关节速度 (deg/s) | 45 |
| `TCP Max Linear Speed` | MoveL TCP 线速度 (m/s) | 0.5 |
| `TCP Max Angular Speed` | MoveL TCP 角速度 (deg/s) | 90 |
| `IK Failure Strategy` | MoveL 失败策略 | FallbackToPTP |
| `IK Compute Mode` | Synchronous / Asynchronous | Synchronous |
| `IK Delta Frame` | World / Base / TCP | World |

> **TCP 挂载要求**：末端执行器 (TCP) 必须挂载在最后一个关节的子物体上，否则 IK 计算会出现偏差。

---

## 架构概览

```
┌──────────────────────────────────────────────────────────────────┐
│  Runtime — RobotArmBehaviour (MonoBehaviour 入口)                 │
├──────────┬───────────────────┬───────────────────────────────────┤
│Interaction│    Controller     │           Data                    │
│ 命令分发   │ RobotArmController│    序列化配置                      │
│ 输入管理   │ FK/IK 调度        │    Inspector 面板                  │
├──────────┴───────────────────┴───────────────────────────────────┤
│  Instructions — MoveJ (关节插值) / MoveL (笛卡尔直线插值)           │
├──────────────────────────────────────────────────────────────────┤
│  Kinematics — FK / IK (四阶段并行求解) / 雅可比 / 关节权重          │
├──────────────────────────────────────────────────────────────────┤
│  IEngine — 引擎抽象层 (Unity 适配)                                 │
├──────────────────────────────────────────────────────────────────┤
│  Math — 纯 C# 数学库 (Vector3/Quaternion/Matrix4x4/MatrixMN)     │
├──────────────────────────────────────────────────────────────────┤
│  辅助套件: RMCollision | RMRecorder | RMPersistence               │
└──────────────────────────────────────────────────────────────────┘
```

---

## 核心算法：四阶段 IK 求解

### Phase 1 — 加权阻尼最小二乘 (Weighted DLS)

```
Δθ = Jw^T · (Jw · Jw^T + λ²I)^(-1) · e
```

- `Jw = J · diag(1/w)` — 加权雅可比矩阵
- 关节靠近限位时，权重 `w` 按二次梯度增大，自动抑制该关节运动
- 权重公式：`w[j] = 1 + maxPenalty × (1 - d_near/buffer)²`

### Phase 2 — 零空间中位回归

利用冗余自由度（仅位置约束时有效），将关节推向行程中位：

```
Δθ_null = (I - J_p^+ · J_p) · q0
```

### Phase 3 — 锁定关节补偿

指定锁定关节后，DLS 自动跳过该关节（权重=0），其余关节补偿以维持 TCP 位置。
应用场景：用户拖动某个关节的 Gizmo，其他关节自动调整。

### Phase 4 — 并行多解搜索

同时运行 N 个线程（`MultiSolutionTrials`），从不同初始解搜索最优：

- **Trial 0**：从当前关节角度出发（保持连续性）
- **Trial 1~N-1**：随机初始角度（探索多解空间）

**四级梯队评分选优**：

| 梯队 | 条件 | 权重系数 |
|------|------|----------|
| Tier 1 | 所有关节 limitFactor ≥ goodThreshold | 1.0 |
| Tier 2 | <50% 关节接近限位 | d |
| Tier 3 | ≥50% 关节接近限位 | d² |
| Tier 4 | 任一关节触碰硬限位 | d³ |

综合评分：`score = a × easing + (1-a) × tierScore`

- `easing = 1/(1 + rotationCost)` — 转动越少分越高（缓动系数）
- `tierScore = d^(tier-1)` — 梯队衰减（`d = TierDecayCoefficient`）
- `a = EasingPriorityCoefficient` — 缓动优先级权重

---

## 运动指令

### MoveJ — 关节空间插值

```csharp
controller.MotionMode = MotionInterpolationMode.MoveJ;
controller.SetMaxInterpolationSpeed(45f * Mathf.Deg2Rad); // 45 deg/s
controller.MoveEndEffectorTo(targetPos, targetRot);
```

- 每个关节独立匀速插值
- 所有关节同步到达目标（最慢关节决定总时间）
- TCP 走曲线，速度快
- 适用场景：大范围移动、无路径精度要求

### MoveL — 笛卡尔直线插值

```csharp
controller.MotionMode = MotionInterpolationMode.MoveL;
controller.SetCartesianSpeed(0.5f, 90f * Mathf.Deg2Rad); // 0.5 m/s, 90 deg/s
controller.MoveEndEffectorTo(targetPos, targetRot);
```

- TCP 沿直线匀速运动（位置 Lerp + 姿态 Slerp）
- 路径上每个路点进行并行多解 IK 求解
- 运动时间 = max(线距离/线速度, 角距离/角速度)
- IK 失败时自动降级到 MoveJ（`FallbackToPTP` 策略）

---

## 键盘控制

| 模式 | 按键 | 功能 |
|------|------|------|
| 通用 | `Tab` | 切换 FK / IK 模式 |
| FK | `Q` / `E` | 选择上一个/下一个关节 |
| FK | `A` / `D` | 负向/正向旋转选中关节 |
| IK | `W` `S` `A` `D` | Z轴/X轴 平移 |
| IK | `Space` / `Ctrl` | Y轴 上/下移动 |
| IK | `I` `K` `J` `L` `U` `O` | 绕 X/Y/Z 轴旋转 |

---

## API 参考

### RobotArmController — 核心控制器

```csharp
// FK 操作
void SetJointAngle(int jointIndex, float angleRad);
void SetJointAngles(float[] anglesRad);
float[] GetJointAngles();
RMMatrix4x4 GetEndEffectorPose();           // DH 基坐标系
RMMatrix4x4 GetEndEffectorWorldPose();      // Unity 世界坐标系

// IK 操作
IKResult MoveEndEffectorTo(RMVector3 targetPos, RMQuaternion targetRot);
IKResult MoveEndEffectorDelta(RMVector3 posDelta, RMVector3 rotDelta);
IKResult AdjustJointKeepTCP(int jointIndex, float newAngleRad);

// 运动配置
void SetMaxInterpolationSpeed(float maxAngularVelocityRad);
void SetCartesianSpeed(float linearMPerSec, float angularRadPerSec);
void SyncIKConfig(IKSolverConfig source);   // 热更新 IK 参数

// 属性
ControlMode Mode { get; set; }              // FK / IK
MotionInterpolationMode MotionMode;         // MoveJ / MoveL
IKDeltaFrame DeltaFrame;                    // World / Base / TCP
IKComputeMode ComputeMode;                  // Synchronous / Asynchronous
IKFailureStrategy FailureStrategy;          // StopAndReport / FallbackToPTP

// 事件
event Action<int, bool> OnJointLimitChanged;       // 关节限位状态变化
event Action<RMVector3, RMQuaternion> OnCartesianIKFailure; // MoveL 失败
```

### RobotArmBehaviour — MonoBehaviour 入口

```csharp
// 核心访问
RobotArmController Controller { get; }
CollisionSuiteController CollisionSuite { get; }
RecorderSuiteController RecorderSuite { get; }
bool IsInitialized { get; }
```

---

## 代码示例

### IK 绝对移动

```csharp
var arm = GetComponent<RobotArmBehaviour>();
var targetPos = new RMVector3(0.5f, 0.2f, 0.3f);
var targetRot = RMQuaternion.Identity;

IKResult result = arm.Controller.MoveEndEffectorTo(targetPos, targetRot);
if (result != null && result.Converged)
    Debug.Log($"IK 成功, 迭代: {result.IterationsUsed}");
```

### IK 增量移动 (Jog)

```csharp
arm.Controller.DeltaFrame = IKDeltaFrame.TCP;  // TCP 坐标系
arm.Controller.MoveEndEffectorDelta(
    new RMVector3(0.01f, 0f, 0f),  // 末端方向前进 1cm
    RMVector3.Zero);
```

### 拖动关节保持 TCP

```csharp
arm.Controller.AdjustJointKeepTCP(0, 90f * Mathf.Deg2Rad);
```

### 运行时热更新 IK 参数

```csharp
var config = new IKSolverConfig {
    MaxIterations = 100,
    DampingFactor = 1f,
    MultiSolutionTrials = 12,
    EasingPriorityCoefficient = 0.3f
};
arm.Controller.SyncIKConfig(config);
```

### 碰撞检测

```csharp
// Inspector 中配置 CollisionConfig 后自动初始化
// 查询碰撞评分：
var scores = arm.CollisionSuite.Detector.AccumulatedScores;
```

### 运动录制

```csharp
arm.RecorderSuite.StartRecording();
// ... 运动 ...
arm.RecorderSuite.StopRecording();
```

---

## 辅助套件

### RMCollision — 碰撞检测

- 自动分析关节层级，生成碰撞分段
- 为每段自动创建碰撞体 (Kinematic Rigidbody + Trigger)
- 初始化时进行静态干涉校准（排除初始接触）
- 支持外部碰撞体（工具、工件）
- 持续监控碰撞对，累积接触评分

### RMRecorder — 运动录制

- 按帧采样关节角度数据
- 平滑滤波处理
- 支持导出持久化文件

### RMPersistence — 数据持久化

- 异步批量写入，不阻塞主线程
- 自动文件路径管理

---

## 参数调优指南

| 目标 | 调整参数 | 方向 |
|------|----------|------|
| 提高精度 | `MaxIterations` | 增大 (200+) |
| 加快收敛 | `PositionThreshold` / `RotationThreshold` | 适当增大 |
| 优先位置 | `RotationWeight` | 减小 (如 0.1) |
| 减少跳变 | `EasingPriorityCoefficient` | 增大 (趋向 1.0) |
| 更优解 | `MultiSolutionTrials` | 增大 (如 16~32) |
| 避免限位 | `SoftLimitActivationZone` | 增大 (如 0.2) |
| 强力回中 | `NullSpaceMidRangeGain` | 增大 (如 1.0~2.0) |
| MoveL 平顺 | `TCP Max Linear Speed` | 减小 |

---

## 项目结构

```
RobotMatrix/
├── Controller/       # RobotArmController — 核心调度
├── Data/             # 序列化配置数据结构
├── Editor/           # Unity 自定义 Inspector
├── Instructions/     # MoveJ / MoveL 插值引擎
├── Interaction/      # 命令模式交互 (键盘/鼠标)
├── Kinematics/       # FK / IK / 雅可比 / 关节权重
├── License/          # 许可证管理 (Editor)
├── Math/             # 纯 C# 数学库 (零依赖)
├── Runtime/          # RobotArmBehaviour 入口
└── Tests/            # 单元测试

IEngine/              # 引擎抽象层 (可移植)
RMCollision/          # 碰撞检测套件
RMRecorder/           # 运动录制套件
RMPersistence/        # 数据持久化
Plugins/RMLicense/    # 许可证验证 DLL
```

---

## 技术规格

| 特性 | 参数 |
|------|------|
| 关节数量 | 无限制 (N-DOF) |
| 关节类型 | Revolute / Prismatic |
| IK 算法 | 四阶段并行 DLS |
| IK 迭代 | 默认 150 次 |
| 位置精度 | 0.001 m (1mm) |
| 旋转精度 | 0.001 rad (~0.057 deg) |
| 并行试验 | 默认 8 路 |
| 运动指令 | MoveJ / MoveL |
| 坐标系 | World / Base / TCP |
| 计算模式 | 同步 / 异步 |
| 依赖 | 零第三方依赖 |
| 平台 | Windows / macOS |

---

> **RobotMatrix** — 工业级机械臂运动学引擎，让 Unity 中的机器人仿真与控制变得专业可靠
