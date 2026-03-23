# RobotMatrix — Unity 工业机械臂运动学引擎

[![Unity](https://img.shields.io/badge/Unity-2021.3+-black.svg)](https://unity.com)
[![C#](https://img.shields.io/badge/C%23-9.0-blue.svg)](https://docs.microsoft.com/dotnet/csharp/)
[![.NET Standard](https://img.shields.io/badge/.NET_Standard-2.1-purple.svg)]()
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20macOS-lightgrey.svg)]()

> 专业级 N-DOF 机械臂逆运动学求解引擎 — 纯 C# 实现，零第三方依赖，开箱即用

---

## 为什么选择 RobotMatrix

- **亚毫米精度** — 位置精度 0.001m，旋转精度 0.001rad，满足工业仿真需求
- **智能多解优选** — 并行搜索多组 IK 解，自动选择最优解，避免关节跳变和限位碰撞
- **工业运动指令** — 完整支持 MoveJ（关节插值）和 MoveL（笛卡尔直线），对标工业机器人编程规范
- **即插即用** — 拖入 Transform、配置参数、点击 Play，三步完成
- **零依赖** — 纯 C# 数学库，不依赖任何第三方插件
- **N-DOF 通用** — 不限关节数量，旋转关节 / 移动关节均支持

---

## 核心功能

### 运动学求解

| 功能 | 说明 |
|------|------|
| 正运动学 (FK) | DH 参数自动计算，实时更新各关节和末端位姿 |
| 逆运动学 (IK) | 多阶段智能求解，自动处理限位、冗余自由度、多解选择 |
| 并行多解搜索 | 多线程同时搜索多组解，智能评分选优 |
| 软限位保护 | 关节接近限位时自动柔性抑制，无需手动干预 |
| 零空间优化 | 冗余自由度自动利用，关节趋向安全中位 |
| 异步计算 | 后台线程求解 IK，不阻塞主线程渲染 |

### 运动指令

| 指令 | 说明 |
|------|------|
| **MoveJ** | 关节空间插值，各关节匀速同步到达，速度快 |
| **MoveL** | 笛卡尔直线插值，TCP 沿直线精确运动，路径可控 |
| 增量 Jog | 支持 World / Base / TCP 三种坐标系的增量控制 |
| 拖动保持 TCP | 拖动指定关节，其他关节自动补偿维持末端位置 |
| 失败降级 | MoveL 路径不可达时自动切换 MoveJ 完成运动 |

### 辅助套件

| 套件 | 功能 |
|------|------|
| **RMCollision** | 关节碰撞检测——自动分段、碰撞体生成、静态干涉校准、实时碰撞评分 |
| **RMRecorder** | 运动录制——按帧采样关节数据，支持回放与导出 |
| **RMPersistence** | 数据持久化——异步批量写入，不阻塞主线程 |

### 可视化 & 交互

- Scene 视图 Gizmo：关节坐标轴、连杆、末端标记、限位热力图、多解幽灵臂
- 键盘控制：FK 单关节旋转 / IK 末端平移旋转，Tab 切换模式
- 自定义 Inspector：所有参数可视化配置，实时热更新

---

## 快速开始

### 1. 许可证激活

首次使用需要激活（激活后离线可用）：

1. Unity 菜单 **RobotMatrix > License > Activate**
2. 输入邮箱，点击 **Request Access**（设备信息自动采集）
3. 管理员审批后你会收到包含 License Key 的邮件
4. 回到窗口，点击 "I already have a license key"，粘贴 Key 点击 **Activate**

### 2. 安装

将整个仓库内容复制到 Unity 项目的 `Assets/` 目录下。

### 3. 挂载组件

1. 构建机械臂关节层级（父子结构或独立节点）
2. 根物体添加 **RobotArmBehaviour** 组件
3. 将关节 Transform 按运动学链顺序拖入 `Joint Transforms`
4. （可选）将末端执行器 Transform 拖入 `End Effector Transform`

> **注意**：末端执行器 (TCP) 必须挂载在最后一个关节的子物体上。

### 4. Inspector 配置

**关节配置** — 为每个关节设置：
- 关节类型（旋转 / 移动）
- 运动轴方向
- 角度限位范围
- 初始角度

**IK 配置** — 可调参数包括：
- 收敛精度与最大迭代次数
- 阻尼因子与位置/姿态权重比
- 软限位灵敏度
- 并行搜索试验数与评分偏好
- 零空间回归强度

**运动配置** — 选择运动模式和速度：
- MoveJ / MoveL 模式切换
- 关节速度 / TCP 线速度 / TCP 角速度
- IK 失败策略（停止 / 自动降级）
- 同步 / 异步计算模式
- 增量坐标系（World / Base / TCP）

**辅助套件** — 在 Inspector 中直接配置碰撞检测、运动录制、数据持久化。

点击 Play 即可开始控制。

---

## 键盘控制

| 模式 | 按键 | 功能 |
|------|------|------|
| 通用 | `Tab` | 切换 FK / IK |
| FK | `Q` / `E` | 选择关节 |
| FK | `A` / `D` | 旋转关节 |
| IK | `W` `S` `A` `D` | XZ 平面移动 |
| IK | `Space` / `Ctrl` | Y 轴升降 |
| IK | `I` `K` `J` `L` `U` `O` | 绕 XYZ 旋转 |

---

## API 快速参考

```csharp
var arm = GetComponent<RobotArmBehaviour>();
var ctrl = arm.Controller;

// FK 控制
ctrl.SetJointAngle(0, 45f * Mathf.Deg2Rad);
ctrl.SetJointAngles(anglesRad);
float[] angles = ctrl.GetJointAngles();

// IK 移动
ctrl.MoveEndEffectorTo(targetPos, targetRot);          // 绝对位姿
ctrl.MoveEndEffectorDelta(posDelta, rotDelta);         // 增量 Jog
ctrl.AdjustJointKeepTCP(jointIndex, newAngleRad);      // 拖动保持 TCP

// 运动配置
ctrl.MotionMode = MotionInterpolationMode.MoveL;       // 切换 MoveL
ctrl.SetCartesianSpeed(0.5f, 90f * Mathf.Deg2Rad);    // TCP 速度
ctrl.SetMaxInterpolationSpeed(45f * Mathf.Deg2Rad);   // MoveJ 速度
ctrl.SyncIKConfig(newConfig);                          // 运行时热更新参数

// 事件
ctrl.OnJointLimitChanged += (idx, atLimit) => { };     // 限位状态变化
ctrl.OnCartesianIKFailure += (pos, rot) => { };        // MoveL 路径不可达

// 辅助套件
arm.RecorderSuite.StartRecording();                    // 开始录制
arm.RecorderSuite.StopRecording();                     // 停止录制
```

---

## 项目结构

```
RobotMatrix/          核心插件
├── Controller/       运动学控制调度
├── Kinematics/       FK / IK 求解引擎
├── Instructions/     MoveJ / MoveL 插值
├── Interaction/      命令模式交互
├── Math/             纯 C# 数学库
├── Data/             序列化配置
├── Editor/           自定义 Inspector
├── Runtime/          MonoBehaviour 入口
└── Tests/            单元测试

IEngine/              引擎抽象层（可移植）
RMCollision/          碰撞检测套件
RMRecorder/           运动录制套件
RMPersistence/        数据持久化
Plugins/RMLicense/    许可证验证
```

---

## 技术规格

| 项目 | 参数 |
|------|------|
| 关节数量 | N-DOF，无限制 |
| 关节类型 | Revolute / Prismatic |
| 位置精度 | 0.001 m |
| 旋转精度 | 0.001 rad |
| 运动指令 | MoveJ / MoveL |
| 坐标系 | World / Base / TCP |
| 计算模式 | 同步 / 异步 |
| 平台 | Windows / macOS |
| 依赖 | 零第三方依赖 |

---

## 联系与支持

如需获取许可证、商业授权或技术支持，请联系开发者。

---

> **RobotMatrix** — 工业级 Unity 机械臂引擎，让机器人仿真变得专业、可靠、简单
