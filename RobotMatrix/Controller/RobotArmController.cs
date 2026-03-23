using System;
using System.Threading;
using System.Threading.Tasks;
using RobotMatrix.Math;
using IEngine;
using RobotMatrix.Kinematics;
using RobotMatrix.Instructions;

namespace RobotMatrix.Controller
{
    /// <summary>
    /// 控制模式枚举。
    /// </summary>
    public enum ControlMode { FK, IK }

    /// <summary>
    /// IK 计算模式枚举。
    /// </summary>
    public enum IKComputeMode { Synchronous, Asynchronous }

    /// <summary>
    /// IK 增量坐标系枚举。
    /// </summary>
    public enum IKDeltaFrame
    {
        /// <summary>增量基于 Unity 世界坐标系。</summary>
        World,
        /// <summary>增量基于机器人基座坐标系（DH 基帧在 Unity 空间中的朝向）。</summary>
        Base,
        /// <summary>增量基于末端执行器（TCP）坐标系。</summary>
        TCP
    }

    /// <summary>
    /// 机械臂控制器：连接引擎层和运动学核心层的桥梁。
    /// 负责模型初始化、FK/IK 调度、场景同步。
    /// </summary>
    public class RobotArmController
    {
        // ===== 内部持有 =====
        private RobotArmModel _model;
        private FKSolver _fkSolver;
        private IKSolver _ikSolver;
        private JacobianCalculator _jacobianCalc;

        private IEngineObject[] _jointObjects;
        private IEngineObject _endEffectorObject;

        private ControlMode _currentMode = ControlMode.FK;

        // ===== 零位基准帧数据 =====
        private RMQuaternion[] _jointInitLocalRotations;
        private RMVector3[] _jointInitLocalPositions;

        // ===== 层级分析与世界坐标统一数据 =====
        private JointHierarchyInfo[] _hierarchyInfos;
        private RMMatrix4x4 _dhToUnityCalibration;
        private RMMatrix4x4 _unityToDHCalibration;
        private RMMatrix4x4[] _fkZeroWorldTransforms;

        // ===== 零位基准帧：关节 0 的初始世界位姿（校准帧计算用）=====
        private RMVector3 _joint0InitWorldPos;
        private RMQuaternion _joint0InitWorldRot;

        // ===== 运行时状态 =====
        private float[] _currentJointAngles;
        private bool _isDirty;

        // ===== IK 增量 Jog 绝对目标 =====
        // 维护绝对目标位姿，避免每帧从 FK 重算导致漂移累积。
        // 位置增量只修改 _ikTargetPos，旋转增量只修改 _ikTargetRot，
        // 从而在旋转时位置不漂移，在平移时姿态不漂移。
        private RMVector3 _ikTargetPos;
        private RMQuaternion _ikTargetRot;
        private bool _ikTargetValid;

        // ===== FK 缓存 =====
        private RMMatrix4x4 _cachedEEPose;
        private bool _fkCacheValid;

        // ===== 异步 IK =====
        private IKComputeMode _ikComputeMode = IKComputeMode.Synchronous;
        private IKDeltaFrame _ikDeltaFrame = IKDeltaFrame.World;
        private IKSolver _asyncIKSolver;
        private Task<IKResult> _ikTask;
        private CancellationTokenSource _ikCancellation;
        private RMVector3 _pendingIKTargetPos;
        private RMQuaternion _pendingIKTargetRot;
        private bool _asyncTargetDirty;  // 任务运行期间是否有新目标到达

        // ===== 关节限位状态 =====
        private bool[] _jointAtLimit;
        private bool[] _prevJointAtLimit;

        // ===== 关节插值器（MoveJ：IK 结果防跳变）=====
        private JointInterpolator _interpolator;
        private float[] _interpolatedAnglesBuffer;

        // ===== 笛卡尔插值器（MoveL：TCP 直线运动）=====
        private CartesianInterpolator _cartesianInterpolator;
        private float[] _cartesianFallbackAngles; // MoveL 回退到 MoveJ 时的目标关节角
        private MotionInterpolationMode _motionMode = MotionInterpolationMode.MoveJ;
        private IKFailureStrategy _ikFailureStrategy = IKFailureStrategy.FallbackToPTP;

        // ===== 多解诊断数据（Phase 4 / 5）=====
        private float[] _lastJointWeights;        // 最近一次 IK 的关节逆权重
        private float[] _lastJointLimitFactors;   // 最近一次 IK 的 limitFactor
        private IKResult[] _lastMultiResults;     // 最近一次多解搜索的所有候选

        /// <summary>
        /// 关节限位状态变更事件。参数：(jointIndex, isAtLimit)。
        /// 仅在状态发生变化时触发（从非限位到限位，或反之），避免每帧重复。
        /// </summary>
        public event System.Action<int, bool> OnJointLimitChanged;

        /// <summary>
        /// MoveL 路径上 IK 求解失败事件。参数：(失败路点位置, 失败路点姿态)。
        /// 仅在 IKFailureStrategy.StopAndReport 策略下触发。
        /// </summary>
        public event Action<RMVector3, RMQuaternion> OnCartesianIKFailure;

        // ===== 公共属性 =====
        public RobotArmModel Model => _model;
        public int DOF => _model != null ? _model.DOF : 0;
        public bool IsInitialized => _model != null;

        /// <summary>关节物体数组（只读引用，供外部套件访问）。</summary>
        public IEngineObject[] JointObjects => _jointObjects;

        /// <summary>运动插值模式（MoveJ / MoveL）。</summary>
        public MotionInterpolationMode MotionMode
        {
            get => _motionMode;
            set => _motionMode = value;
        }

        /// <summary>MoveL 路径上 IK 失败策略。</summary>
        public IKFailureStrategy FailureStrategy
        {
            get => _ikFailureStrategy;
            set => _ikFailureStrategy = value;
        }

        /// <summary>是否正在笛卡尔直线插值中（MoveL 进行中）。</summary>
        public bool IsCartesianInterpolating =>
            _cartesianInterpolator != null && _cartesianInterpolator.HasTarget;

        /// <summary>笛卡尔插值进度（0~1）。</summary>
        public float CartesianProgress =>
            _cartesianInterpolator != null ? _cartesianInterpolator.Progress : 0f;

        /// <summary>最近一次 IK 求解结果（供状态面板显示）。</summary>
        public IKResult LastIKResult { get; private set; }

        /// <summary>最近一次多解搜索的所有候选结果（供可视化）。</summary>
        public IKResult[] LastMultiResults => _lastMultiResults;

        /// <summary>最近一次 IK 求解后各关节的逆权重快照。</summary>
        public float[] LastJointWeights => _lastJointWeights;

        /// <summary>最近一次 IK 求解后各关节的 limitFactor 快照。</summary>
        public float[] LastJointLimitFactors => _lastJointLimitFactors;

        /// <summary>IK 计算模式（同步/异步），默认同步。</summary>
        public IKComputeMode ComputeMode
        {
            get => _ikComputeMode;
            set => _ikComputeMode = value;
        }

        /// <summary>IK 增量坐标系（World/TCP），默认 World。</summary>
        public IKDeltaFrame DeltaFrame
        {
            get => _ikDeltaFrame;
            set => _ikDeltaFrame = value;
        }

        /// <summary>查询指定关节是否处于限位状态。</summary>
        public bool IsJointAtLimit(int index)
        {
            if (_jointAtLimit == null || index < 0 || index >= _jointAtLimit.Length)
                return false;
            return _jointAtLimit[index];
        }

        /// <summary>获取所有关节的限位状态（只读副本）。</summary>
        public bool[] GetJointLimitStatus()
        {
            if (_jointAtLimit == null) return new bool[0];
            var copy = new bool[_jointAtLimit.Length];
            System.Array.Copy(_jointAtLimit, copy, copy.Length);
            return copy;
        }

        // ===== 初始化 =====

        /// <summary>
        /// 从已构建的模型和引擎对象初始化控制器。
        /// </summary>
        public void Initialize(RobotArmModel model,
                               IEngineObject[] jointObjects,
                               IEngineObject endEffectorObject = null,
                               IKSolverConfig ikConfig = null)
        {
            _model = model;
            _jointObjects = jointObjects;
            _endEffectorObject = endEffectorObject;

            _fkSolver = new FKSolver();
            _ikSolver = new IKSolver(ikConfig ?? new IKSolverConfig());
            _jacobianCalc = new JacobianCalculator();

            int n = model.DOF;
            _currentJointAngles = new float[n];

            // 初始化关节限位状态跟踪
            _jointAtLimit = new bool[n];
            _prevJointAtLimit = new bool[n];

            // 初始化关节插值器（MoveJ）
            _interpolator = new JointInterpolator();
            _interpolator.Initialize(n);
            _interpolatedAnglesBuffer = new float[n];

            // 初始化笛卡尔插值器（MoveL）
            _cartesianInterpolator = new CartesianInterpolator();
            _cartesianFallbackAngles = new float[n];

            // 1. 层级分析
            _hierarchyInfos = JointHierarchyAnalyzer.Analyze(jointObjects);

            // 2. 记录零位基准帧
            _jointInitLocalRotations = new RMQuaternion[n];
            _jointInitLocalPositions = new RMVector3[n];
            for (int i = 0; i < n; i++)
            {
                _jointInitLocalRotations[i] = jointObjects[i].LocalRotation;
                _jointInitLocalPositions[i] = jointObjects[i].LocalPosition;
            }

            // 3. 零位 FK，记录各关节 DH 世界变换
            _fkZeroWorldTransforms = new RMMatrix4x4[n];
            var zeroAngles = new float[n];
            _fkSolver.Solve(model, zeroAngles);
            for (int i = 0; i < n; i++)
                _fkZeroWorldTransforms[i] = model.JointStates[i].WorldTransform;

            // 4. 计算 DH→Unity 校准变换
            // DH 第一帧 Z 轴 = (0,0,1)，必须映射到实际旋转轴方向
            // 构建对齐帧：Z = 关节 0 的世界旋转轴，X = 垂直于 Z 的参考轴
            _joint0InitWorldPos = jointObjects[0].WorldPosition;
            _joint0InitWorldRot = jointObjects[0].WorldRotation;
            var alignedFrame = BuildAlignedFrame(model.JointConfigs[0],
                                                 _joint0InitWorldPos, _joint0InitWorldRot);
            var fkZeroT0Inv = RMMatrix4x4.Inverse(_fkZeroWorldTransforms[0]);
            _dhToUnityCalibration = alignedFrame * fkZeroT0Inv;
            _unityToDHCalibration = RMMatrix4x4.Inverse(_dhToUnityCalibration);

            // 5. 计算末端执行器偏移（从最后关节到实际 TCP 的固定变换）
            // EndEffectorOffset = Inv(T_fk_last) * Inv(T_calib) * T_tcp_unity
            // 使得 FK 结果经校准后恰好落在 Unity TCP 位置上
            if (endEffectorObject != null && endEffectorObject.IsValid)
            {
                var tcpUnityMatrix = EngineObjectToMatrix(endEffectorObject);
                var tcpDHMatrix = _unityToDHCalibration * tcpUnityMatrix;
                var lastJointFKInv = RMMatrix4x4.Inverse(_fkZeroWorldTransforms[n - 1]);
                model.EndEffectorOffset = lastJointFKInv * tcpDHMatrix;
            }

            _isDirty = false;
        }

        /// <summary>
        /// 从关节空间信息和配置构建模型并初始化。
        /// </summary>
        public void InitializeFromSpatialConfig(JointSpatialInfo[] spatialInfos,
                                                 JointConfig[] configs,
                                                 IEngineObject[] jointObjects,
                                                 IEngineObject endEffectorObject = null,
                                                 IKSolverConfig ikConfig = null)
        {
            #if UNITY_EDITOR
            if (!RMLicense.LicenseGuard.IsValid) return;
            #endif

            var model = new RobotArmModel();
            model.BuildDHFromSpatialConfig(spatialInfos, configs);
            Initialize(model, jointObjects, endEffectorObject, ikConfig);
        }

        // ===== FK 操作 API =====

        /// <summary>
        /// 设置指定关节的角度值。
        /// </summary>
        public void SetJointAngle(int jointIndex, float angle)
        {
            if (_model == null || jointIndex < 0 || jointIndex >= _model.DOF)
                return;

            var cfg = _model.JointConfigs[jointIndex];
            if (cfg.HasLimits)
                angle = RMMathUtils.Clamp(angle, cfg.MinLimit, cfg.MaxLimit);

            _currentJointAngles[jointIndex] = angle;
            _ikTargetValid = false;
            ApplyFK();
        }

        /// <summary>
        /// 设置所有关节角度。
        /// </summary>
        public void SetJointAngles(float[] angles)
        {
            if (_model == null) return;
            int n = System.Math.Min(angles.Length, _model.DOF);
            for (int i = 0; i < n; i++)
            {
                var cfg = _model.JointConfigs[i];
                _currentJointAngles[i] = cfg.HasLimits
                    ? RMMathUtils.Clamp(angles[i], cfg.MinLimit, cfg.MaxLimit)
                    : angles[i];
            }
            _ikTargetValid = false;
            ApplyFK();
        }

        // ===== IK 操作 API =====

        /// <summary>
        /// 将末端执行器移动到指定位姿（DH 基坐标系）。
        /// 统一使用 SolveMulti，MultiSolutionTrials 控制并行度。
        /// 异步模式下提交后台任务并返回 null，结果通过 PollAsyncIKResult 获取。
        /// </summary>
        public IKResult MoveEndEffectorTo(RMVector3 targetPos, RMQuaternion targetRot)
        {
            if (_model == null) return null;

            if (_ikComputeMode == IKComputeMode.Asynchronous)
            {
                SubmitAsyncIK(targetPos, targetRot);
                return null;
            }

            int trials = _ikSolver.Config.MultiSolutionTrials;
            var result = _ikSolver.SolveMulti(_model, _currentJointAngles,
                targetPos, targetRot, trials, out var allResults);
            HandleIKResult(result, allResults, targetPos, targetRot);
            return result;
        }

        /// <summary>
        /// 增量移动末端执行器（工业机器人 Jog 模式）。
        /// posDelta / rotDelta 的参考坐标系由 DeltaFrame 属性决定：
        ///   - World: Unity 世界坐标系（默认）
        ///   - TCP:   末端执行器（TCP）局部坐标系
        /// 
        /// 使用绝对目标跟踪：位置增量只修改位置目标，旋转增量只修改旋转目标。
        /// 统一使用 SolveMulti，MultiSolutionTrials 控制并行度。
        /// 异步模式下提交后台任务并返回 null，结果通过 PollAsyncIKResult 获取。
        /// </summary>
        public IKResult MoveEndEffectorDelta(RMVector3 posDelta, RMVector3 rotDelta)
        {
            if (_model == null) return null;

            // 首次调用或目标失效时，从当前 FK 初始化绝对目标
            if (!_ikTargetValid)
            {
                var currentEE = _fkCacheValid ? _cachedEEPose
                    : _fkSolver.Solve(_model, _currentJointAngles);
                _ikTargetPos = currentEE.GetPosition();
                _ikTargetRot = currentEE.GetRotation();
                _ikTargetValid = true;
            }

            bool hasPos = System.Math.Abs(posDelta.X) > 1e-10f ||
                          System.Math.Abs(posDelta.Y) > 1e-10f ||
                          System.Math.Abs(posDelta.Z) > 1e-10f;

            bool hasRot = System.Math.Abs(rotDelta.X) > 1e-10f ||
                          System.Math.Abs(rotDelta.Y) > 1e-10f ||
                          System.Math.Abs(rotDelta.Z) > 1e-10f;

            // TCP 坐标系模式：将增量从 TCP 局部帧旋转到 Unity 世界帧
            if (_ikDeltaFrame == IKDeltaFrame.TCP)
            {
                var eeWorldRot = GetEndEffectorWorldPose().GetRotation();
                if (hasPos)
                    posDelta = RMQuaternion.RotateVector(eeWorldRot, posDelta);
                if (hasRot)
                    rotDelta = RMQuaternion.RotateVector(eeWorldRot, rotDelta);
            }
            // Base 坐标系模式：将增量从机器人基座帧旋转到 Unity 世界帧
            else if (_ikDeltaFrame == IKDeltaFrame.Base)
            {
                var baseWorldRot = _dhToUnityCalibration.GetRotation();
                if (hasPos)
                    posDelta = RMQuaternion.RotateVector(baseWorldRot, posDelta);
                if (hasRot)
                    rotDelta = RMQuaternion.RotateVector(baseWorldRot, rotDelta);
            }

            // Unity 世界增量 → DH 基坐标系增量
            if (hasPos)
            {
                var dhPosDelta = _unityToDHCalibration.MultiplyDirection(posDelta);
                _ikTargetPos = _ikTargetPos + dhPosDelta;
            }

            if (hasRot)
            {
                var dhRotDelta = _unityToDHCalibration.MultiplyDirection(rotDelta);
                var deltaQuat = RMQuaternion.EulerToQuaternion(dhRotDelta);
                _ikTargetRot = (deltaQuat * _ikTargetRot).Normalized;
            }

            if (_ikComputeMode == IKComputeMode.Asynchronous)
            {
                SubmitAsyncIK(_ikTargetPos, _ikTargetRot);
                return null;
            }

            int trials = _ikSolver.Config.MultiSolutionTrials;
            var result = _ikSolver.SolveMulti(_model, _currentJointAngles,
                _ikTargetPos, _ikTargetRot, trials, out var allResults);
            HandleIKResult(result, allResults, _ikTargetPos, _ikTargetRot);
            return result;
        }

        /// <summary>
        /// 拖动指定关节到新角度，同时保持 TCP 位置不变（Phase 3: 零空间用户交互）。
        /// 将被拖动关节锁定到目标角度，其他关节通过 IK 补偿以维持末端位置。
        /// 姿态作为冗余自由度尽量少变化。
        /// </summary>
        /// <param name="jointIndex">要拖动的关节索引。</param>
        /// <param name="newAngleRad">目标角度（弧度）。</param>
        /// <returns>IK 求解结果（null 表示无效输入）。</returns>
        public IKResult AdjustJointKeepTCP(int jointIndex, float newAngleRad)
        {
            if (_model == null || jointIndex < 0 || jointIndex >= _model.DOF)
                return null;

            // 限位 clamp
            var cfg = _model.JointConfigs[jointIndex];
            if (cfg.HasLimits)
                newAngleRad = RMMathUtils.Clamp(newAngleRad, cfg.MinLimit, cfg.MaxLimit);

            // 记录当前 TCP 位置（DH 空间）
            var currentEE = _fkCacheValid ? _cachedEEPose
                : _fkSolver.Solve(_model, _currentJointAngles);
            var tcpPos = currentEE.GetPosition();

            // 构建锁定选项
            int n = _model.DOF;
            var lockedJoints = new bool[n];
            var lockedAngles = new float[n];
            lockedJoints[jointIndex] = true;
            lockedAngles[jointIndex] = newAngleRad;

            var options = new IKSolveOptions
            {
                LockedJoints = lockedJoints,
                LockedAngles = lockedAngles
            };

            // 使用 SolvePositionOnly 求解（保持位置，姿态作为冗余自由度）
            // 预设初始角度：将目标关节设为新角度
            var initAngles = new float[n];
            System.Array.Copy(_currentJointAngles, initAngles, n);
            initAngles[jointIndex] = newAngleRad;

            var result = _ikSolver.SolvePositionOnly(_model, initAngles, tcpPos, options);

            if (result != null)
            {
                LastIKResult = result;
                _lastMultiResults = new IKResult[] { result };
                UpdateDiagnosticData(result);
                _ikTargetValid = false; // 姿态可能变了，重置 IK 目标
                ApplyIKResult(result.JointAngles);
            }

            return result;
        }

        // ===== IK 结果处理 =====

        /// <summary>
        /// 统一处理 IK 求解结果：保存诊断数据 + 应用角度 + 更新 IK 目标。
        /// </summary>
        private void HandleIKResult(IKResult result, IKResult[] allResults,
                                     RMVector3 targetPos, RMQuaternion targetRot)
        {
            if (result == null) return;

            LastIKResult = result;
            _lastMultiResults = allResults;
            _ikTargetPos = targetPos;
            _ikTargetRot = targetRot;
            _ikTargetValid = true;
            UpdateDiagnosticData(result);
            ApplyIKResult(result.JointAngles);
        }

        /// <summary>
        /// 更新诊断数据（关节权重和 limitFactor）。
        /// </summary>
        private void UpdateDiagnosticData(IKResult result)
        {
            if (result?.JointAngles == null || _model == null) return;
            int n = _model.DOF;

            if (_lastJointWeights == null || _lastJointWeights.Length != n)
                _lastJointWeights = new float[n];
            if (_lastJointLimitFactors == null || _lastJointLimitFactors.Length != n)
                _lastJointLimitFactors = new float[n];

            JointWeightCalculator.ComputeInverseWeightsInto(
                _model, result.JointAngles,
                _ikSolver.Config.SoftLimitActivationZone,
                _ikSolver.Config.SoftLimitMaxPenalty,
                _lastJointWeights);

            JointWeightCalculator.ComputeLimitFactorsInto(
                _model, result.JointAngles,
                _lastJointLimitFactors);
        }

        // ===== 状态查询 =====

        public float[] GetJointAngles()
        {
            if (_currentJointAngles == null) return new float[0];
            var copy = new float[_currentJointAngles.Length];
            System.Array.Copy(_currentJointAngles, copy, copy.Length);
            return copy;
        }

        public RMMatrix4x4 GetEndEffectorPose()
        {
            if (_model == null) return RMMatrix4x4.Identity;
            if (_fkCacheValid) return _cachedEEPose;
            return _fkSolver.Solve(_model, _currentJointAngles);
        }

        /// <summary>
        /// 获取末端执行器在 Unity 世界坐标系下的位姿（DH 结果 × 校准变换）。
        /// </summary>
        public RMMatrix4x4 GetEndEffectorWorldPose()
        {
            if (_model == null) return RMMatrix4x4.Identity;
            var dhPose = _fkCacheValid ? _cachedEEPose : _fkSolver.Solve(_model, _currentJointAngles);
            return _dhToUnityCalibration * dhPose;
        }

        public ControlMode GetControlMode() => _currentMode;

        /// <summary>
        /// 获取指定关节的 DH 坐标帧在 Unity 世界坐标系下的变换矩阵。
        /// 用于 Gizmo 可视化：列 0=X 轴，列 1=Y 轴，列 2=Z 轴（旋转轴），列 3=原点位置。
        /// </summary>
        public RMMatrix4x4 GetJointDHFrameWorld(int index)
        {
            if (_model == null || index < 0 || index >= _model.DOF)
                return RMMatrix4x4.Identity;

            // FK 链约定 T_i = LinkTransform_i * RotZ(q_i)，
            // 关节 i 的旋转发生在帧 i 的 z 轴上。
            // 由于 RotZ 不改变 z 列和位置列，JointStates[i] 的 z 轴
            // 就是关节 i 的实际旋转轴方向。
            return _dhToUnityCalibration * _model.JointStates[index].WorldTransform;
        }

        public void SetControlMode(ControlMode mode)
        {
            _currentMode = mode;
            _ikTargetValid = false;
            _fkCacheValid = false;
            _interpolator?.Reset();
            _cartesianInterpolator?.Reset();
        }

        public JointConfig GetJointConfig(int index)
        {
            if (_model == null || index < 0 || index >= _model.DOF) return null;
            return _model.JointConfigs[index];
        }

        // ===== 热更新 =====

        /// <summary>
        /// 标记参数已变更，需要在下一帧重建。
        /// </summary>
        public void MarkDirty()
        {
            _isDirty = true;
            _fkCacheValid = false;
        }

        /// <summary>
        /// 处理 dirty flag，在帧末调用。
        /// </summary>
        public void ProcessDirtyFlag(JointSpatialInfo[] newSpatialInfos = null)
        {
            if (!_isDirty) return;
            _isDirty = false;

            // 取消正在运行的异步 IK 任务（模型即将重建，旧结果无效）
            if (_ikTask != null && !_ikTask.IsCompleted)
            {
                _ikCancellation?.Cancel();
                _ikTask = null;
            }
            _asyncTargetDirty = false;

            if (newSpatialInfos != null)
            {
                _model.RebuildDHParameters(newSpatialInfos);
            }

            // 重新计算零位 FK 和校准变换
            var zeroAngles = new float[_model.DOF];
            _fkSolver.Solve(_model, zeroAngles);
            for (int i = 0; i < _model.DOF; i++)
                _fkZeroWorldTransforms[i] = _model.JointStates[i].WorldTransform;

            var alignedFrame = BuildAlignedFrame(_model.JointConfigs[0],
                                                 _joint0InitWorldPos, _joint0InitWorldRot);
            var fkZeroT0Inv = RMMatrix4x4.Inverse(_fkZeroWorldTransforms[0]);
            _dhToUnityCalibration = alignedFrame * fkZeroT0Inv;
            _unityToDHCalibration = RMMatrix4x4.Inverse(_dhToUnityCalibration);

            // 重新计算末端执行器偏移
            if (_endEffectorObject != null && _endEffectorObject.IsValid)
            {
                var tcpUnityMatrix = EngineObjectToMatrix(_endEffectorObject);
                var tcpDHMatrix = _unityToDHCalibration * tcpUnityMatrix;
                var lastJointFKInv = RMMatrix4x4.Inverse(_fkZeroWorldTransforms[_model.DOF - 1]);
                _model.EndEffectorOffset = lastJointFKInv * tcpDHMatrix;
            }

            // 重新应用当前角度
            _ikTargetValid = false;
            _interpolator?.Reset();
            _cartesianInterpolator?.Reset();
            ApplyFK();
        }

        // ===== 内部方法 =====

        /// <summary>
        /// 处理 IK 求解结果：若角度差小则直接应用，否则根据运动模式选择 MoveJ 或 MoveL 插值。
        /// </summary>
        private void ApplyIKResult(float[] targetAngles)
        {
            if (_model == null) return;

            if (_interpolator == null)
            {
                System.Array.Copy(targetAngles, _currentJointAngles, _model.DOF);
                ApplyFK();
                return;
            }

            // 快速路径：若所有关节差异都在单帧 60fps 下的 maxStep 以内，直接应用
            // maxStep at 60fps = 180 deg/s * (1/60)s = 3 deg = 0.0524 rad
            const float fastPathThreshold = 0.0524f;
            bool smallDelta = true;
            for (int i = 0; i < _model.DOF; i++)
            {
                float diff = targetAngles[i] - _currentJointAngles[i];
                if (diff < 0) diff = -diff;
                if (diff > fastPathThreshold)
                {
                    smallDelta = false;
                    break;
                }
            }

            if (smallDelta)
            {
                // 差异很小，直接应用不经过插值
                System.Array.Copy(targetAngles, _currentJointAngles, _model.DOF);
                ApplyFK();
            }
            else if (_motionMode == MotionInterpolationMode.MoveL
                     && _cartesianInterpolator != null
                     && _ikTargetValid)
            {
                // MoveL：笛卡尔直线插值
                // 起始位姿 = 当前 FK 缓存（DH 空间）
                var currentEE = _fkCacheValid ? _cachedEEPose
                    : _fkSolver.Solve(_model, _currentJointAngles);
                var startPos = currentEE.GetPosition();
                var startRot = currentEE.GetRotation();

                // 目标位姿 = 精确的用户意图（非 IK 解的 FK 结果，避免残差）
                var endPos = _ikTargetPos;
                var endRot = _ikTargetRot;

                // 保存 IK 解作为 PTP 回退目标
                System.Array.Copy(targetAngles, _cartesianFallbackAngles, _model.DOF);

                // 启动笛卡尔轨迹
                _cartesianInterpolator.Start(startPos, startRot, endPos, endRot);

                if (_cartesianInterpolator.HasTarget)
                {
                    // 立即推进第一步
                    const float assumedDt = 1f / 60f;
                    AdvanceCartesianStep(assumedDt);
                }
                else
                {
                    // 起止太近，CartesianInterpolator 判定无需插值，直接应用
                    System.Array.Copy(targetAngles, _currentJointAngles, _model.DOF);
                    ApplyFK();
                }
            }
            else
            {
                // MoveJ：关节空间插值（现有逻辑）
                _interpolator.SetTarget(targetAngles);
                // 立即推进第一步（假定 60fps deltaTime），保证状态不完全过期
                const float assumedDt = 1f / 60f;
                _interpolator.Update(assumedDt, _currentJointAngles, _interpolatedAnglesBuffer);
                System.Array.Copy(_interpolatedAnglesBuffer, _currentJointAngles, _model.DOF);
                ApplyFK();
            }
        }

        /// <summary>
        /// 设置插值器的最大角速度（弧度/秒）。
        /// </summary>
        public void SetMaxInterpolationSpeed(float maxAngularVelocityRad)
        {
            if (_interpolator != null)
                _interpolator.MaxAngularVelocityRad = maxAngularVelocityRad;
            // 同步到 IK 求解器，用于并行解速度预筛选
            if (_ikSolver != null && _ikSolver.Config != null)
                _ikSolver.Config.MaxJointSpeedForFilter = maxAngularVelocityRad;
        }

        /// <summary>
        /// 设置笛卡尔插值器的 TCP 速度参数。
        /// </summary>
        /// <param name="linearMPerSec">TCP 最大线速度（米/秒）。</param>
        /// <param name="angularRadPerSec">TCP 最大角速度（弧度/秒）。</param>
        public void SetCartesianSpeed(float linearMPerSec, float angularRadPerSec)
        {
            if (_cartesianInterpolator == null) return;
            _cartesianInterpolator.MaxLinearSpeed = linearMPerSec;
            _cartesianInterpolator.MaxAngularSpeed = angularRadPerSec;
        }

        /// <summary>
        /// 运行时同步 IK 求解器配置参数。
        /// 仅同步允许运行时热调的字段，避免重建求解器。
        /// </summary>
        public void SyncIKConfig(IKSolverConfig source)
        {
            if (_ikSolver == null || _ikSolver.Config == null || source == null) return;
            var cfg = _ikSolver.Config;
            cfg.EnableWeightedDLS = source.EnableWeightedDLS;
            cfg.SoftLimitActivationZone = source.SoftLimitActivationZone;
            cfg.SoftLimitMaxPenalty = source.SoftLimitMaxPenalty;
            cfg.NullSpaceMidRangeGain = source.NullSpaceMidRangeGain;
            cfg.MultiSolutionTrials = source.MultiSolutionTrials;
            cfg.GoodThreshold = source.GoodThreshold;
            cfg.EasingPriorityCoefficient = source.EasingPriorityCoefficient;
            cfg.TierDecayCoefficient = source.TierDecayCoefficient;
        }

        /// <summary>
        /// 每帧推进运动插值。由 Runtime 层在 Update 中调用。
        /// MoveL 活跃时优先处理笛卡尔插值，否则走 MoveJ 关节插值。
        /// </summary>
        public void UpdateInterpolation(float deltaTime)
        {
            if (_model == null) return;

            // MoveL 笛卡尔插值优先
            if (_cartesianInterpolator != null && _cartesianInterpolator.HasTarget)
            {
                AdvanceCartesianStep(deltaTime);
                return;
            }

            // MoveJ 关节插值
            if (_interpolator == null || !_interpolator.HasTarget)
                return;

            _interpolator.Update(deltaTime, _currentJointAngles, _interpolatedAnglesBuffer);
            System.Array.Copy(_interpolatedAnglesBuffer, _currentJointAngles, _model.DOF);
            ApplyFK();
        }

        // ===== MoveL 笛卡尔插值核心 =====

        /// <summary>
        /// 推进一帧笛卡尔直线插值：产出路点 → 并行多解 IK 求解 → 应用或失败处理。
        /// </summary>
        private void AdvanceCartesianStep(float deltaTime)
        {
            bool done = _cartesianInterpolator.Update(deltaTime,
                out var wpPos, out var wpRot);

            if (done)
            {
                // 轨迹完成：直接应用保存的精确 IK 解（避免浮点累积漂移）
                System.Array.Copy(_cartesianFallbackAngles, _currentJointAngles, _model.DOF);
                ApplyFK();
                return;
            }

            // 对路点位姿做完整的并行多解 IK 求解（不绕过底层求解架构）
            int trials = _ikSolver.Config.MultiSolutionTrials;
            var result = _ikSolver.SolveMulti(_model, _currentJointAngles,
                wpPos, wpRot, trials, out var allResults);

            if (result != null && IsWaypointIKAcceptable(result))
            {
                // 成功：应用最优解
                System.Array.Copy(result.JointAngles, _currentJointAngles, _model.DOF);
                ApplyFK();
                LastIKResult = result;
                _lastMultiResults = allResults;
                UpdateDiagnosticData(result);
            }
            else
            {
                // IK 求解失败
                HandleCartesianIKFailure(wpPos, wpRot);
            }
        }

        /// <summary>
        /// 判断路点 IK 结果是否可接受。
        /// 路径中间点允许比收敛阈值稍大的误差（10 倍宽容）。
        /// </summary>
        private bool IsWaypointIKAcceptable(IKResult result)
        {
            if (result.Converged) return true;

            // 宽容接受：误差在收敛阈值的 10 倍以内
            float posThreshold = _ikSolver.Config.PositionThreshold * 10f;
            if (result.PositionError < posThreshold)
                return true;

            return false;
        }

        /// <summary>
        /// MoveL 路径上 IK 求解失败的处理。
        /// </summary>
        private void HandleCartesianIKFailure(RMVector3 failPos, RMQuaternion failRot)
        {
            _cartesianInterpolator.Reset();

            if (_ikFailureStrategy == IKFailureStrategy.StopAndReport)
            {
                // 停止运动，触发事件通知上层
                OnCartesianIKFailure?.Invoke(failPos, failRot);
            }
            else
            {
                // FallbackToPTP：自动回退到关节空间插值完成剩余运动
                _interpolator.SetTarget(_cartesianFallbackAngles);
                const float dt = 1f / 60f;
                _interpolator.Update(dt, _currentJointAngles, _interpolatedAnglesBuffer);
                System.Array.Copy(_interpolatedAnglesBuffer, _currentJointAngles, _model.DOF);
                ApplyFK();
            }
        }

        private void ApplyFK()
        {
            _cachedEEPose = _fkSolver.Solve(_model, _currentJointAngles);
            _fkCacheValid = true;
            SyncToScene(_currentJointAngles);
            UpdateLimitStatus();
        }

        /// <summary>
        /// 将关节角度同步到引擎场景。
        /// 策略：在零位局部旋转基础上，绕关节配置的旋转轴叠加关节角度。
        /// 只修改 LocalRotation，不修改 LocalPosition，
        /// 以保持 Unity 层级结构中的连杆几何（父子位置关系）不变。
        /// DH FK 结果仅用于末端位姿计算（IK / 可视化），不写入中间关节 Transform。
        /// </summary>
        private void SyncToScene(float[] jointAngles)
        {
            if (_jointObjects == null) return;

            for (int i = 0; i < _model.DOF; i++)
            {
                if (!_jointObjects[i].IsValid) continue;

                var cfg = _model.JointConfigs[i];

                // 旋转轴（关节局部坐标系下）
                var localAxis = cfg.AxisDirection.SqrMagnitude > 1e-12f
                    ? RMVector3.Normalize(cfg.AxisDirection)
                    : RMVector3.Forward;

                // 在零位局部旋转基础上，绕局部旋转轴叠加关节角度
                var deltaRot = RMQuaternion.FromAxisAngle(localAxis, jointAngles[i]);
                _jointObjects[i].LocalRotation = _jointInitLocalRotations[i] * deltaRot;
            }
        }

        /// <summary>
        /// 检测每个关节的限位状态，并在状态变化时触发事件。
        /// 判断标准：角度与限位边界的差值小于 epsilon 即视为触碰限位。
        /// </summary>
        private void UpdateLimitStatus()
        {
            if (_jointAtLimit == null || _model == null) return;

            const float epsilon = 1e-4f;
            int n = _model.DOF;

            for (int i = 0; i < n; i++)
            {
                var cfg = _model.JointConfigs[i];
                bool atLimit = false;

                if (cfg.HasLimits)
                {
                    float angle = _currentJointAngles[i];
                    atLimit = (cfg.MaxLimit - cfg.MinLimit) > epsilon &&
                              (System.Math.Abs(angle - cfg.MinLimit) < epsilon ||
                               System.Math.Abs(angle - cfg.MaxLimit) < epsilon);
                }

                _jointAtLimit[i] = atLimit;

                // 状态变化时触发事件
                if (atLimit != _prevJointAtLimit[i])
                {
                    _prevJointAtLimit[i] = atLimit;
                    OnJointLimitChanged?.Invoke(i, atLimit);
                }
            }
        }

        /// <summary>
        /// 从 IEngineObject 构建近似 4x4 变换矩阵（位置+旋转）。
        /// </summary>
        private static RMMatrix4x4 EngineObjectToMatrix(IEngineObject obj)
        {
            return RMMatrix4x4.FromTranslationRotation(obj.WorldPosition, obj.WorldRotation);
        }

        /// <summary>
        /// 构建与 DH 基帧对齐的坐标帧：
        /// Z 轴 = 关节实际旋转轴（世界方向），
        /// X 轴 = 垂直于 Z 的参考方向（与 BuildDHFromSpatialConfig 一致），
        /// 位置 = 关节世界位置（含偏移微调）。
        /// </summary>
        private static RMMatrix4x4 BuildAlignedFrame(JointConfig cfg,
                                                      RMVector3 worldPos,
                                                      RMQuaternion worldRot)
        {
            // 计算世界旋转轴（与 BuildDHFromSpatialConfig 的 zAxes[0] 一致）
            var adjustedRot = worldRot;
            if (cfg.RotationOffset.SqrMagnitude > 1e-12f)
            {
                var offsetQuat = RMQuaternion.EulerToQuaternion(cfg.RotationOffset);
                adjustedRot = worldRot * offsetQuat;
            }

            var localAxis = cfg.AxisDirection.SqrMagnitude > 1e-12f
                ? RMVector3.Normalize(cfg.AxisDirection)
                : RMVector3.Forward;
            var zAxis = RMVector3.Normalize(RMQuaternion.RotateVector(adjustedRot, localAxis));

            // X 轴：优先用关节初始 X 方向投影到垂直于 Z 的平面（Gram-Schmidt），
            // 确保零位时与 Unity 初始帧一致。退化时回退到任意垂直方向。
            // 必须与 BuildDHFromSpatialConfig 的 xAxes[0] 计算一致。
            var initX = RMQuaternion.RotateVector(adjustedRot, RMVector3.Right);
            var xProj = initX - zAxis * RMVector3.Dot(initX, zAxis);
            RMVector3 xAxis;
            if (xProj.SqrMagnitude > 1e-12f)
                xAxis = RMVector3.Normalize(xProj);
            else
                xAxis = RMGeometryUtils.ChoosePerpendicularAxis(zAxis);

            // Y 轴：右手坐标系
            var yAxis = RMVector3.Cross(zAxis, xAxis);

            // 位置（含原点偏移微调）
            var pos = worldPos + cfg.OriginOffset;

            return RMGeometryUtils.BuildFrameMatrix(pos, xAxis, yAxis, zAxis);
        }

        // ===== 异步 IK =====

        /// <summary>
        /// 提交异步 IK 目标。
        /// 如果有任务正在运行，仅更新待处理目标（不取消当前任务），
        /// 待当前任务完成后自动用最新目标再提交一轮，避免连续输入导致任务被反复取消而无法完成。
        /// </summary>
        private void SubmitAsyncIK(RMVector3 targetPos, RMQuaternion targetRot)
        {
            _pendingIKTargetPos = targetPos;
            _pendingIKTargetRot = targetRot;

            // 如果已有任务在跑，只标记有新目标，不打断当前计算
            if (_ikTask != null && !_ikTask.IsCompleted)
            {
                _asyncTargetDirty = true;
                return;
            }

            LaunchAsyncIKTask();
        }

        /// <summary>
        /// 以当前 _pendingIKTargetPos/Rot 和 _currentJointAngles 为快照启动后台 IK 任务。
        /// </summary>
        private void LaunchAsyncIKTask()
        {
            _asyncTargetDirty = false;

            if (_ikCancellation != null)
                _ikCancellation.Dispose();
            _ikCancellation = new CancellationTokenSource();

            if (_asyncIKSolver == null)
                _asyncIKSolver = new IKSolver(_ikSolver.Config);

            var model = _model;
            var currentAngles = new float[_model.DOF];
            System.Array.Copy(_currentJointAngles, currentAngles, _model.DOF);
            var solver = _asyncIKSolver;
            var token = _ikCancellation.Token;
            var targetPos = _pendingIKTargetPos;
            var targetRot = _pendingIKTargetRot;
            int trials = _ikSolver.Config.MultiSolutionTrials;

            _ikTask = Task.Run(() =>
            {
                if (token.IsCancellationRequested) return null;
                return solver.SolveMulti(model, currentAngles, targetPos, targetRot,
                    trials, out _lastMultiResults);
            }, token);
        }

        /// <summary>
        /// 每帧轮询异步 IK 结果。在 Update 中调用。
        /// 如果后台 IK 已完成，将结果应用到关节角度并同步场景。
        /// 若任务运行期间有新目标到达，立即用最新目标和最新关节角度启动下一轮计算。
        /// </summary>
        public void PollAsyncIKResult()
        {
            if (_ikComputeMode != IKComputeMode.Asynchronous) return;
            if (_ikTask == null || !_ikTask.IsCompleted) return;

            IKResult result = null;
            try
            {
                result = _ikTask.Result;
            }
            catch (System.AggregateException)
            {
                // 任务被取消或出错，忽略
            }

            _ikTask = null;

            if (result != null)
            {
                LastIKResult = result;
                _ikTargetPos = _pendingIKTargetPos;
                _ikTargetRot = _pendingIKTargetRot;
                _ikTargetValid = true;
                UpdateDiagnosticData(result);
                ApplyIKResult(result.JointAngles);
            }

            // 任务运行期间有新目标到达 → 用最新状态立即启动下一轮
            if (_asyncTargetDirty)
                LaunchAsyncIKTask();
        }
    }
}
