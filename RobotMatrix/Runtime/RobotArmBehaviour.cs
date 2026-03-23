using UnityEngine;
using RobotMatrix.Math;
using IEngine;
using IEngine.Unity;
using RobotMatrix.Kinematics;
using RobotMatrix.Controller;
using RobotMatrix.Instructions;
using RobotMatrix.Interaction;
using RobotMatrix.Data;
using RMPersistence;
using RMRecorder;
using RMCollision;

namespace RobotMatrix.Runtime
{
    /// <summary>
    /// 机械臂 MonoBehaviour 入口组件。
    /// 将运动学核心、交互层、可视化串联在一起，挂载到场景 GameObject 上。
    /// </summary>
    [AddComponentMenu("RobotMatrix/Robot Arm")]
    public class RobotArmBehaviour : MonoBehaviour
    {
        // ===== Inspector 可序列化字段 =====

        [Header("关节配置")]
        [Tooltip("关节对应的 Transform 数组（按运动学链序排列）")]
        public Transform[] jointTransforms;

        [Tooltip("末端执行器 Transform（可选）")]
        public Transform endEffectorTransform;

        [Tooltip("各关节参数配置")]
        public SerializableJointConfig[] jointConfigs;

        [Header("IK 配置")]
        public SerializableIKConfig ikConfig = new SerializableIKConfig();

        [Header("可视化")]
        public VisualizationConfig visualization = new VisualizationConfig();

        [Header("交互")]
        [Tooltip("是否启用键盘输入")]
        public bool enableKeyboardInput = true;

        [Tooltip("是否启用鼠标输入")]
        public bool enableMouseInput;

        [Tooltip("FK 模式关节旋转速度（度/秒）")]
        [Range(1f, 180f)]
        public float jointSpeedDegPerSec = 30f;

        [Tooltip("IK 模式末端移动速度（米/秒）")]
        [Range(0.01f, 2f)]
        public float endEffectorSpeedMPerSec = 0.2f;

        [Tooltip("IK 模式末端旋转速度（度/秒）")]
        [Range(1f, 360f)]
        public float endEffectorRotSpeedDegPerSec = 90f;

        [Tooltip("关节插值最大角速度（度/秒），防止 IK 结果导致瞬间跳变")]
        [Range(10f, 720f)]
        public float maxJointInterpolationSpeedDegPerSec = 45f;

        [Header("运动模式")]
        [Tooltip("MoveJ = 关节空间插值（TCP 走弧线，速度快）  MoveL = 笛卡尔直线插值（TCP 走直线）")]
        public MotionInterpolationMode motionInterpolationMode = MotionInterpolationMode.MoveJ;

        [Tooltip("MoveL 模式下 TCP 最大线速度（米/秒）")]
        [Range(0.01f, 5f)]
        public float tcpMaxLinearSpeedMPerSec = 0.5f;

        [Tooltip("MoveL 模式下 TCP 最大角速度（度/秒）")]
        [Range(1f, 720f)]
        public float tcpMaxAngularSpeedDegPerSec = 90f;

        [Tooltip("MoveL 路径上 IK 失败时的处理策略")]
        public IKFailureStrategy ikFailureStrategy = IKFailureStrategy.FallbackToPTP;

        [Header("性能")]
        [Tooltip("IK 计算模式：同步（默认）或异步（后台线程）")]
        public IKComputeMode ikComputeMode = IKComputeMode.Synchronous;

        [Tooltip("IK 增量坐标系：World（世界坐标系）或 TCP（末端执行器坐标系）")]
        public IKDeltaFrame ikDeltaFrame = IKDeltaFrame.World;

        [Header("碰撞检测 (RMCollision)")]
        public CollisionConfig collisionConfig = new CollisionConfig();

        [Tooltip("每个关节对应的碰撞体覆盖列表（可选）。\n" +
                 "每个关节可拖入多个带 Collider 的 Transform，留空则使用自动生成。\n" +
                 "数组长度应与关节数一致，不足的关节按 AutoAddColliders 处理。")]
        public JointColliderOverrideEntry[] jointColliderOverrides;

        [Tooltip("外部环境碰撞体列表（可选）。\n" +
                 "拖入 TCP 工具、工件、桌面等外部 Collider。\n" +
                 "系统会自动设置 isTrigger 和 Rigidbody，并在校准阶段排除初始接触。")]
        public Collider[] externalColliders;

        [Header("数据记录 (RMRecorder)")]
        public RecorderConfig recorderConfig = new RecorderConfig();

        [Header("持久化 (RMPersistence)")]
        public PersistenceConfig persistenceConfig = new PersistenceConfig();

        // ===== 内部引用 =====

        private RobotArmController _controller;
        private CommandDispatcher _dispatcher;
        private InputManager _inputManager;
        private KeyboardInputHandler _keyboardHandler;
        private MouseInputHandler _mouseHandler;
        private KeyboardInputConfig _keyboardConfig;
        private IEngineVisualizer _visualizer;
        private IEnginePhysics _physics;

        private IEngineObject[] _jointEngineObjects;
        private IEngineObject _endEffectorEngineObject;

        private bool _initialized;

        private FKSolver _ghostFKSolver;

        // ===== 套件控制器 =====
        private CollisionSuiteController _collisionSuite;
        private RecorderSuiteController _recorderSuite;

        // ===== 公共属性 =====

        /// <summary>底层控制器（供编辑器扩展或外部脚本访问）。</summary>
        public RobotArmController Controller => _controller;

        /// <summary>是否已初始化。</summary>
        public bool IsInitialized => _initialized;

        /// <summary>键盘处理器（可获取当前选中关节等信息）。</summary>
        public KeyboardInputHandler KeyboardHandler => _keyboardHandler;

        /// <summary>碰撞检测套件控制器。</summary>
        public CollisionSuiteController CollisionSuite => _collisionSuite;

        /// <summary>数据记录套件控制器。</summary>
        public RecorderSuiteController RecorderSuite => _recorderSuite;

        // ===== 生命周期 =====

        #if UNITY_EDITOR
        private void Reset()
        {
            if (!RMLicense.LicenseGuard.IsValid)
            {
                UnityEngine.Debug.LogError("[RobotMatrix] License not activated. Please activate via RobotMatrix > License > Activate.");
                UnityEditor.EditorApplication.delayCall += () =>
                {
                    if (this != null)
                        DestroyImmediate(this);
                    UnityEditor.EditorApplication.ExecuteMenuItem("RobotMatrix/License/Activate");
                };
            }
        }
        #endif

        private void Start()
        {
            InitializeArm();
        }

        private void OnDestroy()
        {
            if (_controller != null)
            {
                _controller.OnJointLimitChanged -= OnJointLimitStatusChanged;
                _controller.OnCartesianIKFailure -= OnCartesianIKFailed;
            }
            _collisionSuite?.Dispose();
            _recorderSuite?.Dispose();
        }

        private void Update()
        {
            if (!_initialized) return;

            // 同步 Inspector 可调参数到 Controller（支持运行时实时切换）
            _controller.ComputeMode = ikComputeMode;
            _controller.DeltaFrame = ikDeltaFrame;
            _controller.SetMaxInterpolationSpeed(maxJointInterpolationSpeedDegPerSec * RMMathUtils.Deg2Rad);
            _controller.SyncIKConfig(ikConfig.ToIKSolverConfig());

            // 同步运动模式参数（MoveJ / MoveL）
            _controller.MotionMode = motionInterpolationMode;
            _controller.FailureStrategy = ikFailureStrategy;
            _controller.SetCartesianSpeed(
                tcpMaxLinearSpeedMPerSec,
                tcpMaxAngularSpeedDegPerSec * RMMathUtils.Deg2Rad);

            // 每帧根据 deltaTime 更新步长，使速度与帧率无关
            float dt = Time.deltaTime;
            _keyboardConfig.JointAngleStep = jointSpeedDegPerSec * RMMathUtils.Deg2Rad * dt;
            _keyboardConfig.PositionStep = endEffectorSpeedMPerSec * dt;
            _keyboardConfig.RotationStep = endEffectorRotSpeedDegPerSec * RMMathUtils.Deg2Rad * dt;

            _inputManager.Tick();
            _controller.PollAsyncIKResult();
            _controller.UpdateInterpolation(dt);
            _controller.ProcessDirtyFlag();

            // 碰撞检测套件 Tick
            _collisionSuite?.Tick(dt);

            // 数据记录套件 Tick
            if (_recorderSuite != null && _recorderSuite.IsRecording)
            {
                _recorderSuite.Tick(dt, _controller.GetJointAngles());
            }
        }

        private void OnDrawGizmos()
        {
            // 编辑模式（未初始化）时显示关节预览
            if (!_initialized)
            {
                DrawEditModeGizmos();
                return;
            }

            if (visualization == null || _visualizer == null) return;
            if (jointTransforms == null) return;

            int dof = _controller.DOF;
            bool isFKMode = _controller.GetControlMode() == ControlMode.FK;
            int selectedIdx = (isFKMode && _keyboardHandler != null)
                ? _keyboardHandler.SelectedJointIndex : -1;

            // 获取 limitFactor 用于热力图着色
            var limitFactors = _controller.LastJointLimitFactors;

            for (int i = 0; i < dof; i++)
            {
                if (jointTransforms[i] == null) continue;
                var jointPos = ToRMVector3(jointTransforms[i].position);

                bool isSelected = (i == selectedIdx);
                bool isAtLimit = _controller.IsJointAtLimit(i);

                // 关节球体
                if (visualization.showJointAxes)
                {
                    if (isAtLimit)
                    {
                        // 限位状态：红色实心球 + 半透明红色外圈警示
                        _visualizer.DrawSphere(jointPos, visualization.jointSphereRadius * 1.5f,
                            RMColor.Red);
                        _visualizer.DrawWireSphere(jointPos, visualization.jointSphereRadius * 3f,
                            new RMColor(1f, 0f, 0f, 0.5f));
                    }
                    else if (isSelected)
                    {
                        // FK 模式选中关节：橙色实心球 + 外围大圈
                        _visualizer.DrawSphere(jointPos, visualization.jointSphereRadius * 1.5f,
                            RMColor.Orange);
                        _visualizer.DrawWireSphere(jointPos, visualization.jointSphereRadius * 3f,
                            new RMColor(1f, 0.8f, 0f, 0.5f));
                    }
                    else if (visualization.showLimitHeatmap && limitFactors != null && i < limitFactors.Length)
                    {
                        // 热力图模式：根据 limitFactor 渐变着色（绿→黄→红）
                        float lf = limitFactors[i];
                        var heatColor = LimitFactorToColor(lf);
                        _visualizer.DrawWireSphere(jointPos, visualization.jointSphereRadius, heatColor);
                    }
                    else
                    {
                        // IK 模式统一青色，FK 模式未选中用黄色
                        var color = isFKMode ? RMColor.Yellow : RMColor.Cyan;
                        _visualizer.DrawWireSphere(jointPos, visualization.jointSphereRadius, color);
                    }
                }

                // 连杆线段
                if (visualization.showLinks && i > 0 && jointTransforms[i - 1] != null)
                {
                    var prevPos = ToRMVector3(jointTransforms[i - 1].position);
                    _visualizer.DrawLine(prevPos, jointPos, RMColor.White);
                }

                // 坐标轴
                if (visualization.showJointAxes)
                {
                    float len = isSelected ? visualization.axisLength * 1.8f : visualization.axisLength;
                    DrawAxes(jointTransforms[i], len);
                }
            }

            // 末端执行器标记
            if (visualization.showEndEffector)
            {
                // 实际 Unity TCP 标记（绿色）
                if (endEffectorTransform != null)
                {
                    var eePos = ToRMVector3(endEffectorTransform.position);
                    _visualizer.DrawWireSphere(eePos, visualization.endEffectorRadius, RMColor.Green);

                    if (dof > 0 && jointTransforms[dof - 1] != null)
                    {
                        var lastJointPos = ToRMVector3(jointTransforms[dof - 1].position);
                        _visualizer.DrawLine(lastJointPos, eePos, RMColor.Green);
                    }
                }
                else if (dof > 0 && jointTransforms[dof - 1] != null)
                {
                    // 没配末端 Transform，在最后一个关节处画标记即可
                    var lastPos = ToRMVector3(jointTransforms[dof - 1].position);
                    _visualizer.DrawWireSphere(lastPos, visualization.endEffectorRadius, RMColor.Green);
                }

                // FK 计算的 TCP 位置标记（洋红色）— 修复后应与绿色标记重合
                var fkWorldPose = _controller.GetEndEffectorWorldPose();
                var fkWorldPos = fkWorldPose.GetPosition();
                _visualizer.DrawSphere(fkWorldPos, visualization.endEffectorRadius * 0.6f,
                    new RMColor(1f, 0f, 1f, 0.7f));
            }

            // DH 坐标帧可视化（FK / IK 模式都显示）
            if (visualization.showDHFrames)
            {
                float dhLen = visualization.dhFrameAxisLength;
                // 使用稍浅的颜色与 Unity Transform 坐标轴区分
                var dhRed   = new RMColor(1f, 0.3f, 0.3f, 1f);
                var dhGreen = new RMColor(0.3f, 1f, 0.3f, 1f);
                var dhBlue  = new RMColor(0.3f, 0.3f, 1f, 1f);

                for (int i = 0; i < dof; i++)
                {
                    var dhFrame = _controller.GetJointDHFrameWorld(i);
                    var dhPos = dhFrame.GetPosition();
                    var dhX = dhFrame.GetColumn(0);
                    var dhY = dhFrame.GetColumn(1);
                    var dhZ = dhFrame.GetColumn(2);

                    var xEnd = dhPos + dhX * dhLen;
                    var yEnd = dhPos + dhY * dhLen;
                    var zEnd = dhPos + dhZ * dhLen;

                    _visualizer.DrawLine(dhPos, xEnd, dhRed);
                    _visualizer.DrawLine(dhPos, yEnd, dhGreen);
                    _visualizer.DrawLine(dhPos, zEnd, dhBlue);
                }
            }

            // 幽灵臂（多解候选可视化）
            if (visualization.showAllSolutions)
            {
                DrawGhostArms(dof);
            }

            // 碰撞体可视化
            _collisionSuite?.DrawColliderGizmos(_visualizer);
        }

        // ===== Gizmo 辅助方法 =====

        private void DrawAxes(Transform t, float length)
        {
            var origin = ToRMVector3(t.position);
            var xEnd = ToRMVector3(t.position + t.right * length);
            var yEnd = ToRMVector3(t.position + t.up * length);
            var zEnd = ToRMVector3(t.position + t.forward * length);

            _visualizer.DrawLine(origin, xEnd, RMColor.Red);
            _visualizer.DrawLine(origin, yEnd, RMColor.Green);
            _visualizer.DrawLine(origin, zEnd, RMColor.Blue);
        }

        private static RMVector3 ToRMVector3(Vector3 v)
        {
            return new RMVector3(v.x, v.y, v.z);
        }

        /// <summary>
        /// 将 limitFactor (0~1) 映射为热力图颜色：0=红, 0.5=黄, 1=绿。
        /// </summary>
        private static RMColor LimitFactorToColor(float lf)
        {
            if (lf < 0f) lf = 0f;
            if (lf > 1f) lf = 1f;

            if (lf < 0.5f)
            {
                // 红 → 黄
                float t = lf * 2f;
                return new RMColor(1f, t, 0f, 1f);
            }
            else
            {
                // 黄 → 绿
                float t = (lf - 0.5f) * 2f;
                return new RMColor(1f - t, 1f, 0f, 1f);
            }
        }

        /// <summary>
        /// 梯队 → 幽灵臂颜色（半透明）。
        /// </summary>
        private static RMColor TierToGhostColor(int tier)
        {
            switch (tier)
            {
                case 1: return new RMColor(0f, 1f, 0f, 0.3f);     // 绿
                case 2: return new RMColor(1f, 1f, 0f, 0.3f);     // 黄
                case 3: return new RMColor(1f, 0.5f, 0f, 0.3f);   // 橙
                default: return new RMColor(1f, 0f, 0f, 0.25f);   // 红
            }
        }

        /// <summary>
        /// 渲染多解候选的幽灵臂骨架。
        /// 保存/恢复 model.JointStates 以避免 FK 计算污染当前状态。
        /// </summary>
        private void DrawGhostArms(int dof)
        {
            var allResults = _controller.LastMultiResults;
            if (allResults == null || allResults.Length <= 1) return;

            var model = _controller.Model;
            if (model == null) return;

            if (_ghostFKSolver == null)
                _ghostFKSolver = new FKSolver();

            // 保存当前 JointStates WorldTransform
            var savedTransforms = new RMMatrix4x4[dof];
            for (int i = 0; i < dof; i++)
                savedTransforms[i] = model.JointStates[i].WorldTransform;

            // 找到最佳解索引（跳过，主模型已显示）
            int bestIdx = 0;
            for (int i = 1; i < allResults.Length; i++)
            {
                if (allResults[i] == null) continue;
                if (allResults[bestIdx] == null ||
                    allResults[i].SolutionTier < allResults[bestIdx].SolutionTier ||
                    (allResults[i].SolutionTier == allResults[bestIdx].SolutionTier &&
                     allResults[i].SolutionScore > allResults[bestIdx].SolutionScore))
                {
                    bestIdx = i;
                }
            }

            float sphereR = visualization.jointSphereRadius * 0.6f;

            for (int s = 0; s < allResults.Length; s++)
            {
                if (s == bestIdx) continue;
                var result = allResults[s];
                if (result == null || result.JointAngles == null) continue;
                if (!result.Converged) continue;

                // 用候选角度运行 FK（会写入 model.JointStates）
                _ghostFKSolver.Solve(model, result.JointAngles);
                var ghostColor = TierToGhostColor(result.SolutionTier);

                // 绘制幽灵骨架
                for (int i = 0; i < dof; i++)
                {
                    var pos = model.JointStates[i].WorldTransform.GetPosition();
                    _visualizer.DrawWireSphere(pos, sphereR, ghostColor);

                    if (i > 0)
                    {
                        var prevPos = model.JointStates[i - 1].WorldTransform.GetPosition();
                        _visualizer.DrawLine(prevPos, pos, ghostColor);
                    }
                }

                // 幽灵 TCP
                var ghostEE = (model.JointStates[dof - 1].WorldTransform * model.EndEffectorOffset).GetPosition();
                _visualizer.DrawWireSphere(ghostEE, sphereR * 1.2f, ghostColor);
                if (dof > 0)
                {
                    var lastJPos = model.JointStates[dof - 1].WorldTransform.GetPosition();
                    _visualizer.DrawLine(lastJPos, ghostEE, ghostColor);
                }
            }

            // 恢复原始 JointStates
            for (int i = 0; i < dof; i++)
                model.JointStates[i].WorldTransform = savedTransforms[i];
        }

        /// <summary>
        /// 编辑模式（未运行）下的关节 Transform 预览 Gizmo。
        /// 在关节 Transform 被拖入 Inspector 后，即使 Unity 未运行也能看到
        /// 各关节的坐标轴、连杆线段和运动轴方向。
        /// 当 Inspector 中修改 originOffset / rotationOffsetDeg 时，Gizmo 自动更新。
        /// </summary>
        private void DrawEditModeGizmos()
        {
            if (jointTransforms == null || jointTransforms.Length == 0) return;

            float axisLen = (visualization != null) ? visualization.axisLength : 0.1f;
            float sphereRadius = (visualization != null) ? visualization.jointSphereRadius : 0.03f;

            int count = jointTransforms.Length;

            for (int i = 0; i < count; i++)
            {
                var t = jointTransforms[i];
                if (t == null) continue;

                // 计算调整后的位姿（应用 originOffset 和 rotationOffsetDeg）
                Vector3 adjustedPos = t.position;
                Quaternion adjustedRot = t.rotation;

                if (jointConfigs != null && i < jointConfigs.Length && jointConfigs[i] != null)
                {
                    var cfg = jointConfigs[i];
                    if (cfg.originOffset.sqrMagnitude > 1e-12f)
                        adjustedPos = t.TransformPoint(cfg.originOffset);
                    if (cfg.rotationOffsetDeg.sqrMagnitude > 1e-12f)
                        adjustedRot = t.rotation * Quaternion.Euler(cfg.rotationOffsetDeg);
                }

                // 绘制坐标轴（R=X, G=Y, B=Z）
                Vector3 xEnd = adjustedPos + adjustedRot * Vector3.right * axisLen;
                Vector3 yEnd = adjustedPos + adjustedRot * Vector3.up * axisLen;
                Vector3 zEnd = adjustedPos + adjustedRot * Vector3.forward * axisLen;

                Gizmos.color = Color.red;
                Gizmos.DrawLine(adjustedPos, xEnd);
                Gizmos.color = Color.green;
                Gizmos.DrawLine(adjustedPos, yEnd);
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(adjustedPos, zEnd);

                // 绘制关节球体（半透明黄色）
                Gizmos.color = new Color(1f, 1f, 0f, 0.4f);
                Gizmos.DrawWireSphere(adjustedPos, sphereRadius);

                // 高亮显示运动轴方向（如果已配置）
                if (jointConfigs != null && i < jointConfigs.Length && jointConfigs[i] != null)
                {
                    var axisDir = jointConfigs[i].axisDirection;
                    if (axisDir.sqrMagnitude > 1e-12f)
                    {
                        Vector3 worldAxis = adjustedRot * axisDir.normalized;
                        Vector3 axisEnd = adjustedPos + worldAxis * axisLen * 2f;
                        // 用较亮的橙色粗线标示运动轴
                        Gizmos.color = new Color(1f, 0.6f, 0f, 1f);
                        Gizmos.DrawLine(adjustedPos, axisEnd);
                        Gizmos.DrawWireSphere(axisEnd, sphereRadius * 0.5f);
                    }
                }

                // 连杆线段（到前一个关节）
                if (i > 0 && jointTransforms[i - 1] != null)
                {
                    Vector3 prevPos = jointTransforms[i - 1].position;
                    if (jointConfigs != null && i - 1 < jointConfigs.Length && jointConfigs[i - 1] != null)
                    {
                        var prevCfg = jointConfigs[i - 1];
                        if (prevCfg.originOffset.sqrMagnitude > 1e-12f)
                            prevPos = jointTransforms[i - 1].TransformPoint(prevCfg.originOffset);
                    }
                    Gizmos.color = new Color(1f, 1f, 1f, 0.4f);
                    Gizmos.DrawLine(prevPos, adjustedPos);
                }
            }

            // 末端执行器标记
            if (endEffectorTransform != null)
            {
                Gizmos.color = new Color(0f, 1f, 0f, 0.5f);
                Gizmos.DrawWireSphere(endEffectorTransform.position, sphereRadius * 1.2f);

                if (count > 0 && jointTransforms[count - 1] != null)
                {
                    Gizmos.color = new Color(0f, 1f, 0f, 0.3f);
                    Gizmos.DrawLine(jointTransforms[count - 1].position, endEffectorTransform.position);
                }
            }
        }

        // ===== 初始化 =====

        /// <summary>
        /// 初始化机械臂系统。可在 Start 中自动调用，也可由编辑器手动触发。
        /// </summary>
        public void InitializeArm()
        {
            #if UNITY_EDITOR
            if (!RMLicense.LicenseGuard.IsValid)
            {
                Debug.LogError("[RobotMatrix] License not activated. Please activate via RobotMatrix > License > Activate.");
                return;
            }
            #endif

            if (jointTransforms == null || jointTransforms.Length == 0)
            {
                Debug.LogWarning($"[RobotArm] '{gameObject.name}' 未配置关节 Transform 数组。", gameObject);
                return;
            }

            int dof = jointTransforms.Length;

            // 1. 包装 IEngineObject
            _jointEngineObjects = new IEngineObject[dof];
            for (int i = 0; i < dof; i++)
            {
                if (jointTransforms[i] == null)
                {
                    Debug.LogError($"[RobotArm] '{gameObject.name}' 关节 [{i}] Transform 为空。", gameObject);
                    return;
                }
                _jointEngineObjects[i] = new UnityEngineObject(jointTransforms[i]);
            }

            if (endEffectorTransform != null)
                _endEffectorEngineObject = new UnityEngineObject(endEffectorTransform);

            // 2. 构建关节配置
            var configs = BuildJointConfigs(dof);
            var spatialInfos = BuildSpatialInfos(dof);

            // 3. 创建控制器并初始化
            _controller = new RobotArmController();
            _controller.ComputeMode = ikComputeMode;
            _controller.DeltaFrame = ikDeltaFrame;
            _controller.InitializeFromSpatialConfig(
                spatialInfos, configs, _jointEngineObjects,
                _endEffectorEngineObject, ikConfig.ToIKSolverConfig());

            // 4. 创建交互层
            var factory = new UnityAdapterFactory();
            _dispatcher = new CommandDispatcher(_controller);
            _inputManager = new InputManager(_dispatcher);

            var input = factory.CreateInput();

            _keyboardConfig = new KeyboardInputConfig();
            _keyboardHandler = new KeyboardInputHandler(input, _controller, _keyboardConfig);
            _keyboardHandler.Enabled = enableKeyboardInput;
            _inputManager.Register(_keyboardHandler);

            _mouseHandler = new MouseInputHandler(input, _controller);
            _mouseHandler.Enabled = enableMouseInput;
            _inputManager.Register(_mouseHandler);

            // 5. 创建可视化器
            _visualizer = factory.CreateVisualizer();

            // 6. 订阅关节限位状态变更事件
            _controller.OnJointLimitChanged += OnJointLimitStatusChanged;

            // 7. 订阅 MoveL IK 失败事件
            _controller.OnCartesianIKFailure += OnCartesianIKFailed;

            // 8. 初始化物理引擎抽象
            _physics = factory.CreatePhysics();

            // 9. 初始化碰撞检测套件
            IEngineObject[][] colliderOverrideObjects = null;
            if (jointColliderOverrides != null && jointColliderOverrides.Length > 0)
            {
                colliderOverrideObjects = new IEngineObject[jointColliderOverrides.Length][];
                for (int i = 0; i < jointColliderOverrides.Length; i++)
                {
                    var entry = jointColliderOverrides[i];
                    if (entry == null || entry.colliders == null || entry.colliders.Length == 0)
                        continue;

                    var list = new System.Collections.Generic.List<IEngineObject>();
                    for (int j = 0; j < entry.colliders.Length; j++)
                    {
                        if (entry.colliders[j] != null)
                            list.Add(new UnityEngineObject(entry.colliders[j].transform));
                    }
                    if (list.Count > 0)
                        colliderOverrideObjects[i] = list.ToArray();
                }
            }
            // 转换外部碰撞体
            IEngineObject[] externalColliderObjects = null;
            if (externalColliders != null && externalColliders.Length > 0)
            {
                var extList = new System.Collections.Generic.List<IEngineObject>();
                for (int i = 0; i < externalColliders.Length; i++)
                {
                    if (externalColliders[i] != null)
                        extList.Add(new UnityEngineObject(externalColliders[i].transform));
                }
                if (extList.Count > 0)
                    externalColliderObjects = extList.ToArray();
            }

            _collisionSuite = new CollisionSuiteController();
            _collisionSuite.Initialize(_jointEngineObjects, _physics, collisionConfig,
                persistenceConfig, colliderOverrideObjects, externalColliderObjects);

            // 10. 初始化数据记录套件
            _recorderSuite = new RecorderSuiteController(dof, recorderConfig, persistenceConfig);

            _initialized = true;
        }

        /// <summary>
        /// 关节限位状态变更时输出日志。
        /// </summary>
        private void OnJointLimitStatusChanged(int jointIndex, bool isAtLimit)
        {
            string jointName = (jointConfigs != null && jointIndex < jointConfigs.Length
                && jointConfigs[jointIndex] != null)
                ? jointConfigs[jointIndex].name : $"Joint {jointIndex}";
            float angleDeg = _controller.GetJointAngles()[jointIndex] * RMMathUtils.Rad2Deg;

            if (isAtLimit)
            {
                Debug.LogWarning(
                    $"[RobotArm] 关节 {jointIndex} '{jointName}' 触碰限位: 当前角度 {angleDeg:F1}°",
                    gameObject);
            }
            else
            {
                Debug.Log(
                    $"[RobotArm] 关节 {jointIndex} '{jointName}' 已回到限位范围内",
                    gameObject);
            }
        }

        /// <summary>
        /// MoveL 路径上 IK 求解失败时的日志输出。
        /// </summary>
        private void OnCartesianIKFailed(RMVector3 failPos, RMQuaternion failRot)
        {
            Debug.LogWarning(
                $"[RobotArm] MoveL IK 求解失败 — 路点位置: ({failPos.X:F4}, {failPos.Y:F4}, {failPos.Z:F4})",
                gameObject);
        }

        // ===== 内部构建辅助 =====

        private JointConfig[] BuildJointConfigs(int dof)
        {
            var configs = new JointConfig[dof];
            for (int i = 0; i < dof; i++)
            {
                if (jointConfigs != null && i < jointConfigs.Length)
                    configs[i] = jointConfigs[i].ToJointConfig();
                else
                    configs[i] = new JointConfig();
            }
            return configs;
        }

        private JointSpatialInfo[] BuildSpatialInfos(int dof)
        {
            var infos = new JointSpatialInfo[dof];
            for (int i = 0; i < dof; i++)
            {
                var t = jointTransforms[i];
                var cfg = (jointConfigs != null && i < jointConfigs.Length)
                    ? jointConfigs[i] : null;

                infos[i] = new JointSpatialInfo
                {
                    WorldPosition = new RMVector3(t.position.x, t.position.y, t.position.z),
                    WorldRotation = new RMQuaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                    AxisDirection = cfg != null
                        ? new RMVector3(cfg.axisDirection.x, cfg.axisDirection.y, cfg.axisDirection.z)
                        : RMVector3.Forward,
                    OriginOffset = cfg != null
                        ? new RMVector3(cfg.originOffset.x, cfg.originOffset.y, cfg.originOffset.z)
                        : RMVector3.Zero,
                    RotationOffset = cfg != null
                        ? new RMVector3(
                            cfg.rotationOffsetDeg.x * RMMathUtils.Deg2Rad,
                            cfg.rotationOffsetDeg.y * RMMathUtils.Deg2Rad,
                            cfg.rotationOffsetDeg.z * RMMathUtils.Deg2Rad)
                        : RMVector3.Zero
                };
            }
            return infos;
        }

        // ===== 公共 API 便捷方法 =====

        /// <summary>
        /// 设置指定关节角度（度）。
        /// </summary>
        public void SetJointAngleDeg(int jointIndex, float angleDeg)
        {
            if (_controller != null)
                _controller.SetJointAngle(jointIndex, angleDeg * RMMathUtils.Deg2Rad);
        }

        /// <summary>
        /// 获取所有关节角度（度）。
        /// </summary>
        public float[] GetJointAnglesDeg()
        {
            if (_controller == null) return new float[0];
            var rad = _controller.GetJointAngles();
            var deg = new float[rad.Length];
            for (int i = 0; i < rad.Length; i++)
                deg[i] = rad[i] * RMMathUtils.Rad2Deg;
            return deg;
        }

        /// <summary>
        /// 编辑器下重建机械臂（参数修改后调用）。
        /// </summary>
        public void RebuildArm()
        {
            _initialized = false;
            InitializeArm();
        }
    }
}
