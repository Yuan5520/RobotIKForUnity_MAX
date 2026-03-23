using UnityEngine;
using UnityEditor;
using RobotMatrix.Math;
using RobotMatrix.Controller;
using RobotMatrix.Kinematics;
using RobotMatrix.Data;
using RobotMatrix.Runtime;
using RobotMatrix.Instructions;
using RMCollision;
using RMRecorder;

namespace RobotMatrix.Editor
{
    /// <summary>
    /// RobotArmBehaviour 的自定义 Inspector 面板。
    /// 提供关节配置编辑、实时角度滑块、模式切换和状态监控。
    /// </summary>
    [CustomEditor(typeof(RobotArmBehaviour))]
    public class RobotArmEditor : UnityEditor.Editor
    {
        // ===== 序列化属性 =====
        private SerializedProperty _jointTransforms;
        private SerializedProperty _endEffectorTransform;
        private SerializedProperty _jointConfigs;
        private SerializedProperty _ikConfig;
        private SerializedProperty _visualization;
        private SerializedProperty _enableKeyboardInput;
        private SerializedProperty _enableMouseInput;
        private SerializedProperty _jointSpeedDegPerSec;
        private SerializedProperty _endEffectorSpeedMPerSec;
        private SerializedProperty _endEffectorRotSpeedDegPerSec;
        private SerializedProperty _ikComputeMode;
        private SerializedProperty _ikDeltaFrame;
        private SerializedProperty _maxJointInterpolationSpeedDegPerSec;
        private SerializedProperty _motionInterpolationMode;
        private SerializedProperty _tcpMaxLinearSpeedMPerSec;
        private SerializedProperty _tcpMaxAngularSpeedDegPerSec;
        private SerializedProperty _ikFailureStrategy;

        // ===== 套件序列化属性 =====
        private SerializedProperty _collisionConfig;
        private SerializedProperty _recorderConfig;
        private SerializedProperty _persistenceConfig;

        // ===== 折叠状态 =====
        // 第一组：关节配置、IK 配置、运动模式 —— 默认展开
        private bool _foldoutJoints = true;
        private bool _foldoutIK = true;
        private bool _foldoutMotionMode = true;
        // 第二组：交互配置、可视化 —— 默认收缩
        private bool _foldoutInteraction = false;
        private bool _foldoutVisualization = false;
        // 套件面板 —— 默认收缩
        private bool _foldoutCollision = false;
        private bool _foldoutRecorder = false;
        private bool _foldoutPersistence = false;
        // 运行时监控 —— 非运行时收缩，运行时自动展开
        private bool _foldoutRuntime = false;
        // 运行时子面板 —— 运行时全部展开
        private bool _foldoutIKStatus = true;
        private bool _foldoutMultiSolution = true;
        private bool _foldoutDragCompensation = true;

        // ===== 实时角度控制 =====
        private float[] _editorJointAngles;

        // ===== 编辑模式轴向编辑 =====
        private int _axisEditJointIndex = -1;

        // ===== 运行时状态追踪 =====
        private bool _wasPlaying = false;

        private void OnEnable()
        {
            _jointTransforms = serializedObject.FindProperty("jointTransforms");
            _endEffectorTransform = serializedObject.FindProperty("endEffectorTransform");
            _jointConfigs = serializedObject.FindProperty("jointConfigs");
            _ikConfig = serializedObject.FindProperty("ikConfig");
            _visualization = serializedObject.FindProperty("visualization");
            _enableKeyboardInput = serializedObject.FindProperty("enableKeyboardInput");
            _enableMouseInput = serializedObject.FindProperty("enableMouseInput");
            _jointSpeedDegPerSec = serializedObject.FindProperty("jointSpeedDegPerSec");
            _endEffectorSpeedMPerSec = serializedObject.FindProperty("endEffectorSpeedMPerSec");
            _endEffectorRotSpeedDegPerSec = serializedObject.FindProperty("endEffectorRotSpeedDegPerSec");
            _ikComputeMode = serializedObject.FindProperty("ikComputeMode");
            _ikDeltaFrame = serializedObject.FindProperty("ikDeltaFrame");
            _maxJointInterpolationSpeedDegPerSec = serializedObject.FindProperty("maxJointInterpolationSpeedDegPerSec");
            _motionInterpolationMode = serializedObject.FindProperty("motionInterpolationMode");
            _tcpMaxLinearSpeedMPerSec = serializedObject.FindProperty("tcpMaxLinearSpeedMPerSec");
            _tcpMaxAngularSpeedDegPerSec = serializedObject.FindProperty("tcpMaxAngularSpeedDegPerSec");
            _ikFailureStrategy = serializedObject.FindProperty("ikFailureStrategy");

            _collisionConfig = serializedObject.FindProperty("collisionConfig");
            _recorderConfig = serializedObject.FindProperty("recorderConfig");
            _persistenceConfig = serializedObject.FindProperty("persistenceConfig");
        }

        public override void OnInspectorGUI()
        {
            // License check
            if (!RMLicense.LicenseGuard.IsValid)
            {
                EditorGUILayout.Space(10);
                EditorGUILayout.HelpBox("RobotMatrix is not activated.\nPlease activate your license to use this component.", MessageType.Warning);
                EditorGUILayout.Space(4);
                if (GUILayout.Button("Activate License", GUILayout.Height(30)))
                {
                    EditorApplication.ExecuteMenuItem("RobotMatrix/License/Activate");
                }
                EditorGUILayout.Space(4);
                EditorGUILayout.LabelField("Device ID: " + RMLicense.LicenseGuard.HardwareIdShort, EditorStyles.miniLabel);
                return;
            }

            serializedObject.Update();

            var behaviour = (RobotArmBehaviour)target;

            // 进入运行模式时自动展开运行时监控
            if (Application.isPlaying && !_wasPlaying)
            {
                _foldoutRuntime = true;
                _foldoutIKStatus = true;
                _foldoutMultiSolution = true;
                _foldoutDragCompensation = true;
            }
            _wasPlaying = Application.isPlaying;

            // ===== 1. 运行时监控（非运行时收缩，运行时展开） =====
            DrawRuntimeMonitorSection(behaviour);

            EditorGUILayout.Space(8);

            // ===== 2. 关节配置 =====
            DrawJointConfigSection(behaviour);

            EditorGUILayout.Space(8);

            // ===== 3. IK 求解器配置 =====
            DrawIKConfigSection();

            EditorGUILayout.Space(8);

            // ===== 4. 运动模式 =====
            DrawMotionModeSection(behaviour);

            EditorGUILayout.Space(8);

            // ===== 5. 交互配置 =====
            DrawInteractionSection();

            EditorGUILayout.Space(8);

            // ===== 6. 可视化配置 =====
            DrawVisualizationSection();

            EditorGUILayout.Space(8);

            // ===== 7. 碰撞检测配置 =====
            DrawCollisionSection(behaviour);

            EditorGUILayout.Space(8);

            // ===== 8. 数据记录配置 =====
            DrawRecorderSection(behaviour);

            EditorGUILayout.Space(8);

            // ===== 9. 持久化配置 =====
            DrawPersistenceSection();

            // ===== 操作按钮 =====
            EditorGUILayout.Space(8);
            if (Application.isPlaying)
            {
                if (GUILayout.Button("重新初始化"))
                {
                    behaviour.RebuildArm();
                }
            }

            serializedObject.ApplyModifiedProperties();
        }

        // ================================================================
        // 1. 运行时监控
        // ================================================================

        private void DrawRuntimeMonitorSection(RobotArmBehaviour behaviour)
        {
            _foldoutRuntime = EditorGUILayout.Foldout(_foldoutRuntime, "运行时监控", true);
            if (!_foldoutRuntime) return;

            EditorGUI.indentLevel++;

            bool isRuntime = Application.isPlaying && behaviour.IsInitialized;

            if (isRuntime)
            {
                DrawRuntimePanel(behaviour);
            }
            else
            {
                DrawEditModePanel(behaviour);
            }

            EditorGUI.indentLevel--;
        }

        private void DrawRuntimePanel(RobotArmBehaviour behaviour)
        {
            var ctrl = behaviour.Controller;
            int dof = ctrl.DOF;

            // 控制模式显示
            var mode = ctrl.GetControlMode();
            EditorGUILayout.LabelField("控制模式", mode.ToString());
            EditorGUILayout.LabelField("增量坐标系", ctrl.DeltaFrame.ToString());
            EditorGUILayout.LabelField("运动模式", ctrl.MotionMode.ToString());

            // MoveL 进度
            if (ctrl.IsCartesianInterpolating)
            {
                var rect = EditorGUILayout.GetControlRect(GUILayout.Height(18));
                EditorGUI.ProgressBar(rect, ctrl.CartesianProgress,
                    $"笛卡尔插值进度: {ctrl.CartesianProgress * 100f:F1}%");
            }

            // 当前关节
            if (behaviour.KeyboardHandler != null)
            {
                EditorGUILayout.LabelField("当前选中关节",
                    behaviour.KeyboardHandler.SelectedJointIndex.ToString());
            }

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("关节角度 (\u00b0)", EditorStyles.boldLabel);

            // 初始化编辑器角度缓存
            if (_editorJointAngles == null || _editorJointAngles.Length != dof)
            {
                _editorJointAngles = behaviour.GetJointAnglesDeg();
            }
            else
            {
                var current = behaviour.GetJointAnglesDeg();
                System.Array.Copy(current, _editorJointAngles, dof);
            }

            // 为每个关节绘制滑块
            bool changed = false;
            for (int i = 0; i < dof; i++)
            {
                var cfg = ctrl.GetJointConfig(i);
                string label = (behaviour.jointConfigs != null && i < behaviour.jointConfigs.Length)
                    ? behaviour.jointConfigs[i].name
                    : $"Joint {i}";

                float minDeg = cfg != null && cfg.HasLimits ? cfg.MinLimit * RMMathUtils.Rad2Deg : -180f;
                float maxDeg = cfg != null && cfg.HasLimits ? cfg.MaxLimit * RMMathUtils.Rad2Deg : 180f;

                float newVal = EditorGUILayout.Slider(label, _editorJointAngles[i], minDeg, maxDeg);
                if (Mathf.Abs(newVal - _editorJointAngles[i]) > 0.01f)
                {
                    _editorJointAngles[i] = newVal;
                    changed = true;
                }
            }

            if (changed)
            {
                for (int i = 0; i < dof; i++)
                {
                    behaviour.SetJointAngleDeg(i, _editorJointAngles[i]);
                }
            }

            // 末端位姿
            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("末端位姿", EditorStyles.boldLabel);

            var eePose = ctrl.GetEndEffectorWorldPose();
            var eePos = eePose.GetPosition();
            var eeRot = eePose.GetRotation();
            EditorGUILayout.Vector3Field("位置",
                new Vector3(eePos.X, eePos.Y, eePos.Z));
            var eeEuler = RMQuaternion.QuaternionToEuler(eeRot);
            EditorGUILayout.Vector3Field("姿态 (rad)",
                new Vector3(eeEuler.X, eeEuler.Y, eeEuler.Z));

            // IK 状态
            var lastIK = ctrl.LastIKResult;
            if (lastIK != null)
            {
                EditorGUILayout.Space(4);
                EditorGUILayout.LabelField("IK 状态", EditorStyles.boldLabel);
                EditorGUILayout.LabelField("收敛状态", lastIK.Converged ? "是" : "否");
                EditorGUILayout.LabelField("位置误差", lastIK.PositionError.ToString("E3") + " m");
                EditorGUILayout.LabelField("旋转误差", lastIK.RotationError.ToString("E3") + " rad");
                EditorGUILayout.LabelField("迭代次数", lastIK.IterationsUsed.ToString());
                EditorGUILayout.LabelField("解梯队", $"T{lastIK.SolutionTier}");
                EditorGUILayout.LabelField("梯队内得分", lastIK.SolutionScore.ToString("F3"));
                EditorGUILayout.LabelField("缓动系数 b", lastIK.EasingCoefficient.ToString("F3"));
                EditorGUILayout.LabelField("综合评分", lastIK.CompositeScore.ToString("F3"));
            }

            // ===== 运行时可调参数 =====
            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("运行时可调参数", EditorStyles.boldLabel);

            EditorGUI.BeginChangeCheck();
            float newInterpSpeed = EditorGUILayout.Slider("插值最大角速度 (\u00b0/s)",
                behaviour.maxJointInterpolationSpeedDegPerSec, 10f, 720f);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(behaviour, "Change Max Interp Speed");
                behaviour.maxJointInterpolationSpeedDegPerSec = newInterpSpeed;
                EditorUtility.SetDirty(behaviour);
            }

            EditorGUI.BeginChangeCheck();
            float newEasingA = EditorGUILayout.Slider("缓动优先系数 a",
                behaviour.ikConfig.easingPriorityCoefficient, 0f, 1f);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(behaviour, "Change Easing Priority");
                behaviour.ikConfig.easingPriorityCoefficient = newEasingA;
                EditorUtility.SetDirty(behaviour);
            }

            // IK 求解详情
            EditorGUILayout.Space(4);
            _foldoutIKStatus = EditorGUILayout.Foldout(_foldoutIKStatus, "IK 求解详情", true);
            if (_foldoutIKStatus)
            {
                EditorGUI.indentLevel++;
                DrawIKSolveStatusPanel(ctrl, dof, behaviour);
                EditorGUI.indentLevel--;
            }

            // 多解搜索面板
            _foldoutMultiSolution = EditorGUILayout.Foldout(_foldoutMultiSolution, "多解搜索", true);
            if (_foldoutMultiSolution)
            {
                EditorGUI.indentLevel++;
                DrawMultiSolutionPanel(ctrl, behaviour);
                EditorGUI.indentLevel--;
            }

            // 关节拖动补偿面板
            _foldoutDragCompensation = EditorGUILayout.Foldout(_foldoutDragCompensation, "关节拖动补偿", true);
            if (_foldoutDragCompensation)
            {
                EditorGUI.indentLevel++;
                DrawDragCompensationPanel(ctrl, dof, behaviour);
                EditorGUI.indentLevel--;
            }

            // 持续重绘以显示实时数据
            Repaint();
        }

        private void DrawEditModePanel(RobotArmBehaviour behaviour)
        {
            EditorGUILayout.HelpBox("进入运行模式后可查看实时监控数据", MessageType.Info);

            int dof = behaviour.jointTransforms != null ? behaviour.jointTransforms.Length : 0;
            EditorGUILayout.LabelField("自由度", dof.ToString());

            if (behaviour.jointConfigs != null && behaviour.jointConfigs.Length > 0)
            {
                EditorGUILayout.Space(4);
                EditorGUILayout.LabelField("关节配置概览", EditorStyles.boldLabel);

                for (int i = 0; i < behaviour.jointConfigs.Length; i++)
                {
                    var cfg = behaviour.jointConfigs[i];
                    if (cfg == null) continue;

                    string jointLabel = string.IsNullOrEmpty(cfg.name) ? $"Joint {i}" : cfg.name;
                    EditorGUILayout.LabelField(jointLabel, EditorStyles.boldLabel);

                    EditorGUI.indentLevel++;
                    EditorGUI.BeginDisabledGroup(true);
                    EditorGUILayout.LabelField("类型", cfg.type.ToString());
                    EditorGUILayout.Vector3Field("旋转轴", cfg.axisDirection);
                    if (cfg.hasLimits)
                    {
                        EditorGUILayout.LabelField("限位范围",
                            $"{cfg.minLimitDeg:F1}\u00b0 ~ {cfg.maxLimitDeg:F1}\u00b0");
                    }
                    else
                    {
                        EditorGUILayout.LabelField("限位范围", "无限位");
                    }
                    EditorGUILayout.FloatField("初始角度 (\u00b0)", cfg.initialAngleDeg);
                    EditorGUI.EndDisabledGroup();
                    EditorGUI.indentLevel--;
                }
            }

            // 末端位姿
            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("末端位姿", EditorStyles.boldLabel);
            EditorGUILayout.LabelField("位置", "---");
            EditorGUILayout.LabelField("姿态", "---");

            // IK 状态
            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("IK 状态", EditorStyles.boldLabel);
            EditorGUILayout.LabelField("收敛状态", "---");
        }

        // ================================================================
        // 2. 关节配置
        // ================================================================

        private void DrawJointConfigSection(RobotArmBehaviour behaviour)
        {
            _foldoutJoints = EditorGUILayout.Foldout(_foldoutJoints, "关节配置", true);
            if (!_foldoutJoints) return;

            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(_jointTransforms, new GUIContent("关节 Transform 列表"), true);
            EditorGUILayout.PropertyField(_endEffectorTransform, new GUIContent("末端执行器 Transform"));

            EditorGUILayout.Space(4);
            EditorGUILayout.PropertyField(_jointConfigs, new GUIContent("关节参数配置"), true);

            // 轴向编辑按钮（每个关节一个）
            if (!Application.isPlaying && behaviour.jointConfigs != null && behaviour.jointConfigs.Length > 0)
            {
                EditorGUILayout.Space(4);
                EditorGUILayout.LabelField("轴向编辑", EditorStyles.boldLabel);
                for (int i = 0; i < behaviour.jointConfigs.Length; i++)
                {
                    var cfg = behaviour.jointConfigs[i];
                    if (cfg == null) continue;
                    string jointLabel = string.IsNullOrEmpty(cfg.name) ? $"Joint {i}" : cfg.name;
                    bool isEditing = (_axisEditJointIndex == i);

                    EditorGUILayout.BeginHorizontal();
                    var prevBg = GUI.backgroundColor;
                    if (isEditing) GUI.backgroundColor = new Color(1f, 0.8f, 0f);

                    EditorGUILayout.LabelField($"  {jointLabel}  (轴: {cfg.axisDirection})", GUILayout.MinWidth(200));
                    string btnLabel = isEditing ? "停止编辑" : "编辑轴向";
                    if (GUILayout.Button(btnLabel, GUILayout.Width(80)))
                    {
                        _axisEditJointIndex = isEditing ? -1 : i;
                        SceneView.RepaintAll();
                    }

                    GUI.backgroundColor = prevBg;
                    EditorGUILayout.EndHorizontal();
                }
            }

            // 同步按钮
            EditorGUILayout.Space(4);
            if (GUILayout.Button("同步配置数组长度"))
            {
                SyncConfigArrayLength(behaviour);
            }

            EditorGUI.indentLevel--;
        }

        // ================================================================
        // 3. IK 求解器配置
        // ================================================================

        private void DrawIKConfigSection()
        {
            _foldoutIK = EditorGUILayout.Foldout(_foldoutIK, "IK 求解器配置", true);
            if (!_foldoutIK) return;

            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(_ikConfig, true);

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("运行时参数", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(_ikComputeMode, new GUIContent("计算模式"));
            EditorGUILayout.PropertyField(_ikDeltaFrame, new GUIContent("增量坐标系"));
            EditorGUI.indentLevel--;
        }

        // ================================================================
        // 4. 运动模式
        // ================================================================

        private void DrawMotionModeSection(RobotArmBehaviour behaviour)
        {
            _foldoutMotionMode = EditorGUILayout.Foldout(_foldoutMotionMode, "运动模式", true);
            if (!_foldoutMotionMode) return;

            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(_motionInterpolationMode, new GUIContent("插值模式"));

            // MoveL 参数仅在 MoveL 模式下显示
            if (behaviour.motionInterpolationMode == MotionInterpolationMode.MoveL)
            {
                EditorGUILayout.Space(4);
                EditorGUILayout.LabelField("MoveL 参数", EditorStyles.boldLabel);
                EditorGUILayout.PropertyField(_tcpMaxLinearSpeedMPerSec, new GUIContent("TCP 线速度 (m/s)"));
                EditorGUILayout.PropertyField(_tcpMaxAngularSpeedDegPerSec, new GUIContent("TCP 角速度 (\u00b0/s)"));
                EditorGUILayout.PropertyField(_ikFailureStrategy, new GUIContent("IK 失败策略"));
            }

            EditorGUI.indentLevel--;
        }

        // ================================================================
        // 5. 交互配置
        // ================================================================

        private void DrawInteractionSection()
        {
            _foldoutInteraction = EditorGUILayout.Foldout(_foldoutInteraction, "交互配置", true);
            if (!_foldoutInteraction) return;

            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(_enableKeyboardInput, new GUIContent("启用键盘输入"));

            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("运动速度", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(_jointSpeedDegPerSec, new GUIContent("FK 关节速度 (\u00b0/s)"));
            EditorGUILayout.PropertyField(_endEffectorSpeedMPerSec, new GUIContent("IK 移动速度 (m/s)"));
            EditorGUILayout.PropertyField(_endEffectorRotSpeedDegPerSec, new GUIContent("IK 旋转速度 (\u00b0/s)"));
            EditorGUILayout.PropertyField(_maxJointInterpolationSpeedDegPerSec, new GUIContent("插值最大角速度 (\u00b0/s)"));
            EditorGUI.indentLevel--;
        }

        // ================================================================
        // 6. 可视化配置
        // ================================================================

        private void DrawVisualizationSection()
        {
            _foldoutVisualization = EditorGUILayout.Foldout(_foldoutVisualization, "可视化配置", true);
            if (!_foldoutVisualization) return;

            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(_visualization, true);
            EditorGUI.indentLevel--;
        }

        // ================================================================
        // Scene 视图手柄
        // ================================================================

        private void OnSceneGUI()
        {
            var behaviour = (RobotArmBehaviour)target;

            // 编辑模式：轴向选择手柄
            if (!Application.isPlaying)
            {
                if (_axisEditJointIndex >= 0)
                {
                    HandleUtility.AddDefaultControl(
                        GUIUtility.GetControlID(FocusType.Passive));
                }
                DrawAxisSelectionHandles(behaviour);
                return;
            }

            // 运行模式：IK 末端执行器手柄
            if (!behaviour.IsInitialized) return;

            var ctrl = behaviour.Controller;
            if (ctrl.GetControlMode() != ControlMode.IK) return;

            var eePoseWorld = ctrl.GetEndEffectorWorldPose();
            var eePos = eePoseWorld.GetPosition();
            var eeRot = eePoseWorld.GetRotation();

            var unityPos = new Vector3(eePos.X, eePos.Y, eePos.Z);
            var unityRot = new Quaternion(eeRot.X, eeRot.Y, eeRot.Z, eeRot.W);

            // 位置手柄
            EditorGUI.BeginChangeCheck();
            var newPos = Handles.PositionHandle(unityPos, unityRot);
            if (EditorGUI.EndChangeCheck())
            {
                ctrl.MoveEndEffectorTo(
                    new RMVector3(newPos.x, newPos.y, newPos.z), eeRot);
                SceneView.RepaintAll();
            }

            // 旋转手柄
            EditorGUI.BeginChangeCheck();
            var newRot = Handles.RotationHandle(unityRot, unityPos);
            if (EditorGUI.EndChangeCheck())
            {
                ctrl.MoveEndEffectorTo(eePos,
                    new RMQuaternion(newRot.x, newRot.y, newRot.z, newRot.w));
                SceneView.RepaintAll();
            }
        }

        private void DrawAxisSelectionHandles(RobotArmBehaviour behaviour)
        {
            if (behaviour.jointTransforms == null || behaviour.jointConfigs == null) return;
            if (_axisEditJointIndex < 0 || _axisEditJointIndex >= behaviour.jointTransforms.Length) return;
            if (_axisEditJointIndex >= behaviour.jointConfigs.Length) return;

            var t = behaviour.jointTransforms[_axisEditJointIndex];
            if (t == null) return;

            var cfg = behaviour.jointConfigs[_axisEditJointIndex];
            if (cfg == null) return;

            Vector3 pos = t.position;
            Quaternion rot = t.rotation;

            if (cfg.originOffset.sqrMagnitude > 1e-12f)
                pos = t.TransformPoint(cfg.originOffset);
            if (cfg.rotationOffsetDeg.sqrMagnitude > 1e-12f)
                rot = t.rotation * Quaternion.Euler(cfg.rotationOffsetDeg);

            float handleSize = HandleUtility.GetHandleSize(pos);
            float arrowLen = handleSize * 1.5f;

            var prevZTest = Handles.zTest;
            Handles.zTest = UnityEngine.Rendering.CompareFunction.Always;

            Vector3 currentAxis = cfg.axisDirection.normalized;

            Vector3[] localDirs = { Vector3.right, Vector3.up, Vector3.forward };
            Color[] axisColors = { Color.red, Color.green, Color.blue };
            string[] axisLabels = { "X", "Y", "Z" };

            for (int a = 0; a < 3; a++)
            {
                Vector3 localDir = localDirs[a];
                Vector3 worldDir = rot * localDir;
                Vector3 arrowEnd = pos + worldDir * arrowLen;

                bool isSelected = Vector3.Dot(currentAxis, localDir) > 0.99f;

                Color drawColor = isSelected ? new Color(1f, 0.85f, 0f, 1f) : axisColors[a];
                float lineThickness = isSelected ? 4f : 2f;
                float capSize = handleSize * (isSelected ? 0.18f : 0.12f);
                float pickSize = handleSize * 0.2f;

                Handles.color = drawColor;
                Handles.DrawLine(pos, arrowEnd, lineThickness);

                Vector3 capUp = Mathf.Abs(Vector3.Dot(worldDir, Vector3.up)) > 0.99f
                    ? Vector3.forward : Vector3.up;
                Quaternion capRot = Quaternion.LookRotation(worldDir, capUp);

                if (Handles.Button(arrowEnd, capRot, capSize, pickSize, Handles.ConeHandleCap))
                {
                    Undo.RecordObject(behaviour, "Set Joint Axis Direction");
                    cfg.axisDirection = localDir;
                    EditorUtility.SetDirty(behaviour);
                    SceneView.RepaintAll();
                    Repaint();
                }

                Handles.color = drawColor;
                string labelText = isSelected ? $"[{axisLabels[a]}]" : axisLabels[a];
                var labelStyle = new GUIStyle(EditorStyles.boldLabel)
                {
                    normal = { textColor = drawColor },
                    fontSize = isSelected ? 14 : 11
                };
                Handles.Label(arrowEnd + worldDir * handleSize * 0.1f, labelText, labelStyle);

                if (isSelected)
                {
                    Handles.color = new Color(drawColor.r, drawColor.g, drawColor.b, 0.15f);
                    Handles.DrawSolidDisc(pos, worldDir, arrowLen * 0.4f);
                    Handles.color = new Color(drawColor.r, drawColor.g, drawColor.b, 0.5f);
                    Handles.DrawWireDisc(pos, worldDir, arrowLen * 0.4f);
                }
            }

            string jointName = !string.IsNullOrEmpty(cfg.name) ? cfg.name : $"Joint {_axisEditJointIndex}";
            Handles.Label(pos + Vector3.up * handleSize * 0.7f,
                $"轴向编辑: {jointName}",
                new GUIStyle(EditorStyles.boldLabel)
                {
                    normal = { textColor = new Color(1f, 0.85f, 0f) },
                    fontSize = 13
                });

            Handles.zTest = prevZTest;
        }

        // ================================================================
        // 运行时子面板
        // ================================================================

        private void DrawIKSolveStatusPanel(RobotArmController ctrl, int dof, RobotArmBehaviour behaviour)
        {
            var limitFactors = ctrl.LastJointLimitFactors;
            var weights = ctrl.LastJointWeights;

            if (limitFactors == null || weights == null)
            {
                EditorGUILayout.HelpBox("尚无 IK 求解数据", MessageType.None);
                return;
            }

            for (int i = 0; i < dof && i < limitFactors.Length; i++)
            {
                string label = (behaviour.jointConfigs != null && i < behaviour.jointConfigs.Length)
                    ? behaviour.jointConfigs[i].name : $"Joint {i}";

                EditorGUILayout.BeginHorizontal();

                float lf = limitFactors[i];
                var barColor = Color.Lerp(Color.red, Color.green, lf);
                var prevBg = GUI.backgroundColor;
                GUI.backgroundColor = barColor;
                EditorGUI.ProgressBar(
                    EditorGUILayout.GetControlRect(GUILayout.Height(16)),
                    lf,
                    $"{label}: LF={lf:F2}  W={1f / Mathf.Max(weights[i], 1e-6f):F1}");
                GUI.backgroundColor = prevBg;

                EditorGUILayout.EndHorizontal();
            }
        }

        private void DrawMultiSolutionPanel(RobotArmController ctrl, RobotArmBehaviour behaviour)
        {
            EditorGUI.BeginChangeCheck();
            int newTrials = EditorGUILayout.IntSlider("并行试验数",
                behaviour.ikConfig.multiSolutionTrials, 1, 32);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(behaviour, "Change MultiSolutionTrials");
                behaviour.ikConfig.multiSolutionTrials = newTrials;
                EditorUtility.SetDirty(behaviour);
            }

            EditorGUI.BeginChangeCheck();
            float newThreshold = EditorGUILayout.Slider("安全阈值",
                behaviour.ikConfig.goodThreshold, 0.05f, 0.4f);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(behaviour, "Change GoodThreshold");
                behaviour.ikConfig.goodThreshold = newThreshold;
                EditorUtility.SetDirty(behaviour);
            }

            // 候选解信息
            var allResults = ctrl.LastMultiResults;
            var lastIK = ctrl.LastIKResult;

            if (lastIK != null)
            {
                EditorGUILayout.Space(2);
                EditorGUILayout.LabelField("当前解",
                    $"T{lastIK.SolutionTier}  得分 {lastIK.SolutionScore:F3}  " +
                    $"b={lastIK.EasingCoefficient:F3}  综合={lastIK.CompositeScore:F3}");
            }

            if (allResults != null && allResults.Length > 1)
            {
                int[] tierCount = new int[5];
                int convergedCount = 0;
                for (int i = 0; i < allResults.Length; i++)
                {
                    if (allResults[i] == null) continue;
                    if (allResults[i].Converged) convergedCount++;
                    int t = Mathf.Clamp(allResults[i].SolutionTier, 1, 4);
                    tierCount[t]++;
                }

                EditorGUILayout.LabelField("候选解数",
                    $"{allResults.Length} ({convergedCount} 收敛)");
                EditorGUILayout.LabelField("梯队分布",
                    $"T1:{tierCount[1]} T2:{tierCount[2]} T3:{tierCount[3]} T4:{tierCount[4]}");

                EditorGUI.BeginChangeCheck();
                bool showAll = EditorGUILayout.Toggle("显示所有候选解",
                    behaviour.visualization.showAllSolutions);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(behaviour, "Toggle Show All Solutions");
                    behaviour.visualization.showAllSolutions = showAll;
                    EditorUtility.SetDirty(behaviour);
                    SceneView.RepaintAll();
                }
            }
        }

        // 关节拖动补偿面板用的编辑器状态
        private int _dragJointIndex;
        private float _dragTargetAngleDeg;
        private bool _dragAngleInitialized;
        private int _lastDragJointIndex = -1;

        private void DrawDragCompensationPanel(RobotArmController ctrl, int dof, RobotArmBehaviour behaviour)
        {
            if (dof <= 0)
            {
                EditorGUILayout.HelpBox("无可用关节", MessageType.None);
                return;
            }

            _dragJointIndex = Mathf.Clamp(_dragJointIndex, 0, dof - 1);
            string[] jointNames = new string[dof];
            for (int i = 0; i < dof; i++)
            {
                jointNames[i] = (behaviour.jointConfigs != null && i < behaviour.jointConfigs.Length)
                    ? behaviour.jointConfigs[i].name : $"Joint {i}";
            }
            _dragJointIndex = EditorGUILayout.Popup("目标关节", _dragJointIndex, jointNames);

            var cfg = ctrl.GetJointConfig(_dragJointIndex);
            float minDeg = cfg != null && cfg.HasLimits ? cfg.MinLimit * RMMathUtils.Rad2Deg : -180f;
            float maxDeg = cfg != null && cfg.HasLimits ? cfg.MaxLimit * RMMathUtils.Rad2Deg : 180f;

            if (!_dragAngleInitialized || _lastDragJointIndex != _dragJointIndex)
            {
                var initAngles = behaviour.GetJointAnglesDeg();
                if (initAngles.Length > _dragJointIndex)
                    _dragTargetAngleDeg = initAngles[_dragJointIndex];
                _lastDragJointIndex = _dragJointIndex;
                _dragAngleInitialized = true;
            }

            _dragTargetAngleDeg = EditorGUILayout.Slider("目标角度 (\u00b0)",
                _dragTargetAngleDeg, minDeg, maxDeg);

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("应用（保持 TCP）"))
            {
                float targetRad = _dragTargetAngleDeg * RMMathUtils.Deg2Rad;
                ctrl.AdjustJointKeepTCP(_dragJointIndex, targetRad);
                _dragAngleInitialized = false;
                SceneView.RepaintAll();
            }
            if (GUILayout.Button("重置"))
            {
                _dragAngleInitialized = false;
            }
            EditorGUILayout.EndHorizontal();
        }

        // ================================================================
        // 工具方法
        // ================================================================

        private void SyncConfigArrayLength(RobotArmBehaviour behaviour)
        {
            int targetLen = behaviour.jointTransforms != null ? behaviour.jointTransforms.Length : 0;

            Undo.RecordObject(behaviour, "Sync Config Array Length");
            bool changed = false;

            // 同步关节配置数组
            if (behaviour.jointConfigs == null || behaviour.jointConfigs.Length != targetLen)
            {
                var newConfigs = new SerializableJointConfig[targetLen];
                int copyLen = 0;
                if (behaviour.jointConfigs != null)
                    copyLen = Mathf.Min(behaviour.jointConfigs.Length, targetLen);

                for (int i = 0; i < copyLen; i++)
                    newConfigs[i] = behaviour.jointConfigs[i];

                for (int i = copyLen; i < targetLen; i++)
                {
                    newConfigs[i] = new SerializableJointConfig
                    {
                        name = behaviour.jointTransforms[i] != null
                            ? behaviour.jointTransforms[i].name
                            : $"Joint {i}"
                    };
                }

                behaviour.jointConfigs = newConfigs;
                changed = true;
            }

            // 同步碰撞体覆盖数组
            if (behaviour.jointColliderOverrides == null || behaviour.jointColliderOverrides.Length != targetLen)
            {
                var newOverrides = new JointColliderOverrideEntry[targetLen];
                if (behaviour.jointColliderOverrides != null)
                {
                    int copyLen = Mathf.Min(behaviour.jointColliderOverrides.Length, targetLen);
                    for (int i = 0; i < copyLen; i++)
                        newOverrides[i] = behaviour.jointColliderOverrides[i];
                }
                for (int i = 0; i < targetLen; i++)
                {
                    if (newOverrides[i] == null)
                        newOverrides[i] = new JointColliderOverrideEntry();
                }
                behaviour.jointColliderOverrides = newOverrides;
                changed = true;
            }

            if (changed)
                EditorUtility.SetDirty(behaviour);
        }

        // ================================================================
        // 7. 碰撞检测配置
        // ================================================================

        private void DrawCollisionSection(RobotArmBehaviour behaviour)
        {
            _foldoutCollision = EditorGUILayout.Foldout(_foldoutCollision, "碰撞检测 (RMCollision)", true);
            if (!_foldoutCollision) return;

            EditorGUI.indentLevel++;
            DrawPropertyChildren(_collisionConfig);

            // 碰撞体覆盖拖入口 —— 按关节名逐项显示（每个关节支持多个碰撞体）
            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("碰撞体覆盖（按关节）", EditorStyles.boldLabel);

            int dof = behaviour.jointTransforms != null ? behaviour.jointTransforms.Length : 0;

            if (behaviour.jointColliderOverrides == null || behaviour.jointColliderOverrides.Length != dof)
            {
                if (dof > 0 && GUILayout.Button("同步碰撞体覆盖数组"))
                {
                    Undo.RecordObject(behaviour, "Sync Collider Overrides");
                    var newOverrides = new JointColliderOverrideEntry[dof];
                    if (behaviour.jointColliderOverrides != null)
                    {
                        int copyLen = Mathf.Min(behaviour.jointColliderOverrides.Length, dof);
                        for (int i = 0; i < copyLen; i++)
                            newOverrides[i] = behaviour.jointColliderOverrides[i];
                    }
                    for (int i = 0; i < dof; i++)
                    {
                        if (newOverrides[i] == null)
                            newOverrides[i] = new JointColliderOverrideEntry();
                    }
                    behaviour.jointColliderOverrides = newOverrides;
                    EditorUtility.SetDirty(behaviour);
                }
            }
            else if (dof > 0)
            {
                EditorGUILayout.HelpBox("每个关节可拖入多个 Collider 组件，留空则自动生成。", MessageType.None);
                for (int i = 0; i < dof; i++)
                {
                    string jointLabel = (behaviour.jointConfigs != null && i < behaviour.jointConfigs.Length
                                         && behaviour.jointConfigs[i] != null
                                         && !string.IsNullOrEmpty(behaviour.jointConfigs[i].name))
                        ? behaviour.jointConfigs[i].name
                        : $"Joint {i}";

                    var entry = behaviour.jointColliderOverrides[i];
                    if (entry == null)
                    {
                        entry = new JointColliderOverrideEntry();
                        behaviour.jointColliderOverrides[i] = entry;
                        EditorUtility.SetDirty(behaviour);
                    }

                    int colliderCount = entry.colliders != null ? entry.colliders.Length : 0;
                    EditorGUILayout.BeginHorizontal();
                    EditorGUILayout.LabelField($"[{i}] {jointLabel} ({colliderCount})", EditorStyles.boldLabel);

                    if (GUILayout.Button("+", GUILayout.Width(24)))
                    {
                        Undo.RecordObject(behaviour, "Add Collider Override");
                        var list = new System.Collections.Generic.List<Collider>(
                            entry.colliders ?? new Collider[0]);
                        list.Add(null);
                        entry.colliders = list.ToArray();
                        EditorUtility.SetDirty(behaviour);
                    }
                    EditorGUILayout.EndHorizontal();

                    if (entry.colliders != null)
                    {
                        int removeIdx = -1;
                        for (int j = 0; j < entry.colliders.Length; j++)
                        {
                            EditorGUILayout.BeginHorizontal();
                            EditorGUI.BeginChangeCheck();
                            var newVal = (Collider)EditorGUILayout.ObjectField(
                                $"    碰撞体 {j}",
                                entry.colliders[j],
                                typeof(Collider), true);
                            if (EditorGUI.EndChangeCheck())
                            {
                                Undo.RecordObject(behaviour, "Set Collider Override");
                                entry.colliders[j] = newVal;
                                EditorUtility.SetDirty(behaviour);
                            }

                            if (GUILayout.Button("-", GUILayout.Width(24)))
                            {
                                removeIdx = j;
                            }
                            EditorGUILayout.EndHorizontal();
                        }

                        if (removeIdx >= 0)
                        {
                            Undo.RecordObject(behaviour, "Remove Collider Override");
                            var list = new System.Collections.Generic.List<Collider>(entry.colliders);
                            list.RemoveAt(removeIdx);
                            entry.colliders = list.ToArray();
                            EditorUtility.SetDirty(behaviour);
                        }
                    }
                }
            }

            // 外部环境碰撞体列表
            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField("外部环境碰撞体", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox("拖入 TCP 工具、工件、桌面等外部 Collider，系统自动设置 Trigger 和 Rigidbody。", MessageType.None);

            int extCount = behaviour.externalColliders != null ? behaviour.externalColliders.Length : 0;
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField($"数量: {extCount}");
            if (GUILayout.Button("+", GUILayout.Width(24)))
            {
                Undo.RecordObject(behaviour, "Add External Collider");
                var list = new System.Collections.Generic.List<Collider>(
                    behaviour.externalColliders ?? new Collider[0]);
                list.Add(null);
                behaviour.externalColliders = list.ToArray();
                EditorUtility.SetDirty(behaviour);
            }
            EditorGUILayout.EndHorizontal();

            if (behaviour.externalColliders != null)
            {
                int removeIdx = -1;
                for (int i = 0; i < behaviour.externalColliders.Length; i++)
                {
                    EditorGUILayout.BeginHorizontal();
                    EditorGUI.BeginChangeCheck();
                    var newVal = (Collider)EditorGUILayout.ObjectField(
                        $"    [{i}]",
                        behaviour.externalColliders[i],
                        typeof(Collider), true);
                    if (EditorGUI.EndChangeCheck())
                    {
                        Undo.RecordObject(behaviour, "Set External Collider");
                        behaviour.externalColliders[i] = newVal;
                        EditorUtility.SetDirty(behaviour);
                    }

                    if (GUILayout.Button("-", GUILayout.Width(24)))
                    {
                        removeIdx = i;
                    }
                    EditorGUILayout.EndHorizontal();
                }

                if (removeIdx >= 0)
                {
                    Undo.RecordObject(behaviour, "Remove External Collider");
                    var list = new System.Collections.Generic.List<Collider>(behaviour.externalColliders);
                    list.RemoveAt(removeIdx);
                    behaviour.externalColliders = list.ToArray();
                    EditorUtility.SetDirty(behaviour);
                }
            }

            // 运行时状态
            if (Application.isPlaying && behaviour.IsInitialized)
            {
                var suite = behaviour.CollisionSuite;
                if (suite != null && suite.IsInitialized)
                {
                    EditorGUILayout.Space(4);
                    EditorGUILayout.LabelField("运行时状态", EditorStyles.boldLabel);

                    var detector = suite.Detector;
                    if (detector != null)
                    {
                        int activeCount = 0;
                        foreach (var kvp in detector.AccumulatedScores)
                        {
                            if (kvp.Value > 0.01f) activeCount++;
                        }
                        EditorGUILayout.LabelField("活跃碰撞对", activeCount.ToString());

                        foreach (var kvp in detector.AccumulatedScores)
                        {
                            if (kvp.Value < 0.01f) continue;
                            var barColor = Color.Lerp(Color.yellow, Color.red,
                                Mathf.Clamp01(kvp.Value / 5f));
                            var prevBg = GUI.backgroundColor;
                            GUI.backgroundColor = barColor;
                            EditorGUI.ProgressBar(
                                EditorGUILayout.GetControlRect(GUILayout.Height(16)),
                                Mathf.Clamp01(kvp.Value / 10f),
                                $"Seg {kvp.Key.SegmentA} <-> Seg {kvp.Key.SegmentB}: {kvp.Value:F3}");
                            GUI.backgroundColor = prevBg;
                        }
                    }

                    if (suite.Segments != null)
                    {
                        EditorGUILayout.LabelField("分段数", suite.Segments.Length.ToString());
                    }
                }
                else
                {
                    EditorGUILayout.HelpBox("碰撞检测未启用。设置 Enabled = true 后重新初始化。",
                        MessageType.Info);
                }
            }

            EditorGUI.indentLevel--;
        }

        // ================================================================
        // 8. 数据记录配置
        // ================================================================

        private void DrawRecorderSection(RobotArmBehaviour behaviour)
        {
            _foldoutRecorder = EditorGUILayout.Foldout(_foldoutRecorder, "数据记录 (RMRecorder)", true);
            if (!_foldoutRecorder) return;

            EditorGUI.indentLevel++;
            DrawPropertyChildren(_recorderConfig);

            // 运行时录制控制
            if (Application.isPlaying && behaviour.IsInitialized)
            {
                var suite = behaviour.RecorderSuite;
                if (suite != null)
                {
                    EditorGUILayout.Space(4);

                    bool isRecording = suite.IsRecording;

                    EditorGUILayout.BeginHorizontal();
                    GUI.backgroundColor = isRecording ? new Color(1f, 0.3f, 0.3f) : new Color(0.3f, 1f, 0.3f);
                    string btnLabel = isRecording ? "停止录制" : "开始录制";
                    if (GUILayout.Button(btnLabel, GUILayout.Height(28)))
                    {
                        if (isRecording)
                            suite.StopRecording();
                        else
                            suite.StartRecording();
                    }
                    GUI.backgroundColor = Color.white;
                    EditorGUILayout.EndHorizontal();

                    if (isRecording && suite.CurrentSession != null)
                    {
                        var session = suite.CurrentSession;
                        EditorGUILayout.LabelField("录制状态", "录制中...");
                        EditorGUILayout.LabelField("已录帧数", session.RecordCount.ToString());
                        EditorGUILayout.LabelField("持续时间", $"{session.ElapsedTime:F1} 秒");
                        EditorGUILayout.LabelField("输出文件", session.FilePath);
                    }
                }
            }

            EditorGUI.indentLevel--;
        }

        // ================================================================
        // 9. 持久化配置
        // ================================================================

        private void DrawPersistenceSection()
        {
            _foldoutPersistence = EditorGUILayout.Foldout(_foldoutPersistence, "持久化 (RMPersistence)", true);
            if (!_foldoutPersistence) return;

            EditorGUI.indentLevel++;
            DrawPropertyChildren(_persistenceConfig);
            EditorGUI.indentLevel--;
        }

        // ================================================================
        // 辅助：遍历序列化属性的子字段直接绘制（无额外收缩栏）
        // ================================================================

        private void DrawPropertyChildren(SerializedProperty property)
        {
            var iter = property.Copy();
            var end = property.GetEndProperty();
            iter.NextVisible(true);
            while (!SerializedProperty.EqualContents(iter, end))
            {
                EditorGUILayout.PropertyField(iter, true);
                if (!iter.NextVisible(false)) break;
            }
        }
    }
}
