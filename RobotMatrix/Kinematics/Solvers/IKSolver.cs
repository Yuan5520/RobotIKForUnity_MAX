using System;
using System.Threading.Tasks;
using RobotMatrix.Math;

namespace RobotMatrix.Kinematics
{
    /// <summary>
    /// DLS（阻尼最小二乘法）逆运动学求解器。
    /// 
    /// 核心改进：
    ///   Phase 1 - 加权 DLS：Δθ = Jw^T * (Jw*J^T + λ²I)^{-1} * e
    ///             其中 Jw = J * diag(wInv)，wInv 为关节空间动态逆权重。
    ///             关节靠近限位时权重增大，抑制该关节运动。
    ///   Phase 2 - 零空间中位回归：在 SolvePositionOnly 模式下，
    ///             利用冗余自由度将关节推向中位。
    ///   Phase 3 - 锁定关节求解：通过 IKSolveOptions 指定锁定关节，
    ///             DLS 自动跳过锁定关节（wInv=0）。
    ///   Phase 4 - 并行多解搜索：SolveMulti 同时运行多个随机初始解，
    ///             四级梯队评分自动选取最优解。
    /// 
    /// 支持任意 N-DOF 串联机械臂。
    /// 支持两种模式：
    ///   - 全 6D 模式（3 位置 + 3 姿态）: Solve()
    ///   - 仅位置 3D 模式: SolvePositionOnly()
    /// </summary>
    public class IKSolver
    {
        public IKSolverConfig Config;

        private readonly FKSolver _fkSolver = new FKSolver();
        private readonly JacobianCalculator _jacobianCalc = new JacobianCalculator();

        // 预分配缓冲区（在 Prepare 中按 DOF 初始化）
        private RMMatrixMN _jBuffer;       // 6 x N  原始雅可比
        private RMMatrixMN _jtBuffer;      // N x 6  J^T
        private RMMatrixMN _jjtBuffer;     // 6 x 6
        private RMMatrixMN _jjtDamped;     // 6 x 6
        private RMMatrixMN _dampingBuffer; // 6 x 6
        private RMMatrixMN _jjtInvBuffer;  // 6 x 6
        private RMMatrixMN _identity6;     // 6 x 6
        private float[] _errorVec;         // 6 x 1
        private float[] _tempVecResult;    // 6 x 1
        private float[] _deltaThetaBuf;    // N x 1
        private float[] _thetaBuffer;      // N x 1
        private float[,] _augBuffer6;      // 6 x 12

        // 加权 DLS 缓冲区（Phase 1）
        private float[] _wInvBuffer;       // N x 1  逆权重
        private RMMatrixMN _jwBuffer6;     // 6 x N  Jw = J * diag(wInv)

        // SolvePositionOnly 专用缓冲区
        private RMMatrixMN _jpBuffer;      // 3 x N
        private RMMatrixMN _jpTBuffer;     // N x 3
        private RMMatrixMN _jpJpTBuffer;   // 3 x 3
        private RMMatrixMN _jpJpTDamped;   // 3 x 3
        private RMMatrixMN _dampingBuffer3;// 3 x 3
        private RMMatrixMN _jpJpTInvBuf;   // 3 x 3
        private RMMatrixMN _identity3;     // 3 x 3
        private float[] _posErrorVec;      // 3 x 1
        private float[] _tempVecResult3;   // 3 x 1
        private float[] _deltaThetaBuf3;   // N x 1
        private float[,] _augBuffer3;      // 3 x 6
        private RMMatrixMN _jwpBuffer;     // 3 x N  加权位置雅可比

        // 零空间缓冲区（Phase 2）
        private float[] _q0Buffer;         // N x 1  中位回归偏好向量
        private float[] _nullCorrBuf;      // N x 1  零空间修正量

        // 多解求解器池（Phase 4）
        private IKSolver[] _solverPool;
        private RobotArmModel[] _modelPool; // 每个线程独立 model 副本
        private int _solverPoolSize;

        // 随机数用于多解初始化
        private static readonly object _rngLock = new object();
        private static readonly System.Random _rng = new System.Random();

        private int _preparedDOF;

        public IKSolver()
        {
            Config = new IKSolverConfig();
        }

        public IKSolver(IKSolverConfig config)
        {
            Config = config ?? new IKSolverConfig();
        }

        /// <summary>
        /// 预分配内部缓冲区（按 DOF 维度）。
        /// 首次调用 Solve 时自动触发，DOF 变化时自动重新分配。
        /// </summary>
        private void Prepare(int dof)
        {
            if (_preparedDOF == dof && _jBuffer != null)
                return;

            _preparedDOF = dof;

            // 全 6D 模式缓冲区
            _jBuffer = new RMMatrixMN(6, dof);
            _jtBuffer = new RMMatrixMN(dof, 6);
            _jjtBuffer = new RMMatrixMN(6, 6);
            _jjtDamped = new RMMatrixMN(6, 6);
            _dampingBuffer = new RMMatrixMN(6, 6);
            _jjtInvBuffer = new RMMatrixMN(6, 6);
            _identity6 = RMMatrixMN.Identity(6);
            _errorVec = new float[6];
            _tempVecResult = new float[6];
            _deltaThetaBuf = new float[dof];
            _thetaBuffer = new float[dof];
            _augBuffer6 = new float[6, 12];

            // 加权 DLS 缓冲区
            _wInvBuffer = new float[dof];
            _jwBuffer6 = new RMMatrixMN(6, dof);

            // 仅位置模式缓冲区
            _jpBuffer = new RMMatrixMN(3, dof);
            _jpTBuffer = new RMMatrixMN(dof, 3);
            _jpJpTBuffer = new RMMatrixMN(3, 3);
            _jpJpTDamped = new RMMatrixMN(3, 3);
            _dampingBuffer3 = new RMMatrixMN(3, 3);
            _jpJpTInvBuf = new RMMatrixMN(3, 3);
            _identity3 = RMMatrixMN.Identity(3);
            _posErrorVec = new float[3];
            _tempVecResult3 = new float[3];
            _deltaThetaBuf3 = new float[dof];
            _augBuffer3 = new float[3, 6];
            _jwpBuffer = new RMMatrixMN(3, dof);

            // 零空间缓冲区
            _q0Buffer = new float[dof];
            _nullCorrBuf = new float[dof];
        }

        // ===== Phase 1 & 3: 加权 DLS 全 6D 求解 =====

        /// <summary>
        /// 全 6D IK 求解（位置 + 姿态）。
        /// </summary>
        public IKResult Solve(RobotArmModel model,
                              float[] currentJointAngles,
                              RMVector3 targetPosition,
                              RMQuaternion targetRotation)
        {
            return Solve(model, currentJointAngles, targetPosition, targetRotation, IKSolveOptions.Default);
        }

        /// <summary>
        /// 全 6D IK 求解（带锁定关节选项）。
        /// </summary>
        public IKResult Solve(RobotArmModel model,
                              float[] currentJointAngles,
                              RMVector3 targetPosition,
                              RMQuaternion targetRotation,
                              IKSolveOptions options)
        {
            int n = model.DOF;
            Prepare(n);

            // 工作副本
            Array.Copy(currentJointAngles, _thetaBuffer, n);

            // 锁定关节：强制设定初始角度
            if (options.HasLockedJoints)
            {
                for (int j = 0; j < n; j++)
                {
                    if (options.LockedJoints[j])
                        _thetaBuffer[j] = options.LockedAngles[j];
                }
            }

            float lambdaSq = Config.DampingFactor * Config.DampingFactor;
            float wPos = Config.PositionWeight;
            float wRot = Config.RotationWeight;
            float posErr = float.MaxValue;
            float rotErr = float.MaxValue;
            bool converged = false;
            int iter = 0;

            for (iter = 0; iter < Config.MaxIterations; iter++)
            {
                // 1. FK
                var eePose = _fkSolver.Solve(model, _thetaBuffer);
                var eePos = eePose.GetPosition();
                var eeRot = eePose.GetRotation();

                // 2. 位置误差
                var ePosVec = targetPosition - eePos;
                posErr = ePosVec.Magnitude;

                // 3. 旋转误差
                var eRotVec = RMMathUtils.ComputeRotationError(targetRotation, eeRot);
                rotErr = eRotVec.Magnitude;

                // 4. 收敛检查
                if (posErr < Config.PositionThreshold && rotErr < Config.RotationThreshold)
                {
                    converged = true;
                    break;
                }

                // 5. 组合加权误差向量
                _errorVec[0] = ePosVec.X * wPos;
                _errorVec[1] = ePosVec.Y * wPos;
                _errorVec[2] = ePosVec.Z * wPos;
                _errorVec[3] = eRotVec.X * wRot;
                _errorVec[4] = eRotVec.Y * wRot;
                _errorVec[5] = eRotVec.Z * wRot;

                // 6. 雅可比 J (6×N)
                _jacobianCalc.ComputeInto(model, _thetaBuffer, _jBuffer, eePos, skipFK: true);
                ScaleJacobianRows(_jBuffer, wPos, wRot, n);

                // 7. 计算关节空间逆权重
                ComputeWeights(model, _thetaBuffer, options);

                // 8. 加权 DLS: Δθ = Jw^T * (Jw * J^T + λ²I)^{-1} * e
                //    Jw = J * diag(wInv)
                RMMatrixMN.DiagScaleColumnsInto(_jBuffer, _wInvBuffer, _jwBuffer6);

                RMMatrixMN.TransposeInto(_jBuffer, _jtBuffer);             // J^T
                RMMatrixMN.MultiplyInto(_jwBuffer6, _jtBuffer, _jjtBuffer); // Jw * J^T
                RMMatrixMN.ScalarMultiplyInto(_identity6, lambdaSq, _dampingBuffer);
                RMMatrixMN.AddInto(_jjtBuffer, _dampingBuffer, _jjtDamped);

                if (!RMMatrixMN.InverseNxNInto(_jjtDamped, _jjtInvBuffer, _augBuffer6))
                    break;

                RMMatrixMN.MultiplyVectorInto(_jjtInvBuffer, _errorVec, _tempVecResult);
                RMMatrixMN.TransposeMultiplyInto(_jwBuffer6, _tempVecResult, _deltaThetaBuf);

                // 9. 更新关节角度
                for (int j = 0; j < n; j++)
                    _thetaBuffer[j] += _deltaThetaBuf[j];

                // 9b. 锁定关节：强制还原
                if (options.HasLockedJoints)
                {
                    for (int j = 0; j < n; j++)
                    {
                        if (options.LockedJoints[j])
                            _thetaBuffer[j] = options.LockedAngles[j];
                    }
                }

                // 10. NaN 守卫
                if (ContainsNaN(_thetaBuffer))
                {
                    Array.Copy(currentJointAngles, _thetaBuffer, n);
                    break;
                }

                // 11. Clamp
                ClampToLimits(model, _thetaBuffer);
            }

            NormalizeAngles(model, _thetaBuffer);

            var resultAngles = new float[n];
            Array.Copy(_thetaBuffer, resultAngles, n);

            // 计算解质量
            JointWeightCalculator.ComputeSolutionTierAndScore(
                model, resultAngles, Config.GoodThreshold, out int tier, out float score);

            return new IKResult
            {
                JointAngles = resultAngles,
                Converged = converged,
                PositionError = posErr,
                RotationError = rotErr,
                IterationsUsed = iter,
                SolutionTier = tier,
                SolutionScore = score
            };
        }

        // ===== Phase 1, 2 & 3: 加权 DLS 仅位置求解 + 零空间中位回归 =====

        /// <summary>
        /// 仅位置 3D IK 求解。
        /// </summary>
        public IKResult SolvePositionOnly(RobotArmModel model,
                                           float[] currentJointAngles,
                                           RMVector3 targetPosition)
        {
            return SolvePositionOnly(model, currentJointAngles, targetPosition, IKSolveOptions.Default);
        }

        /// <summary>
        /// 仅位置 3D IK 求解（带锁定关节选项）。
        /// 使用 3×N 位置雅可比，对 6-DOF 臂留出 3 个冗余自由度。
        /// 当 NullSpaceMidRangeGain > 0 时，利用零空间将冗余自由度推向关节中位。
        /// </summary>
        public IKResult SolvePositionOnly(RobotArmModel model,
                                           float[] currentJointAngles,
                                           RMVector3 targetPosition,
                                           IKSolveOptions options)
        {
            int n = model.DOF;
            Prepare(n);

            Array.Copy(currentJointAngles, _thetaBuffer, n);

            // 锁定关节
            if (options.HasLockedJoints)
            {
                for (int j = 0; j < n; j++)
                {
                    if (options.LockedJoints[j])
                        _thetaBuffer[j] = options.LockedAngles[j];
                }
            }

            float lambdaSq = Config.DampingFactor * Config.DampingFactor;
            float nullGain = Config.NullSpaceMidRangeGain;
            float posErr = float.MaxValue;
            bool converged = false;
            int iter = 0;

            for (iter = 0; iter < Config.MaxIterations; iter++)
            {
                // 1. FK
                var eePose = _fkSolver.Solve(model, _thetaBuffer);
                var eePos = eePose.GetPosition();

                // 2. 位置误差
                var ePosVec = targetPosition - eePos;
                posErr = ePosVec.Magnitude;

                if (posErr < Config.PositionThreshold)
                {
                    converged = true;
                    break;
                }

                _posErrorVec[0] = ePosVec.X;
                _posErrorVec[1] = ePosVec.Y;
                _posErrorVec[2] = ePosVec.Z;

                // 3. 雅可比，提取位置行 → 3×N
                _jacobianCalc.ComputeInto(model, _thetaBuffer, _jBuffer, eePos, skipFK: true);
                _jBuffer.SubRowsInto(0, 3, _jpBuffer);

                // 4. 计算关节空间逆权重
                ComputeWeights(model, _thetaBuffer, options);

                // 5. 加权 DLS: Δθ = Jwp^T * (Jwp * Jp^T + λ²I₃)⁻¹ * e_pos
                RMMatrixMN.DiagScaleColumnsInto(_jpBuffer, _wInvBuffer, _jwpBuffer);

                RMMatrixMN.TransposeInto(_jpBuffer, _jpTBuffer);              // Jp^T
                RMMatrixMN.MultiplyInto(_jwpBuffer, _jpTBuffer, _jpJpTBuffer); // Jwp * Jp^T
                RMMatrixMN.ScalarMultiplyInto(_identity3, lambdaSq, _dampingBuffer3);
                RMMatrixMN.AddInto(_jpJpTBuffer, _dampingBuffer3, _jpJpTDamped);

                if (!RMMatrixMN.InverseNxNInto(_jpJpTDamped, _jpJpTInvBuf, _augBuffer3))
                    break;

                RMMatrixMN.MultiplyVectorInto(_jpJpTInvBuf, _posErrorVec, _tempVecResult3);
                RMMatrixMN.TransposeMultiplyInto(_jwpBuffer, _tempVecResult3, _deltaThetaBuf3);

                // 6. 零空间中位回归（Phase 2）
                //    利用冗余自由度将关节推向中位，不影响 TCP 位置精度。
                //    Δθ_null = (I - Jwp^T * (Jwp*Jp^T+λ²I)^{-1} * Jp) * q0
                //    = q0 - Jwp^T * ((Jwp*Jp^T+λ²I)^{-1} * (Jp * q0))
                //    复用已计算的逆矩阵 _jpJpTInvBuf，零额外矩阵求逆开销。
                if (nullGain > 0f)
                {
                    // 构建偏好向量 q0
                    for (int j = 0; j < n; j++)
                    {
                        var cfg = model.JointConfigs[j];
                        if (options.HasLockedJoints && options.LockedJoints[j])
                        {
                            _q0Buffer[j] = 0f; // 锁定关节不参与零空间
                        }
                        else if (cfg.HasLimits)
                        {
                            float mid = (cfg.MinLimit + cfg.MaxLimit) * 0.5f;
                            _q0Buffer[j] = nullGain * (mid - _thetaBuffer[j]);
                        }
                        else
                        {
                            _q0Buffer[j] = 0f; // 无限位关节，无中位目标
                        }
                    }

                    // v = Jp * q0 (3×1)，复用 _posErrorVec 作临时存储
                    RMMatrixMN.MultiplyVectorInto(_jpBuffer, _q0Buffer, _posErrorVec);

                    // u = (Jwp*Jp^T+λ²I)^{-1} * v (3×1)，复用 _tempVecResult3
                    RMMatrixMN.MultiplyVectorInto(_jpJpTInvBuf, _posErrorVec, _tempVecResult3);

                    // corr = Jwp^T * u (N×1)，使用 _nullCorrBuf
                    RMMatrixMN.TransposeMultiplyInto(_jwpBuffer, _tempVecResult3, _nullCorrBuf);

                    // Δθ_total = Δθ_main + (q0 - corr)
                    for (int j = 0; j < n; j++)
                        _deltaThetaBuf3[j] += _q0Buffer[j] - _nullCorrBuf[j];
                }

                // 7. 更新关节角度
                for (int j = 0; j < n; j++)
                    _thetaBuffer[j] += _deltaThetaBuf3[j];

                // 7b. 锁定关节还原
                if (options.HasLockedJoints)
                {
                    for (int j = 0; j < n; j++)
                    {
                        if (options.LockedJoints[j])
                            _thetaBuffer[j] = options.LockedAngles[j];
                    }
                }

                if (ContainsNaN(_thetaBuffer))
                {
                    Array.Copy(currentJointAngles, _thetaBuffer, n);
                    break;
                }

                ClampToLimits(model, _thetaBuffer);
            }

            NormalizeAngles(model, _thetaBuffer);

            var resultAngles = new float[n];
            Array.Copy(_thetaBuffer, resultAngles, n);

            JointWeightCalculator.ComputeSolutionTierAndScore(
                model, resultAngles, Config.GoodThreshold, out int tier, out float score);

            return new IKResult
            {
                JointAngles = resultAngles,
                Converged = converged,
                PositionError = posErr,
                RotationError = 0f,
                IterationsUsed = iter,
                SolutionTier = tier,
                SolutionScore = score
            };
        }

        // ===== Phase 4: 并行多解搜索 =====

        /// <summary>
        /// 并行多解搜索（全 6D 模式）。
        /// 同时运行 trials 个不同初始解的 IK 求解，
        /// 按四级梯队评分自动选取最优解。
        /// </summary>
        /// <param name="model">机械臂模型。</param>
        /// <param name="currentJointAngles">当前关节角度。</param>
        /// <param name="targetPosition">目标位置。</param>
        /// <param name="targetRotation">目标姿态。</param>
        /// <param name="trials">并行试验数（1 = 传统单解）。</param>
        /// <param name="allResults">输出所有试验结果（可为 null，不返回）。</param>
        /// <returns>最优解。</returns>
        public IKResult SolveMulti(RobotArmModel model,
                                    float[] currentJointAngles,
                                    RMVector3 targetPosition,
                                    RMQuaternion targetRotation,
                                    int trials,
                                    out IKResult[] allResults)
        {
            return SolveMulti(model, currentJointAngles, targetPosition, targetRotation,
                              trials, IKSolveOptions.Default, out allResults);
        }

        /// <summary>
        /// 并行多解搜索（全 6D 模式，带锁定关节选项）。
        /// </summary>
        public IKResult SolveMulti(RobotArmModel model,
                                    float[] currentJointAngles,
                                    RMVector3 targetPosition,
                                    RMQuaternion targetRotation,
                                    int trials,
                                    IKSolveOptions options,
                                    out IKResult[] allResults)
        {
            if (trials <= 1)
            {
                var single = Solve(model, currentJointAngles, targetPosition, targetRotation, options);
                allResults = new IKResult[] { single };
                return single;
            }

            EnsureSolverPool(trials, model);
            int n = model.DOF;
            var results = new IKResult[trials];
            var initAngles = GenerateTrialAngles(model, currentJointAngles, trials);

            Parallel.For(0, trials, i =>
            {
                _solverPool[i].Config = Config;
                results[i] = _solverPool[i].Solve(_modelPool[i], initAngles[i],
                    targetPosition, targetRotation, options);
            });

            allResults = results;
            return SelectBest(results, currentJointAngles, model);
        }

        /// <summary>
        /// 并行多解搜索（仅位置模式）。
        /// </summary>
        public IKResult SolvePositionOnlyMulti(RobotArmModel model,
                                                float[] currentJointAngles,
                                                RMVector3 targetPosition,
                                                int trials,
                                                out IKResult[] allResults)
        {
            return SolvePositionOnlyMulti(model, currentJointAngles, targetPosition,
                                           trials, IKSolveOptions.Default, out allResults);
        }

        /// <summary>
        /// 并行多解搜索（仅位置模式，带锁定关节选项）。
        /// </summary>
        public IKResult SolvePositionOnlyMulti(RobotArmModel model,
                                                float[] currentJointAngles,
                                                RMVector3 targetPosition,
                                                int trials,
                                                IKSolveOptions options,
                                                out IKResult[] allResults)
        {
            if (trials <= 1)
            {
                var single = SolvePositionOnly(model, currentJointAngles, targetPosition, options);
                allResults = new IKResult[] { single };
                return single;
            }

            EnsureSolverPool(trials, model);
            int n = model.DOF;
            var results = new IKResult[trials];
            var initAngles = GenerateTrialAngles(model, currentJointAngles, trials);

            Parallel.For(0, trials, i =>
            {
                _solverPool[i].Config = Config;
                results[i] = _solverPool[i].SolvePositionOnly(_modelPool[i], initAngles[i],
                    targetPosition, options);
            });

            allResults = results;
            return SelectBest(results, currentJointAngles, model);
        }

        // ===== 内部方法 =====

        /// <summary>
        /// 计算逆权重（综合加权 DLS 开关和锁定关节）。
        /// </summary>
        private void ComputeWeights(RobotArmModel model, float[] theta, IKSolveOptions options)
        {
            if (Config.EnableWeightedDLS)
            {
                JointWeightCalculator.ComputeInverseWeightsInto(
                    model, theta,
                    Config.SoftLimitActivationZone,
                    Config.SoftLimitMaxPenalty,
                    _wInvBuffer);
            }
            else
            {
                // 未启用加权 DLS 时，所有权重 = 1
                for (int j = 0; j < model.DOF; j++)
                    _wInvBuffer[j] = 1f;
            }

            // 锁定关节
            if (options.HasLockedJoints)
                JointWeightCalculator.ApplyLockedJointWeights(options.LockedJoints, _wInvBuffer);
        }

        /// <summary>
        /// 确保求解器池和模型池大小足够。
        /// 每个池内求解器独立持有缓冲区，每个模型副本独立持有 JointStates，支持并发。
        /// </summary>
        private void EnsureSolverPool(int requiredSize, RobotArmModel sourceModel)
        {
            if (_solverPool != null && _solverPoolSize >= requiredSize)
                return;

            _solverPool = new IKSolver[requiredSize];
            _modelPool = new RobotArmModel[requiredSize];
            for (int i = 0; i < requiredSize; i++)
            {
                _solverPool[i] = new IKSolver(Config);
                _modelPool[i] = sourceModel.CloneForParallel();
            }
            _solverPoolSize = requiredSize;
        }

        /// <summary>
        /// 生成多解试验的初始角度集。
        /// index 0 = 当前角度（保持当前姿态），index 1..N-1 = 随机初始角度。
        /// </summary>
        private static float[][] GenerateTrialAngles(RobotArmModel model,
                                                      float[] currentAngles,
                                                      int trials)
        {
            int n = model.DOF;
            var result = new float[trials][];

            // Trial 0: 当前角度
            result[0] = new float[n];
            Array.Copy(currentAngles, result[0], n);

            // Trial 1..N-1: 随机初始角度
            for (int t = 1; t < trials; t++)
            {
                var angles = new float[n];
                lock (_rngLock)
                {
                    for (int j = 0; j < n; j++)
                    {
                        var cfg = model.JointConfigs[j];
                        if (cfg.HasLimits)
                        {
                            angles[j] = cfg.MinLimit + (float)_rng.NextDouble() * (cfg.MaxLimit - cfg.MinLimit);
                        }
                        else
                        {
                            // 无限位关节，在当前角度 ±π 范围内随机
                            angles[j] = currentAngles[j] + (float)(_rng.NextDouble() * 2.0 - 1.0) * 3.14159f;
                        }
                    }
                }
                result[t] = angles;
            }

            return result;
        }

        /// <summary>
        /// 综合评分选取最优解。
        /// 
        /// 步骤：
        ///   1. 预筛选：若任一关节角度差 > 2 * MaxJointSpeedForFilter，排除该解
        ///   2. 缓动系数 b = 1 / (1 + rotationCost)，rotationCost = Σ(|Δθ_j| / range_j)
        ///   3. 相对解优度系数 c = 梯队映射 (T1→0.4, T2→0.3, T3→0.2, T4→0.1)
        ///   4. 综合评分 = a * b + (1-a) * c，a = EasingPriorityCoefficient
        ///   5. 选综合评分最高的收敛解；全未收敛则选位置误差最小的
        /// </summary>
        private IKResult SelectBest(IKResult[] results, float[] currentAngles, RobotArmModel model)
        {
            int n = model.DOF;
            float a = Config.EasingPriorityCoefficient;
            float speedLimit = Config.MaxJointSpeedForFilter;
            float speedThreshold = speedLimit > 0f ? speedLimit * 2f : float.MaxValue;
            float d = Config.TierDecayCoefficient;

            IKResult best = null;
            IKResult bestUnconverged = null;

            for (int i = 0; i < results.Length; i++)
            {
                var r = results[i];
                if (r == null || r.JointAngles == null) continue;

                // 1. 速度预筛选：任一关节角度差 > 2 * maxSpeed 则排除
                bool rejected = false;
                float rotationCost = 0f;
                for (int j = 0; j < n; j++)
                {
                    float diff = System.Math.Abs(r.JointAngles[j] - currentAngles[j]);
                    if (diff > speedThreshold)
                    {
                        rejected = true;
                        break;
                    }
                    var cfg = model.JointConfigs[j];
                    float range = cfg.HasLimits ? (cfg.MaxLimit - cfg.MinLimit) : 6.2832f;
                    if (range > 1e-6f)
                        rotationCost += diff / range;
                }

                if (rejected) continue;

                // 2. 缓动系数 b：转动越少越高
                float b = 1f / (1f + rotationCost);

                // 3. 相对解优度系数 c = d^(tier-1)：T1=1, T2=d, T3=d², T4=d³
                int tier = r.SolutionTier < 1 ? 1 : r.SolutionTier;
                float c = 1f;
                for (int k = 1; k < tier; k++)
                    c *= d;

                // 4. 综合评分
                float composite = a * b + (1f - a) * c;
                r.EasingCoefficient = b;
                r.CompositeScore = composite;

                if (r.Converged)
                {
                    if (best == null || composite > best.CompositeScore)
                        best = r;
                }
                else
                {
                    if (bestUnconverged == null || r.PositionError < bestUnconverged.PositionError)
                        bestUnconverged = r;
                }
            }

            return best ?? bestUnconverged ?? results[0];
        }

        private static void ClampToLimits(RobotArmModel model, float[] theta)
        {
            for (int i = 0; i < model.DOF; i++)
            {
                var cfg = model.JointConfigs[i];
                if (cfg.HasLimits)
                {
                    if (theta[i] < cfg.MinLimit)
                        theta[i] = cfg.MinLimit;
                    else if (theta[i] > cfg.MaxLimit)
                        theta[i] = cfg.MaxLimit;
                }
            }
        }

        private static void NormalizeAngles(RobotArmModel model, float[] theta)
        {
            for (int i = 0; i < model.DOF; i++)
            {
                if (model.JointConfigs[i].Type == JointType.Revolute &&
                    !model.JointConfigs[i].HasLimits)
                {
                    theta[i] = RMMathUtils.NormalizeAngle(theta[i]);
                }
            }
        }

        private static bool ContainsNaN(float[] arr)
        {
            for (int i = 0; i < arr.Length; i++)
            {
                if (float.IsNaN(arr[i]) || float.IsInfinity(arr[i]))
                    return true;
            }
            return false;
        }

        /// <summary>
        /// 对 6×N 雅可比矩阵的行施加任务空间权重。
        /// </summary>
        private static void ScaleJacobianRows(RMMatrixMN j, float wPos, float wRot, int cols)
        {
            for (int c = 0; c < cols; c++)
            {
                j[0, c] *= wPos;
                j[1, c] *= wPos;
                j[2, c] *= wPos;
                j[3, c] *= wRot;
                j[4, c] *= wRot;
                j[5, c] *= wRot;
            }
        }
    }
}
