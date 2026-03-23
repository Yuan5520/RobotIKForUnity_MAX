using System;
using RobotMatrix.Math;

namespace RobotMatrix.Kinematics
{
    /// <summary>
    /// 机械臂数学模型（纯数据+算法，引擎无关）。
    /// 持有关节配置、运行时状态和 DH 参数表。
    /// </summary>
    public class RobotArmModel
    {
        public JointConfig[] JointConfigs;
        public JointState[] JointStates;

        /// <summary>自由度数量（= 关节数）。</summary>
        public int DOF;

        /// <summary>末端执行器相对于最后一个关节的固定偏移变换。</summary>
        public RMMatrix4x4 EndEffectorOffset = RMMatrix4x4.Identity;

        /// <summary>
        /// 创建线程安全的浅克隆：共享 JointConfigs（只读），独立 JointStates（读写）。
        /// 用于 Parallel.For 并行 IK 求解，避免多线程写入同一 JointStates。
        /// </summary>
        public RobotArmModel CloneForParallel()
        {
            var clone = new RobotArmModel
            {
                DOF = DOF,
                JointConfigs = JointConfigs, // 只读，可共享
                EndEffectorOffset = EndEffectorOffset,
                JointStates = new JointState[DOF]
            };
            for (int i = 0; i < DOF; i++)
                clone.JointStates[i] = new JointState();
            return clone;
        }

        /// <summary>
        /// 使用预定义的 DH 参数表直接构建模型（用于已知 DH 参数的场景）。
        /// </summary>
        public static RobotArmModel FromDHParameters(DHParameters[] dhParams, JointType[] types = null)
        {
            int n = dhParams.Length;
            var model = new RobotArmModel
            {
                DOF = n,
                JointConfigs = new JointConfig[n],
                JointStates = new JointState[n]
            };

            for (int i = 0; i < n; i++)
            {
                var dh = dhParams[i];
                model.JointConfigs[i] = new JointConfig
                {
                    Type = types != null ? types[i] : JointType.Revolute,
                    DH = dh,
                    AxisDirection = RMVector3.Forward,
                    // 向后兼容：已知 DH 参数时，LinkTransform = DHMatrix(a, α, d, θ_offset)
                    LinkTransform = RMMatrix4x4.DHMatrix(dh.A, dh.Alpha, dh.D, dh.ThetaOffset)
                };
                model.JointStates[i] = new JointState();
            }

            return model;
        }

        /// <summary>
        /// 从关节空间配置自动计算 DH 参数并构建模型。
        /// 适用于从 Unity 场景中采集关节信息的场景。
        /// </summary>
        public void BuildDHFromSpatialConfig(JointSpatialInfo[] spatialInfos, JointConfig[] configs)
        {
            int n = spatialInfos.Length;
            DOF = n;
            JointConfigs = configs;
            JointStates = new JointState[n];

            for (int i = 0; i < n; i++)
                JointStates[i] = new JointState();

            // 计算每个关节的实际坐标系（应用用户微调）
            var positions = new RMVector3[n];
            var zAxes = new RMVector3[n];     // 各关节的 z 轴方向（世界坐标）
            var xAxes = new RMVector3[n];     // 各关节的 x 轴方向（世界坐标）

            for (int i = 0; i < n; i++)
            {
                var info = spatialInfos[i];

                // 应用用户旋转微调
                var adjustedRot = info.WorldRotation;
                if (info.RotationOffset.SqrMagnitude > 1e-12f)
                {
                    var offsetQuat = RMQuaternion.EulerToQuaternion(info.RotationOffset);
                    adjustedRot = info.WorldRotation * offsetQuat;
                }

                // 应用用户平移微调
                var adjustedPos = info.WorldPosition + info.OriginOffset;

                positions[i] = adjustedPos;

                // z 轴 = 关节运动轴在世界坐标下的方向
                var localAxis = info.AxisDirection.SqrMagnitude > 1e-12f
                    ? RMVector3.Normalize(info.AxisDirection)
                    : RMVector3.Forward;
                zAxes[i] = RMVector3.Normalize(RMQuaternion.RotateVector(adjustedRot, localAxis));

                // x 轴暂时留空，后续根据 DH 约定计算
                xAxes[i] = RMVector3.Zero;
            }

            // 按 DH 约定计算每个关节的 x 轴和 DH 参数
            for (int i = 0; i < n; i++)
            {
                if (i == 0)
                {
                    // 第一个关节：x 轴优先使用关节初始 X 方向投影到垂直于 z0 的平面
                    // （Gram-Schmidt），确保零位时校准帧与 Unity 初始帧一致，
                    // 避免零位视觉跳变。退化时回退到任意垂直方向。
                    var rot0 = spatialInfos[0].WorldRotation;
                    if (spatialInfos[0].RotationOffset.SqrMagnitude > 1e-12f)
                    {
                        var offQ = RMQuaternion.EulerToQuaternion(spatialInfos[0].RotationOffset);
                        rot0 = rot0 * offQ;
                    }
                    var initX = RMQuaternion.RotateVector(rot0, RMVector3.Right);
                    var xProj = initX - zAxes[0] * RMVector3.Dot(initX, zAxes[0]);
                    if (xProj.SqrMagnitude > 1e-12f)
                        xAxes[0] = RMVector3.Normalize(xProj);
                    else
                        xAxes[0] = RMGeometryUtils.ChoosePerpendicularAxis(zAxes[0]);
                    // 基座关节的 DH 参数全为零
                    configs[0].DH = new DHParameters(0f, 0f, 0f, 0f);
                }
                else
                {
                    var zi_1 = zAxes[i - 1]; // z_{i-1}
                    var zi = zAxes[i];       // z_i
                    var pi_1 = positions[i - 1];
                    var pi = positions[i];

                    // 计算公法线方向
                    var cross = RMVector3.Cross(zi_1, zi);
                    float crossMag = cross.Magnitude;

                    RMVector3 xi;
                    if (crossMag > 1e-6f)
                    {
                        // 两轴不平行：公法线 = z_{i-1} × z_i
                        xi = cross * (1f / crossMag);
                    }
                    else
                    {
                        // 两轴平行或重合：取连线方向投影到垂直于 z 的平面
                        var diff = pi - pi_1;
                        var projection = diff - zi_1 * RMVector3.Dot(diff, zi_1);
                        float projMag = projection.Magnitude;
                        if (projMag > 1e-6f)
                            xi = projection * (1f / projMag);
                        else
                            xi = RMGeometryUtils.ChoosePerpendicularAxis(zi_1);
                    }

                    xAxes[i] = xi;

                    // ===== 提取 DH 四参数 =====

                    // alpha = 相邻 z 轴的夹角（绕 x_i 轴）
                    float alpha = (float)System.Math.Atan2(
                        RMVector3.Dot(RMVector3.Cross(zi_1, zi), xi),
                        RMVector3.Dot(zi_1, zi));

                    // a = 公垂线距离（沿 x_i 方向，z_{i-1} 到 z_i）
                    var dp = pi - pi_1;
                    float a = RMVector3.Dot(dp, xi);

                    // d = 沿 z_{i-1} 方向的偏距
                    float d = RMVector3.Dot(dp, zi_1);

                    // theta_offset = x_{i-1} 到 x_i 绕 z_{i-1} 的角度
                    float thetaOffset = 0f;
                    if (i > 0 && xAxes[i - 1].SqrMagnitude > 1e-12f)
                    {
                        thetaOffset = (float)System.Math.Atan2(
                            RMVector3.Dot(RMVector3.Cross(xAxes[i - 1], xi), zi_1),
                            RMVector3.Dot(xAxes[i - 1], xi));
                    }

                    configs[i].DH = new DHParameters(a, alpha, d, thetaOffset);
                }
            }

            // ===== 阶段三：构建完整 LinkTransform（精确逐关节变换） =====
            // 为每个关节构建世界 DH 帧矩阵 F_i = [x_i | y_i | z_i | p_i]，
            // 然后 LinkTransform_i = Inv(F_{i-1}) * F_i，
            // 精确描述帧间变换，不丢失任何位移分量。

            var worldFrames = new RMMatrix4x4[n];
            for (int i = 0; i < n; i++)
            {
                var yi = RMVector3.Cross(zAxes[i], xAxes[i]);
                worldFrames[i] = RMGeometryUtils.BuildFrameMatrix(positions[i], xAxes[i], yi, zAxes[i]);
            }

            configs[0].LinkTransform = RMMatrix4x4.Identity;
            for (int i = 1; i < n; i++)
            {
                configs[i].LinkTransform = RMMatrix4x4.Inverse(worldFrames[i - 1]) * worldFrames[i];
            }
        }

        /// <summary>
        /// 运行时重新计算 DH 参数（热更新）。
        /// 需外部重新采集 JointSpatialInfo 后调用。
        /// </summary>
        public void RebuildDHParameters(JointSpatialInfo[] spatialInfos)
        {
            BuildDHFromSpatialConfig(spatialInfos, JointConfigs);
        }

        /// <summary>
        /// 获取所有关节的 DH 参数表。
        /// </summary>
        public DHParameters[] GetDHTable()
        {
            var table = new DHParameters[DOF];
            for (int i = 0; i < DOF; i++)
                table[i] = JointConfigs[i].DH;
            return table;
        }
    }
}
