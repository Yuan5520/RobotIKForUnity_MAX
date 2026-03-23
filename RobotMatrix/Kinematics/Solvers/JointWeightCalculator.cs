namespace RobotMatrix.Kinematics
{
    /// <summary>
    /// 关节空间权重计算器（静态工具类，零 GC 分配）。
    /// 提供：
    ///   1. 动态逆权重计算（软限位惩罚）
    ///   2. 锁定关节权重覆盖
    ///   3. 四级梯队解质量评估
    /// </summary>
    public static class JointWeightCalculator
    {
        /// <summary>
        /// 计算各关节的逆权重 wInv[j] = 1/w[j]，写入预分配数组。
        /// 当关节远离限位时 wInv≈1（权重≈1），
        /// 靠近限位时 wInv→0（权重→∞），抑制该关节运动。
        /// 
        /// 权重公式：w = 1 + maxPenalty * (1 - d_near/buffer)²
        ///   d_near = min(theta - minLimit, maxLimit - theta)
        ///   buffer = range * activationZone
        /// </summary>
        /// <param name="model">机械臂模型（读取关节限位）。</param>
        /// <param name="theta">当前关节角度数组。</param>
        /// <param name="activationZone">激活区占行程比例（0~0.5）。</param>
        /// <param name="maxPenalty">到达限位时的最大惩罚倍数。</param>
        /// <param name="wInvOut">输出逆权重数组 [N]。</param>
        public static void ComputeInverseWeightsInto(
            RobotArmModel model,
            float[] theta,
            float activationZone,
            float maxPenalty,
            float[] wInvOut)
        {
            int n = model.DOF;
            for (int j = 0; j < n; j++)
            {
                var cfg = model.JointConfigs[j];
                if (!cfg.HasLimits)
                {
                    // 无限位关节，权重恒为 1
                    wInvOut[j] = 1f;
                    continue;
                }

                float range = cfg.MaxLimit - cfg.MinLimit;
                if (range < 1e-8f)
                {
                    // 行程接近零，视为锁死
                    wInvOut[j] = 0f;
                    continue;
                }

                float buffer = range * activationZone;
                if (buffer < 1e-8f)
                {
                    wInvOut[j] = 1f;
                    continue;
                }

                float dLow = theta[j] - cfg.MinLimit;
                float dHigh = cfg.MaxLimit - theta[j];
                float dNear = dLow < dHigh ? dLow : dHigh;

                if (dNear >= buffer)
                {
                    // 安全区内，权重 = 1
                    wInvOut[j] = 1f;
                }
                else
                {
                    // 激活区内：二次惩罚
                    if (dNear < 0f) dNear = 0f;
                    float t = 1f - dNear / buffer; // t ∈ [0,1]，0=缓冲边界，1=限位
                    float w = 1f + maxPenalty * t * t;
                    wInvOut[j] = 1f / w;
                }
            }
        }

        /// <summary>
        /// 对锁定关节强制设置 wInv = 0（等效权重 = ∞），
        /// 使 DLS 自然不产生该关节的增量，无需改变矩阵维度。
        /// </summary>
        /// <param name="lockedMask">锁定掩码（null 或 bool[N]，true = 锁定）。</param>
        /// <param name="wInvOut">逆权重数组，将被就地修改。</param>
        public static void ApplyLockedJointWeights(bool[] lockedMask, float[] wInvOut)
        {
            if (lockedMask == null) return;
            int n = lockedMask.Length < wInvOut.Length ? lockedMask.Length : wInvOut.Length;
            for (int j = 0; j < n; j++)
            {
                if (lockedMask[j])
                    wInvOut[j] = 0f;
            }
        }

        /// <summary>
        /// 评估一组关节角度的解质量：计算梯队（Tier 1~4）和梯队内得分。
        /// 
        /// limitFactor[j] = d_near[j] / halfRange[j]，范围 [0,1]：
        ///   1.0 = 关节在正中，0.0 = 关节在限位。
        /// 
        /// 梯队定义：
        ///   Tier 1: 所有关节 limitFactor >= goodThreshold（全安全）
        ///   Tier 2: &lt;50% 关节低于 goodThreshold
        ///   Tier 3: >=50% 关节低于 goodThreshold
        ///   Tier 4: 任一关节 limitFactor ≈ 0（触碰硬限位）
        /// 
        /// 梯队内得分 = mean(limitFactor)，越高越好。
        /// </summary>
        /// <param name="model">机械臂模型。</param>
        /// <param name="theta">关节角度数组。</param>
        /// <param name="goodThreshold">安全阈值（0~1），低于此值视为接近限位。</param>
        /// <param name="tier">输出梯队等级（1~4）。</param>
        /// <param name="score">输出梯队内得分（0~1，越高越好）。</param>
        public static void ComputeSolutionTierAndScore(
            RobotArmModel model,
            float[] theta,
            float goodThreshold,
            out int tier,
            out float score)
        {
            int n = model.DOF;
            float sum = 0f;
            int nearLimitCount = 0;
            bool anyAtHardLimit = false;
            int limitedJointCount = 0; // 有限位的关节数

            const float hardLimitEps = 1e-4f;

            for (int j = 0; j < n; j++)
            {
                var cfg = model.JointConfigs[j];
                if (!cfg.HasLimits)
                {
                    sum += 1f; // 无限位关节视为完美
                    continue;
                }

                limitedJointCount++;

                float range = cfg.MaxLimit - cfg.MinLimit;
                float halfRange = range * 0.5f;

                if (halfRange < 1e-8f)
                {
                    // 行程为零
                    anyAtHardLimit = true;
                    nearLimitCount++;
                    continue;
                }

                float dLow = theta[j] - cfg.MinLimit;
                float dHigh = cfg.MaxLimit - theta[j];
                float dNear = dLow < dHigh ? dLow : dHigh;
                if (dNear < 0f) dNear = 0f;

                float limitFactor = dNear / halfRange;
                if (limitFactor > 1f) limitFactor = 1f;

                sum += limitFactor;

                if (dNear < hardLimitEps)
                    anyAtHardLimit = true;

                if (limitFactor < goodThreshold)
                    nearLimitCount++;
            }

            score = n > 0 ? sum / n : 1f;

            // 梯队判定
            if (anyAtHardLimit)
            {
                tier = 4;
            }
            else if (limitedJointCount == 0)
            {
                tier = 1; // 无限位关节，全安全
            }
            else if (nearLimitCount == 0)
            {
                tier = 1;
            }
            else if (nearLimitCount < limitedJointCount * 0.5f)
            {
                tier = 2;
            }
            else
            {
                tier = 3;
            }
        }

        /// <summary>
        /// 计算各关节的 limitFactor（归一化安全距离），写入预分配数组。
        /// limitFactor = d_near / halfRange，1=中位，0=限位。
        /// 无限位关节输出 1.0。
        /// </summary>
        public static void ComputeLimitFactorsInto(
            RobotArmModel model,
            float[] theta,
            float[] limitFactorsOut)
        {
            int n = model.DOF;
            for (int j = 0; j < n; j++)
            {
                var cfg = model.JointConfigs[j];
                if (!cfg.HasLimits)
                {
                    limitFactorsOut[j] = 1f;
                    continue;
                }

                float range = cfg.MaxLimit - cfg.MinLimit;
                float halfRange = range * 0.5f;
                if (halfRange < 1e-8f)
                {
                    limitFactorsOut[j] = 0f;
                    continue;
                }

                float dLow = theta[j] - cfg.MinLimit;
                float dHigh = cfg.MaxLimit - theta[j];
                float dNear = dLow < dHigh ? dLow : dHigh;
                if (dNear < 0f) dNear = 0f;

                float lf = dNear / halfRange;
                limitFactorsOut[j] = lf > 1f ? 1f : lf;
            }
        }
    }
}
