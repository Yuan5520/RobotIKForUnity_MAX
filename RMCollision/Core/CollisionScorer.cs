using System;
using RobotMatrix.Math;

namespace RMCollision
{
    /// <summary>
    /// 碰撞评分器。
    /// 根据碰撞事件的接触面积、相对速度等数据计算碰撞严重程度评分。
    /// </summary>
    public static class CollisionScorer
    {
        /// <summary>
        /// 计算单次碰撞事件的严重程度评分。
        /// 综合考虑：
        ///   - 碰撞速度（对动态碰撞更敏感）
        ///   - 接触面积（面积越大越严重）
        /// </summary>
        /// <param name="relativeVelocity">碰撞相对速度。</param>
        /// <param name="contactArea">估算接触面积。</param>
        /// <returns>碰撞评分（0~无上限，值越大越严重）。</returns>
        public static float CalculateScore(RMVector3 relativeVelocity, float contactArea)
        {
            float speed = relativeVelocity.Magnitude;
            // 速度权重 70%，面积权重 30%
            float score = speed * 0.7f + contactArea * 0.3f;
            return Math.Max(0f, score);
        }

        /// <summary>
        /// 应用指数衰减更新累积评分。
        /// newAccumulated = oldAccumulated * decay + newEventScore
        /// </summary>
        public static float ApplyDecay(float accumulated, float newScore, float decayRate, float deltaTime)
        {
            float decay = (float)Math.Pow(decayRate, deltaTime);
            return accumulated * decay + newScore;
        }
    }
}
