using System;

namespace RMCollision
{
    /// <summary>
    /// 碰撞检测套件配置。
    /// </summary>
    [Serializable]
    public class CollisionConfig
    {
        /// <summary>是否启用碰撞检测。</summary>
        public bool Enabled = false;

        /// <summary>是否自动为机械臂零件添加网格碰撞体。</summary>
        public bool AutoAddColliders = true;

        /// <summary>是否在 Scene 视图中显示碰撞体边界线框。</summary>
        public bool ShowCollidersGizmo = false;

        /// <summary>
        /// 静态干涉校准时长（秒）。
        /// 启动后的这段时间内记录所有碰撞对作为基线，之后排除这些碰撞对。
        /// </summary>
        public float CalibrationDurationSec = 1f;

        /// <summary>是否将碰撞事件持久化到 JSON 文件。</summary>
        public bool EnablePersistence = true;

        /// <summary>碰撞评分指数衰减率（每秒）。取值 0~1，越接近 1 衰减越慢。</summary>
        public float ScoreDecayRate = 0.95f;
    }
}
