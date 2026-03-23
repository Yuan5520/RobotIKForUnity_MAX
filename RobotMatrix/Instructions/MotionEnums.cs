namespace RobotMatrix.Instructions
{
    /// <summary>
    /// 运动插值模式，对应工业机器人 MoveJ / MoveL 指令。
    /// </summary>
    public enum MotionInterpolationMode
    {
        /// <summary>MoveJ：关节空间插值，TCP 路径不可控但速度快。</summary>
        MoveJ,

        /// <summary>MoveL：笛卡尔直线插值，TCP 沿直线运动。</summary>
        MoveL
    }

    /// <summary>
    /// MoveL 路径上 IK 求解失败时的处理策略。
    /// </summary>
    public enum IKFailureStrategy
    {
        /// <summary>停止运动并触发事件通知上层。</summary>
        StopAndReport,

        /// <summary>自动回退到关节空间插值（MoveJ）完成剩余运动。</summary>
        FallbackToPTP
    }
}
