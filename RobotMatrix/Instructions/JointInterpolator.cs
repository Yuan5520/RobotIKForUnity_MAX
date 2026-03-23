using RobotMatrix.Math;

namespace RobotMatrix.Instructions
{
    /// <summary>
    /// 关节角速度限制器（MoveJ 指令核心）。
    /// 限制每个关节每帧的最大角度变化量，防止 IK 求解结果导致的瞬间跳变。
    /// 引擎无关的纯算法实现。
    /// </summary>
    public class JointInterpolator
    {
        private float _maxAngularVelocityRad;
        private float[] _targetAngles;
        private bool _hasTarget;
        private int _dof;

        /// <summary>是否有待插值的目标。</summary>
        public bool HasTarget => _hasTarget;

        /// <summary>
        /// 最大角速度（弧度/秒）。
        /// </summary>
        public float MaxAngularVelocityRad
        {
            get => _maxAngularVelocityRad;
            set => _maxAngularVelocityRad = value > 0f ? value : _maxAngularVelocityRad;
        }

        public JointInterpolator()
        {
            // 默认最大角速度 180°/s
            _maxAngularVelocityRad = 180f * RMMathUtils.Deg2Rad;
        }

        /// <summary>
        /// 初始化或重新分配内部缓冲区。
        /// </summary>
        public void Initialize(int dof)
        {
            _dof = dof;
            _targetAngles = new float[dof];
            _hasTarget = false;
        }

        /// <summary>
        /// 设置新的目标关节角度（来自 IK 求解结果）。
        /// </summary>
        public void SetTarget(float[] targetAngles)
        {
            if (_targetAngles == null || targetAngles.Length != _dof) return;
            System.Array.Copy(targetAngles, _targetAngles, _dof);
            _hasTarget = true;
        }

        /// <summary>
        /// 每帧更新：将 currentAngles 向 targetAngles 移动，限制最大角速度。
        /// 结果写入 output 数组。
        /// 返回 true 表示所有关节已到达目标（已收敛）。
        /// </summary>
        public bool Update(float deltaTime, float[] currentAngles, float[] output)
        {
            if (!_hasTarget || currentAngles == null || output == null)
                return true;

            float maxStep = _maxAngularVelocityRad * deltaTime;
            bool allSettled = true;

            for (int i = 0; i < _dof; i++)
            {
                float diff = _targetAngles[i] - currentAngles[i];
                float absDiff = diff < 0 ? -diff : diff;

                if (absDiff <= maxStep)
                {
                    // 差值在允许步长内，直接到达目标
                    output[i] = _targetAngles[i];
                }
                else
                {
                    // 超出步长，按最大速度趋近
                    output[i] = currentAngles[i] + (diff > 0 ? maxStep : -maxStep);
                    allSettled = false;
                }
            }

            if (allSettled)
                _hasTarget = false;

            return allSettled;
        }

        /// <summary>
        /// 重置插值状态，清除目标。
        /// </summary>
        public void Reset()
        {
            _hasTarget = false;
        }
    }
}
