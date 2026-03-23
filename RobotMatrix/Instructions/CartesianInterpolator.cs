using RobotMatrix.Math;

namespace RobotMatrix.Instructions
{
    /// <summary>
    /// 笛卡尔直线插值器（MoveL 指令核心）。
    /// 在起始位姿与目标位姿之间按速度约束线性插值，每帧产出一个中间路点。
    /// 引擎无关的纯算法实现，与 JointInterpolator 平行设计。
    /// </summary>
    public class CartesianInterpolator
    {
        private RMVector3 _startPos;
        private RMQuaternion _startRot;
        private RMVector3 _endPos;
        private RMQuaternion _endRot;

        private float _totalDuration;   // 总运动时间（秒）
        private float _elapsed;         // 已经过时间（秒）
        private bool _hasTarget;

        private float _maxLinearSpeed;   // TCP 最大线速度（米/秒）
        private float _maxAngularSpeed;  // TCP 最大角速度（弧度/秒）

        /// <summary>是否有活跃轨迹。</summary>
        public bool HasTarget => _hasTarget;

        /// <summary>当前插值进度（0~1）。</summary>
        public float Progress => _hasTarget && _totalDuration > 0f
            ? RMMathUtils.Clamp(_elapsed / _totalDuration, 0f, 1f)
            : 0f;

        /// <summary>TCP 最大线速度（米/秒）。</summary>
        public float MaxLinearSpeed
        {
            get => _maxLinearSpeed;
            set => _maxLinearSpeed = value > 0f ? value : _maxLinearSpeed;
        }

        /// <summary>TCP 最大角速度（弧度/秒）。</summary>
        public float MaxAngularSpeed
        {
            get => _maxAngularSpeed;
            set => _maxAngularSpeed = value > 0f ? value : _maxAngularSpeed;
        }

        public CartesianInterpolator()
        {
            _maxLinearSpeed = 0.5f;                        // 默认 0.5 m/s
            _maxAngularSpeed = 90f * RMMathUtils.Deg2Rad;  // 默认 90°/s
        }

        /// <summary>
        /// 启动一段笛卡尔直线轨迹。
        /// </summary>
        /// <param name="startPos">起始位置（DH 基坐标系）。</param>
        /// <param name="startRot">起始姿态。</param>
        /// <param name="endPos">目标位置。</param>
        /// <param name="endRot">目标姿态。</param>
        public void Start(RMVector3 startPos, RMQuaternion startRot,
                          RMVector3 endPos, RMQuaternion endRot)
        {
            _startPos = startPos;
            _startRot = startRot;
            _endPos = endPos;
            _endRot = endRot;

            float linearDist = RMVector3.Distance(startPos, endPos);
            float angularDist = RMQuaternion.Angle(startRot, endRot); // 弧度

            // 总时间取线速度和角速度约束中较大者
            float tLinear = _maxLinearSpeed > 1e-9f ? linearDist / _maxLinearSpeed : 0f;
            float tAngular = _maxAngularSpeed > 1e-9f ? angularDist / _maxAngularSpeed : 0f;
            _totalDuration = tLinear > tAngular ? tLinear : tAngular;

            if (_totalDuration < 1e-6f)
            {
                // 起止太近，无需插值
                _hasTarget = false;
                return;
            }

            _elapsed = 0f;
            _hasTarget = true;
        }

        /// <summary>
        /// 每帧推进插值。
        /// </summary>
        /// <param name="deltaTime">帧时间（秒）。</param>
        /// <param name="waypointPos">输出：当前帧路点位置。</param>
        /// <param name="waypointRot">输出：当前帧路点姿态。</param>
        /// <returns>true = 轨迹已完成，false = 仍在进行中。</returns>
        public bool Update(float deltaTime, out RMVector3 waypointPos, out RMQuaternion waypointRot)
        {
            if (!_hasTarget)
            {
                waypointPos = _endPos;
                waypointRot = _endRot;
                return true;
            }

            _elapsed += deltaTime;
            float t = _elapsed / _totalDuration;
            if (t >= 1f)
                t = 1f;

            waypointPos = RMVector3.Lerp(_startPos, _endPos, t);
            waypointRot = RMQuaternion.Slerp(_startRot, _endRot, t);

            if (t >= 1f)
            {
                _hasTarget = false;
                return true;
            }

            return false;
        }

        /// <summary>
        /// 重置插值状态，清除轨迹。
        /// </summary>
        public void Reset()
        {
            _hasTarget = false;
            _elapsed = 0f;
        }
    }
}
