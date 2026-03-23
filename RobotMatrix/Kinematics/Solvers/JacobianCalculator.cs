using RobotMatrix.Math;

namespace RobotMatrix.Kinematics
{
    /// <summary>
    /// 几何法雅可比矩阵计算器。
    /// 输出 6xN 矩阵（6 = 任务空间维度：3 位置 + 3 姿态，N = 关节数）。
    /// </summary>
    public class JacobianCalculator
    {
        private readonly FKSolver _fkSolver = new FKSolver();

        // 预分配列向量缓冲区
        private RMVector3[] _jPosColumns;
        private RMVector3[] _jRotColumns;
        private int _preparedDOF;

        private void Prepare(int dof)
        {
            if (_preparedDOF == dof && _jPosColumns != null)
                return;
            _preparedDOF = dof;
            _jPosColumns = new RMVector3[dof];
            _jRotColumns = new RMVector3[dof];
        }

        /// <summary>
        /// 计算几何雅可比矩阵。
        /// </summary>
        public RMMatrixMN Compute(RobotArmModel model, float[] jointAngles)
        {
            int n = model.DOF;
            Prepare(n);

            var eePose = _fkSolver.Solve(model, jointAngles);
            var eePos = eePose.GetPosition();

            ComputeColumns(model, eePos, n);
            return RMMatrixMN.FromColumnVectors(_jPosColumns, _jRotColumns);
        }

        /// <summary>
        /// 计算几何雅可比矩阵并写入预分配目标（零 GC 版本）。
        /// </summary>
        /// <param name="skipFK">为 true 时跳过内部 FK 调用（调用方已确保 JointStates 最新）。</param>
        public void ComputeInto(RobotArmModel model, float[] jointAngles,
                                RMMatrixMN target, RMVector3 eePos, bool skipFK = false)
        {
            int n = model.DOF;
            Prepare(n);

            if (!skipFK)
            {
                var eePose = _fkSolver.Solve(model, jointAngles);
                eePos = eePose.GetPosition();
            }

            ComputeColumns(model, eePos, n);
            RMMatrixMN.FromColumnVectorsInto(_jPosColumns, _jRotColumns, target);
        }

        private void ComputeColumns(RobotArmModel model, RMVector3 eePos, int n)
        {
            for (int i = 0; i < n; i++)
            {
                var Ti = model.JointStates[i].WorldTransform;
                var zi = new RMVector3(Ti[0, 2], Ti[1, 2], Ti[2, 2]);
                var pi = new RMVector3(Ti[0, 3], Ti[1, 3], Ti[2, 3]);

                if (model.JointConfigs[i].Type == JointType.Revolute)
                {
                    _jPosColumns[i] = RMVector3.Cross(zi, eePos - pi);
                    _jRotColumns[i] = zi;
                }
                else
                {
                    _jPosColumns[i] = zi;
                    _jRotColumns[i] = RMVector3.Zero;
                }
            }
        }
    }
}
