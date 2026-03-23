using NUnit.Framework;
using RobotMatrix.Math;
using RobotMatrix.Kinematics;

namespace RobotMatrix.Tests
{
    public class RobotArmModelTests
    {
        private const float Epsilon = 1e-4f;
        private static readonly float Pi = RMMathUtils.PI;

        // ===== FromDHParameters 工厂方法 =====

        [Test]
        public void FromDHParameters_CreatesCorrectModel()
        {
            var dhParams = new[]
            {
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(1f, 0f, 0f, 0f)
            };

            var model = RobotArmModel.FromDHParameters(dhParams);

            Assert.AreEqual(2, model.DOF);
            Assert.AreEqual(2, model.JointConfigs.Length);
            Assert.AreEqual(2, model.JointStates.Length);
            Assert.AreEqual(1f, model.JointConfigs[0].DH.A, Epsilon);
            Assert.AreEqual(1f, model.JointConfigs[1].DH.A, Epsilon);
        }

        // ===== BuildDHFromSpatialConfig =====

        [Test]
        public void BuildDH_2R_Planar_CorrectDHParameters()
        {
            // 2R 平面机械臂：两个旋转关节，连杆长度各 1
            // J0 在原点，z 轴向上（0,1,0），x 轴向右（1,0,0）
            // J1 在 (1,0,0)，z 轴向上（0,1,0）
            // 标准 DH: a0=1, alpha0=0, d0=0; a1=1, alpha1=0, d1=0
            // 注: 第一个关节 DH 全零（基座约定）

            var spatialInfos = new[]
            {
                new JointSpatialInfo
                {
                    WorldPosition = new RMVector3(0f, 0f, 0f),
                    WorldRotation = RMQuaternion.Identity,
                    AxisDirection = RMVector3.Up  // 绕 Y 轴旋转
                },
                new JointSpatialInfo
                {
                    WorldPosition = new RMVector3(1f, 0f, 0f),
                    WorldRotation = RMQuaternion.Identity,
                    AxisDirection = RMVector3.Up  // 绕 Y 轴旋转
                }
            };

            var configs = new[]
            {
                new JointConfig { Type = JointType.Revolute },
                new JointConfig { Type = JointType.Revolute }
            };

            var model = new RobotArmModel();
            model.BuildDHFromSpatialConfig(spatialInfos, configs);

            Assert.AreEqual(2, model.DOF);

            // J0: 基座关节，DH 全零
            Assert.AreEqual(0f, configs[0].DH.A, Epsilon, "J0.a");
            Assert.AreEqual(0f, configs[0].DH.Alpha, Epsilon, "J0.alpha");
            Assert.AreEqual(0f, configs[0].DH.D, Epsilon, "J0.d");

            // J1: 两轴平行（都是 Y 轴），a=1（沿 x 方向偏移），alpha=0，d=0
            Assert.AreEqual(1f, configs[1].DH.A, Epsilon, "J1.a");
            Assert.AreEqual(0f, configs[1].DH.Alpha, Epsilon, "J1.alpha");
            Assert.AreEqual(0f, configs[1].DH.D, Epsilon, "J1.d");
        }

        [Test]
        public void BuildDH_3R_ZAxis_StackedVertically()
        {
            // 3 个关节都绕 Z 轴旋转，沿 Z 轴间隔 1 堆叠
            // J0 在 (0,0,0), J1 在 (0,0,1), J2 在 (0,0,2)
            // 所有轴平行（都是 Z），应得到: d=1 (沿 z 偏距), a=0, alpha=0

            var spatialInfos = new[]
            {
                new JointSpatialInfo
                {
                    WorldPosition = new RMVector3(0f, 0f, 0f),
                    WorldRotation = RMQuaternion.Identity,
                    AxisDirection = RMVector3.Forward
                },
                new JointSpatialInfo
                {
                    WorldPosition = new RMVector3(0f, 0f, 1f),
                    WorldRotation = RMQuaternion.Identity,
                    AxisDirection = RMVector3.Forward
                },
                new JointSpatialInfo
                {
                    WorldPosition = new RMVector3(0f, 0f, 2f),
                    WorldRotation = RMQuaternion.Identity,
                    AxisDirection = RMVector3.Forward
                }
            };

            var configs = new[]
            {
                new JointConfig { Type = JointType.Revolute },
                new JointConfig { Type = JointType.Revolute },
                new JointConfig { Type = JointType.Revolute }
            };

            var model = new RobotArmModel();
            model.BuildDHFromSpatialConfig(spatialInfos, configs);

            // J1: d=1, a=0, alpha=0
            Assert.AreEqual(0f, configs[1].DH.A, Epsilon, "J1.a");
            Assert.AreEqual(0f, configs[1].DH.Alpha, Epsilon, "J1.alpha");
            Assert.AreEqual(1f, configs[1].DH.D, Epsilon, "J1.d");

            // J2: d=1, a=0, alpha=0
            Assert.AreEqual(0f, configs[2].DH.A, Epsilon, "J2.a");
            Assert.AreEqual(0f, configs[2].DH.Alpha, Epsilon, "J2.alpha");
            Assert.AreEqual(1f, configs[2].DH.D, Epsilon, "J2.d");
        }

        [Test]
        public void BuildDH_PerpendicularAxes_Alpha90()
        {
            // J0 绕 Z 轴，J1 绕 X 轴（正交）
            // 两轴在同一点（即 d=0, a=0），但 alpha=90° (π/2)

            var spatialInfos = new[]
            {
                new JointSpatialInfo
                {
                    WorldPosition = new RMVector3(0f, 0f, 0f),
                    WorldRotation = RMQuaternion.Identity,
                    AxisDirection = RMVector3.Forward  // Z
                },
                new JointSpatialInfo
                {
                    WorldPosition = new RMVector3(0f, 0f, 0f),
                    WorldRotation = RMQuaternion.Identity,
                    AxisDirection = RMVector3.Right     // X
                }
            };

            var configs = new[]
            {
                new JointConfig { Type = JointType.Revolute },
                new JointConfig { Type = JointType.Revolute }
            };

            var model = new RobotArmModel();
            model.BuildDHFromSpatialConfig(spatialInfos, configs);

            // alpha 应为 ±π/2（两轴正交）
            float absAlpha = System.Math.Abs(configs[1].DH.Alpha);
            Assert.AreEqual(Pi / 2f, absAlpha, Epsilon, "J1.alpha should be ±90°");
            Assert.AreEqual(0f, configs[1].DH.A, Epsilon, "J1.a");
            Assert.AreEqual(0f, configs[1].DH.D, Epsilon, "J1.d");
        }

        [Test]
        public void BuildDH_WithOriginOffset_Applied()
        {
            // J0 在原点，J1 在 (0,0,0) 但有 OriginOffset=(2,0,0)
            // 实际位置应为 (2,0,0)

            var spatialInfos = new[]
            {
                new JointSpatialInfo
                {
                    WorldPosition = new RMVector3(0f, 0f, 0f),
                    WorldRotation = RMQuaternion.Identity,
                    AxisDirection = RMVector3.Up
                },
                new JointSpatialInfo
                {
                    WorldPosition = new RMVector3(0f, 0f, 0f),
                    WorldRotation = RMQuaternion.Identity,
                    AxisDirection = RMVector3.Up,
                    OriginOffset = new RMVector3(2f, 0f, 0f)
                }
            };

            var configs = new[]
            {
                new JointConfig { Type = JointType.Revolute },
                new JointConfig { Type = JointType.Revolute }
            };

            var model = new RobotArmModel();
            model.BuildDHFromSpatialConfig(spatialInfos, configs);

            // 平行轴，偏距 a=2
            Assert.AreEqual(2f, configs[1].DH.A, Epsilon, "J1.a with offset");
        }

        [Test]
        public void GetDHTable_ReturnsCorrectCount()
        {
            var dhParams = new[]
            {
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(0.5f, Pi / 2f, 0.2f, 0f),
                new DHParameters(0.8f, 0f, 0f, Pi / 4f)
            };

            var model = RobotArmModel.FromDHParameters(dhParams);
            var table = model.GetDHTable();

            Assert.AreEqual(3, table.Length);
            Assert.AreEqual(0.5f, table[1].A, Epsilon);
            Assert.AreEqual(Pi / 2f, table[1].Alpha, Epsilon);
            Assert.AreEqual(0.2f, table[1].D, Epsilon);
        }

        [Test]
        public void RebuildDHParameters_UpdatesExisting()
        {
            var spatialInfos = new[]
            {
                new JointSpatialInfo
                {
                    WorldPosition = new RMVector3(0f, 0f, 0f),
                    WorldRotation = RMQuaternion.Identity,
                    AxisDirection = RMVector3.Up
                },
                new JointSpatialInfo
                {
                    WorldPosition = new RMVector3(1f, 0f, 0f),
                    WorldRotation = RMQuaternion.Identity,
                    AxisDirection = RMVector3.Up
                }
            };

            var configs = new[]
            {
                new JointConfig { Type = JointType.Revolute },
                new JointConfig { Type = JointType.Revolute }
            };

            var model = new RobotArmModel();
            model.BuildDHFromSpatialConfig(spatialInfos, configs);
            Assert.AreEqual(1f, configs[1].DH.A, Epsilon);

            // 热更新：J1 位置从 (1,0,0) 改为 (3,0,0)
            spatialInfos[1].WorldPosition = new RMVector3(3f, 0f, 0f);
            model.RebuildDHParameters(spatialInfos);
            Assert.AreEqual(3f, configs[1].DH.A, Epsilon, "After rebuild, a should be 3");
        }
    }
}
