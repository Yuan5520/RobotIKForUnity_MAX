using NUnit.Framework;
using RobotMatrix.Math;
using IEngine;
using RobotMatrix.Controller;

namespace RobotMatrix.Tests
{
    /// <summary>
    /// 轻量 Mock IEngineObject，用于测试层级分析。
    /// </summary>
    public class MockEngineObject : IEngineObject
    {
        private readonly string _name;
        private readonly MockEngineObject _parent;

        public MockEngineObject(string name, MockEngineObject parent = null)
        {
            _name = name;
            _parent = parent;
        }

        public RMVector3 LocalPosition { get; set; }
        public RMQuaternion LocalRotation { get; set; } = RMQuaternion.Identity;
        public RMVector3 LocalScale { get; set; } = new RMVector3(1, 1, 1);
        public RMVector3 WorldPosition { get; set; }
        public RMQuaternion WorldRotation { get; set; } = RMQuaternion.Identity;

        public RMVector3 TransformPoint(RMVector3 localPoint) => localPoint + WorldPosition;
        public RMVector3 InverseTransformPoint(RMVector3 worldPoint) => worldPoint - WorldPosition;
        public RMVector3 TransformDirection(RMVector3 localDir) => localDir;

        public IEngineObject Parent => _parent;
        public IEngineObject[] Children => new IEngineObject[0];
        public string Name => _name;
        public bool IsValid => true;

        public override bool Equals(object obj)
        {
            return obj is MockEngineObject other && _name == other._name;
        }

        public override int GetHashCode()
        {
            return _name != null ? _name.GetHashCode() : 0;
        }
    }

    public class JointHierarchyAnalyzerTests
    {
        // ===== 严格父子链 → 全部 AncestorDescendant =====

        [Test]
        public void Analyze_StrictParentChain_AllAncestorDescendant()
        {
            // Root → J0 → J1 → J2
            var root = new MockEngineObject("Root");
            var j0 = new MockEngineObject("J0", root);
            var j1 = new MockEngineObject("J1", j0);
            var j2 = new MockEngineObject("J2", j1);

            var joints = new IEngineObject[] { j0, j1, j2 };
            var infos = JointHierarchyAnalyzer.Analyze(joints);

            Assert.AreEqual(3, infos.Length);
            Assert.AreEqual(JointRelationType.AncestorDescendant, infos[0].Relation);
            Assert.AreEqual(JointRelationType.AncestorDescendant, infos[1].Relation);
            Assert.AreEqual(JointRelationType.AncestorDescendant, infos[2].Relation);
            Assert.AreEqual(-1, infos[0].ParentJointIndex);
            Assert.AreEqual(0, infos[1].ParentJointIndex);
            Assert.AreEqual(1, infos[2].ParentJointIndex);
        }

        // ===== 中间夹杂模型节点 → 仍然 AncestorDescendant =====

        [Test]
        public void Analyze_WithIntermediateNodes_StillAncestorDescendant()
        {
            // Root → J0 → Model1 → Model2 → J1
            // J0 是 J1 的祖先（中间有非关节节点）
            var root = new MockEngineObject("Root");
            var j0 = new MockEngineObject("J0", root);
            var model1 = new MockEngineObject("Model1", j0);
            var model2 = new MockEngineObject("Model2", model1);
            var j1 = new MockEngineObject("J1", model2);

            var joints = new IEngineObject[] { j0, j1 };
            var infos = JointHierarchyAnalyzer.Analyze(joints);

            Assert.AreEqual(JointRelationType.AncestorDescendant, infos[0].Relation);
            Assert.AreEqual(JointRelationType.AncestorDescendant, infos[1].Relation);
        }

        // ===== 同级关节 → Independent =====

        [Test]
        public void Analyze_SiblingJoints_Independent()
        {
            // Root → J0
            // Root → J1
            // J0 和 J1 是同级（共同父级 Root，但互不是祖先后代）
            var root = new MockEngineObject("Root");
            var j0 = new MockEngineObject("J0", root);
            var j1 = new MockEngineObject("J1", root);

            var joints = new IEngineObject[] { j0, j1 };
            var infos = JointHierarchyAnalyzer.Analyze(joints);

            Assert.AreEqual(JointRelationType.AncestorDescendant, infos[0].Relation);
            Assert.AreEqual(JointRelationType.Independent, infos[1].Relation);
        }

        // ===== 混合拓扑 =====

        [Test]
        public void Analyze_MixedTopology()
        {
            // Root → J0 → J1
            // Root → J2
            // J0→J1 是父子，J1→J2 是独立
            var root = new MockEngineObject("Root");
            var j0 = new MockEngineObject("J0", root);
            var j1 = new MockEngineObject("J1", j0);
            var j2 = new MockEngineObject("J2", root);

            var joints = new IEngineObject[] { j0, j1, j2 };
            var infos = JointHierarchyAnalyzer.Analyze(joints);

            Assert.AreEqual(JointRelationType.AncestorDescendant, infos[0].Relation);
            Assert.AreEqual(JointRelationType.AncestorDescendant, infos[1].Relation);
            Assert.AreEqual(JointRelationType.Independent, infos[2].Relation);
        }

        // ===== IsAncestorOf 边界 =====

        [Test]
        public void IsAncestorOf_NullInputs_ReturnsFalse()
        {
            var obj = new MockEngineObject("A");
            Assert.IsFalse(JointHierarchyAnalyzer.IsAncestorOf(null, obj));
            Assert.IsFalse(JointHierarchyAnalyzer.IsAncestorOf(obj, null));
            Assert.IsFalse(JointHierarchyAnalyzer.IsAncestorOf(null, null));
        }

        [Test]
        public void IsAncestorOf_SameObject_ReturnsFalse()
        {
            // 自身不是自身的祖先
            var obj = new MockEngineObject("A");
            Assert.IsFalse(JointHierarchyAnalyzer.IsAncestorOf(obj, obj));
        }

        [Test]
        public void IsAncestorOf_DirectParent_ReturnsTrue()
        {
            var parent = new MockEngineObject("Parent");
            var child = new MockEngineObject("Child", parent);
            Assert.IsTrue(JointHierarchyAnalyzer.IsAncestorOf(parent, child));
        }

        [Test]
        public void IsAncestorOf_Grandparent_ReturnsTrue()
        {
            var grandparent = new MockEngineObject("Grandparent");
            var parent = new MockEngineObject("Parent", grandparent);
            var child = new MockEngineObject("Child", parent);
            Assert.IsTrue(JointHierarchyAnalyzer.IsAncestorOf(grandparent, child));
        }

        // ===== 单关节 =====

        [Test]
        public void Analyze_SingleJoint_AncestorDescendant()
        {
            var j0 = new MockEngineObject("J0");
            var joints = new IEngineObject[] { j0 };
            var infos = JointHierarchyAnalyzer.Analyze(joints);

            Assert.AreEqual(1, infos.Length);
            Assert.AreEqual(JointRelationType.AncestorDescendant, infos[0].Relation);
            Assert.AreEqual(-1, infos[0].ParentJointIndex);
        }
    }
}
