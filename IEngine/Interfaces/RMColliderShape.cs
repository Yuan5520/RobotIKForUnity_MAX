using RobotMatrix.Math;

namespace IEngine
{
    /// <summary>
    /// 碰撞体的实际几何形状描述（引擎无关）。
    /// 用于可视化绘制碰撞体的真实外形而非 AABB 包围盒。
    /// </summary>
    public struct RMColliderShape
    {
        /// <summary>碰撞体类型。</summary>
        public RMColliderType Type;

        /// <summary>碰撞体世界坐标中心。</summary>
        public RMVector3 WorldCenter;

        /// <summary>物体世界旋转。</summary>
        public RMQuaternion WorldRotation;

        // ----- Box -----
        /// <summary>Box 碰撞体的本地尺寸（全长，非半径）。</summary>
        public RMVector3 BoxSize;

        // ----- Sphere -----
        /// <summary>Sphere 碰撞体的世界缩放后半径。</summary>
        public float SphereRadius;

        // ----- Capsule -----
        /// <summary>Capsule 碰撞体的世界缩放后半径。</summary>
        public float CapsuleRadius;
        /// <summary>Capsule 碰撞体的世界缩放后总高度。</summary>
        public float CapsuleHeight;
        /// <summary>Capsule 碰撞体方向：0=X, 1=Y, 2=Z。</summary>
        public int CapsuleDirection;

        // ----- Mesh -----
        /// <summary>Mesh 碰撞体的世界坐标顶点。</summary>
        public RMVector3[] MeshVertices;
        /// <summary>Mesh 碰撞体的三角形索引（每3个一组）。</summary>
        public int[] MeshTriangles;
    }
}
