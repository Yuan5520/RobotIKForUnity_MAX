using RobotMatrix.Math;

namespace IEngine
{
    /// <summary>
    /// 场景物体的抽象引用，包装引擎原生对象（如 Unity 的 GameObject/Transform）。
    /// 提供空间变换读写、局部/世界坐标转换、层级管理。
    /// </summary>
    public interface IEngineObject
    {
        // ===== 空间变换 =====

        RMVector3 LocalPosition { get; set; }
        RMQuaternion LocalRotation { get; set; }
        RMVector3 LocalScale { get; set; }
        RMVector3 WorldPosition { get; set; }
        RMQuaternion WorldRotation { get; set; }

        // ===== 局部/世界坐标转换 =====

        /// <summary>
        /// 将局部坐标点转换为世界坐标。
        /// </summary>
        RMVector3 TransformPoint(RMVector3 localPoint);

        /// <summary>
        /// 将世界坐标点转换为局部坐标。
        /// </summary>
        RMVector3 InverseTransformPoint(RMVector3 worldPoint);

        /// <summary>
        /// 将局部方向向量转换为世界方向（忽略位移，仅旋转+缩放）。
        /// </summary>
        RMVector3 TransformDirection(RMVector3 localDir);

        // ===== 层级管理 =====

        /// <summary>
        /// 父级物体。若为场景根节点则返回 null。
        /// </summary>
        IEngineObject Parent { get; }

        /// <summary>
        /// 直接子级物体数组。
        /// </summary>
        IEngineObject[] Children { get; }

        /// <summary>
        /// 物体名称。
        /// </summary>
        string Name { get; }

        /// <summary>
        /// 底层引擎对象是否仍然有效（未被销毁）。
        /// </summary>
        bool IsValid { get; }
    }
}
