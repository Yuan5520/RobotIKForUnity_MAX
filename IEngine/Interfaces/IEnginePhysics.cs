using RobotMatrix.Math;

namespace IEngine
{
    /// <summary>
    /// 物理引擎抽象接口。
    /// 封装碰撞体管理、层级查询、碰撞事件监听等物理相关操作。
    /// </summary>
    public interface IEnginePhysics
    {
        // ===== 碰撞体管理 =====

        /// <summary>
        /// 为目标物体添加盒型碰撞体。
        /// </summary>
        void AddBoxCollider(IEngineObject target, RMVector3 center, RMVector3 size);

        /// <summary>
        /// 为目标物体添加网格碰撞体。
        /// </summary>
        void AddMeshCollider(IEngineObject target, bool convex);

        /// <summary>
        /// 获取目标物体上的所有碰撞体信息。
        /// </summary>
        RMColliderInfo[] GetColliders(IEngineObject target);

        /// <summary>
        /// 启用/禁用目标物体上的所有碰撞体。
        /// </summary>
        void SetCollidersEnabled(IEngineObject target, bool enabled);

        /// <summary>
        /// 设置目标物体上所有碰撞体的 isTrigger 状态。
        /// </summary>
        void SetCollidersIsTrigger(IEngineObject target, bool isTrigger);

        /// <summary>
        /// 移除目标物体上的所有碰撞体。
        /// </summary>
        void RemoveAllColliders(IEngineObject target);

        // ===== 层级查询 =====

        /// <summary>
        /// 获取根节点下的所有后代物体（包含自身）。
        /// </summary>
        IEngineObject[] GetAllDescendants(IEngineObject root);

        /// <summary>
        /// 获取根节点下的后代物体，遇到 stopBefore 节点时跳过该分支。
        /// 用于获取两个连续关节根节点之间的所有物体。
        /// </summary>
        IEngineObject[] GetDescendantsUntil(IEngineObject root, IEngineObject stopBefore);

        // ===== 碰撞事件 =====

        /// <summary>
        /// 为目标物体注册碰撞事件监听器。
        /// </summary>
        void RegisterCollisionListener(IEngineObject target, ICollisionListener listener);

        /// <summary>
        /// 注销目标物体的碰撞事件监听器。
        /// </summary>
        void UnregisterCollisionListener(IEngineObject target);

        // ===== Rigidbody 管理 =====

        /// <summary>
        /// 确保目标物体拥有运动学刚体（isKinematic=true, useGravity=false）。
        /// 碰撞检测需要至少一方拥有 Rigidbody。
        /// </summary>
        void EnsureKinematicRigidbody(IEngineObject target);

        // ===== 碰撞体边界查询 =====

        /// <summary>
        /// 获取目标物体所有碰撞体的合并包围盒。
        /// </summary>
        RMBounds GetColliderBounds(IEngineObject target);

        /// <summary>
        /// 获取目标物体所有碰撞体的实际几何形状描述。
        /// 用于精确的碰撞体可视化。
        /// </summary>
        RMColliderShape[] GetColliderShapes(IEngineObject target);
    }
}
