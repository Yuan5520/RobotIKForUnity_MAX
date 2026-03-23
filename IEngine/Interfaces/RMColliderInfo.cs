namespace IEngine
{
    /// <summary>
    /// 碰撞体类型枚举。
    /// </summary>
    public enum RMColliderType
    {
        Box,
        Mesh,
        Sphere,
        Capsule
    }

    /// <summary>
    /// 碰撞体信息（引擎无关）。
    /// </summary>
    public struct RMColliderInfo
    {
        /// <summary>碰撞体类型。</summary>
        public RMColliderType Type;

        /// <summary>碰撞体是否启用。</summary>
        public bool Enabled;

        /// <summary>所属物体名称。</summary>
        public string OwnerName;
    }
}
