namespace IEngine
{
    /// <summary>
    /// 碰撞事件监听器接口。
    /// </summary>
    public interface ICollisionListener
    {
        /// <summary>碰撞开始。</summary>
        void OnCollisionEnter(RMCollisionEventData data);

        /// <summary>碰撞持续中。</summary>
        void OnCollisionStay(RMCollisionEventData data);

        /// <summary>碰撞结束。</summary>
        void OnCollisionExit(RMCollisionEventData data);
    }
}
