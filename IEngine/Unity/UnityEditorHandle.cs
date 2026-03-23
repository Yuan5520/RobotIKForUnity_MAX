#if UNITY_EDITOR
using UnityEditor;
using RobotMatrix.Math;
using IEngine;

namespace IEngine.Unity
{
    /// <summary>
    /// 编辑器环境下的交互式控制柄实现，使用 Handles API。
    /// 仅在 OnSceneGUI 回调中调用才有效。
    /// </summary>
    public class UnityEditorHandle : IEngineHandle
    {
        public RMVector3 PositionHandle(RMVector3 position, RMQuaternion rotation)
        {
            var newPos = Handles.PositionHandle(
                TypeConverter.ToUnity(position),
                TypeConverter.ToUnity(rotation));
            return TypeConverter.ToRM(newPos);
        }

        public RMQuaternion RotationHandle(RMQuaternion rotation, RMVector3 position)
        {
            var newRot = Handles.RotationHandle(
                TypeConverter.ToUnity(rotation),
                TypeConverter.ToUnity(position));
            return TypeConverter.ToRM(newRot);
        }

        public bool IsAvailable => true;
    }
}
#endif
