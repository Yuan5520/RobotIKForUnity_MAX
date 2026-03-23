using UnityEngine;
using RobotMatrix.Math;
using IEngine;

namespace IEngine.Unity
{
    /// <summary>
    /// Unity 输入系统适配器，包装 Input 类实现 IEngineInput。
    /// </summary>
    public class UnityEngineInput : IEngineInput
    {
        private Camera _camera;

        public UnityEngineInput(Camera camera = null)
        {
            _camera = camera;
        }

        private Camera ActiveCamera => _camera != null ? _camera : Camera.main;

        public RMVector3 GetMouseWorldPosition(RMVector3 screenPos, float depth)
        {
            var cam = ActiveCamera;
            if (cam == null)
                return RMVector3.Zero;

            var uScreen = TypeConverter.ToUnity(screenPos);
            uScreen.z = depth;
            var worldPos = cam.ScreenToWorldPoint(uScreen);
            return TypeConverter.ToRM(worldPos);
        }

        public bool GetMouseButton(int button)
        {
            return Input.GetMouseButton(button);
        }

        public bool GetMouseButtonDown(int button)
        {
            return Input.GetMouseButtonDown(button);
        }

        public bool GetKey(string keyName)
        {
            return Input.GetKey(keyName);
        }

        public bool GetKeyDown(string keyName)
        {
            return Input.GetKeyDown(keyName);
        }

        public float GetAxis(string axisName)
        {
            return Input.GetAxis(axisName);
        }
    }
}
