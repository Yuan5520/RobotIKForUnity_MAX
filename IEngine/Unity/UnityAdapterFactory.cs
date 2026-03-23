using UnityEngine;
using IEngine;

namespace IEngine.Unity
{
    /// <summary>
    /// Unity 引擎适配器工厂实现。
    /// 根据编辑器/运行时环境注入不同的 Visualizer 和 Handle 实现。
    /// </summary>
    public class UnityAdapterFactory : IEngineAdapterFactory
    {
        public IEngineObject WrapObject(object nativeObject)
        {
            if (nativeObject is GameObject go)
                return new UnityEngineObject(go);
            if (nativeObject is Transform t)
                return new UnityEngineObject(t);
            return null;
        }

        public IEngineVisualizer CreateVisualizer()
        {
#if UNITY_EDITOR
            return new UnityEditorVisualizer();
#else
            return new UnityRuntimeVisualizer();
#endif
        }

        public IEngineHandle CreateHandle()
        {
#if UNITY_EDITOR
            return new UnityEditorHandle();
#else
            return new NullHandle();
#endif
        }

        public IEngineInput CreateInput()
        {
            return new UnityEngineInput();
        }

        public IEnginePhysics CreatePhysics()
        {
            return new UnityEnginePhysics();
        }
    }
}
