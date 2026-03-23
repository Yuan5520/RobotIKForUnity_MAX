using UnityEngine;
using RobotMatrix.Math;
using IEngine;

namespace IEngine.Unity
{
    /// <summary>
    /// Unity 引擎物体适配器，包装 GameObject/Transform 实现 IEngineObject。
    /// </summary>
    public class UnityEngineObject : IEngineObject
    {
        private readonly Transform _transform;

        public UnityEngineObject(GameObject gameObject)
        {
            _transform = gameObject != null ? gameObject.transform : null;
        }

        public UnityEngineObject(Transform transform)
        {
            _transform = transform;
        }

        /// <summary>
        /// 获取底层 Unity Transform（供 Unity 适配层内部使用）。
        /// </summary>
        public Transform UnityTransform => _transform;

        // ===== 空间变换 =====

        public RMVector3 LocalPosition
        {
            get => TypeConverter.ToRM(_transform.localPosition);
            set => _transform.localPosition = TypeConverter.ToUnity(value);
        }

        public RMQuaternion LocalRotation
        {
            get => TypeConverter.ToRM(_transform.localRotation);
            set => _transform.localRotation = TypeConverter.ToUnity(value);
        }

        public RMVector3 LocalScale
        {
            get => TypeConverter.ToRM(_transform.localScale);
            set => _transform.localScale = TypeConverter.ToUnity(value);
        }

        public RMVector3 WorldPosition
        {
            get => TypeConverter.ToRM(_transform.position);
            set => _transform.position = TypeConverter.ToUnity(value);
        }

        public RMQuaternion WorldRotation
        {
            get => TypeConverter.ToRM(_transform.rotation);
            set => _transform.rotation = TypeConverter.ToUnity(value);
        }

        // ===== 坐标转换 =====

        public RMVector3 TransformPoint(RMVector3 localPoint)
        {
            return TypeConverter.ToRM(_transform.TransformPoint(TypeConverter.ToUnity(localPoint)));
        }

        public RMVector3 InverseTransformPoint(RMVector3 worldPoint)
        {
            return TypeConverter.ToRM(_transform.InverseTransformPoint(TypeConverter.ToUnity(worldPoint)));
        }

        public RMVector3 TransformDirection(RMVector3 localDir)
        {
            return TypeConverter.ToRM(_transform.TransformDirection(TypeConverter.ToUnity(localDir)));
        }

        // ===== 层级管理 =====

        public IEngineObject Parent
        {
            get
            {
                var parent = _transform.parent;
                return parent != null ? new UnityEngineObject(parent) : null;
            }
        }

        public IEngineObject[] Children
        {
            get
            {
                int count = _transform.childCount;
                var children = new IEngineObject[count];
                for (int i = 0; i < count; i++)
                    children[i] = new UnityEngineObject(_transform.GetChild(i));
                return children;
            }
        }

        public string Name => _transform != null ? _transform.name : string.Empty;

        public bool IsValid => _transform != null;

        // ===== 相等性比较（用于层级分析中的祖先检测）=====

        public override bool Equals(object obj)
        {
            if (obj is UnityEngineObject other)
                return ReferenceEquals(_transform, other._transform);
            return false;
        }

        public override int GetHashCode()
        {
            return _transform != null ? _transform.GetHashCode() : 0;
        }
    }
}
