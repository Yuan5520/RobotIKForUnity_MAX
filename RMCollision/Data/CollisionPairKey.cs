using System;

namespace RMCollision
{
    /// <summary>
    /// 碰撞对标识。确保 SegmentA &lt; SegmentB 以保证唯一性。
    /// </summary>
    public struct CollisionPairKey : IEquatable<CollisionPairKey>
    {
        public int SegmentA;
        public int SegmentB;

        public CollisionPairKey(int a, int b)
        {
            if (a <= b)
            {
                SegmentA = a;
                SegmentB = b;
            }
            else
            {
                SegmentA = b;
                SegmentB = a;
            }
        }

        public bool Equals(CollisionPairKey other)
        {
            return SegmentA == other.SegmentA && SegmentB == other.SegmentB;
        }

        public override bool Equals(object obj)
        {
            return obj is CollisionPairKey other && Equals(other);
        }

        public override int GetHashCode()
        {
            return (SegmentA * 397) ^ SegmentB;
        }

        public override string ToString()
        {
            return $"({SegmentA}, {SegmentB})";
        }
    }
}
