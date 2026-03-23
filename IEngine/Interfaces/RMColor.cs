namespace IEngine
{
    /// <summary>
    /// 引擎无关的颜色结构体（RGBA，各分量 0~1）。
    /// </summary>
    public struct RMColor
    {
        public float R;
        public float G;
        public float B;
        public float A;

        public RMColor(float r, float g, float b, float a = 1f)
        {
            R = r;
            G = g;
            B = b;
            A = a;
        }

        // ===== 常用颜色 =====

        public static readonly RMColor Red = new RMColor(1f, 0f, 0f);
        public static readonly RMColor Green = new RMColor(0f, 1f, 0f);
        public static readonly RMColor Blue = new RMColor(0f, 0f, 1f);
        public static readonly RMColor Yellow = new RMColor(1f, 1f, 0f);
        public static readonly RMColor White = new RMColor(1f, 1f, 1f);
        public static readonly RMColor Gray = new RMColor(0.5f, 0.5f, 0.5f);
        public static readonly RMColor Orange = new RMColor(1f, 0.5f, 0f);
        public static readonly RMColor Cyan = new RMColor(0f, 1f, 1f);
    }
}
