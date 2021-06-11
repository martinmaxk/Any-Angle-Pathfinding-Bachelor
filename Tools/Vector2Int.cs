using System;

namespace MathTools
{
    public struct Vector2Int : IEquatable<Vector2Int>
    {
        private static Vector2Int zeroPoint = new Vector2Int();

        public int x;
        public int y;

        public static Vector2Int Zero
        {
            get { return zeroPoint; }
        }

        public Vector2Int(int x, int y)
        {
            this.x = x;
            this.y = y;
        }

        public static float Distance(Vector2Int value1, Vector2Int value2)
        {
            float v1 = value1.x - value2.x, v2 = value1.y - value2.y;
            return (float)Math.Sqrt((v1 * v1) + (v2 * v2));
        }

        public static float DistanceSquared(Vector2Int value1, Vector2Int value2)
        {
            float v1 = value1.x - value2.x, v2 = value1.y - value2.y;
            return (v1 * v1) + (v2 * v2);
        }

        public static bool operator ==(Vector2Int a, Vector2Int b)
        {
            return a.Equals(b);
        }

        public static bool operator !=(Vector2Int a, Vector2Int b)
        {
            return !a.Equals(b);
        }

        public bool Equals(Vector2Int other)
        {
            return ((x == other.x) && (y == other.y));
        }

        public override bool Equals(object obj)
        {
            return (obj is Vector2Int) ? Equals((Vector2Int)obj) : false;
        }

        public override int GetHashCode()
        {
            return x ^ y;
        }

        public override string ToString()
        {
            return string.Format("{{X:{0} Y:{1}}}", x, y);
        }

        public static implicit operator Vector2(Vector2Int v)
        {
            return new Vector2(v.x, v.y);
        }

        public static Vector2Int operator +(Vector2Int value1, Vector2Int value2)
        {
            value1.x += value2.x;
            value1.y += value2.y;
            return value1;
        }

        public static Vector2Int operator -(Vector2Int value1, Vector2Int value2)
        {
            value1.x -= value2.x;
            value1.y -= value2.y;
            return value1;
        }

        public static Vector2Int operator *(Vector2Int value1, Vector2Int value2)
        {
            value1.x *= value2.x;
            value1.y *= value2.y;
            return value1;
        }

        public static Vector2Int operator *(Vector2Int value, int scaleFactor)
        {
            value.x *= scaleFactor;
            value.y *= scaleFactor;
            return value;
        }

        public static Vector2Int operator *(int scaleFactor, Vector2Int value)
        {
            value.x *= scaleFactor;
            value.y *= scaleFactor;
            return value;
        }

        public static Vector2Int operator /(Vector2Int value1, Vector2Int value2)
        {
            value1.x /= value2.x;
            value1.y /= value2.y;
            return value1;
        }

        public static Vector2Int operator /(Vector2Int value1, int divider)
        {
            value1.x /= divider;
            value1.y /= divider;
            return value1;
        }
    }
}
