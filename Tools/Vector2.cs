using System;

namespace MathTools
{
    public struct Vector2 : IEquatable<Vector2>
    {
        public float x;
        public float y;

        private static Vector2 zeroVector = new Vector2(0f, 0f);
        private static Vector2 unitVector = new Vector2(1f, 1f);
        private static Vector2 unitXVector = new Vector2(1f, 0f);
        private static Vector2 unitYVector = new Vector2(0f, 1f);
        public static Vector2 Zero
        {
            get { return zeroVector; }
        }
        
        public static Vector2 One
        {
            get { return unitVector; }
        }

        public static Vector2 UnitX
        {
            get { return unitXVector; }
        }

        public static Vector2 UnitY
        {
            get { return unitYVector; }
        }

        public Vector2(float x, float y)
        {
            this.x = x;
            this.y = y;
        }

        public Vector2(float value)
        {
            this.x = value;
            this.y = value;
        }

        public void Abs(){
            this.x = Math.Max(this.x, - this.x);
            this.y = Math.Max(this.y, - this.y);
        }

        public float LengthSquared()
        {
            return (x * x) + (y * y);
        }

        public static float Distance(Vector2 value1, Vector2 value2)
        {
            float v1 = value1.x - value2.x, v2 = value1.y - value2.y;
            return (float)Math.Sqrt((v1 * v1) + (v2 * v2));
        }

        public static float DistanceSquared(Vector2 value1, Vector2 value2)
        {
            float v1 = value1.x - value2.x, v2 = value1.y - value2.y;
            return (v1 * v1) + (v2 * v2);
        }

        public static float Dot(Vector2 value1, Vector2 value2)
        {
            return (value1.x * value2.x) + (value1.y * value2.y);
        }

        public static void Dot(ref Vector2 value1, ref Vector2 value2, out float result)
        {
            result = (value1.x * value2.x) + (value1.y * value2.y);
        }

        private const float epsilonNormSqrt = 1e-15f;
        /// <summary>
        /// Get the smaller angle between two vectors
        /// </summary>
        public static float Angle(Vector2 from, Vector2 to)
        {
            // sqrt(a) * sqrt(b) = sqrt(a * b)
            float denominator = (float)Math.Sqrt(from.LengthSquared() * to.LengthSquared());
            if (denominator < epsilonNormSqrt)
                return 0f;

            // dot(a, b) = |a||b|cos theta
            // dot(a, b) / (|a||b|) = cos theta
            // cos theta is valid between [-1, 1] (unit circle)
            float cosAngle = MathExt.Clamp(Dot(from, to) / denominator, -1f, 1f);
            return (float)Math.Acos(cosAngle);
        }

        public override bool Equals(object obj)
        {
            if (obj is Vector2)
                return Equals((Vector2)this);

            return false;
        }

        public bool Equals(Vector2 other)
        {
            return (x == other.x) && (y == other.y);
        }

        public override int GetHashCode()
        {
            return x.GetHashCode() + y.GetHashCode();
        }

        public override string ToString()
        {
            return string.Format("({0}, {1})", x, y);
        }

        public static explicit operator Vector2D(Vector2 value)
        {
            return new Vector2D(value.x, value.y);
        }

        public static Vector2 operator -(Vector2 value)
        {
            value.x = -value.x;
            value.y = -value.y;
            return value;
        }

        public static bool operator ==(Vector2 value1, Vector2 value2)
        {
            return value1.x == value2.x && value1.y == value2.y;
        }

        public static bool operator !=(Vector2 value1, Vector2 value2)
        {
            return value1.x != value2.x || value1.y != value2.y;
        }

        public static Vector2 operator +(Vector2 value1, Vector2 value2)
        {
            value1.x += value2.x;
            value1.y += value2.y;
            return value1;
        }

        public static Vector2 operator -(Vector2 value1, Vector2 value2)
        {
            value1.x -= value2.x;
            value1.y -= value2.y;
            return value1;
        }

        public static Vector2 operator *(Vector2 value1, Vector2 value2)
        {
            value1.x *= value2.x;
            value1.y *= value2.y;
            return value1;
        }

        public static Vector2 operator *(Vector2 value, float scaleFactor)
        {
            value.x *= scaleFactor;
            value.y *= scaleFactor;
            return value;
        }

        public static Vector2 operator *(float scaleFactor, Vector2 value)
        {
            value.x *= scaleFactor;
            value.y *= scaleFactor;
            return value;
        }

        public static Vector2 operator /(Vector2 value1, Vector2 value2)
        {
            value1.x /= value2.x;
            value1.y /= value2.y;
            return value1;
        }

        public static Vector2 operator /(Vector2 value1, float divider)
        {
            float factor = 1 / divider;
            value1.x *= factor;
            value1.y *= factor;
            return value1;
        }
    }
}
