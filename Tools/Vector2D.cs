using System;

namespace MathTools
{
    public struct Vector2D : IEquatable<Vector2D>
    {
        public double x;
        public double y;

        private static Vector2D zeroVector = new Vector2D(0f, 0f);
        private static Vector2D unitVector = new Vector2D(1f, 1f);
        private static Vector2D unitXVector = new Vector2D(1f, 0f);
        private static Vector2D unitYVector = new Vector2D(0f, 1f);
        public static Vector2D Zero
        {
            get { return zeroVector; }
        }
        
        public static Vector2D One
        {
            get { return unitVector; }
        }

        public static Vector2D UnitX
        {
            get { return unitXVector; }
        }

        public static Vector2D UnitY
        {
            get { return unitYVector; }
        }

        public Vector2D(double x, double y)
        {
            this.x = x;
            this.y = y;
        }

        public Vector2D(double value)
        {
            this.x = value;
            this.y = value;
        }

        public void Abs(){
            this.x = Math.Max(this.x, - this.x);
            this.y = Math.Max(this.y, - this.y);
        }

        public double LengthSquared()
        {
            return (x * x) + (y * y);
        }

        public static double Distance(Vector2D value1, Vector2D value2)
        {
            double v1 = value1.x - value2.x, v2 = value1.y - value2.y;
            return (double)Math.Sqrt((v1 * v1) + (v2 * v2));
        }

        public static double DistanceSquared(Vector2D value1, Vector2D value2)
        {
            double v1 = value1.x - value2.x, v2 = value1.y - value2.y;
            return (v1 * v1) + (v2 * v2);
        }

        public static double Dot(Vector2D value1, Vector2D value2)
        {
            return (value1.x * value2.x) + (value1.y * value2.y);
        }

        public static void Dot(ref Vector2D value1, ref Vector2D value2, out double result)
        {
            result = (value1.x * value2.x) + (value1.y * value2.y);
        }

        public override bool Equals(object obj)
        {
            if (obj is Vector2D)
                return Equals((Vector2D)this);

            return false;
        }

        public bool Equals(Vector2D other)
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

        public static explicit operator Vector2(Vector2D value)
        {
            return new Vector2((float)value.x, (float)value.y);
        }

        public static Vector2D operator -(Vector2D value)
        {
            value.x = -value.x;
            value.y = -value.y;
            return value;
        }

        public static bool operator ==(Vector2D value1, Vector2D value2)
        {
            return value1.x == value2.x && value1.y == value2.y;
        }

        public static bool operator !=(Vector2D value1, Vector2D value2)
        {
            return value1.x != value2.x || value1.y != value2.y;
        }

        public static Vector2D operator +(Vector2D value1, Vector2D value2)
        {
            value1.x += value2.x;
            value1.y += value2.y;
            return value1;
        }

        public static Vector2D operator -(Vector2D value1, Vector2D value2)
        {
            value1.x -= value2.x;
            value1.y -= value2.y;
            return value1;
        }

        public static Vector2D operator *(Vector2D value1, Vector2D value2)
        {
            value1.x *= value2.x;
            value1.y *= value2.y;
            return value1;
        }

        public static Vector2D operator *(Vector2D value, double scaleFactor)
        {
            value.x *= scaleFactor;
            value.y *= scaleFactor;
            return value;
        }

        public static Vector2D operator *(double scaleFactor, Vector2D value)
        {
            value.x *= scaleFactor;
            value.y *= scaleFactor;
            return value;
        }

        public static Vector2D operator /(Vector2D value1, Vector2D value2)
        {
            value1.x /= value2.x;
            value1.y /= value2.y;
            return value1;
        }

        public static Vector2D operator /(Vector2D value1, double divider)
        {
            double factor = 1 / divider;
            value1.x *= factor;
            value1.y *= factor;
            return value1;
        }
    }
}
