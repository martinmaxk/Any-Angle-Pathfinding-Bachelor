using System;
using System.Collections.Generic;
using Xunit;
using MathTools;

namespace RpsTests
{
    public class GeometryTests
    {
        private const float MaxValue = MathExt.MaxValue, Epsilon = MathExt.Epsilon, 
            CollEpsilon = MathExt.CollEpsilon;

        [Fact]
        public void LineToLinePrecision()
        {
            Vector2 intersection;
            Assert.True(MathExt.LineToLine(new Vector2(0, MaxValue + Epsilon), new Vector2(MaxValue, MaxValue),
                new Vector2(MaxValue - (Epsilon * 2), MaxValue), new Vector2(MaxValue - Epsilon, MaxValue + Epsilon), out intersection));
            Assert.False(MathExt.LineToLine(new Vector2(0, MaxValue - Epsilon), new Vector2(MaxValue, MaxValue),
                new Vector2(MaxValue - (Epsilon * 2), MaxValue), new Vector2(MaxValue - Epsilon, MaxValue + Epsilon), out intersection));
        }

        public static IEnumerable<object[]> OrientationCollinearDataGen()
        {
            yield return new object[] { new Vector2(0, 0), new Vector2(MaxValue, MaxValue) / 2, 
                new Vector2(MaxValue, MaxValue) };
        }

        public static IEnumerable<object[]> OrientationCwDataGen()
        {
            yield return new object[] { new Vector2(0, 0), new Vector2(MaxValue, MaxValue) / 2,
                new Vector2(MaxValue - Epsilon, MaxValue) };
        }

        public static IEnumerable<object[]> OrientationCcwDataGen()
        {
            yield return new object[] { new Vector2(0, 0), new Vector2(MaxValue, MaxValue) / 2,
                new Vector2(MaxValue + Epsilon, MaxValue) };
        }

        [Theory]
        [MemberData(nameof(OrientationCollinearDataGen))]
        public void OrientationCollinear(Vector2 p1, Vector2 p2, Vector2 p3)
        {
            float orientation = MathExt.Orientation(p1, p2, p3);
            Assert.True(MathExt.Collinear(orientation));
        }

        [Theory]
        [MemberData(nameof(OrientationCwDataGen))]
        public void OrientationCw(Vector2 p1, Vector2 p2, Vector2 p3)
        {
            float orientation = MathExt.Orientation(p1, p2, p3);
            Assert.True(orientation < -CollEpsilon);
        }

        [Theory]
        [MemberData(nameof(OrientationCcwDataGen))]
        public void OrientationCcw(Vector2 p1, Vector2 p2, Vector2 p3)
        {
            float orientation = MathExt.Orientation(p1, p2, p3);
            Assert.True(orientation > CollEpsilon);
        }
    }
}
