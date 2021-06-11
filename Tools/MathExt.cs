using System;
using System.Runtime.CompilerServices;
using System.Collections.Generic;

namespace MathTools
{
    public static class MathExt
    {
        public static void MinkowskiSum(List<Vector2> P, List<Vector2> R, List<Vector2> result,
            int bottomLeftIdxP, int bottomLeftIdxR)
        {
            int i = bottomLeftIdxP;
            int j = bottomLeftIdxR;
            int n = P.Count + i;
            int m = R.Count + j;
            while (!(i == n && j == m))
            {
                Vector2 v_i = P[i % P.Count]; // i % P.Length to satisfy v_{n+1} = v_1 and v_{n+2} = v_2
                Vector2 w_j = R[j % R.Count];
                Vector2 v_ip1 = P[(i + 1) % P.Count]; //v_{i+1}
                Vector2 w_jp1 = R[(j + 1) % R.Count];
                result.Add(v_i + w_j);
                
                double angle_i = CcwAngle(v_i, v_ip1);
                double angle_j = CcwAngle(w_j, w_jp1);

                if (i == n)
                    j++;
                else if (j == m)
                    i++;
                //perform equality check first to account for possible floating point error
                else if (ApproxAngle(angle_i, angle_j))
                {
                    i++;
                    j++;
                }
                else if (angle_i < angle_j)
                    i++;
                else
                    j++;
            }
        }

        public static int BottomLeftIndex(List<Vector2> character)
        {
            Vector2 bottomLeft = character[0];
            int bottomLeftIndex = 0;
            for (int i = 0; i < character.Count; i++)
            {
                if (character[i].y > bottomLeft.y || 
                    (MathExt.Approximately(character[i].y, bottomLeft.y) && character[i].x < bottomLeft.x))
                {
                    bottomLeft = character[i];
                    bottomLeftIndex = i;
                }
            }
            return bottomLeftIndex;
        }

        // counter-clockwise angle from a to b
        public static double CcwAngle(Vector2 a, Vector2 b)
        {
            return CcwAngle((Vector2D)a, (Vector2D)b);
        }
        public static double CcwAngle(Vector2D a, Vector2D b)
        {
            var diff = b - a;
            double angle = Math.Atan2(-diff.y, diff.x);
            if (angle < 0f)
                angle += 2 * Math.PI;
            return angle;
        }
        public const float Epsilon = 0.1f, Epsilon01 = Epsilon / MaxValue, CollEpsilon = Epsilon01;
        public const double EpsilonD = 1e-8, EpsilonD01 = EpsilonD / MaxValue, 
            AngleEpsilon = 1e-12, CollEpsilonD = EpsilonD01;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int NextPowerOfTwo(int x)
        {
            x--;
            x |= (x >> 1);
            x |= (x >> 2);
            x |= (x >> 4);
            x |= (x >> 8);
            x |= (x >> 16);

            return (x + 1);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int CeilDiv(int dividend, int divisor)
        {
            return (dividend + (divisor - 1)) / divisor;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Log2Ceil(int n)
        {
            int bits = 0;
            while (n > 1)
            {
                bits++;
                n >>= 1;
            }
            return bits;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Clamp(float value, float min, float max)
        {
            if (value < min)
                return min;

            if (value > max)
                return max;
            return value;
        }

        public static bool ApproxAngle(double angle1, double angle2)
        {
            return Approximately(angle1, angle2, AngleEpsilon);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Approximately(double value1, double value2, double tolerance = EpsilonD)
        {
            return Math.Abs(value1 - value2) <= tolerance;
        }

        public static bool Collinear(double orientation)
        {
            return Approximately(orientation, 0, CollEpsilonD);
        }

        public static bool Collinear(float orientation)
        {
            return Approximately(orientation, 0f, CollEpsilon);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Approximately(float value1, float value2, float tolerance = Epsilon)
        {
            /*float diff = value1 - value2;
            return diff > -Epsilon && diff < Epsilon;*/
            return Math.Abs(value1 - value2) <= tolerance;
        }

        public static bool Approximately(Vector2 v1, Vector2 v2, float tolerance = Epsilon)
        {
            return Approximately(v1.x, v2.x, tolerance) && Approximately(v1.y, v2.y, tolerance);
        }

        public static bool Approximately(Vector2D v1, Vector2D v2, double tolerance = EpsilonD)
        {
            return Approximately(v1.x, v2.x, tolerance) && Approximately(v1.y, v2.y, tolerance);
        }

        public static int Compare(float value1, float value2, float tolerance = Epsilon)
        {
            if (Approximately(value1, value2, tolerance))
                return 0;
            if (value1 < value2)
                return -1;
            return 1;
        }

        // True if angle is 0, false if 180
        // Assumes three points are collinear
        public static bool IsVectorAngle0(Vector2 center, Vector2 angleP1, Vector2 angleP2)
        {
            int cmpX1 = Compare(center.x, angleP1.x);
            int cmpX2 = Compare(center.x, angleP2.x);
            int cmpY1 = Compare(center.y, angleP1.y);
            int cmpY2 = Compare(center.y, angleP2.y);
            if ((cmpX1 == 0 && cmpY1 == 0) || (cmpX2 == 0 && cmpY2 == 0))
                return true;
            return cmpX1 == cmpX2 &&
                   cmpY1 == cmpY2;
        }

        // https://www.geeksforgeeks.org/orientation-3-ordered-points/
        // To find orientation of ordered triplet    
        // (p1, p2, p3). The function returns    
        // following values    
        // 0 --> p, q and r are colinear 
        // (positive) --> Counterclockwise 
        // (negative) --> Clockwise 
        // z-coordinate of Cross product (determinant) p2p3 x p1p2
        // Precision problem: Collinear(OrientationF((0, 100), (91.666664, 0), (55, 40))) = true, 
        // yet is not even close to being collinear
        public static float Orientation(Vector2 p1, Vector2 p2, Vector2 p3)
        {
            float orientation = (p2.y - p1.y) * (p3.x - p2.x) -
                            (p2.x - p1.x) * (p3.y - p2.y);
            #if DEBUG
            if (Collinear(orientation) != Collinear(OrientationD(p1, p2, p3)))
            {
                throw new Exception(string.Format(
                    "Not enough precision for Orientation({0}, {1}, {2}) = Approx0F({3}) != Approx0D({4})",
                    p1, p2, p3, orientation, OrientationD(p1, p2, p3)));
            }
            #endif
            return orientation;
        }

        public static double OrientationD(Vector2 p1, Vector2 p2, Vector2 p3)
        {
            return OrientationD((Vector2D)p1, (Vector2D)p2, (Vector2D)p3);
        }

        public static double OrientationD(Vector2D p1, Vector2D p2, Vector2D p3)
        {
            return (p2.y - p1.y) * (p3.x - p2.x) -
                            (p2.x - p1.x) * (p3.y - p2.y);
        }

        public static int Orientation(Vector2Int p1, Vector2Int p2, Vector2Int p3)
        {
            long orientation = (long)(p2.y - p1.y) * (p3.x - p2.x) -
                            (long)(p2.x - p1.x) * (p3.y - p2.y);
            return Math.Sign(orientation);
        }

        // https://www.element84.com/blog/determining-the-winding-of-a-polygon-given-as-a-set-of-ordered-points
        // Assumes left hand coordinate system (y-axis is inverted)
        public static bool IsPolygonClockwise(List<Vector2> polygon)
        {
            // Calculate signed area
            float area = 0f;
            var prevP = polygon[polygon.Count - 1];
            for (int i = 0; i < polygon.Count; i++)
            {
                var p = polygon[i];
                area += (p.x - prevP.x) * (p.y + prevP.y);
                prevP = p;
            }
            return area < 0f;
        }

        // Checks if p2 is a convex vertex in a simple polygon in clockwise order 
        // (in left hand coordinate system) p1 is point before p2 and p3 is point after p2
        public static bool IsConvex(Vector2 p1, Vector2 p2, Vector2 p3)
        {
            // The interior angle of polygon is clockwise angle from p2p3 to p1p2
            // so if p2p3 turns clockwise towards p1p2 then inside angle < 180 (convex)
            return OrientationD(p1, p2, p3) < -CollEpsilonD;
        }

        // Check if p is in clockwise angle (exclusive) between center angleP1 and center angleP2
        // Assumes angleP1, angleP2, center is in counter-clockwise order
        public static bool PointInCwAngle(Vector2 center, Vector2 angleP1, Vector2 angleP2, Vector2 p,
            double epsilon = CollEpsilonD)
        {
            double orientation1 = OrientationD(center, angleP1, p);
            double orientation2 = OrientationD(center, angleP2, p);
            // (Not collinear) center angleP1 p is a clockwise turn and
            // center angleP2 p is a ccw turn
            if (MathExt.IsConvex(angleP2, center, angleP1))
                return orientation1 < -epsilon && orientation2 > epsilon;
            else // Orientation check does not work if angle space > 180, so negate ccw check
                return orientation1 < -epsilon || orientation2 > epsilon;
        }

        public static void Swap<T>(ref T value1, ref T value2)
        {
            var temp = value1;
            value1 = value2;
            value2 = temp;
        }

        public static void MinMax(ref double min, ref double max)
        {
            if (max < min)
                Swap(ref min, ref max);
        }

        public static void MinMax(ref float min, ref float max)
        {
            if (max < min)
                Swap(ref min, ref max);
        }

        // If between is between start and end return true (exclusive)
        // Returns true if start = between = end (approx)
        private static bool BetweenExcl(double start, double between, double end,
            double epsilon = EpsilonD)
        {
            if (Approximately(start, end, epsilon))
                return Approximately(start, between, epsilon);
            MinMax(ref start, ref end);
            return between > (start + epsilon) && between < (end - epsilon);
        }

        public static bool Between(float start, float between, float end)
        {
            if (Approximately(start, end))
                return Approximately(start, between);
            MinMax(ref start, ref end);
            return between >= start && between <= end;
        }

        public static bool OnSegmentExcl(Vector2D start, Vector2D between, Vector2D end)
        {
            return BetweenExcl(start.x, between.x, end.x) && BetweenExcl(start.y, between.y, end.y);
        }

        public static bool OnSegmentIncl(Vector2 start, Vector2 between, Vector2 end)
        {
            return Between(start.x, between.x, end.x) && Between(start.y, between.y, end.y);
        }

        // Max/min value without causing precision errors in context
        public const float MaxValue = 999999f, MinValue = -MaxValue;

        // from: https://github.com/prime31/Nez/blob/master/Nez.Portable/Physics/Collisions.cs
        // Does not calculate intersection if there is none 
        // Intersection within epsilon of an endpoint does not count
        /*public static bool LineToLineOpt(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2, out Vector2 intersection)
        {
            intersection = new Vector2(float.NaN, float.NaN);

            var b = a2 - a1;
            var d = b2 - b1;
            float bDotDPerp = b.x * d.y - b.y * d.x;

            // If b dot d == 0, it means the lines are parallel so have infinite intersection points
            if (Approximately(bDotDPerp, 0))
                return false;

            var c = b1 - a1;
            float t = (c.x * d.y - c.y * d.x) / bDotDPerp;
            if (t < Epsilon || t > 1 - Epsilon)
                return false;

            var u = (c.x * b.y - c.y * b.x) / bDotDPerp;
            if (u < Epsilon || u > 1 - Epsilon)
                return false;

            intersection = a1 + t * b;

            return true;
        }*/

        public static bool LineToLine(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2, out Vector2 intersection)
        {
            return LineToLine(a1, a2, b1, b2, false, out intersection);
        }

        public static bool HalfLineToLine(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2, out Vector2 intersection)
        {
            return LineToLine(a1, a2, b1, b2, true, out intersection);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool LineToLine(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2, bool halfLine,
            out Vector2 intersection)
        {
            Vector2D intersectionD;
            bool intersects = LineToLine((Vector2D)a1, (Vector2D)a2, (Vector2D)b1, (Vector2D)b2, 
                halfLine, out intersectionD);
            intersection = (Vector2)intersectionD;
            return intersects;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool LineToLine(Vector2D a1, Vector2D a2, Vector2D b1, Vector2D b2, bool halfLine, 
            out Vector2D intersection)
        {
            Vector2D b;
            double t;
            bool intersects = LineToLine(a1, a2, b1, b2, halfLine, out b, out t);
            intersection = a1 + t * b;
            return intersects && OnSegmentExcl(b1, intersection, b2);
            /*return OnSegment(a1, intersection, a2) && 
                   OnSegment(b1, intersection, b2);*/
            //return !(t < 0 || t > 1 || u < 0 || u > 1);
        }
        
        // If halfLine then HalfLineToLine (not segment), else LineSegmentToLine
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool LineToLine(Vector2D a1, Vector2D a2, Vector2D b1, Vector2D b2,
            bool halfLine, out Vector2D b, out double t)
        {
            t = double.NaN;

            b = a2 - a1;
            var d = b2 - b1;
            double bDotDPerp = b.x * d.y - b.y * d.x;

            // If b dot d == 0, it means the lines are parallel so have infinite intersection points
            if (Approximately(bDotDPerp, 0))
                return false;

            var c = b1 - a1;
            t = (c.x * d.y - c.y * d.x) / bDotDPerp;
            return t > EpsilonD01 && (halfLine || t < 1 - EpsilonD01);
        }

        //https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
        public static bool IsInside(Vector2[] polygon, int n, Vector2 p)
        {
            // There must be at least 3 vertices in polygon[] 
            if (n < 3)
            {
                return false;
            }

            // Create a point for line segment from p to infinite 
            Vector2 extreme = new Vector2(float.PositiveInfinity, p.y);

            // Count intersections of the above line 
            // with sides of polygon 
            int count = 0, i = 0;
            do
            {
                int next = (i + 1) % n;

                // Check if the line segment from 'p' to 
                // 'extreme' intersects with the line 
                // segment from 'polygon[i]' to 'polygon[next]' 
                if (LineToLine(polygon[i], polygon[next], p, extreme, out Vector2 _))
                {
                    //we do not check if p lies on a
                    //line segment of a polygon since
                    //the polygons in our case are defined
                    //as open sets
                    count++;
                }
                i = next;
            } while (i != 0);

            // Return true if count is odd, false otherwise 
            return (count % 2 == 1);
        }

        public static double PathCost(List<Vector2> path)
        {
            double total = 0;
            for (int i = 0; i < path.Count - 1; i++)
                total += Vector2D.Distance((Vector2D)path[i], (Vector2D)path[i + 1]);
            return total;
        }

        public static double PathCost(List<Vector2Int> path)
        {
            double total = 0;
            for (int i = 0; i < path.Count - 1; i++)
                total += Vector2D.Distance((Vector2D)(Vector2)path[i], (Vector2D)(Vector2)path[i + 1]);
            return total;
        }
    }
}
