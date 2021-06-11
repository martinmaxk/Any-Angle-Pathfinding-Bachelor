using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;
using MathTools;
using Advanced.Algorithms.DataStructures;
using PolyBoolOpMartinez;
using Debugging;

namespace Pathfinding.VG
{
    public class RotationalPlaneSweep
    {
        private VisVertices obstacles;
        private DynamicArray<int> sortedIds;
        // Counter-clockwise angle/distance between p and w_i
        private double[] angles;
        private float[] squaredDists;
        // Tree of edges, key is distance between p and edge
        private RedBlackTree<EdgeNode> tree;

        private AngleComparer angleComparer;
        private EdgeDistComparer edgeComparer;
        // Place points that are not polygon vertices at the end
        private int lastPolyEndI;
        // Closest collinear vertices with an edge on both sides (blocks rho)
        private int closestCcwCwId;
        public bool Reduced { get; private set; }

        public RotationalPlaneSweep(bool reduced)
        {
            edgeComparer = new EdgeDistComparer(this);
            tree = new RedBlackTree<EdgeNode>(edgeComparer, new EdgeIdEqComparer());
            this.Reduced = reduced;
        }

        // Is point with id a vertex on a polygon?
        private bool IsPoly(int id)
        {
            return id <= lastPolyEndI;
        }

        // Is p in non-taut region of polyPId?
        private bool IsInNonTautRegion(int polyPId, Vector2 p)
        {
            // Translate all points by -center, so center is origo (0, 0)
            var center = obstacles.V(polyPId);
            // Mirror the angle points in center point, i.e. origo, so negate
            var angleP1 = -(obstacles.NextV(polyPId) - center);
            var angleP2 = -(obstacles.PrevV(polyPId) - center);
            p -= center;
            // is p - polyP in clockwise angle between 
            // -(polyPId nextPolyId) and -(polyId prevPolyId)
            return MathExt.PointInCwAngle(Vector2.Zero, angleP1, angleP2, p);
        }

        private bool Visible(Vector2 p, int wId, bool prevVisible,
            float intersectSqrdDist)
        {
            var w = obstacles.V(wId);
            // This check handles the case where
            // pw_i is blocked by the interior of the polygon that w_i is on
            if (IsPoly(wId) && MathExt.PointInCwAngle(w, obstacles.NextV(wId), obstacles.PrevV(wId), p))
                return false;
            // w_i does not have same angle as w_i-1 so w_i
            // is not on the same line segment as w_i-1 
            // add non-poly points even if this is a reduced vg
            if (prevVisible)
                return squaredDists[wId] <= intersectSqrdDist;
            else
                return false;
        }

        private int ModifyTreeAux(int pId, int wId, int incidentId, bool insert)
        {
            // Insert into queue the obstacle edges incident to w_i on ccw side of rho
            // Delete from queue the obstacle edges incident to w_i on cw side of rho

            // Priority is distance from p to edge, so order in which rho hits edge
            // use squared distance from p to intersection between rho and edge

            var node = new EdgeNode(incidentId);
            if (wId == incidentId)
                incidentId = obstacles.NextID(wId);
            var p = obstacles.V(pId);
            double orientation = MathExt.OrientationD(p, obstacles.V(wId), obstacles.V(incidentId));
            // Ignore edges collinear with rho
            if (MathExt.Collinear(orientation))
                return 0;
            if (orientation > 0)
            {
                if (insert)
                    tree.Insert(node);
                return 1;
            }
            else if (orientation < 0)
            {
                if (!insert && !MathExt.ApproxAngle(angles[wId], 0))
                {
                    int delIndex = tree.Delete(node);
                    Debug.Assert(delIndex != -1, "Node was not found in tree");
                }
                return -1;
            }
            else
                throw new Exception("Orientation = 0, yet was not detected by Approx0");
        }

        // Returns should loop continue?
        private bool ModifyTree(int pId, int startI, int i, bool insert)
        {
            int wId = sortedIds[i];
            bool nextCollinear = i != sortedIds.Count - 1 &&
                MathExt.ApproxAngle(angles[wId], angles[sortedIds[i + 1]]);
            if (!IsPoly(wId))
                return nextCollinear;
            edgeComparer.SetW(obstacles.V(wId));
            int prevPolyPId = obstacles.PrevID(wId);
            // Edge going from prevPolyId to wId
            int orientation1 = ModifyTreeAux(pId, wId, prevPolyPId, insert);
            // Edge with id wId is going from wId to nextPolyId
            int orientation2 = ModifyTreeAux(pId, wId, wId, insert);
            return nextCollinear;
        }

        public void VisibleVertices(int pId, VisVertices obstacles, ref DynamicArray<Edge> edges)
        {
            edges.Clear();
            this.obstacles = obstacles;
            var p = obstacles.V(pId);
            lastPolyEndI = obstacles.vs.Count - 1;
            while (!obstacles.IsDescriptor(lastPolyEndI))
                lastPolyEndI--;
            bool isPPoly = pId <= lastPolyEndI;
            // If Reduced & p is concave, do not add any edges
            bool isPConvex = true;

            if (angles == null || angles.Length < obstacles.vs.Count)
                angles = new double[obstacles.vs.Count + 2];
            if (squaredDists == null || squaredDists.Length < obstacles.vs.Count)
                squaredDists = new float[obstacles.vs.Count + 2];
            angleComparer = angleComparer ?? new AngleComparer();
            angleComparer.Reset(obstacles.vs.arr, angles, squaredDists);

            // If isPPoly or p is on an edge, sweep exterior angle between incident edges of p
            // otherwise sweep 360 degrees from angle 0
            double initAngle = 0, stopAngle = 2 * Math.PI;
            var initRho = new Vector2(MathExt.MaxValue, p.y);
            if (isPPoly)
            {
                isPConvex = MathExt.IsConvex(obstacles.PrevV(pId), p, obstacles.NextV(pId));
                initRho = obstacles.NextV(pId);
                initAngle = MathExt.CcwAngle(p, initRho);
                stopAngle = MathExt.CcwAngle(p, obstacles.PrevV(pId));
                stopAngle -= initAngle;
                if (stopAngle < 0)
                    stopAngle += 2 * Math.PI;
            }
            else
            {
                for (int i = 0; i < obstacles.vs.Count; i++)
                {
                    var vertex = obstacles.V(i);
                    if (i == pId || VisVertices.IsDescriptor(vertex) || !IsPoly(i))
                        continue;
                    var nextV = obstacles.NextV(i);
                    if (MathExt.Collinear(MathExt.OrientationD(vertex, p, nextV)) &&
                        MathExt.OnSegmentExcl((Vector2D)vertex, (Vector2D)p, (Vector2D)nextV))
                    {
                        initRho = nextV;
                        initAngle = MathExt.CcwAngle(p, initRho);
                        stopAngle = MathExt.CcwAngle(p, vertex);
                        stopAngle -= initAngle;
                        if (stopAngle < 0)
                            stopAngle += 2 * Math.PI;
                    }
                }
            }
            stopAngle += MathExt.AngleEpsilon;
            edgeComparer.SetP(p);
            edgeComparer.SetW(initRho);
            if (sortedIds.arr == null || sortedIds.arr.Length < obstacles.vs.Count)
                sortedIds.arr = new int[obstacles.vs.Count + 2];
            for (int i = 0; i < obstacles.vs.Count; i++)
            {
                var vertex = obstacles.V(i);
                if (i == pId || VisVertices.IsDescriptor(vertex))
                    continue;
                double angle = MathExt.CcwAngle(p, vertex) - initAngle;
                if (angle < 0)
                    angle += 2 * Math.PI;
                angles[i] = angle;
                // Insert all obstacle edges intersected by initRho in tree
                if (IsPoly(i) && MathExt.HalfLineToLine(p, initRho, vertex, obstacles.NextV(i), out Vector2 _))
                    tree.Insert(new EdgeNode(i));
                if (angle > stopAngle && vertex != p)
                    continue;
                sortedIds.Add(i);
                var diff = vertex - p;
                squaredDists[i] = diff.LengthSquared();
            }

            for (int i = 0; i < sortedIds.Count; i++)
            {
                int id = sortedIds[i];
                var vertex = obstacles.V(id);
                if (IsPoly(id) && vertex == p)
                {
                    double angle1 = angles[obstacles.PrevID(id)];
                    double angle2 = angles[obstacles.NextID(id)];
                    double interiorAngle = (angle2 - angle1);
                    if (interiorAngle < 0)
                        interiorAngle += 2 * Math.PI;
                    double midAngle = angle1 + (interiorAngle * 0.5);
                    angles[id] = midAngle;
                }
            }

            Array.Sort<int>(sortedIds.arr, 0, sortedIds.Count, angleComparer);

            bool prevVisible = false;
            var intersection = new Vector2(float.NaN, float.NaN);
            bool closestCcwCwVisible;
            // Is p on or outside a polygon, is p outside the level?
            bool isPInFreespace = false;
            for (int i = 0; i < sortedIds.Count;)
            {
                closestCcwCwVisible = false;
                closestCcwCwId = -1;
                prevVisible = true;
                int startColI = i;
                while (ModifyTree(pId, startColI, i, insert: false) && i < sortedIds.Count - 1)
                    i++;
                int endColI = i;
                var edgeIter = tree.FindMin();
                float intersectSqrdDist = float.MaxValue;
                if (!edgeIter.IsNull() && obstacles.LineToEdge(p, obstacles.V(sortedIds[endColI]),
                    edgeIter.Value.id, out intersection))
                {
                    intersectSqrdDist = Vector2.DistanceSquared(p, intersection);
                }
                //bool nextCollinear = false;
                var firstW = obstacles.V(sortedIds[startColI]);
                for (i = startColI; i <= endColI;)
                {
                    if (closestCcwCwVisible) //&& intersection != w)
                        break;
                    int wId = sortedIds[i];
                    var w = obstacles.V(wId);
                    bool prevCollinear = w != firstW; //nextCollinear;
                    //nextCollinear = i != endI;
                    bool visible = true;
                    int startSameI = i;
                    do
                    {
                        visible &= Visible(p, wId, prevVisible, intersectSqrdDist);
                        i++;
                        if (i == sortedIds.Count)
                            break;
                        wId = sortedIds[i];
                    } while (w == obstacles.V(wId));
                    int endSameI = i - 1;
                    for (i = startSameI; i <= endSameI; i++)
                    {
                        wId = sortedIds[i];
                        bool isWPoly = IsPoly(wId);
                        isPInFreespace |= visible & isWPoly;
                        // Add wId to visiblity tree at p, cost is euclidean distance
                        if (visible & (!prevCollinear | !Reduced))
                        {
                            bool isConvex = !isWPoly ||
                                MathExt.IsConvex(obstacles.PrevV(wId), w, obstacles.NextV(wId));
                            if (!Reduced || (!prevCollinear && isConvex && isPConvex))
                            {
                                if (!Reduced || !isPPoly ||
                                    (!IsInNonTautRegion(pId, w) && !IsInNonTautRegion(wId, p)))
                                    edges.Add(new Edge(wId, (float)Math.Sqrt(squaredDists[wId])));
                            }
                            if (isWPoly)
                            {
                                if (!isConvex)
                                    closestCcwCwId = wId;
                                if (p == w)
                                    closestCcwCwId = wId;
                                // Vertex with edge on each side will always block rho
                                if (closestCcwCwId == wId)
                                {
                                    intersection = w;
                                    closestCcwCwVisible = true;
                                }
                            }
                        }
                    }
                    prevVisible = visible;
                }
                for (i = startColI; i <= endColI; i++)
                    ModifyTree(pId, startColI, i, insert: true);
            }
            tree.Clear();
            sortedIds.Clear();
            // No polygon points are visible, so p must be inside a polygon
            if (!isPInFreespace)
                edges.Clear();
        }

        // Assumes polygons of even depth is in clockwise order
        // and polygons of odd depth (holes, inverse polygons) 
        // are kept in ccw order, so vectors for orientation are also swapped
        public void VisibilityGraph(List<List<Vector2>> polygons,
            ref VisVertices obstaclesRef,
            ref DynamicArray<DynamicArray<Edge>> allEdges)
        {
            this.obstacles = obstaclesRef;

            int count = 0;
            for (int i = 0; i < polygons.Count; i++)
            {
                var polygon = polygons[i];
                count += polygon.Count + 2;
            }
            if (obstacles.vs.arr.Length < count + 2)
                obstacles.vs.arr = new Vector2[count + 2];
            obstacles.vs.ResizeAndExtendTo(count);
            lastPolyEndI = obstacles.vs.Count;
            count = 0;
            for (int i = 0; i < polygons.Count; i++)
            {
                var polygon = polygons[i];
                int head = count++;
                for (int j = 0; j < polygon.Count; j++)
                    obstacles.vs[count++] = polygon[j];

                obstacles.SetDescriptor(head, count - 1);
                obstacles.SetDescriptor(count++, head + 1);
            }

            if (allEdges.arr.Length < count + 2)
                allEdges.arr = new DynamicArray<Edge>[count + 2];
            allEdges.ResizeAndExtendTo(obstacles.vs.Count);

            for (int i = 0; i < obstacles.vs.Count; i++)
            {
                if (obstacles.IsDescriptor(i))
                    continue;
                var edges = allEdges[i];
                if (edges.arr == null)
                    edges = new DynamicArray<Edge>(0);
                else
                    edges.Clear();
                // Only calculate visibility from convex vertices
                if (Reduced &&
                    !MathExt.IsConvex(obstacles.PrevV(i), obstacles.V(i), obstacles.NextV(i)))
                    continue;
                VisibleVertices(i, obstacles, ref edges);
                allEdges[i] = edges;
            }
            obstaclesRef = this.obstacles;
        }

        private class AngleComparer : IComparer<int>
        {
            private Vector2[] vertices;
            private double[] angles;
            private float[] squaredDists;

            public void Reset(Vector2[] vertices, double[] angles, float[] squaredDists)
            {
                this.vertices = vertices;
                this.angles = angles;
                this.squaredDists = squaredDists;
            }

            // Primary comparison between angles, secondary between distances from p
            public int Compare(int id1, int id2)
            {
                if (id1 == id2)
                    return 0;
                double angle1 = angles[id1];
                double angle2 = angles[id2];
                if (MathExt.ApproxAngle(angle1, angle2))
                {
                    int compare = squaredDists[id1].CompareTo(squaredDists[id2]);
                    // Sort non-poly points last on ties 
                    // if rho_i intersects interior of poly then poly point will be invisible 
                    // and so non-poly point should also be invisible (!prevVisible)
                    if (compare == 0)
                        return id1 < id2 ? -1 : 1;
                    return compare;
                }
                return angle1 < angle2 ? -1 : 1;
            }
        }

        private struct EdgeNode
        {
            public int id;

            public EdgeNode(int id)
            {
                this.id = id;
            }

            public override string ToString()
            {
                return "EdgeNode ID: " + id;
            }
        }

        private class EdgeDistComparer : Comparer<EdgeNode>
        {
            private Vector2 p, w;
            private readonly RotationalPlaneSweep rps;

            public EdgeDistComparer(RotationalPlaneSweep rps)
            {
                this.rps = rps;
            }

            public void SetP(Vector2 p)
            {
                this.p = p;
            }

            public void SetW(Vector2 w)
            {
                this.w = w;
            }

            public override int Compare(EdgeNode edge1, EdgeNode edge2)
            {
                Vector2 intersection1, intersection2;
                float sqrdDist1 = GetSqrdDist(edge1, out intersection1);
                float sqrdDist2 = GetSqrdDist(edge2, out intersection2);
                // edge1 and edge2 are incident edges and intersect rho in their endpoints
                if (MathExt.Approximately(sqrdDist1, sqrdDist2))
                {
                    // oppA and oppB are opposite endpoints of where the two edges meet
                    Vector2 a1, a2, b1, b2, intersection, oppA, oppB;
                    rps.obstacles.E(edge1.id, out a1, out a2);
                    rps.obstacles.E(edge2.id, out b1, out b2);
                    oppA = MathExt.Approximately(a1, intersection1) ? a2 : a1;
                    oppB = MathExt.Approximately(b1, intersection2) ? b2 : b1;
                    if (MathExt.LineToLine(p, oppB, a1, a2, out intersection))
                    {
                        sqrdDist1 = Vector2.DistanceSquared(p, intersection);
                        sqrdDist2 = Vector2.DistanceSquared(p, oppB);
                    }
                    else
                    {
                        MathExt.LineToLine(p, oppA, b1, b2, out intersection);
                        sqrdDist1 = Vector2.DistanceSquared(p, oppA);
                        sqrdDist2 = Vector2.DistanceSquared(p, intersection);
                    }
                }
                return sqrdDist1.CompareTo(sqrdDist2);
            }

            private float GetSqrdDist(EdgeNode edge, out Vector2 intersection)
            {
                Vector2 a1, a2;
                rps.obstacles.E(edge.id, out a1, out a2);
                MathExt.LineToLine(p, w, a1, a2, out intersection);
                return Vector2.DistanceSquared(p, intersection);
            }
        }

        private class EdgeIdEqComparer : EqualityComparer<EdgeNode>
        {
            public override bool Equals(EdgeNode edge1, EdgeNode edge2)
            {
                return edge1.id == edge2.id;
            }

            public override int GetHashCode(EdgeNode edge)
            {
                return edge.id;
            }
        }
    }

    // Type punning (interpret same bytes as either float or integer)
    [StructLayout(LayoutKind.Explicit)]
    public struct FloatInt
    {
        [FieldOffset(0)]
        public float f;
        [FieldOffset(0)]
        public int i;
    }

    // Vertices of polygons in clockwise order, merged into 1 array
    public struct VisVertices
    {
        public DynamicArray<Vector2> vs;
        private const float DescVal = float.NegativeInfinity;

        // Is vertex a polygon header/footer where index of footer/header is in v.y?
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsDescriptor(Vector2 v)
        {
            return v.x == DescVal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsDescriptor(int vId)
        {
            return IsDescriptor(vs[vId]);
        }

        // Each polygon has a header and footer where you can obtain the index to 
        // first/last point. These entries are marked by x = DescVal
        // We do this to avoid casting because float cannot represent all integers
        public static Vector2 Descriptor(int otherEndI)
        {
            return new Vector2(DescVal, new FloatInt() { i = otherEndI }.f);
        }

        public void SetDescriptor(int vId, int otherEndI)
        {
            vs[vId] = Descriptor(otherEndI);
        }

        // Get prev/next point in polygon
        // If id is at a descriptor, then return its index to opposite end
        public int PrevNextID(int prevNextId)
        {
            var descriptor = vs[prevNextId];
            if (IsDescriptor(descriptor))
                return new FloatInt() { f = descriptor.y }.i;
            return prevNextId;
        }

        public int PrevID(int polyPId)
        {
            return PrevNextID(polyPId - 1);
        }

        public int NextID(int polyPId)
        {
            return PrevNextID(polyPId + 1);
        }

        public Vector2 PrevV(int polyPId)
        {
            return vs[PrevID(polyPId)];
        }

        public Vector2 NextV(int polyPId)
        {
            return vs[NextID(polyPId)];
        }

        public Vector2 V(int polyPId)
        {
            return vs[polyPId];
        }

        public void E(int edgeId, out Vector2 v1, out Vector2 v2)
        {
            v1 = V(edgeId);
            v2 = NextV(edgeId);
        }

        public bool LineToEdge(Vector2 a1, Vector2 a2, int edgeId, out Vector2 intersection)
        {
            return MathExt.LineToLine(a1, a2, V(edgeId), NextV(edgeId), out intersection);
        }
    }
}
