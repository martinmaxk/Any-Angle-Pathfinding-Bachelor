using System;
using System.Collections.Generic;
using MathTools;
using Advanced.Algorithms.DataStructures;

namespace Triangulation
{
    public partial class PolygonTriangulator
    {
        private PolygonMonotoneDecomposer decomposer = new PolygonMonotoneDecomposer();
        private MonotonePolygonTriangulator triangulator = new MonotonePolygonTriangulator();
        private List<EdgeNode> monoHeads = new List<EdgeNode>(),
            triHeads = new List<EdgeNode>(), allNodes = new List<EdgeNode>();
        private List<RedBlackTree<EdgeNode>> edges = new List<RedBlackTree<EdgeNode>>();
        private List<Diagonal> diagonals = new List<Diagonal>();
        private EdgePool edgePool = new EdgePool(4);
        private EdgeNodeDuplicateComparer duplicateComparer = new EdgeNodeDuplicateComparer();
        private EdgeNodeAngleComparer angleComparer = new EdgeNodeAngleComparer();

        // Stores result in triangles
        public void Triangulate(List<Vector2> polygon, List<List<Vector2>> triangles, ref int start)
        {
            angleComparer.polygon = polygon;
            while (edges.Count < polygon.Count)
                edges.Add(new RedBlackTree<EdgeNode>(angleComparer));
            for (int i = 0; i < polygon.Count; i++)
                edges[i].Clear();
            // Create edge nodes and resolve duplicate vertices
            SplitDuplicateVertices(polygon, triHeads);
            for (int i = 0; i < triHeads.Count; i++)
                decomposer.Decompose(polygon, triHeads[i], monoHeads, edges, diagonals, edgePool);
            triHeads.Clear();
            for (int i = 0; i < monoHeads.Count; i++)
                triangulator.Triangulate(polygon, monoHeads[i], triHeads, edges, diagonals, edgePool);
            while (triangles.Count < start + triHeads.Count)
                triangles.Add(new List<Vector2>(3));
            var expanded = new List<PolyBoolOpMartinez.Polygon>();
            for (int i = 0; i < triHeads.Count; i++)
            {
                var edgeNode = triHeads[i];
                var triangle = triangles[start + i];
                triangle.Clear();
                for (int j = 0; j < 3; j++)
                {
                    triangle.Add(polygon[edgeNode.id]);
                    edgeNode = edgeNode.next;
                }
                Debugging.Debug.Assert(edgeNode.id == triHeads[i].id, "Not a triangle");
            }
            start += triHeads.Count;
            monoHeads.Clear();
            triHeads.Clear();
            edgePool.ReturnAll();
        }

        public void SplitDuplicateVertices(List<Vector2> polygon, List<List<Vector2>> resPolys)
        {
            SplitDuplicateVertices(polygon, triHeads);
            for (int i = 0; i < triHeads.Count; i++)
                resPolys.Add(ToList(polygon, triHeads[i]));
            triHeads.Clear();
            edgePool.ReturnAll();
        }

        private void SplitDuplicateVertices(List<Vector2> polygon, List<EdgeNode> resHeads)
        {
            var edgeHead = edgePool.Create(id: 0);
            allNodes.Add(edgeHead);
            resHeads.Add(edgeHead);
            var edgeTail = edgeHead;
            for (int i = 1; i < polygon.Count; i++)
            {
                var edgeNode = edgePool.Create(id: i);
                allNodes.Add(edgeNode);
                edgeTail.ConnectTo(edgeNode);
                edgeTail = edgeNode;
            }
            edgeTail.ConnectTo(edgeHead);
            duplicateComparer.polygon = polygon;
            allNodes.Sort(duplicateComparer);
            for (int i = 0; i < allNodes.Count - 1; i++)
            {
                var node1 = allNodes[i];
                var node2 = allNodes[i + 1];
                if (polygon[node1.id] != polygon[node2.id])
                    continue;
                var temp = node2.prev;
                node1.prev.ConnectTo(node2);
                temp.ConnectTo(node1);
                resHeads.Add(node1);
                resHeads.Add(node2);
            }
            DistinctPolygons(resHeads);
            allNodes.Clear();
        }

        private static void LogPoly(List<Vector2> polygon, EdgeNode edgeHead)
        {
            var poly = new PolyBoolOpMartinez.Polygon();
            poly.FromEnumerable(ToList(polygon, edgeHead));
            Console.WriteLine(poly.ToGeogebra());
        }

        private static List<Vector2> ToList(List<Vector2> polygon, EdgeNode edgeHead)
        {
            var list = new List<Vector2>();
            var n = edgeHead;
            do
            {
                list.Add(polygon[n.id]);
                n = n.next;
            } while (n.id != edgeHead.id);
            return list;
        }

        // Does p lie above q?
        private static bool IsAbove(Vector2 p, Vector2 q)
        {
            return (MathExt.Approximately(p.y, q.y) && p.x < q.x) ||
                    p.y < q.y;
        }

        private static bool IsInteriorRightOf(List<Vector2> polygon, EdgeNode edgeNode)
        {
            var prev = polygon[edgeNode.prev.id];
            var cur = polygon[edgeNode.id];
            // if current is above prev or below next and polygon is clockwise 
            // then interior of polygon is on the right side of current
            // Does not hold when prev.y = cur.y or next.y = cur.y resp.,
            if (MathExt.Approximately(prev.y, cur.y))
                return cur.y > polygon[edgeNode.next.id].y;
            // but prev.y = cur.y = next.y is impossible
            return cur.y < prev.y; 
        }
        
        private static void DistinctPolygons(List<EdgeNode> edgeHeads)
        {
            for (int i = 0; i < edgeHeads.Count; i++)
            {
                var edgeHead = edgeHeads[i];
                var edgeNode = edgeHead;
                // Remove duplicate head of polygon
                if (edgeNode.id < 0)
                {
                    edgeHeads[i] = edgeHeads[edgeHeads.Count - 1];
                    edgeHeads.RemoveAt(edgeHeads.Count - 1);
                    i--;
                    continue;
                }
                do
                {
                    // Mark each node in polygon connected to this head
                    edgeNode.id = -(edgeNode.id + 1);
                    edgeNode = edgeNode.next;
                } while (edgeNode.id != edgeHead.id);
            }
            // All edgeNode.id are now negated, so negate them back
            for (int i = 0; i < edgeHeads.Count; i++)
            {
                var edgeHead = edgeHeads[i];
                var edgeNode = edgeHead;
                do
                {
                    edgeNode.id = (-edgeNode.id) - 1;
                    edgeNode = edgeNode.next;
                } while (edgeNode.id != edgeHead.id);
            }
        }

        private static void ApplyDiagonalPartitions(List<Vector2> polygon, List<EdgeNode> edgeHeads, 
            List<RedBlackTree<EdgeNode>> edges, List<Diagonal> diagonals, EdgePool edgePool)
        {
            for (int i = 0; i < diagonals.Count; i++)
            {
                var diagonal = diagonals[i];
                DiagonalPartition(polygon, edgeHeads, edges, diagonal, edgePool);
            }
        }

        private static void RemoveCollinearEdges(List<Vector2> polygon, List<EdgeNode> edgeHeads)
        {
            for (int i = 0; i < edgeHeads.Count; i++)
            {
                var edgeHead = edgeHeads[i];
                RemoveCollinearEdges(polygon, ref edgeHead);
                edgeHeads[i] = edgeHead;
            }
        }

        private static void RemoveCollinearEdges(List<Vector2> polygon, ref EdgeNode edgeHead)
        {
            var edgeNode = edgeHead;
            while (true)
            {
                if (MathExt.Collinear(MathExt.OrientationD(polygon[edgeNode.prev.id], 
                    polygon[edgeNode.id], polygon[edgeNode.next.id])))
                {
                    edgeNode.prev.ConnectTo(edgeNode.next);
                    var temp = edgeNode.next;
                    edgeNode.next = edgeNode.prev = null;
                    if (edgeNode.id == edgeHead.id)
                    {
                        edgeHead = edgeNode = temp;
                        continue;
                    }
                    edgeNode = temp;
                }
                else
                    edgeNode = edgeNode.next;
                if (edgeNode.id == edgeHead.id)
                    break;
            }
        }

        // Insert a diagonal between v1 and v2 which splits the polygon in 2 pieces
        private static void DiagonalPartition(List<Vector2> polygon, List<EdgeNode> edgeHeads,
           List<RedBlackTree<EdgeNode>> edges, Diagonal diagonal, EdgePool edgePool)
        {
            // Add diagonal as head of each subpolygon
            // Ignore old head for now
            var diagonal1 = edgePool.Create(id: diagonal.toVId);
            edgeHeads.Add(diagonal1);
            var diagonal2 = edgePool.Create(id: diagonal.fromVId);
            edgeHeads.Add(diagonal2);
            // Temp set next for AngleComparer
            diagonal1.next = diagonal2;
            diagonal2.next = diagonal1;

            var v1Edges = edges[diagonal.fromVId];
            var v2Edges = edges[diagonal.toVId];

            var v1Edge = FindEdge(polygon, v1Edges, diagonal2);
            var v2Edge = FindEdge(polygon, v2Edges, diagonal1);

            // v1Edge and v2Edge are always in opposite polygons
            v1Edge.prev.ConnectTo(diagonal2);
            diagonal1.ConnectTo(v1Edge);
            v2Edge.prev.ConnectTo(diagonal1);
            diagonal2.ConnectTo(v2Edge);

            edges[diagonal.fromVId].Insert(diagonal2);
            edges[diagonal.toVId].Insert(diagonal1);
        }

        private static EdgeNode FindEdge(List<Vector2> polygon, RedBlackTree<EdgeNode> edges, EdgeNode queryNode)
        {
            BSTIterator<EdgeNode> predIter, succIter;
            edges.FindPredSucc(queryNode, out predIter, out succIter);
            var toV = polygon[queryNode.next.id];
            // If no edge with lesser ccw angle exists, pick max as pred
            if (predIter.IsNull())
                predIter = edges.FindMax();
            if (!predIter.IsNull())
            {
                var pred = predIter.Value;
                if (MathExt.PointInCwAngle(polygon[pred.id], polygon[pred.next.id], polygon[pred.prev.id], toV))
                    return pred;
            }
            // If no edge with greater ccw angle exists, pick min as succ
            if (succIter.IsNull())
                succIter = edges.FindMin();
            var succ = succIter.Value;
            Debugging.Debug.Assert(
                MathExt.PointInCwAngle(polygon[succ.id], polygon[succ.next.id], polygon[succ.prev.id], toV));
            return succ;
        }

        private struct Diagonal
        {
            public int fromVId, toVId;

            public Diagonal(int fromVId, int toVId)
            {
                this.fromVId = fromVId;
                this.toVId = toVId;
            }
        }

        // A double-linked list node in a list of edges
        private class EdgeNode
        {
            public int id;
            // Helper is actually vertex helper.id, but 
            // we store a reference to the incident edges of the vertex
            public EdgeNode helper;
            public EdgeNode prev, next;

            public void ConnectTo(EdgeNode other)
            {
                next = other;
                other.prev = this;
            }
        }

        // Avoid generating garbage by reusing nodes across multiple triangulate calls
        // Stack where top moves, but elements are never overwritten
        private class EdgePool
        {
            private EdgeNode[] arr;
            private int top = -1, count;

            public EdgePool(int capacity)
            {
                arr = new EdgeNode[capacity];
            }

            public EdgeNode Create(int id)
            {
                EdgeNode node;
                if (top >= 0)
                    node = arr[top--];
                else
                {
                    if (count == arr.Length)
                        Array.Resize(ref arr, count * 2);
                    node = arr[count++] = new EdgeNode();
                }
                node.id = id;
                // Assumes pointers can only be to nodes in arr
                // Outside nodes are kept alive by references after ReturnAll
                node.helper = node.next = node.prev = null;
                return node;
            }

            // Make all nodes in pool available
            public void ReturnAll()
            {
                top = count - 1;
            }
        }

        private class EdgeNodeDuplicateComparer : Comparer<EdgeNode>
        {
            public List<Vector2> polygon;

            public override int Compare(EdgeNode node1, EdgeNode node2)
            {
                var v1 = polygon[node1.id];
                var v2 = polygon[node2.id];
                if (v1.x != v2.x)
                    return v1.x.CompareTo(v2.x);
                if (v1.y != v2.y)
                    return v1.y.CompareTo(v2.y);
                return node1.id.CompareTo(node2.id);
            }
        }

        private class EdgeNodeAngleComparer : Comparer<EdgeNode>
        {
            public List<Vector2> polygon;

            public override int Compare(EdgeNode node1, EdgeNode node2)
            {
                Debugging.Debug.Assert(node1.id == node2.id, "Edge must start from same endpoint");
                var center = polygon[node1.id];
                var v1 = polygon[node1.next.id];
                var v2 = polygon[node2.next.id];
                double orientation = MathExt.OrientationD(center, v2, v1);
                Debugging.Debug.Assert(!(MathExt.Collinear(orientation) && 
                    MathExt.IsVectorAngle0(center, v2, v1)), 
                    "Diagonals cannot be collinear");
                int cmp1 = MathExt.Compare(v1.y, center.y);
                int cmp2 = MathExt.Compare(v2.y, center.y);
                if (cmp1 == cmp2)
                {
                    // If v1.y == v2.y 
                    // then if v1 is right of center, then v1 is 0 deg and v2 is 180 deg else reverse
                    if (cmp1 == 0)
                        return v1.x > center.x ? -1 : 1;
                    if (orientation > 0)
                        return 1;
                    else
                        return -1;
                }
                if (cmp1 == 0)
                    return v1.x > center.x ? -1 : -cmp2;
                if (cmp2 == 0)
                    return v2.x > center.x ? 1 : cmp1;
                return cmp1;
            }
        }
    }
}
