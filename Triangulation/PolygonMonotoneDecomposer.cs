using System.Collections.Generic;
using MathTools;
using Advanced.Algorithms.DataStructures;

namespace Triangulation
{
    public partial class PolygonTriangulator
    {
        // Class to decompose a simple polygon into monotone pieces
        private class PolygonMonotoneDecomposer
        {
            private List<SweepVertex> sweepVertices = new List<SweepVertex>();
            private SweeplineVertexComparer vertexComparer = new SweeplineVertexComparer();
            private EdgeNodeComparer edgeComparer = new EdgeNodeComparer();
            // Used to perform vertex query on edge tree (find edge directly left of vertex)
            private EdgeNode dummyNode1 = new EdgeNode(), dummyNode2 = new EdgeNode();

            private List<Vector2> polygon;
            private List<EdgeNode> edgeHeads;
            private List<Diagonal> diagonals;
            private EdgePool edgePool;
            private RedBlackTree<EdgeNode> tree;

            public PolygonMonotoneDecomposer()
            {
                tree = new RedBlackTree<EdgeNode>(edgeComparer);
                dummyNode1.ConnectTo(dummyNode2);
            }

            // Polygon must be in clockwise order
            // Stores result as doubly linked lists in edgeHeads
            public void Decompose(List<Vector2> polygon, EdgeNode edgeHead, List<EdgeNode> edgeHeads,
                List<RedBlackTree<EdgeNode>> edges, List<Diagonal> diagonals, EdgePool edgePool)
            {
                this.polygon = polygon;
                this.edgeHeads = edgeHeads;
                this.diagonals = diagonals;
                this.edgePool = edgePool;
                vertexComparer.polygon = edgeComparer.polygon = polygon;
                edgeHeads.Add(edgeHead);
                var edgeNode = edgeHead;
                do
                {
                    edges[edgeNode.id].Clear();
                    edges[edgeNode.id].Insert(edgeNode);
                    sweepVertices.Add(new SweepVertex(edgeNode));
                    edgeNode = edgeNode.next;
                } while (edgeNode.id != edgeHead.id);
                sweepVertices.Sort(vertexComparer);
                dummyNode1.id = polygon.Count;
                polygon.Add(default(Vector2));
                dummyNode2.id = polygon.Count;
                polygon.Add(default(Vector2));
                for (int i = 0; i < sweepVertices.Count; i++)
                {
                    edgeNode = sweepVertices[i].edgeNode;
                    var vPos = polygon[edgeNode.id];
                    edgeComparer.sweepY = vPos.y;
                    switch (GetVType(edgeNode))
                    {
                        case VType.Start:
                            tree.Insert(edgeNode.prev);
                            edgeNode.prev.helper = edgeNode;
                            break;
                        case VType.End:
                            OldHelperMerge(edgeNode);
                            break;
                        case VType.Split:
                            var leftEdge = EdgeLeftOf(vPos);
                            diagonals.Add(new Diagonal(edgeNode.id, leftEdge.helper.id));
                            leftEdge.helper = edgeNode;
                            tree.Insert(edgeNode.prev);
                            edgeNode.prev.helper = edgeNode;
                            break;
                        case VType.Merge:
                            OldHelperMerge(edgeNode);
                            HelperOfEdgeLeftOf(edgeNode);
                            break;
                        case VType.Regular:
                            if (IsInteriorRightOf(polygon, edgeNode))
                            {
                                OldHelperMerge(edgeNode);
                                tree.Insert(edgeNode.prev);
                                edgeNode.prev.helper = edgeNode;
                            }
                            else
                                HelperOfEdgeLeftOf(edgeNode);
                            break;
                    }
                }
                ApplyDiagonalPartitions(polygon, edgeHeads, edges, diagonals, edgePool);
                diagonals.Clear();
                DistinctPolygons(edgeHeads);
                RemoveCollinearEdges(polygon, edgeHeads);
                polygon.RemoveAt(polygon.Count - 1);
                polygon.RemoveAt(polygon.Count - 1);
                this.polygon = null;
                sweepVertices.Clear();
                tree.Clear();
            }

            private void OldHelperMerge(EdgeNode edgeNode)
            {
                if (GetVType(edgeNode.helper) == VType.Merge)
                    diagonals.Add(new Diagonal(edgeNode.id, edgeNode.helper.id));
                tree.Delete(edgeNode);
            }

            private void HelperOfEdgeLeftOf(EdgeNode edgeNode)
            {
                var leftEdge = EdgeLeftOf(polygon[edgeNode.id]);
                if (GetVType(leftEdge.helper) == VType.Merge)
                    diagonals.Add(new Diagonal(edgeNode.id, leftEdge.helper.id));
                leftEdge.helper = edgeNode;
            }

            private VType GetVType(EdgeNode edgeNode)
            {
                var v = polygon[edgeNode.id];
                var prevV = polygon[edgeNode.prev.id];
                var nextV = polygon[edgeNode.next.id];
                bool prevAbove = IsAbove(prevV, v);
                bool nextAbove = IsAbove(nextV, v);
                if (prevAbove != nextAbove)
                    return VType.Regular;
                bool isConvex = MathExt.IsConvex(prevV, v, nextV);
                if (prevAbove)
                    return isConvex ? VType.End : VType.Merge;
                return isConvex ? VType.Start : VType.Split;
            }

            private EdgeNode EdgeLeftOf(Vector2 v)
            {
                Debugging.Debug.Assert(tree.Count != 0, "Tree should not be empty");
                polygon[dummyNode1.id] = new Vector2(v.x, MathExt.MinValue);
                polygon[dummyNode2.id] = new Vector2(v.x, MathExt.MaxValue);
                return tree.FindPred(dummyNode1).Value;
            }

            private static Vector2 GetPos(List<Vector2> polygon, SweepVertex vertex)
            {
                return polygon[vertex.edgeNode.id];
            }

            private class SweeplineVertexComparer : Comparer<SweepVertex>
            {
                public List<Vector2> polygon;
                
                public override int Compare(SweepVertex v1, SweepVertex v2)
                {
                    return IsAbove(GetPos(polygon, v1), GetPos(polygon, v2)) ? -1 : 1;
                }
            }

            private struct SweepVertex
            {
                public EdgeNode edgeNode;

                public SweepVertex(EdgeNode edgeNode)
                {
                    this.edgeNode = edgeNode;
                }
            }

            private class EdgeNodeComparer : Comparer<EdgeNode>
            {
                public float sweepY;
                public List<Vector2> polygon;

                public override int Compare(EdgeNode node1, EdgeNode node2)
                {
                    var node1P1 = polygon[node1.id];
                    var node1P2 = polygon[node1.next.id];
                    var node2P1 = polygon[node2.id];
                    var node2P2 = polygon[node2.next.id];
                    var sweepP1 = new Vector2(MathExt.MinValue, sweepY);
                    var sweepP2 = new Vector2(MathExt.MaxValue, sweepY);
                    var intersection2 = default(Vector2);
                    if (!MathExt.LineToLine(node1P1, node1P2, sweepP1, sweepP2, out Vector2 intersection1))
                        intersection1.x = (node1P1.x + node1P2.x) / 2;
                    if (!MathExt.LineToLine(node2P1, node2P2, sweepP1, sweepP2, out intersection2))
                        intersection2.x = (node2P1.x + node2P2.x) / 2;
                    return intersection1.x.CompareTo(intersection2.x);
                }
            }

            private enum VType
            {
                Start,
                End,
                Split,
                Merge,
                Regular
            }
        }
    }
}
