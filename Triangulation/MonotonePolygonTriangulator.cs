using System.Collections.Generic;
using MathTools;
using System;
using Advanced.Algorithms.DataStructures;

namespace Triangulation
{
    public partial class PolygonTriangulator
    {
        // Class to triangulate monotone polygons
        private class MonotonePolygonTriangulator
        {
            private Stack<EdgeNode> stack = new Stack<EdgeNode>();

            // Stores result as doubly linked lists in edgeHeads
            public void Triangulate(List<Vector2> polygon, EdgeNode edgeHead,
                List<EdgeNode> edgeHeads, List<RedBlackTree<EdgeNode>> edges, List<Diagonal> diagonals, 
                EdgePool edgePool)
            {
                var topNode = FindTopVertex(polygon, edges, edgeHead);
                var leftNode = topNode;
                var rightNode = topNode;
                var prevNode = NextNode(polygon, ref leftNode, ref rightNode);
                var curNode = NextNode(polygon, ref leftNode, ref rightNode);
                stack.Push(topNode);
                stack.Push(prevNode);
                while (leftNode.next.id != rightNode.id && leftNode.prev.id != rightNode.id)
                {
                    var peekNode = stack.Peek();
                    if (IsInteriorRightOf(polygon, curNode) != IsInteriorRightOf(polygon, peekNode))
                    {
                        while (stack.Count > 1)
                            diagonals.Add(new Diagonal(curNode.id, stack.Pop().id));
                        stack.Pop();
                        stack.Push(prevNode);
                        stack.Push(curNode);
                    }
                    else
                    {
                        var lastPopped = stack.Pop();
                        var v = polygon[curNode.id];
                        var nextV = polygon[curNode.next.id];
                        var prevV = polygon[curNode.prev.id];
                        while (stack.Count > 0)
                        {
                            var peeked = stack.Peek();
                            var popV = polygon[peeked.id];
                            var popNextV = polygon[peeked.next.id];
                            var popPrevV = polygon[peeked.prev.id];
                            // Stop if diagonal is not in polygon
                            // if we are on the left chain, and the current turn was a left (ccw) turn, then
                            // diagonal is not in interior
                            double orientation = MathExt.OrientationD(v, polygon[lastPopped.id], popV);
                            bool isInteriorRightOfV = IsInteriorRightOf(polygon, curNode);
                            if ((isInteriorRightOfV && orientation > -MathExt.CollEpsilon) ||
                                (!isInteriorRightOfV && orientation < MathExt.CollEpsilon))
                                break;
                            diagonals.Add(new Diagonal(curNode.id, peeked.id));
                            var diagonal = diagonals[diagonals.Count - 1];
                            stack.Pop();
                            lastPopped = peeked;
                        }
                        stack.Push(lastPopped);
                        stack.Push(curNode);
                    }
                    prevNode = curNode;
                    curNode = NextNode(polygon, ref leftNode, ref rightNode);
                }
                // Since the above while loop terminated leftNode and rightNode 
                // are neighbors and the lowest is u_n
                var lastNode = IsAbove(polygon[leftNode.id], polygon[rightNode.id]) ? rightNode : leftNode;
                stack.TryPop(out EdgeNode _);
                while (stack.Count > 1)
                    diagonals.Add(new Diagonal(lastNode.id, stack.Pop().id));
                stack.TryPop(out _);
                ApplyDiagonalPartitions(polygon, edgeHeads, edges, diagonals, edgePool);
                // We need this in case input is a triangle
                edgeHeads.Add(edgeHead);
                diagonals.Clear();
                DistinctPolygons(edgeHeads);
                RemoveCollinearEdges(polygon, edgeHeads);
            }

            private EdgeNode FindTopVertex(List<Vector2> polygon, List<RedBlackTree<EdgeNode>> edges, 
                EdgeNode edgeHead)
            {
                var topVertex = edgeHead;
                var currentVertex = edgeHead;
                do
                {
                    edges[currentVertex.id].Clear();
                    edges[currentVertex.id].Insert(currentVertex);
                    if (IsAbove(polygon[currentVertex.id], polygon[topVertex.id]))
                        topVertex = currentVertex;
                    currentVertex = currentVertex.next;
                } while (edgeHead.id != currentVertex.id);
                return topVertex;
            }

            // Advance left/right chain depending on above next vertex and return it
            private EdgeNode NextNode(List<Vector2> polygon, ref EdgeNode leftNode, ref EdgeNode rightNode)
            {
                if (IsAbove(polygon[leftNode.prev.id], polygon[rightNode.next.id]))
                    return leftNode = leftNode.prev;
                else
                    return rightNode = rightNode.next;
            }
        }
    }   
}
