using System;
using System.Linq;
using System.Collections.Generic;
using Xunit;
using MathTools;
using Pathfinding.VG;
using Pathfinding;
using PolyBoolOpMartinez;

namespace RpsTests
{
    public class VisibilityGraphTests
    {
        private List<List<Vector2>> polygons;
        private VisVertices obstacles;
        private DynamicArray<DynamicArray<Edge>> allEdges = 
            new DynamicArray<DynamicArray<Edge>>(4);
        private DynamicArray<Edge> edges = new DynamicArray<Edge>(0);

        private VisibilityGraph vg = new VisibilityGraph();
        private PlaneSweepConnector rpsConnector = new PlaneSweepConnector();
        private RotationalPlaneSweep rps;

        public VisibilityGraphTests()
        {
            polygons = new List<List<Vector2>>
            {
                new List<Vector2>() { new Vector2(50, 50), new Vector2(60, 50), new Vector2(60, 60), 
                    new Vector2(50, 60) },
                new List<Vector2>() { new Vector2(50, 40), new Vector2(60, 30), new Vector2(55, 40) },
                new List<Vector2>() { new Vector2(55, 65), new Vector2(60, 70), new Vector2(65, 65), 
                    new Vector2(65, 75), new Vector2(55, 75) },
                new List<Vector2>() { new Vector2(60, 10), new Vector2(70, 10), new Vector2(70, 20) },
                new List<Vector2>() { new Vector2(55, 25), new Vector2(60, 20), new Vector2(60, 25) },
                new List<Vector2>() { new Vector2(60, 16), new Vector2(64, 16), new Vector2(64, 18) },
                new List<Vector2>() { new Vector2(0, 0), new Vector2(0, 100), new Vector2(100, 100), 
                    new Vector2(100, 0) }
            };
            int count = 0;
            for (int i = 0; i < polygons.Count; i++)
                count += polygons[i].Count + 2;
            obstacles = new VisVertices() { vs = new DynamicArray<Vector2>(count) };
            obstacles.vs.Extend(count);
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
        }

        private void VisibleVertices(bool reduced, int pId, Vector2 p)
        {
            edges.Clear();
            AssertApprox(obstacles.vs[pId], p);
            var rps = new RotationalPlaneSweep(reduced);
            rps.VisibleVertices(pId, obstacles, ref edges);
        }

        private void VisibleVertices(bool reduced)
        {
            VisibleVertices(reduced, 2, new Vector2(60, 50));
        }

        [Fact]
        public void VerifyEdges()
        {
            VisibleVertices(false);
            AssertEqEdge(0, new Vector2(60, 60));
            AssertEqEdge(1, new Vector2(60, 70));
            AssertEqEdge(2, new Vector2(65, 65));
            AssertEqEdge(3, new Vector2(100, 100));
            AssertEqEdge(4, new Vector2(100, 0));
            AssertEqEdge(5, new Vector2(70, 20));
            AssertEqEdge(6, new Vector2(64, 18));
            AssertEqEdge(7, new Vector2(60, 30));
            AssertEqEdge(8, new Vector2(60, 25));
            AssertEqEdge(9, new Vector2(60, 20));
            AssertEqEdge(10, new Vector2(60, 16));
            AssertEqEdge(11, new Vector2(60, 10));
            AssertEqEdge(12, new Vector2(55, 40));
            AssertEqEdge(13, new Vector2(50, 40));
            AssertEqEdge(14, new Vector2(0, 0));
            AssertEqEdge(15, new Vector2(50, 50));
            Assert.Equal(16, edges.Count);
        }

        [Fact]
        public void VerifyReducedEdges()
        {
            VisibleVertices(true);
            AssertEqEdge(0, new Vector2(60, 60));
            AssertEqEdge(1, new Vector2(65, 65));
            AssertEqEdge(2, new Vector2(60, 30));
            AssertEqEdge(3, new Vector2(50, 40));
            AssertEqEdge(4, new Vector2(50, 50));
            Assert.Equal(5, edges.Count);
        }

        public static IEnumerable<object[]> ConnectDataGen()
        {
            yield return new object[] 
            {   
                new Vector2(30, 28), new Vector2(70, 26), false,
                new List<Vector2> { new Vector2(60, 25), new Vector2(55, 25), new Vector2(60, 20),
                new Vector2(64, 18), new Vector2(60, 16), new Vector2(60, 10), new Vector2(0, 0), 
                new Vector2(0, 100), new Vector2(55, 75), new Vector2(50, 60), new Vector2(50, 50), 
                new Vector2(60, 50), new Vector2(50, 40), new Vector2(60, 30), new Vector2(70, 26) }, 
                new List<Vector2> { new Vector2(100, 0), new Vector2(70, 20), new Vector2(70, 10),
                new Vector2(64, 16), new Vector2(64, 18), new Vector2(60, 16), new Vector2(60, 20),
                new Vector2(60, 25), new Vector2(55, 25), new Vector2(60, 30), new Vector2(55, 40), 
                new Vector2(0, 100), new Vector2(50, 50), new Vector2(60, 50), new Vector2(60, 60), 
                new Vector2(60, 70), new Vector2(65, 65), new Vector2(65, 75), new Vector2(100, 100), 
                new Vector2(30, 28) },
            };
        }

        [Theory]
        [MemberData(nameof(ConnectDataGen))]
        public void VerifyConnectEdges(Vector2 start, Vector2 goal, 
            bool reduced,
            List<Vector2> startEdgesExpect, List<Vector2> goalEdgesExpect)
        {
            VisibilityGraph(reduced);
            var path1 = new List<Vector2>();
            rpsConnector.Initialize(new RotationalPlaneSweep(rps.Reduced));
            VgFindPath(start, goal, path1, rpsConnector, startEdgesExpect, goalEdgesExpect);
        }

        private void VisibilityGraph(bool reduced)
        {
            if (rps != null && reduced == rps.Reduced)
                return;
            allEdges.Clear();
            rps = new RotationalPlaneSweep(reduced);
            rps.VisibilityGraph(polygons, ref obstacles, ref allEdges);
        }

        private bool VgFindPath(Vector2 start, Vector2 goal, List<Vector2> path,
            IVgConnector connector, List<Vector2> startEdgesExpect,
            List<Vector2> goalEdgesExpect)
        {
            vg.Initialize(obstacles, allEdges, connector);
            path.Clear();
            bool found = vg.FindPath(new AStarPathfinder(obstacles.vs.Count + 2), start, goal,
                path);
            AssertEqConnectVgEdges(allEdges.Count, startEdgesExpect);
            AssertEqConnectVgEdges(allEdges.Count + 1, goalEdgesExpect);
            return found;
        }

        private List<Vector2> GetEdgesV(int fromId)
        {
            var edges = new List<Vector2>();
            vg.BeginIterEdges(fromId);
            while (vg.MoveNext())
                edges.Add(obstacles.V(vg.CurrentEdge.vertexId));
            return edges;
        }

        private void AssertEqConnectVgEdges(int fromId, List<Vector2> edgesExpect)
        {
            AssertEq(edgesExpect, GetEdgesV(fromId));
        }

        private void AssertEqPaths(List<Vector2> pathExpect, List<Vector2> path)
        {
            double costExpect = MathExt.PathCost(pathExpect);
            double costActual = MathExt.PathCost(path);
            AssertApprox(costExpect, costActual, MathExt.Epsilon);
        }

        private void AssertEq(List<Vector2> expected, List<Vector2> actual)
        {
            Assert.Equal(expected.Count, actual.Count);
            actual.Sort(Vector2Comparer.Instance);
            expected.Sort(Vector2Comparer.Instance);
            for (int i = 0; i < expected.Count; i++)
                AssertApprox(expected[i], actual[i]);
        }

        private class Vector2Comparer : Comparer<Vector2>
        {
            public static readonly Vector2Comparer Instance = new Vector2Comparer();

            public override int Compare(Vector2 value1, Vector2 value2)
            {
                if (value1.x == value2.x)
                    return value1.y.CompareTo(value2.y);
                return value1.x.CompareTo(value2.x);
            }
        }

        private void AssertEqEdge(int edgeIndex, Vector2 endV)
        {
            var edge = edges[edgeIndex];
            AssertApprox(obstacles.vs[edge.vertexId], endV);
        }

        private void AssertApprox(float value1, float value2, float epsilon = MathExt.Epsilon)
        {
            Assert.True(MathExt.Approximately(value1, value2, epsilon),
                "!Approx(" + value1 + "," + value2 + ")");
        }

        private void AssertApprox(double value1, double value2, double epsilon = MathExt.EpsilonD)
        {
            Assert.True(MathExt.Approximately(value1, value2, epsilon),
                "!Approx(" + value1 + "," + value2 + ")");
        }

        private void AssertApprox(Vector2 value1, Vector2 value2, float epsilon = MathExt.Epsilon)
        {
            Assert.True(MathExt.Approximately(value1, value2, epsilon), 
                "!Approx(" + value1 + "," + value2 + ")");
        }
    }
}
