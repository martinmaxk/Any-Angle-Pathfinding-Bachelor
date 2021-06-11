using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;
using MathTools;
using Pathfinding.Internal.TiledUtils;
using Pathfinding.VG;
using PolyBoolOpMartinez;
using Triangulation;

namespace Pathfinding.VG
{
    class Program
    {
        private const int NumQueries = 1000;
        private static double[] costs = new double[NumQueries];
        private static List<double> times = new List<double>();

        static void Main(string[] args)
        {
            string mapName = args[0];
            var character = new List<Vector2>() { new Vector2(0, 0), new Vector2(31, 0), 
                new Vector2(31, 31), new Vector2(0, 31) };
            var startGoals = new List<Vector2>(); //{ new Vector2(277, 13) * 32, new Vector2(52, 41) * 32 };
            var consoleOut = Console.Out;
            Console.SetOut(TextWriter.Null);
            string vgFileName = "TestMaps/" + mapName;
            var allPolys = new Polygon();
            allPolys.ReadFromLines(File.ReadAllLines(vgFileName));
            var testVg = BuildVg(vgFileName, character, allPolys, true, out Bbox levelBbox, 
                out IVgConnector _, out AStarPathfinder pathfinder);
            var testVg2 = BuildVg(vgFileName, character, allPolys, false, out Bbox _,
                out IVgConnector _, out AStarPathfinder pathfinder2);
            string gridFileName = "Grid/BinTestMaps/" + mapName + ".bin";
            var testGrid = LoadGridGraph(gridFileName, character, out Vector2Int characterSize);
            var testGridPathfinder = new AStarPathfinder(testGrid.GraphSize);
            var random = new Random(0);
            var tempPath = new List<Vector2>();
            var tempPath2 = new List<Vector2>();
            var tempGridPath = new List<Vector2Int>();
            //Console.SetOut(consoleOut);
            int m = 0;
            while (startGoals.Count < NumQueries * 2)
            {
                var randStart = new Vector2(random.Next((int)levelBbox.xmin / 32, (int)levelBbox.xmax / 32),
                    random.Next((int)levelBbox.ymin / 32, (int)levelBbox.ymax / 32)) * 32;
                var randGoal = new Vector2(random.Next((int)levelBbox.xmin / 32, (int)levelBbox.xmax / 32),
                    random.Next((int)levelBbox.ymin / 32, (int)levelBbox.ymax / 32)) * 32;
                bool res1 = testVg.FindPath(pathfinder, randStart, randGoal, tempPath);
                bool res2 = testVg2.FindPath(pathfinder2, randStart, randGoal, tempPath2);
                var startGrid = WorldToGrid(randStart);
                var goalGrid = WorldToGrid(randGoal);
                if (!MathExt.Approximately(MathExt.PathCost(tempPath), MathExt.PathCost(tempPath2),
                    1f) || res1 != res2)
                {
                    Console.WriteLine(m);
                    Console.WriteLine(res1);
                    Console.WriteLine(tempPath.Count + " " + tempPath2.Count);
                    Console.WriteLine(randStart / 32f + " " + randGoal / 32f);
                    foreach (var p in tempPath)
                        Console.WriteLine(p / 32f);
                    foreach (var p in tempPath2)
                        Console.WriteLine(p / 32f);
                    Console.WriteLine(MathExt.PathCost(tempPath));
                    Console.WriteLine(MathExt.PathCost(tempPath2));
                    throw new Exception();
                }
                if (res1 && testGrid.FindPath(testGridPathfinder, startGrid, goalGrid, characterSize.x,
                    characterSize.y, tempGridPath, false))
                {
                    costs[startGoals.Count / 2] = MathExt.PathCost(tempPath);
                    startGoals.Add(randStart);
                    startGoals.Add(randGoal);
                }
                if (m % 1000 == 0)
                    Console.WriteLine(m);
                m++;
            }
            // First run is for warmup (JIT compiler)
            for (int i = 0; i < 2; i++)
            {
                if (i == 1)
                    Console.SetOut(consoleOut);
                BenchmarkVg(vgFileName, character, startGoals);
                BenchmarkGrid(gridFileName, character, startGoals);
                GC.Collect();
            }
        }

        private static void BenchmarkVg(string fileName, List<Vector2> character, List<Vector2> startGoals)
        {
            var allPolys = new Polygon();
            allPolys.ReadFromLines(File.ReadAllLines(fileName));
            BenchmarkVg(fileName, startGoals, character, allPolys, false);
            BenchmarkVg(fileName, startGoals, character, allPolys, true);
        }

        private static VisibilityGraph BuildVg(string fileName, List<Vector2> character, Polygon allPolys,
            bool reduced, out Bbox levelBbox, 
            out IVgConnector connector, out AStarPathfinder pathfinder)
        {
            Console.WriteLine("Reduced: " + reduced);
            var sw = new Stopwatch();
            sw.Start();
            character = new List<Vector2>(character);
            for (int i = 0; i < character.Count; i++)
                character[i] = -character[i];
            character.Reverse();
            int charBlIndex = MathExt.BottomLeftIndex(character);

            var triangulator = new PolygonTriangulator();
            var triangles = new List<List<Vector2>>();
            var polygon = new List<Vector2>();
            levelBbox = new Bbox(float.MaxValue, float.MaxValue, float.MinValue, float.MinValue);
            int trianglesStart = 0;
            for (int i = 0; i < allPolys.Ncontours; i++)
            {
                var contour = allPolys.Contour(i);
                levelBbox = levelBbox.Union((Bbox)contour.Bbox());
                for (int j = 0; j < contour.Nvertices; j++)
                    polygon.Add((Vector2)contour.Vertex(j));
                //triangles.Add(polygon);
                //polygon = new List<Vector2>();
                triangulator.Triangulate(polygon, triangles, ref trianglesStart);
                polygon.Clear();
            }
            double triTime = sw.Elapsed.TotalMilliseconds;

            /*var levelMid = new Vector2Int(MathExt.NextPowerOfTwo(levelBbox.xmax + levelBbox.xmin) / 2,
                MathExt.NextPowerOfTwo(levelBbox.ymax + levelBbox.ymin) / 2);*/
            var levelMid = new Vector2(levelBbox.xmax + levelBbox.xmin, levelBbox.ymax + levelBbox.ymin) / 2;

            var expanded = new List<Polygon>(triangles.Count);
            var tempList = new List<Vector2>();

            for (int i = 0; i < triangles.Count; i++)
            {
                Debugging.Debug.Assert(MathExt.IsPolygonClockwise(triangles[i]));
                triangles[i].Reverse();
                MathExt.MinkowskiSum(triangles[i], character, tempList,
                    MathExt.BottomLeftIndex(triangles[i]), charBlIndex);
                tempList.Reverse();
                var expandedPoly = new Polygon();
                var expandedContour = expandedPoly.AddNewContour();
                for (int j = 0; j < tempList.Count; j++)
                    expandedContour.Add((Vector2D)tempList[j]);
                expanded.Add(expandedPoly);
                triangles[i].Reverse();
                tempList.Clear();
            }
            double minkowskiTime = sw.Elapsed.TotalMilliseconds - triTime;

            var union = new Polygon();
            var polyBoolOp = new PolyBoolOp();
            polyBoolOp.UnionAll(expanded, union);
            var polygons = new List<List<Vector2>>(union.Ncontours);
            for (int i = 0; i < union.Ncontours; i++)
            {
                var contour = union.Contour(i);
                var poly = new List<Vector2>(contour.Nvertices);
                for (int j = 0; j < contour.Nvertices; j++)
                    poly.Add((Vector2)contour.Vertex(j));
                // Union can merge polygons that overlap in a vertex,
                // split these illegal non-simple polygons here
                triangulator.SplitDuplicateVertices(poly, polygons);
            }
            double polyUnionTime = sw.Elapsed.TotalMilliseconds - minkowskiTime;
            sw.Reset();
            Console.WriteLine("Triangulation time ms: " + triTime);
            Console.WriteLine("Minkowski sum time ms: " + minkowskiTime);
            Console.WriteLine("Poly union time ms: " + polyUnionTime);

            sw.Start();
            long GC_MemoryStart = System.GC.GetTotalMemory(true);
            var rotPlaneSweep = new RotationalPlaneSweep(reduced);
            var visVertices = new VisVertices() { vs = new DynamicArray<Vector2>(4) };
            var allEdges = new DynamicArray<DynamicArray<Edge>>(4);
            rotPlaneSweep.VisibilityGraph(polygons, ref visVertices, ref allEdges);
            long GC_MemoryEnd = System.GC.GetTotalMemory(true);
            Console.WriteLine("Vg memory usage: " + (double)(GC_MemoryEnd - GC_MemoryStart) / (1024 * 1024));
            sw.Stop();
            double vgBuildTime = sw.Elapsed.TotalMilliseconds;
            Console.WriteLine("Vg build time ms: " + vgBuildTime);
            Console.WriteLine("Total preprocessing time ms: " + (triTime + minkowskiTime + polyUnionTime +
                vgBuildTime));
            int numVertices = 0, numEdges = 0;
            for (int i = 0; i < visVertices.vs.Count; i++)
            {
                if (visVertices.IsDescriptor(i))
                    continue;
                numVertices++;
                numEdges += allEdges[i].Count;
            }
            Console.WriteLine("Num vertices: " + numVertices);
            Console.WriteLine("Num edges: " + numEdges);
            sw.Reset();

            var outPolyVg = new Polygon();
            var charContour = outPolyVg.AddNewContour();
            for (int i = character.Count - 1; i >= 0; i--)
                charContour.Add((Vector2D)(-character[i]));
            for (int i = 0; i < visVertices.vs.Count; i++)
            {
                if (visVertices.IsDescriptor(i))
                    continue;
                var contour = outPolyVg.AddNewContour();
                contour.Add((Vector2D)visVertices.V(i));
                for (int j = 0; j < allEdges[i].Count; j++)
                    contour.Add((Vector2D)visVertices.V(allEdges[i][j].vertexId));
            }
            File.WriteAllText("outPolyVg" +
                fileName.Split("/").Last() + (rotPlaneSweep.Reduced ? "Reduced" : ""),
                outPolyVg.ToString());


            var rpsConnector = new TimePlaneSweepConnector();
            rpsConnector.Initialize(rotPlaneSweep);
            connector = rpsConnector;
            var visibilityGraph = new VisibilityGraph();
            visibilityGraph.Initialize(visVertices, allEdges, connector);
            pathfinder = new AStarPathfinder(visVertices.vs.Count + 2);
            return visibilityGraph;
        }

        private static void BenchmarkVg(string fileName, List<Vector2> startGoals,
            List<Vector2> character, Polygon allPolys,
            bool reduced)
        {
            Bbox levelBbox;
            IVgConnector connector;
            AStarPathfinder pathfinder;
            var visibilityGraph = BuildVg(fileName, character, allPolys, reduced, 
                out levelBbox, out connector, out pathfinder);

            var path = new List<Vector2>();
            double totalConnectTime = 0, totalPathLengths = 0;
            long totalNumExpansions = 0;
            int worstTimeIdx = 0;
            var totalSw = new Stopwatch();
            int numPaths = 0;
            times.Clear();
            for (int i = 0; i < startGoals.Count; i += 2)
            {
                var start = startGoals[i];
                var goal = startGoals[i + 1];
                double timeBef = totalSw.Elapsed.TotalMilliseconds;
                totalSw.Start();
                bool success = visibilityGraph.FindPath(pathfinder, start, goal, path);
                totalSw.Stop();
                times.Add(totalSw.Elapsed.TotalMilliseconds - timeBef);
                if (times.Last() > times[worstTimeIdx / 2])
                    worstTimeIdx = i;
                double connectStartTime, connectGoalTime;
                if (connector is TimePlaneSweepConnector timeRpsConnector)
                {
                    connectStartTime = timeRpsConnector.startTime;
                    connectGoalTime = timeRpsConnector.goalTime;
                }
                else
                    throw new Exception("Invalid connector");
                totalConnectTime += connectStartTime + connectGoalTime;
                totalNumExpansions += pathfinder.numExpansions;
                totalPathLengths += MathExt.PathCost(path);
                numPaths++;
            }
            double totalTime = totalSw.Elapsed.TotalMilliseconds;
            double meanTime = totalTime / numPaths;
            Console.WriteLine("Avg time ms: " + meanTime);
            Console.WriteLine("Avg connect time ms: " + (totalConnectTime / numPaths));
            Console.WriteLine("Avg search time ms: " + ((totalTime - totalConnectTime) / numPaths));
            double worstTime = times[worstTimeIdx / 2];
            Console.WriteLine("Worst time ms: " + worstTime + 
                " with start tile: " + startGoals[worstTimeIdx] / 32 + " and goal tile: " + 
                startGoals[worstTimeIdx + 1] / 32);
            Console.WriteLine("Standard deviation time ms: " + StandardDeviation(times, meanTime));
            Console.WriteLine("Avg path length: " + (totalPathLengths / numPaths));
            Console.WriteLine("Avg node expansions: " + (totalNumExpansions / (double)numPaths));
            Console.WriteLine();
        }

        private static GridGraph LoadGridGraph(string fileName, 
            List<Vector2> character, out Vector2Int characterSize)
        {
            var binaryReader = new BinaryReader(File.OpenRead(fileName));
            TileExtensions.Initialize(binaryReader.ReadInt32());
            int mapSizeX = binaryReader.ReadInt32();
            int mapSizeY = binaryReader.ReadInt32();
            var charPoly = new Polygon();
            charPoly.FromEnumerable(character);
            var charBbox = charPoly.Bbox();
            characterSize = new Vector2Int(
                (int)Math.Ceiling((charBbox.xmax - charBbox.xmin) / TileExtensions.TileSize),
                (int)Math.Ceiling((charBbox.ymax - charBbox.ymin) / TileExtensions.TileSize));
            var tileGrid = new TileGrid(mapSizeX, mapSizeY);
            for (int i = 0; i < tileGrid.raw.Length; i++)
                tileGrid[i] = (TileShape)binaryReader.ReadByte();
            binaryReader.Dispose();

            return new GridGraph(tileGrid);
        }

        private static void BenchmarkGrid(string fileName, List<Vector2> character, List<Vector2> startGoals)
        {
            var gridGraph = LoadGridGraph(fileName, character, out Vector2Int characterSize);
            BenchmarkGrid(gridGraph, new AStarPathfinder(gridGraph.GraphSize), startGoals,
                characterSize);
            BenchmarkGrid(gridGraph, new AStarPathfinder(gridGraph.GraphSize), startGoals,
                characterSize, true);
            BenchmarkGrid(gridGraph, new ThetaStarPathfinder(gridGraph.GraphSize), startGoals,
                characterSize);
            BenchmarkGrid(gridGraph, new LazyThetaStarPathfinder(gridGraph.GraphSize), startGoals,
                characterSize);
        }

        private static void BenchmarkGrid(GridGraph gridGraph, AStarPathfinder pathfinder, 
            List<Vector2> startGoals,
            Vector2Int characterSize, bool smooth = false)
        {
            var gridPath = new List<Vector2Int>();
            var sw = new Stopwatch();
            decimal totalPathLengths = 0;
            long totalNumExpansions = 0, totalLosChecks = 0;
            int worstTimeIdx = 0;
            int numPaths = 0;
            times.Clear();
            for (int i = 0; i < startGoals.Count; i += 2)
            {
                var start = WorldToGrid(startGoals[i]);
                var goal = WorldToGrid(startGoals[i + 1]);
                double timeBef = sw.Elapsed.TotalMilliseconds;
                sw.Start();
                bool success = gridGraph.FindPath(pathfinder, start, goal,
                    characterSize.x, characterSize.y, gridPath, smooth);
                sw.Stop();
                times.Add(sw.Elapsed.TotalMilliseconds - timeBef);
                if (times.Last() > times[worstTimeIdx / 2])
                    worstTimeIdx = i;
                totalNumExpansions += pathfinder.numExpansions;
                totalLosChecks += gridGraph.numLosChecks;
                totalPathLengths += (decimal)MathExt.PathCost(gridPath);
                numPaths++;
                if (costs[i / 2] > (MathExt.PathCost(gridPath) * 32) + 1)
                {
                    Console.WriteLine(costs[i / 2] + " " + MathExt.PathCost(gridPath) * 32);
                    Console.WriteLine(start + " " + goal);
                    var poly = new Polygon();
                    var cont = poly.AddNewContour();
                    cont.Add(new Vector2D(0, 0));
                    cont.Add(new Vector2D(characterSize.x, 0) * 32);
                    cont.Add(new Vector2D(characterSize.x, characterSize.y) * 32);
                    cont.Add(new Vector2D(0, characterSize.y) * 32);
                    cont = poly.AddNewContour();
                    for (int j = 0; j < gridPath.Count; j++)
                        cont.Add(new Vector2D(gridPath[j].x * 32, gridPath[j].y * 32));
                    File.WriteAllText("path", poly.ToString());
                    throw new Exception();
                }
            }
            Console.WriteLine(pathfinder + " Smoothing: " + smooth);
            double meanTime = (sw.Elapsed.TotalMilliseconds / numPaths);
            Console.WriteLine("Avg time ms: " + meanTime);
            double worstTime = times[worstTimeIdx / 2];
            Console.WriteLine("Worst time ms: " + worstTime +
                " with start tile: " + startGoals[worstTimeIdx] / 32 + " and goal tile: " +
                startGoals[worstTimeIdx + 1] / 32);
            Console.WriteLine("Standard deviation time ms: " + StandardDeviation(times, meanTime));
            Console.WriteLine("Avg path length: " + (totalPathLengths / numPaths) * TileExtensions.TileSize);
            Console.WriteLine("Avg node expansions: " + (totalNumExpansions / (double)numPaths));
            if (pathfinder is ThetaStarPathfinder)
                Console.WriteLine("Avg LOS checks: " + (totalLosChecks / (double)numPaths));
            /*if (pathfinder is ThetaStarPathfinder)
            {
                var poly = new Polygon();
                var cont = poly.AddNewContour();
                cont.Add(new Vector2D(0, 0));
                cont.Add(new Vector2D(characterSize.x, 0) * 32);
                cont.Add(new Vector2D(characterSize.x, characterSize.y) * 32);
                cont.Add(new Vector2D(0, characterSize.y) * 32);
                cont = poly.AddNewContour();
                for (int j = 0; j < gridPath.Count; j++)
                    cont.Add(new Vector2D(gridPath[j].x * 32, gridPath[j].y * 32));
                File.WriteAllText("path", poly.ToString());
            }*/
            Console.WriteLine();
        }

        private static double StandardDeviation(List<double> data, double mean)
        {
            decimal squaredSum = 0;
            for (int i = 0; i < data.Count; i++)
            {
                decimal diff = (decimal)(data[i] - mean);
                squaredSum += diff * diff;
            }
            double variance = (double)(squaredSum / data.Count);
            return Math.Sqrt(variance);
        }

        private static Vector2Int WorldToGrid(Vector2 world)
        {
            return new Vector2Int((int)Math.Floor(world.x), (int)Math.Floor(world.y)) / TileExtensions.TileSize;
        }

        private class TimePlaneSweepConnector : PlaneSweepConnector
        {
            private Stopwatch sw = new Stopwatch();
            public double startTime, goalTime;

            public override bool FindEdges(int sourceId, int goalId, VisVertices visVertices,
                DynamicArray<DynamicArray<Edge>> allEdges)
            {
                sw.Restart();
                bool connected = base.FindEdges(sourceId, goalId, visVertices, allEdges);
                sw.Stop();
                if (sourceId == goalId)
                    goalTime = sw.Elapsed.TotalMilliseconds;
                else
                    startTime = sw.Elapsed.TotalMilliseconds;
                return connected;
            }
        }
    }
}
