using System;
using System.Collections.Generic;
using MathTools;
using Debugging;

namespace Pathfinding.VG
{
    public class VisibilityGraph : IAstarGraph
    {
        private VisVertices visVertices;
        private DynamicArray<DynamicArray<Edge>> allEdges;
        private int edgeIndex;
        private DynamicArray<Edge> edges;

        private List<int> pathIds;
        private IVgConnector connector;

        public int GraphSize
        {
            get { return visVertices.vs.Count; }
        }

        public void Initialize(VisVertices visVerticesIn, DynamicArray<DynamicArray<Edge>> allEdgesIn,
            IVgConnector connector)
        {
            this.visVertices = visVerticesIn;
            this.allEdges = allEdgesIn;
            this.connector = connector;
            pathIds = new List<int>();
            visVertices.vs.ResizeAndExtendTo(visVertices.vs.Count + 2);
            // Start and end vertex
            if (allEdges.Extend(2))
            {
                allEdges.arr[allEdges.Count - 2].ClearOrInit(32);
                allEdges.arr[allEdges.Count - 1].ClearOrInit(32);
            }
            else
            {
                allEdges.Add(new DynamicArray<Edge>(32));
                allEdges.Add(new DynamicArray<Edge>(32));
            }
        }

        public bool FindPath(AStarPathfinder pathfinder, Vector2 start, Vector2 goal, List<Vector2> path)
        {
            path.Clear();
            allEdges.arr[visVertices.vs.Count - 1].Clear();
            // Could not connect start or goal to graph, no path exists
            if (!AddStartAndGoal(start, goal) ||
                !pathfinder.Search(this, visVertices.vs.Count - 2, visVertices.vs.Count - 1, pathIds))
            {
                RemoveStartAndGoal();
                return false;
            }
            for (int i = 0; i < pathIds.Count; i++)
                path.Add(visVertices.V(pathIds[i]));
            RemoveStartAndGoal();
            return true;
        }

        public bool IsTaut(int vId1, int vId2, int vId3)
        {
            //Debug.Assert(pathfinder.Search(this, vId1, vId3, pathIds, allVertices), "Invalid path given");
            if (pathIds.Count != 3)
                return false;
            return pathIds[0] == vId1 && pathIds[1] == vId2 && pathIds[2] == vId3;
        }

        private bool AddStartAndGoal(Vector2 start, Vector2 goal)
        {
            int startId = visVertices.vs.Count - 2;
            int goalId = visVertices.vs.Count - 1;
            visVertices.vs[startId] = start;
            visVertices.vs[goalId] = goal;
            return connector.FindEdges(goalId, goalId, visVertices, allEdges) &&
                connector.FindEdges(startId, goalId, visVertices, allEdges);
        }

        private void RemoveStartAndGoal()
        {
            var goalEdges = allEdges[allEdges.Count - 1];
            for (int i = 0; i < goalEdges.Count; i++)
            {
                var edge = goalEdges[i];
                if (edge.vertexId == visVertices.vs.Count - 2)
                    continue;
                allEdges.arr[edge.vertexId].RemoveLast();
                Debug.Assert(allEdges[edge.vertexId].Count == 0 || allEdges.arr[edge.vertexId].Last().vertexId != allEdges.Count - 1);
            }
        }

        #region IAstarGraph
        public Edge CurrentEdge { get; private set; }

        public void BeginIterEdges(int fromId)
        {
            edgeIndex = 0;
            edges = allEdges[fromId];
        }

        public float Cost(int sourceId, int neighborId)
        {
            return Heuristic(sourceId, neighborId);
        }

        public float Heuristic(int sourceId, int goalId)
        {
            return Vector2.Distance(visVertices.V(sourceId), visVertices.V(goalId));
        }

        // Move to next edge in edge iteration
        public bool MoveNext()
        {
            if (edgeIndex < edges.Count)
            {
                CurrentEdge = edges[edgeIndex];
                edgeIndex++;
                return true;
            }
            return false;
        }

        public bool LineOfSight(int startId, int endId)
        {
            return connector.LineOfSight(startId, endId);
        }
        #endregion
    }

    public interface IVgConnector
    {
        bool FindEdges(int sourceId, int goalId, 
            VisVertices visVertices, DynamicArray<DynamicArray<Edge>> allEdges);

        bool LineOfSight(int startId, int endId);
    }

    public abstract class BaseVgConnector : IVgConnector
    {
        public abstract bool FindEdges(int sourceId, int goalId, 
            VisVertices visVertices, DynamicArray<DynamicArray<Edge>> allEdges);

        public virtual bool LineOfSight(int fromId, int toId)
        {
            return false;
        }
    }

    public class PlaneSweepConnector : BaseVgConnector
    {
        private RotationalPlaneSweep rotPlaneSweep;

        public void Initialize(RotationalPlaneSweep rotPlaneSweep)
        {
            this.rotPlaneSweep = rotPlaneSweep;
        }

        public override bool FindEdges(int sourceId, int goalId, VisVertices visVertices, 
            DynamicArray<DynamicArray<Edge>> allEdges)
        {
            bool isGoal = sourceId == goalId;
            rotPlaneSweep.VisibleVertices(sourceId, visVertices, ref allEdges.arr[sourceId]);
            var sourceEdges = allEdges[sourceId];
            bool anyEdges = sourceEdges.Count > 0;
            if (!isGoal)
                return anyEdges;
            // Connect to goal vertex (if isGoal)
            for (int i = 0; i < sourceEdges.Count; i++)
            {
                var edge = sourceEdges[i];
                // Start has not been connected yet (avoid duplicate edge)
                if (edge.vertexId == visVertices.vs.Count - 2)
                    continue;
                var endEdges = allEdges[edge.vertexId];
                endEdges.Add(new Edge(sourceId, edge.cost));
                allEdges[edge.vertexId] = endEdges;
            }
            return anyEdges;
        }
    }
}
