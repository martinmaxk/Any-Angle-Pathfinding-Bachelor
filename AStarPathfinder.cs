using System;
using System.Collections.Generic;
using MathTools;
using Debugging;

namespace Pathfinding
{
    /// <summary>
    /// calculates paths given an IAstarGraph and start/goal positions
    /// </summary>
    public class AStarPathfinder
    {
        public int numExpansions;
        protected int sessionID = 1;
        private PriorityQueue<DefaultPqNode> openSet;
        protected NodeInfo[] nodeInfos;

        protected const int NoParent = -1;

        protected struct NodeInfo
        {
            public int parent;
            public float gCost, hCost;
            // sessionID - in open set
            // sessionID + 1 - in closed set
            public int visited;

            public NodeInfo(int parent, float gCost, float hCost, int sessionID)
            {
                this.parent = parent;
                this.gCost = gCost;
                // Cache hCost
                this.hCost = hCost;
                this.visited = sessionID;
            }

            public void Update(int parent, float gCost)
            {
                this.parent = parent;
                this.gCost = gCost;
            }

            public void Update(int visited)
            {
                this.visited = visited;
            }
        }

        public AStarPathfinder(int graphSize)
        {
            nodeInfos = new NodeInfo[graphSize];
            openSet = new PriorityQueue<DefaultPqNode>(graphSize, new DefaultPqNodeComparer());
        }

        /// <summary>
        /// Gets a path from start to goal if possible. If no path is found null is returned.
        /// </summary>
        public bool Search(IAstarGraph graph, int startID, int goalID, List<int> path)
        {
            numExpansions = 0;
            path.Clear();
            bool foundPath = false;
            float startHCost = graph.Heuristic(startID, goalID);
            openSet.Enqueue(new DefaultPqNode(startID, startHCost, startHCost));

            nodeInfos[startID] = new NodeInfo(-1, 0f, startHCost, sessionID);

            while (openSet.Count > 0)
            {
                numExpansions++;
                var current = openSet.Dequeue();
                nodeInfos[current.id].visited = sessionID + 1;
                SetVertex(graph, current.id, goalID);

                if (current.id == goalID)
                {
                    foundPath = true;
                    break;
                }

                float currentGCost = nodeInfos[current.id].gCost;
                // Obsolete enqueued value (multiple nodes of same id on queue)
                if (current.priority > (currentGCost + nodeInfos[current.id].hCost + MathExt.Epsilon))
                    continue;

                graph.BeginIterEdges(current.id);
                while (graph.MoveNext())
                {
                    var edge = graph.CurrentEdge;
                    float newGCost = currentGCost + edge.cost;
                    UpdateVertex(graph, sourceID: current.id, neighborID: edge.vertexId, newGCost: newGCost, goalID: goalID);
                }
            }
            openSet.Reset();

            // Make sure we are reset
            if (sessionID >= int.MaxValue - 1)
            {
                for (int i = 0; i < nodeInfos.Length; i++)
                    nodeInfos[i].visited = 0;
                sessionID = 0;
            }
            sessionID += 2;

            if (foundPath)
                RecontructPath(startID, goalID, path);
            return foundPath;
        }

        protected virtual void SetVertex(IAstarGraph graph, int sourceID, int goalID)
        { }

        // If node has not been visited (neither open nor closed) or newGCost < nodeInfo.gCost, enqueue it, otherwise do nothing
        // this will result in duplicates in priority queue with same id, check this when dequeueing if priority is up to date
        // now we do not need to decrease priority, so no need to find a node in the queue
        protected virtual void UpdateVertex(IAstarGraph graph, int sourceID, int neighborID, float newGCost, int goalID)
        {
            var nodeInfo = nodeInfos[neighborID];
            // Node has not been visited, so add it to the queue (explore)
            if (nodeInfo.visited < sessionID)
            {
                float hCost = graph.Heuristic(neighborID, goalID);
                openSet.Enqueue(new DefaultPqNode(neighborID, newGCost + hCost, hCost));
                nodeInfos[neighborID] = new NodeInfo(sourceID, newGCost, hCost, sessionID);
            }
            // Node has been visited, but a shorter path has been found to it, so update cost
            else if (newGCost < nodeInfo.gCost)
            {
                openSet.Enqueue(new DefaultPqNode(neighborID, newGCost + nodeInfo.hCost, nodeInfo.hCost));
                nodeInfos[neighborID] = new NodeInfo(sourceID, newGCost, nodeInfo.hCost, sessionID);
            }
        }
        
        /// <summary>
        /// Reconstructs a path from the parents
        /// </summary>
        private List<int> RecontructPath(int startID, int goalID, List<int> path)
        {
            int currentID = goalID;
            path.Add(goalID);

            while (currentID != startID)
            {
                currentID = nodeInfos[currentID].parent;
                path.Add(currentID);

                /*if (path.Count > 1000)
                {
                    Debug.Log(currentID);
                    throw new OutOfMemoryException();
                }*/
            }

            path.Reverse();

            return path;
        }
    }

    public struct Edge
    {
        // ID of vertex endpoint
        public int vertexId;
        public float cost;

        public Edge(int vertexId, float cost)
        {
            this.vertexId = vertexId;
            this.cost = cost;
        }
    }
}
