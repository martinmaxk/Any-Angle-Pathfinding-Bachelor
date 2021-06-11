namespace Pathfinding
{
    public class LazyThetaStarPathfinder : ThetaStarPathfinder
    {
        public LazyThetaStarPathfinder(int graphSize) : base(graphSize)
        {
            this.lazy = true;
        }
        
        protected override void SetVertex(IAstarGraph graph, int sourceID, int goalID)
        {
            int parentID = nodeInfos[sourceID].parent;
            if (parentID == NoParent || graph.LineOfSight(parentID, sourceID))
                return;
            float currentGCost = nodeInfos[sourceID].gCost;
            // Go over potential parents and select the one that gives the lowest g-value for source
            nodeInfos[sourceID].gCost = float.MaxValue;
            graph.BeginIterEdges(sourceID);
            bool check = false;
            while (graph.MoveNext())
            {
                var edge = graph.CurrentEdge;
                if (nodeInfos[edge.vertexId].visited != sessionID + 1)
                    continue;
                check = true;
                float newGCost = nodeInfos[edge.vertexId].gCost + edge.cost;
                if (newGCost < nodeInfos[sourceID].gCost)
                    nodeInfos[sourceID].Update(edge.vertexId, newGCost);
            }
            if (!check)
                throw new System.ArgumentException();
        }
    }
}
