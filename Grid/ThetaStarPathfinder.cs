namespace Pathfinding
{
	public class ThetaStarPathfinder : AStarPathfinder
	{
        protected bool lazy = false;

        public ThetaStarPathfinder(int graphSize) : base(graphSize) 
		{ }

        protected override void UpdateVertex(IAstarGraph graph, int sourceID, int neighborID, float newGCost, int goalID)
        {
            if (nodeInfos[neighborID].visited == sessionID + 1)
                return;
            // If this is not the start node and there is line of sight from p(s) to n(s)
            int parentID = nodeInfos[sourceID].parent;
            if (parentID != NoParent && (lazy || graph.LineOfSight(parentID, neighborID)))
            {
                newGCost = nodeInfos[parentID].gCost + graph.Cost(parentID, neighborID);
                base.UpdateVertex(graph, parentID, neighborID, newGCost, goalID);
            }
            else
                base.UpdateVertex(graph, sourceID, neighborID, newGCost, goalID);
        }
    }
}
