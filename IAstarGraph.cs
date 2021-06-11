using System.Collections.Generic;

namespace Pathfinding
{
    /// <summary>
    /// interface for a graph that can be fed to the AstarPathfinder.search method
    /// </summary>
    public interface IAstarGraph
    {
        /// <summary>
        /// Total number of nodes in graph
        /// </summary>
        int GraphSize { get; }

        /// <summary>
        /// Returns current edge in iteration
        /// </summary>
        Edge CurrentEdge { get; }

        /// <summary>
        /// Begins iterations of all edges of fromID
        /// </summary>
        void BeginIterEdges(int fromID);

        /// <summary>
        /// Returns true if there is a next element; false if the sequence has terminated
        /// </summary>
        bool MoveNext();

        /// <summary>
        /// Calculate accurate cost of moving from source to a neighbor
        /// </summary>
        float Cost(int sourceID, int neighborID);

        /// <summary>
        /// Calculates the heuristic (estimate) to get from 'node' to 'goal'
        /// </summary>
        float Heuristic(int sourceID, int goalID);

        /// <summary>
        /// For Theta*
        /// </summary>
        bool LineOfSight(int startIndex, int endIndex);
    }
}
