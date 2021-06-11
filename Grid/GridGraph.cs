using Pathfinding;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Pathfinding.Internal.TiledUtils;
using Pathfinding.Internal;
using MathTools;

namespace Pathfinding
{
    public class GridGraph : IAstarGraph
    {
        private readonly TileGrid tiles;
        // Added to 1D index to move in direction in xy coordinates
        private readonly int[] directions;
        // Added to 1D index to get to start tile
        private readonly int[] overlapOffset = new int[4];
        private readonly int[] overlapOffsetCorner = new int[4];
        // How many tiles to check
        private readonly int[] overlapCount = new int[4];
        // Added to 1D index to move to next tile
        private readonly int[] overlapAdd = new int[4];
        private List<int> pathIndices;
        private int characterWidth, characterHeight;

        public int GraphSize
        {
            get { return tiles.raw.Length; }
        }

        public GridGraph(TileGrid tiles)
        {
            this.tiles = tiles;
            // left - right - up - down
            directions = new int[] { -1, 1, -tiles.Width, tiles.Width };
            pathIndices = new List<int>();
        }

        public void Initialize(int characterWidth, int characterHeight)
        {
            this.characterWidth = characterWidth;
            this.characterHeight = characterHeight;

            //int height1D = tiles.Width * characterHeight;
            overlapCount[(int)Direction.Left] = overlapCount[(int)Direction.Right] = characterHeight;
            overlapAdd[(int)Direction.Left] = overlapAdd[(int)Direction.Right] = tiles.Width;
            overlapCount[(int)Direction.Up] = overlapCount[(int)Direction.Down] = characterWidth;
            overlapAdd[(int)Direction.Up] = overlapAdd[(int)Direction.Down] = 1;

            overlapOffset[(int)Direction.Left] = -1;
            overlapOffset[(int)Direction.Down] = characterHeight * tiles.Width;
            overlapOffset[(int)Direction.Right] = characterWidth;
            overlapOffset[(int)Direction.Up] = -1 * tiles.Width;

            overlapOffsetCorner[(int)Direction.Left] = overlapOffsetCorner[(int)Direction.Up] = 0;
            overlapOffsetCorner[(int)Direction.Right] = characterWidth;
            overlapOffsetCorner[(int)Direction.Down] = characterHeight * tiles.Width;
        }

        // start is top-left tile of character
        private List<int> smoothPath = new List<int>();
        public bool FindPath(AStarPathfinder pathfinder, Vector2Int start, Vector2Int goal,
            int characterWidth, int characterHeight, List<Vector2Int> path, bool smooth)
        {
            numLosChecks = 0;
            path.Clear();
            Initialize(characterWidth, characterHeight);

            this.goalPos = goal;
            int startIndex = tiles.GetIndex(start.x, start.y);
            int goalIndex = tiles.GetIndex(goal.x, goal.y);

            // If there is line of sight from start to goal, path is trivial
            if (LineOfSight(startIndex, goalIndex))
            {
                path.Add(start);
                path.Add(goal);
                return true;
            }
            if (!pathfinder.Search(this, startIndex, goalIndex, pathIndices))
                return false;
            SimplifyPath(pathIndices);
            if (smooth)
            {
                GreedySmoothPath(pathIndices, smoothPath);
                pathIndices = new List<int>(smoothPath);
            }
            //StringPulling(pathIndices, lineIndices);
            /*SmoothPath(pathIndices);
            SimplifyPath(pathIndices);*/
            for (int i = 0; i < pathIndices.Count; i++)
                path.Add(tiles.GetPos(pathIndices[i]));
            return true;
        }

        private void GreedySmoothPath(List<int> path, List<int> res)
        {
            res.Clear();
            int k = 0;
            res.Add(path[0]);
            for (int i = 1; i < path.Count - 1; i++)
            {
                if (!LineOfSight(res[k], path[i + 1]))
                {
                    k++;
                    res.Add(path[i]);
                }
            }
            res.Add(path[path.Count - 1]);
        }

        // Only keep nodes where the path changes direction
        private void SimplifyPath(List<int> path)
        {
            if (path.Count < 2)
                return;
            int curDir = path[1] - path[0];
            int count = 1;
            for (int i = 1; i < path.Count - 1; i++)
            {
                int prevDir = curDir;
                curDir = path[i + 1] - path[i];
                // Only keep nodes where direction changes
                if (prevDir != curDir)
                    path[count++] = path[i];
            }
            // Last node must be in the path
            path[count++] = path[path.Count - 1];
            while (path.Count > count)
                path.RemoveAt(path.Count - 1);
        }

        #region IAstarGraph
        private int curDirIndex;
        private Vector2Int curPos, goalPos;
        private int fromIndex;
        public Edge CurrentEdge { get; private set; }

        private Edge[] curEdges = new Edge[8];
        private int curEdgesCount;
        public void BeginIterEdges(int fromID)
        {
            curDirIndex = 0;
            if (this.fromIndex == fromID)
                return;
            this.fromIndex = fromID;
            curEdgesCount = 0;
            int moveDirs = 0;
            for (; curDirIndex < 4; curDirIndex++)
            {
                // Check tiles at left or right or up or down side of character
                if (IsDirBlocked(fromIndex + overlapOffset[curDirIndex],
                    overlapAdd[curDirIndex], (Direction)curDirIndex))
                    continue;
                moveDirs |= (1 << curDirIndex);
                curEdges[curEdgesCount++] = new Edge(fromIndex + directions[curDirIndex], 1);
            }
            for (; curDirIndex < 8; curDirIndex++)
            {
                int horzDir = (curDirIndex - 4) / 2;
                int vertDir = (curDirIndex % 2) + 2;
                int dir = (1 << horzDir) | (1 << vertDir);
                if ((moveDirs & dir) != dir)
                    continue;
                if (IsBlocked(fromIndex + overlapOffset[horzDir] + overlapOffset[vertDir]))
                    continue;
                curEdges[curEdgesCount++] = new Edge(fromIndex + directions[horzDir] + directions[vertDir], Sqrt2);
            }
            curDirIndex = 0;
        }

        public float Cost(int sourceID, int neighborID)
        {
            return Heuristic(sourceID, neighborID);
        }

        public float Heuristic(int id, int goalID)
        {
            var curPos = tiles.GetPos(id);
            var goalPos = tiles.GetPos(goalID);
            return Vector2Int.Distance(curPos, goalPos);
            /*int xDist = Math.Abs(curPos.x - goalPos.x);
            int yDist = Math.Abs(curPos.y - goalPos.y);
            return xDist + yDist;*/
        }

        private const float Sqrt2 = 1.41f;
        // Move to next edge in edge iteration
        public bool MoveNext()
        {
            if (curDirIndex < curEdgesCount)
            {
                CurrentEdge = curEdges[curDirIndex++];
                return true;
            }
            return false;
        }

        public bool LineOfSight(int startIndex, int endIndex)
        {
            /*if (LineOfSightCells(startIndex, endIndex, null) != LineOfSightCells(endIndex, startIndex, null))
                throw new Exception();*/
            return LineOfSightCells(startIndex, endIndex, null);
        }

        public int numLosChecks;
        public bool LineOfSightCells(int startIndex, int endIndex, List<Vector2Int> res)
        {
            numLosChecks++;
            bool binary = res == null;
            var startPos = tiles.GetPos(startIndex);
            var endPos = tiles.GetPos(endIndex);
            int dx = endPos.x - startPos.x;
            int dy = endPos.y - startPos.y;

            int width = characterWidth, height = characterHeight;
            if (dx == 0)
                width--;
            else if (dy == 0)
                height--;
            for (int i = 0; i <= width; i++)
            {
                for (int j = 0; j <= height; j++)
                {
                    if (!LineOfSightCellsPoint(tiles.GetIndex(startPos.x + i, startPos.y + j),
                        tiles.GetIndex(endPos.x + i, endPos.y + j), res) && binary)
                        return false;
                }
            }
            return binary || res.Count == 0;
        }

        public bool LineOfSightPoint(int startIndex, int endIndex)
        {
            return LineOfSightCellsPoint(startIndex, endIndex, null);
        }

        public bool LineOfSightCellsPoint(int startIndex, int endIndex, List<Vector2Int> res)
        {
            bool binary = res == null;
            var startPos = tiles.GetPos(startIndex);
            var endPos = tiles.GetPos(endIndex);
            int x0 = startPos.x;
            int y0 = startPos.y;
            int x1 = endPos.x;
            int y1 = endPos.y;
            int dx = x1 - x0;
            int dy = y1 - y0;
            int sy;
            if (dy < 0)
            {
                dy = -dy;
                sy = -1;
            }
            else
                sy = 1;
            int sx;
            if (dx < 0)
            {
                dx = -dx;
                sx = -1;
            }
            else
                sx = 1;
            int f = 0;
            if (dx >= dy)
            {
                while (x0 != x1)
                {
                    int xCheck = x0 + (sx - 1) / 2;
                    int yCheck = y0 + (sy - 1) / 2;
                    f = f + dy;
                    if (f >= dx)
                    {
                        if (tiles[xCheck, yCheck].IsBlocked())
                        {
                            if (binary) return false;
                            res.Add(new Vector2Int(xCheck, yCheck));
                        }
                        y0 = y0 + sy;
                        yCheck = y0 + (sy - 1) / 2;
                        f = f - dx;
                    }
                    if (f != 0 && tiles[xCheck, yCheck].IsBlocked())
                    {
                        if (binary) return false;
                        res.Add(new Vector2Int(xCheck, yCheck));
                    }
                    if (dy == 0 && tiles[xCheck, y0].IsBlocked()) //&& tiles[xCheck, y0 - 1].IsBlocked())
                    {
                        if (binary) return false;
                        res.Add(new Vector2Int(xCheck, y0));
                        res.Add(new Vector2Int(xCheck, y0 - 1));
                    }
                    x0 = x0 + sx;
                }
            }
            else
            {
                while (y0 != y1)
                {
                    int xCheck = x0 + (sx - 1) / 2;
                    int yCheck = y0 + (sy - 1) / 2;
                    f = f + dx;
                    if (f >= dy)
                    {
                        if (tiles[xCheck, yCheck].IsBlocked())
                        {
                            if (binary) return false;
                            res.Add(new Vector2Int(xCheck, yCheck));
                        }
                        x0 = x0 + sx;
                        xCheck = x0 + (sx - 1) / 2;
                        f = f - dy;
                    }
                    if (f != 0 && tiles[xCheck, yCheck].IsBlocked())
                    {
                        if (binary) return false;
                        res.Add(new Vector2Int(xCheck, yCheck));
                    }
                    if (dx == 0 && tiles[x0, yCheck].IsBlocked()) //&& tiles[x0 - 1, yCheck].IsBlocked())
                    {
                        if (binary) return false;
                        res.Add(new Vector2Int(x0, yCheck));
                        res.Add(new Vector2Int(x0 - 1, yCheck));
                    }
                    y0 = y0 + sy;
                }
            }
            return binary || res.Count == 0;
        }

        #endregion

        // Is the tile at index blocked?
        private bool IsBlocked(int index)
        {
            return !tiles[index].IsPassable();
        }

        /// <summary>
        /// Is there a blocked tile in the scanline at the side of the character?
        /// </summary>
        /// <param name="index">Current position of the scanline</param>
        /// <param name="add">Scan movement step</param>
        /// <param name="dir">Scan this side of the character</param>
        private bool IsDirBlocked(int index, int add, Direction dir)
        {
            for (int i = 0; i < overlapCount[(int)dir]; i++)
            {
                if (IsBlocked(index))
                    return true;
                index += add;
            }
            return false;
        }

        private enum Direction : byte
        {
            Left,
            Right,
            Up,
            Down
        }
    }
}
