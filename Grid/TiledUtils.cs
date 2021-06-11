using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using MathTools;

namespace Pathfinding.Internal.TiledUtils
{
    public enum TileShape : byte
    {
        None,
        Solid,
        GroundHalfSolid,
        Ground45Left,
        Ground45Right,
        Ground22TopLeft,
        Ground22BottomLeft,
        Ground22BottomRight,
        Ground22TopRight,
        CeilingHalfSolid,
        Ceiling45Left,
        Ceiling45Right,
        Ceiling22BottomLeft,
        Ceiling22TopLeft,
        Ceiling22TopRight,
        Ceiling22BottomRight,
        OneWay,
        Ladder,
        LadderTop
    }

    [Flags]
    public enum TileShapeFlags : int
    {
        Null = 0,
        None = 1 << 0,
        Solid = 1 << 1,
        GroundHalfSolid = 1 << 2,
        Ground45Left = 1 << 3,
        Ground45Right = 1 << 4,
        Ground22TopLeft = 1 << 5,
        Ground22BottomLeft = 1 << 6,
        Ground22BottomRight = 1 << 7,
        Ground22TopRight = 1 << 8,
        CeilingHalfSolid = 1 << 9,
        Ceiling45Left = 1 << 10,
        Ceiling45Right = 1 << 11,
        Ceiling22BottomLeft = 1 << 12,
        Ceiling22TopLeft = 1 << 13,
        Ceiling22TopRight = 1 << 14,
        Ceiling22BottomRight = 1 << 15,
        OneWay = 1 << 16,
        Ladder = 1 << 17,
        LadderTop = 1 << 18,


        GroundSlopeMask = Ground45Left | Ground45Right | Ground22TopLeft | Ground22BottomLeft | Ground22BottomRight | Ground22TopRight,
        CeilingSlopeMask = Ceiling45Left | Ceiling45Right | Ceiling22BottomLeft | Ceiling22TopLeft | Ceiling22TopRight | Ceiling22BottomRight,
        SlopeMask = GroundSlopeMask | CeilingSlopeMask,
        CeilingMask = CeilingHalfSolid | CeilingSlopeMask,
        LadderMask = Ladder | LadderTop,
        OneWayMask = OneWay | LadderTop,
        PassableMask = None | LadderMask | OneWay,
    }

    public static class TileExtensions
    {
        /// <summary>
        /// Can bodies pass through this tile shape?
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsPassable(this TileShape collisionShape)
        {
            return TileShapeFlags.PassableMask.Contains(collisionShape);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsBlocked(this TileShape collisionShape)
        {
            return !collisionShape.IsPassable();
        }

        // Does the flag set completely contain the other flag/flag set?
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Contains(this TileShapeFlags flags, TileShapeFlags collisionShape)
        {
            return (flags & collisionShape) == collisionShape;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Contains(this TileShapeFlags flags, TileShape collisionShape)
        {
            return Contains(flags, ToMask(collisionShape));
        }

        // Convert to mask
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static TileShapeFlags ToMask(this TileShape collisionShape)
        {
            return (TileShapeFlags)(1 << (int)collisionShape);
        }

        public static int TileSize { get; private set; }
        public static int HalfTileSize { get; private set; }

        public static void Initialize(int tileSize)
        {
            TileExtensions.TileSize = tileSize;
            TileExtensions.HalfTileSize = tileSize / 2;
        }

        public static bool OutOfBounds(Vector2Int tilePoint, int mapSizeX, int mapSizeY)
        {
            return tilePoint.x < 0 || tilePoint.x >= mapSizeX ||
                tilePoint.y < 0 || tilePoint.y >= mapSizeY;
        }
    }

    /// <summary>
    /// A 2D grid of Tile objects.
    /// </summary>
    public class TileGrid
    {
        public readonly TileShape[] raw;

        /// <summary>
        /// Gets or sets a Tile at a given index.
        /// </summary>
        /// <param name="x">The X index.</param>
        /// <param name="y">The Y index.</param>
        /// <returns></returns>
        public TileShape this[int x, int y]
        {
            get { return raw[GetIndex(x, y)]; }
            set { raw[GetIndex(x, y)] = value; }
        }

        public TileShape this[int index]
        {
            get { return raw[index]; }
            set { raw[index] = value; }
        }

        /// <summary>
        /// Gets the width of the grid.
        /// </summary>
        public int Width { get; private set; }

        /// <summary>
        /// Gets the height of the grid.
        /// </summary>
        public int Height { get; private set; }

        /// <summary>
        /// Creates a new TileGrid.
        /// </summary>
        /// <param name="width">The width of the grid.</param>
        /// <param name="height">The height of the grid.</param>
        public TileGrid(int width, int height)
        {
            raw = new TileShape[width * height];
            Width = width;
            Height = height;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetIndex(int x, int y)
        {
            return (y * Width) + x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetIndex(Vector2Int pos)
        {
            return GetIndex(pos.x, pos.y);
        }

        public Vector2Int GetPos(int index)
        {
            return new Vector2Int(index % Width, index / Width);
        }
    }
}
