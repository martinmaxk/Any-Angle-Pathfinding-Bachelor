/***************************************************************************
 *   Developer: Francisco Martínez del Río (2012)                          *  
 *   fmartin@ujaen.es                                                      *
 *   Version: 1.0                                                          *
 *                                                                         *
 *   This is a public domain program                                       *
 ***************************************************************************/

using Advanced.Algorithms.DataStructures;
using MathTools;
using System;
using System.Collections.Generic;
using BoolOpUtils = PolyBoolOpMartinez.PolyBoolOp.BoolOpUtils;

namespace PolyBoolOpMartinez
{
    public class Contour
    {
        // Set of points conforming the external contour
        private Vector2D[] points = new Vector2D[3];
        // Holes of the contour. They are stored as the indexes of the holes in a polygon class
        private int[] holes = new int[1];
        public bool External { get; set; } = true; // is the contour an external contour? (i.e., is it not a hole?)
        private bool precomputedCC = false;
        private bool cc;

        public int Nvertices { get; private set; }
        public int Nedges => Nvertices;
        public int Nholes { get; private set; }

        public void CopyFrom(Contour contour)
        {
            if (points.Length < contour.Nvertices)
                points = new Vector2D[contour.Nvertices];
            if (holes.Length < contour.Nholes)
                holes = new int[contour.Nholes];
            for (int i = 0; i < contour.Nvertices; i++)
                points[i] = contour.points[i];
            for (int i = 0; i < contour.Nholes; i++)
                holes[i] = contour.holes[i];
            Nvertices = contour.Nvertices;
            Nholes = contour.Nholes;
            External = contour.External;
            precomputedCC = contour.precomputedCC;
            cc = contour.cc;
        }

        /* Get the p-th vertex of the external contour */
        public Vector2D Vertex(int p)
        {
            return points[p];
        }

        public Segment Segment(int p)
        {
            if (p == Nvertices - 1)
                return new Segment(Last(), points[0]);
            return new Segment(points[p], points[p + 1]);
        }

        // Get the bounding box
        public BboxD Bbox()
        {
            if (Nvertices == 0)
                return new BboxD();
            var b = BoolOpUtils.Bbox(Vertex(0));
            for (int i = 1; i < Nvertices; i++)
                b = b.Union(BoolOpUtils.Bbox(Vertex(i)));
            return b;
        }

        // Return if the contour is counterclockwise oriented
        public bool Counterclockwise()
        {
            if (precomputedCC)
                return cc;
            precomputedCC = true;
            double area = 0.0;
            for (int c = 0; c < Nvertices - 1; c++)
                area += Vertex(c).x * Vertex(c + 1).y - Vertex(c + 1).x * Vertex(c).y;
            area += Vertex(Nvertices - 1).x * Vertex(0).y - Vertex(0).x * Vertex(Nvertices - 1).y;
            cc = area < 0.0;
            return cc;
        }

        /* Return if the contour is clockwise oriented */
        public bool Clockwise()
        {
            return !Counterclockwise();
        }

        public void ChangeOrientation()
        {
            Array.Reverse(points, 0, Nvertices);
            cc = !cc;
        }

        public void SetClockwise()
        {
            if (Counterclockwise())
                ChangeOrientation();
        }

        public void SetCounterClockwise()
        {
            if (Clockwise())
                ChangeOrientation();
        }

        public void Move(double x, double y)
        {
            for (int i = 0; i < Nvertices; i++)
                points[i] = new Vector2D(points[i].x + x, points[i].y + y);
        }

        public void Add(Vector2D s)
        {
            if (Nvertices == points.Length)
                Array.Resize(ref points, points.Length * 2);
            points[Nvertices++] = s;
        }

        public void Clear()
        {
            Nvertices = Nholes = 0;
            External = true;
            precomputedCC = false;
        }

        public void ClearHoles()
        {
            Nholes = 0;
        }

        public Vector2D Last()
        {
            return points[Nvertices - 1];
        }

        public void AddHole(int index)
        {
            if (Nholes == holes.Length)
                Array.Resize(ref holes, holes.Length * 2);
            holes[Nholes++] = index;
        }

        public int Hole(int p)
        {
            return holes[p];
        }

        public void RemoveCollinearEdges()
        {
            int newNVertices = 0;
            for (int i = 0; i < Nvertices; i++)
            {
                int prevIdx = i == 0 ? Nvertices - 1 : i - 1;
                int nextIdx = i == Nvertices - 1 ? 0 : (i + 1);
                if (!BoolOpUtils.Collinear(points[prevIdx], points[i], points[nextIdx]))
                    points[newNVertices++] = points[i];
            }
            Nvertices = newNVertices;
        }

        public string ToGeogebra()
        {
            string output = "Polygon(";
            if (Nvertices == 0)
                return output + ")";
            for (int i = 0; i < Nvertices; i++)
            {
                var point = points[i];
                output += "(" + point.x + ", " + (-point.y) + "), ";
            }
            output = output.Remove(output.Length - 2);
            return output + ")";
        }

        public override string ToString()
        {
            string output = "";
            output += Nvertices + "\n";
            for (int i = 0; i < Nvertices; i++)
            {
                var point = points[i];
                output += "\t" + point.x + " " + point.y + "\n";
            }
            return output;
        }
    }

    public class Polygon
    {
        // Set of contours conforming the polygon
        private Contour[] contours = new Contour[1];
        public int Ncontours { get; private set; }

        public void CopyFrom(Polygon pol)
        {
            for (int i = 0; i < Ncontours; i++)
                contours[i].CopyFrom(pol.contours[i]);
            while (Ncontours < pol.Ncontours)
            {
                var contour = AddNewContour();
                contour.CopyFrom(pol.contours[Ncontours - 1]);
            }
            Ncontours = pol.Ncontours;
        }

        public void Join(Polygon pol)
        {
            int size = Ncontours;
            for (int i = 0; i < pol.Ncontours; i++)
            {
                var other = pol.Contour(i);
                var contour = AddNewContour();
                contour.CopyFrom(other);
                contour.ClearHoles();
                for (int j = 0; j < contour.Nholes; j++)
                    contour.AddHole(other.Hole(j) + size);
            }
        }

        // Get the p-th contour
        public Contour Contour(int p)
        {
            return contours[p];
        }

        public int Nvertices()
        {
            int nv = 0;
            for (int i = 0; i < Ncontours; i++)
                nv += contours[i].Nvertices;
            return nv;
        }

        public BboxD Bbox()
        {
            if (Ncontours == 0)
                return new BboxD();
            var bb = contours[0].Bbox();
            for (int i = 1; i < Ncontours; i++)
                bb = bb.Union(contours[i].Bbox());
            return bb;
        }

        public void Move(double x, double y)
        {
            for (int i = 0; i < Ncontours; i++)
                contours[i].Move(x, y);
        }

        public Contour AddNewContour()
        {
            Contour contour;
            if (Ncontours == contours.Length)
            {
                Array.Resize(ref contours, contours.Length * 2);
                contour = contours[Ncontours] = new Contour();
            }
            else
            {
                contour = contours[Ncontours];
                if (contour == null)
                    contour = contours[Ncontours] = new Contour();
                else
                    contour.Clear();
            }
            Ncontours++;
            return contour;
        }

        public Contour Last()
        {
            return contours[Ncontours - 1];
        }

        public void Clear()
        {
            Ncontours = 0;
        }

        public void RemoveCollinearEdges()
        {
            for (int i = 0; i < Ncontours; i++)
                contours[i].RemoveCollinearEdges();
        }

        public void FromEnumerable(IEnumerable<Vector2D> list)
        {
            var contour = AddNewContour();
            foreach (var v in list)
                contour.Add(v);
        }

        public void FromEnumerable(IEnumerable<Vector2> list)
        {
            var contour = AddNewContour();
            foreach (var v in list)
                contour.Add((Vector2D)v);
        }

        public string ToGeogebra()
        {
            string output = "";
            for (int i = 0; i < Ncontours; i++)
                output += Contour(i).ToGeogebra() + "\n";
            for (int i = 0; i < Ncontours; i++)
            { // write the holes of every contour
                if (Contour(i).Nholes > 0)
                {
                    output += i + ": ";
                    for (int j = 0; j < Contour(i).Nholes; j++)
                        output += Contour(i).Hole(j) + (j == Contour(i).Nholes - 1 ? "\n" : " ");
                }
            }
            return output;
        }

        public override string ToString()
        {
            string output = "";
            output += Ncontours + "\n";
            for (int i = 0; i < Ncontours; i++) // write the contours
                output += Contour(i).ToString();
            for (int i = 0; i < Ncontours; i++)
            { // write the holes of every contour
                if (Contour(i).Nholes > 0)
                {
                    output += i + ": ";
                    for (int j = 0; j < Contour(i).Nholes; j++)
                        output += Contour(i).Hole(j) + (j == Contour(i).Nholes - 1 ? "\n" : " ");
                }
            }
            return output;
        }

        private static readonly char[] whitespaceChars = new char[] { ' ', '\t', '\r' };
        private const StringSplitOptions splitOpt = StringSplitOptions.RemoveEmptyEntries;
        public void ReadFromString(string str)
        {
            ReadFromLines(str.Split(new char[] { '\n' }, splitOpt));
        }

        public void ReadFromLines(string[] lines)
        {
            // read the contours
            int lineIdx = 0;
            int Ncontours = int.Parse(lines[lineIdx++]);
            double px, py;
            string[] words;
            for (int i = 0; i < Ncontours; i++)
            {
                int npoints = int.Parse(lines[lineIdx++]);
                var contour = AddNewContour();
                for (int j = 0; j < npoints; j++)
                {
                    words = lines[lineIdx++].Split(whitespaceChars, splitOpt);
                    px = double.Parse(words[0]);
                    py = double.Parse(words[1]);
                    if (j > 0 && BoolOpUtils.Approx(px, contour.Last().x) &&
                        BoolOpUtils.Approx(py, contour.Last().y))
                        continue;
                    if (j == npoints - 1 && BoolOpUtils.Approx(px, contour.Vertex(0).x) &&
                        BoolOpUtils.Approx(py, contour.Vertex(0).y))
                        continue;
                    contour.Add(new Vector2D(px, py));
                }
                if (contour.Nvertices < 3)
                {
                    Ncontours--;
                    continue;
                }
            }
            // read holes information
            while (lineIdx < lines.Length)
            {
                string line = lines[lineIdx++];
                if (!line.Contains(":"))
                    throw new FormatException("Expected ':' in holes line " + line);
                line = line.Replace(":", "");
                words = line.Split(whitespaceChars, splitOpt);
                int contourId = int.Parse(words[0]);
                for (int i = 1; i < words.Length; i++)
                {
                    int hole = int.Parse(words[i++]);
                    Contour(contourId).AddHole(hole);
                    Contour(hole).External = false;
                }
            }
        }

        public void ComputeHoles()
        {
            int curSeId = 0;
            if (Ncontours < 2)
            {
                if (Ncontours == 1 && Contour(0).Clockwise())
                    Contour(0).ChangeOrientation();
                return;
            }
            var ev = new List<SweepEvent>(Nvertices() * 2);
            for (int i = 0; i < Ncontours; i++)
            {
                Contour(i).SetCounterClockwise();
                for (int j = 0; j < Contour(i).Nedges; j++)
                {
                    var s = Contour(i).Segment(j);
                    if (s.IsVertical()) // vertical segments are not processed
                        continue;
                    var se1 = new SweepEvent(curSeId++, s.source, true, i);
                    ev.Add(se1);
                    var se2 = new SweepEvent(curSeId++, s.target, true, i);
                    ev.Add(se2);
                    se1.otherEvent = se2;
                    se2.otherEvent = se1;
                    if (se1.point.x < se2.point.x)
                    {
                        se2.left = false;
                        se1.inOut = false;
                    }
                    else
                    {
                        se1.left = false;
                        se2.inOut = true;
                    }
                }
            }
            ev.Sort(new SweepEventComp());

            var sl = new RedBlackTree<SweepEvent>(new SegmentComp()); // Status line
            var processed = new bool[Ncontours];
            var holeOf = new int[Ncontours];
            for (int i = 0; i < holeOf.Length; i++)
                holeOf[i] = -1;
            int nprocessed = 0;
            for (int i = 0; i < ev.Count && nprocessed < Ncontours; i++)
            {
                var e = ev[i];

                if (e.left)
                { // the segment must be inserted into S
                    e.posSL = sl.Insert(e);
                    if (!processed[e.pos])
                    {
                        processed[e.pos] = true;
                        nprocessed++;
                        if (!e.posSL.NextLower().IsNull())
                            Contour(e.pos).SetCounterClockwise();
                        else
                        {
                            var prev = e.posSL.NextLower();
                            if (!prev.Value.inOut)
                            {
                                holeOf[e.pos] = prev.Value.pos;
                                Contour(e.pos).External = false;
                                Contour(prev.Value.pos).AddHole(e.pos);
                                if (Contour(prev.Value.pos).Counterclockwise())
                                    Contour(e.pos).SetClockwise();
                                else
                                    Contour(e.pos).SetCounterClockwise();
                            }
                            else if (holeOf[prev.Value.pos] != -1)
                            {
                                holeOf[e.pos] = holeOf[prev.Value.pos];
                                Contour(e.pos).External = false;
                                Contour(holeOf[e.pos]).AddHole(e.pos);
                                if (Contour(holeOf[e.pos]).Counterclockwise())
                                    Contour(e.pos).SetClockwise();
                                else
                                    Contour(e.pos).SetCounterClockwise();
                            }
                            else
                                Contour(e.pos).SetCounterClockwise();
                        }
                    }
                }
                else // the segment must be removed from S
                    sl.Delete(e.otherEvent.posSL);
            }
        }
    }
}
