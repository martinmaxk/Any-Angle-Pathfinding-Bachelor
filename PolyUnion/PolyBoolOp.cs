/***************************************************************************
 *   Developer: Francisco Martínez del Río (2012)                          *  
 *   fmartin@ujaen.es                                                      *
 *   Version: 1.0                                                          *
 *                                                                         *
 *   This is a public domain program                                       *
 ***************************************************************************/

using Advanced.Algorithms.DataStructures;
using MathTools;
using Debugging;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using BoolOpUtils = PolyBoolOpMartinez.PolyBoolOp.BoolOpUtils;

namespace PolyBoolOpMartinez
{
    public enum BoolOpType { Intersection, Union, Difference, Xor };
    public enum EdgeType { Normal, NonContributing, SameTransition, DifferentTransition };
    public enum PolygonType { Subject, Clipping };

    public struct Segment
    {
        // Segment endpoints 
        public Vector2D source, target;

        // Constructor from two points
        public Segment(Vector2D source, Vector2D target)
        {
            this.source = source;
            this.target = target;
        }

        // Return the point of the segment with lexicographically smallest coordinate
        public Vector2D Min()
        {
            return (source.x < target.x) || (BoolOpUtils.Approx(source.x, target.x) && source.y < target.y) ? source : target;
        }

        // Return the point of the segment with lexicographically largest coordinate
        public Vector2D Max()
        {
            return (source.x > target.x) || (BoolOpUtils.Approx(source.x, target.x) && source.y > target.y) ? source : target;
        }

        public bool Degenerate() { return BoolOpUtils.Approx(source, target); }
        public bool IsVertical() { return BoolOpUtils.Approx(source.x, target.x); }
        // Change the segment orientation
        public Segment ChangeOrientation() { return new Segment(target, source); }

        public override string ToString()
        {
            return source + "-" + target;
        }
    };

    public class SweepEvent
    {
        public readonly int id; // used for consistent sorting
        public bool left;              // is point the left endpoint of the edge (point, otherEvent->point)?
        public Vector2D point;          // point associated with the event
        public SweepEvent otherEvent; // event associated to the other endpoint of the edge
        public PolygonType pol;        // Polygon to which the associated segment belongs to
        public EdgeType type;
        // The following fields are only used in "left" events
        /* Does segment (point, otherEvent->p) represent an inside-outside transition 
           in the polygon for a vertical ray from (p.x, -infinite)? */
        public bool inOut;
        public bool otherInOut; // inOut transition for the segment from the other polygon preceding this segment in sl
        public BSTIterator<SweepEvent> posSL; // Position of the event (line segment) in sl
        public SweepEvent prevInResult; // previous segment in sl belonging to the result of the boolean operation
        public bool inResult;
        public int pos;
        public bool resultInOut;
        public int contourId;

        public SweepEvent(int id, bool left, Vector2D p, SweepEvent other, PolygonType pt, EdgeType et = default(EdgeType))
        {
            this.id = id;
            this.left = left;
            this.point = p;
            this.otherEvent = other;
            this.pol = pt;
            this.type = et;
        }

        // Used in Polygon.ComputeHoles
        public SweepEvent(int id, Vector2D p, bool left, int pos)
        {
            this.id = id;
            this.point = p;
            this.left = left;
            this.pos = pos;
        }

        // Is the line segment (point, otherEvent->point) below point p 
        public bool Below(Vector2D p)
        {
            return left ? BoolOpUtils.SignedArea(point, otherEvent.point, p) > 0 :
                                   BoolOpUtils.SignedArea(otherEvent.point, point, p) > 0;
        }
        // Is the line segment (point, otherEvent->point) above point p
        public bool Above(Vector2D p) { return !Below(p); }
        // Is the line segment (point, otherEvent->point) a vertical line segment
        public bool Vertical() { return BoolOpUtils.Approx(point.x, otherEvent.point.x); }
        
        // Is the line segment below segment of cmpEvent
        public bool Below(SweepEvent cmpEvent)
        {
            var seg = left ? Segment() : otherEvent.Segment();
            var cmpSeg = cmpEvent.left ? cmpEvent.Segment() : cmpEvent.otherEvent.Segment();
            double area1 = BoolOpUtils.SignedArea(seg.source, seg.target, cmpSeg.source);
            if (!BoolOpUtils.Approx(area1, 0))
                return area1 > 0;
            return BoolOpUtils.SignedArea(seg.source, seg.target, cmpSeg.target) > 0;
        }
        // Is the line segment above segment of cmpEvent
        public bool Above(SweepEvent cmpEvent) { return !Below(cmpEvent); }

        // Return the line segment associated to the SweepEvent
        public Segment Segment() { return new Segment(point, otherEvent.point); }

        public override string ToString()
        {
            string output = "";
            output += point;
            output += " (" + (left ? "left" : "right") + ')';
            var s = new Segment(point, otherEvent.point);
            output += " S:[" + s.Min() + " - " + s.Max() + "]";
            output += " (" + pol + ')';
            output += " (" + type + ')';
            output += " (" + (inOut ? "inOut" : "outIn") + ')';
            output += " otherInOut: (" + (otherInOut ? "inOut" : "outIn") + ')';
            return output;
        }
    }

    // For sorting sweep events
    public class SweepEventComp : Comparer<SweepEvent>
    {
        public override int Compare(SweepEvent e1, SweepEvent e2)
        {
            return ComesAfter(e1, e2) ? 1 : -1;
        }

        // Compare two sweep events
        // Return true means that e1 is placed at the event queue after e2, i.e., e1 is processed by the algorithm after e2
        public static bool ComesAfter(SweepEvent e1, SweepEvent e2)
        {
            // Different x-coordinate
            if (!BoolOpUtils.Approx(e1.point.x, e2.point.x))
                return e1.point.x > e2.point.x;
            // Different points, but same x-coordinate. The event with lower y-coordinate is processed first
            if (!BoolOpUtils.Approx(e1.point.y, e2.point.y))
                return e1.point.y > e2.point.y;
            // Same point, but one is a left endpoint and the other a right endpoint. The right endpoint is processed first
            if (e1.left != e2.left)
                return e1.left;
            // Same point, both events are left endpoints or both are right endpoints.
            // Not collinear
            if (!BoolOpUtils.Approx(BoolOpUtils.SignedArea(e1.point, e1.otherEvent.point, e2.otherEvent.point), 0))
                return e1.Above(e2.otherEvent.point); // the event associate to the bottom segment is processed first
            return e1.pol > e2.pol;
        }
    }

    public class SegmentComp : Comparer<SweepEvent>
    {
        public override int Compare(SweepEvent le1, SweepEvent le2)
        {
            return ComesBefore(le1, le2) ? -1 : 1;
        }

        // le1 and le2 are the left events of line segments 
        // (le1.point, le1.otherEvent.point) and (le2.point, le2.otherEvent.point)
        private bool ComesBefore(SweepEvent le1, SweepEvent le2)
        {
            if (le1 == le2)
                return false;
            bool collinear1 =
                BoolOpUtils.Approx(BoolOpUtils.SignedArea(le1.point, le1.otherEvent.point, le2.point), 0);
            bool collinear2 = BoolOpUtils.Approx(
                BoolOpUtils.SignedArea(le1.point, le1.otherEvent.point, le2.otherEvent.point), 0);
            if (!collinear1 || !collinear2)
            {
                // Segments are not collinear
                // If they share their left endpoint use the right endpoint to sort
                if (BoolOpUtils.Approx(le1.point, le2.point))
                    return le1.Below(le2.otherEvent.point);
                // Different left endpoint: use the left endpoint to sort
                if (BoolOpUtils.Approx(le1.point.x, le2.point.x))
                    return le1.point.y < le2.point.y;
                // Has the line segment associated to e1 been inserted into S after the line segment associated to e2?
                if (SweepEventComp.ComesAfter(le1, le2))
                {
                    if (BoolOpUtils.Collinear(le2.point, le2.otherEvent.point, le1.point))
                        return le2.Above(le1.otherEvent.point);
                    else
                        return le2.Above(le1.point);
                }
                // The line segment associated to e2 has been inserted into S after the line segment associated to e1
                return le1.Below(collinear1 ? le2.otherEvent.point : le2.point);
            }
            // Segments are collinear
            if (le1.pol != le2.pol)
                return le1.pol < le2.pol;
            // Just a consistent criterion is used
            if (BoolOpUtils.Approx(le1.point, le2.point))
                return le1.id < le2.id;
            return SweepEventComp.ComesAfter(le1, le2);
        }
    }

    public struct Bbox
    {
        public float xmin, xmax, ymin, ymax;

        public Bbox(float xmin = 0, float ymin = 0, float xmax = 0, float ymax = 0)
        {
            this.xmin = xmin; this.ymin = ymin;
            this.xmax = xmax; this.ymax = ymax;
        }

        public Bbox Union(Bbox b)
        {
            return new Bbox(Math.Min(xmin, b.xmin), Math.Min(ymin, b.ymin),
                Math.Max(xmax, b.xmax), Math.Max(ymax, b.ymax));
        }

        // Returns Bbox expanded such that it encloses both this Bbox and p
        public Bbox Enclose(Vector2 p)
        {
            return new Bbox(Math.Min(xmin, p.x), Math.Min(ymin, p.y),
                Math.Max(xmax, p.x), Math.Max(ymax, p.y));
        }
    }

    public struct BboxD
    {
        public double xmin, xmax, ymin, ymax;

        public BboxD(double xmin = 0, double ymin = 0, double xmax = 0, double ymax = 0)
        {
            this.xmin = xmin; this.ymin = ymin;
            this.xmax = xmax; this.ymax = ymax;
        }

        public BboxD Union(BboxD b)
        {
            return new BboxD(Math.Min(xmin, b.xmin), Math.Min(ymin, b.ymin),
                Math.Max(xmax, b.xmax), Math.Max(ymax, b.ymax));
        }

        // Returns Bbox expanded such that it encloses both this Bbox and p
        public BboxD Enclose(Vector2D p)
        {
            return new BboxD(Math.Min(xmin, p.x), Math.Min(ymin, p.y),
                Math.Max(xmax, p.x), Math.Max(ymax, p.y));
        }

        public static explicit operator Bbox(BboxD value)
        {
            return new Bbox((float)value.xmin, (float)value.ymin, (float)value.xmax, (float)value.ymax);
        }
    }

    public class PolyBoolOp
    {
        // Unique Sweep Event ID generator (used for consistent sorting)
        private int curSeId;
        private Polygon subject, clipping, result;
        private BoolOpType operation;
        private PriorityQueue<SweepEvent> eq; // event queue (sorted events to be processed)
        private RedBlackTree<SweepEvent> sl;  // segments intersecting the sweep line
        private List<SweepEvent> sortedEvents, tempSortedEvents = new List<SweepEvent>(),
            tempResultEvents = new List<SweepEvent>();
        private List<bool> tempProcessed = new List<bool>();
        private List<int> tempDepth = new List<int>(), tempHoleOf = new List<int>();

        public PolyBoolOp(double epsilonD = 1e-8)
        {
            BoolOpUtils.SetEpsilon(epsilonD);
            this.eq = new PriorityQueue<SweepEvent>(1, new SweepEventComp());
            this.sl = new RedBlackTree<SweepEvent>(new SegmentComp());
            this.sortedEvents = new List<SweepEvent>();
        }

        /*public Polygon UnionAll(Polygon[] polygons, int start, int end)
        {
            if (start == end)
                return polygons[start];
            var res = new Polygon();
            int mid = (start + end) / 2;
            Union(UnionAll(polygons, start, mid), UnionAll(polygons, mid + 1, end), res);
            return res;
        }*/

        private Polygon[] polyPool;
        public void UnionAll(List<Polygon> polygons, Polygon finalRes)
        {
            // First polygon will be for temp storage
            int poolCount = MathExt.CeilDiv(polygons.Count, 2) + 1;
            int prevCount = polyPool == null ? 0 : polyPool.Length;
            for (int i = 0; i < prevCount; i++)
                polyPool[i].Clear();
            if (prevCount < poolCount)
            {
                polyPool = new Polygon[poolCount];
                for (int i = prevCount; i < polyPool.Length; i++)
                    polyPool[i] = new Polygon();
            }
            for (int i = 0; i < polygons.Count - 1; i += 2)
                Union(polygons[i], polygons[i + 1], polyPool[(i / 2) + 1]);
            // Merge last union result with spare polygon on odd number of polygons
            if (polygons.Count % 2 != 0)
            {
                UnionSwap(ref polyPool[poolCount - 2], polygons[polygons.Count - 1], ref polyPool[poolCount - 1]);
                poolCount--;
            }
            poolCount--;
            while (poolCount > 1)
            {
                for (int i = 1; i < poolCount; i += 2)
                    Union(polyPool[i], polyPool[i + 1], polyPool[i / 2]);
                prevCount = poolCount;
                poolCount /= 2;
                if (prevCount % 2 != 0)
                    UnionSwap(ref polyPool[poolCount - 1], polyPool[prevCount], ref polyPool[poolCount]);
                BoolOpUtils.Swap(ref polyPool[0], ref polyPool[poolCount]);
            }
            finalRes.CopyFrom(polyPool[1]);
        }

        // Perform union between subject and clip with res as temp buffer and then swap subject and res
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void UnionSwap(ref Polygon subject, Polygon clip, ref Polygon res)
        {
            Union(subject, clip, res);
            BoolOpUtils.Swap(ref subject, ref res);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Union(Polygon subject, Polygon clip, Polygon res)
        {
            Run(subject, clip, res, BoolOpType.Union);
        }

        public void Run(Polygon subject, Polygon clip, Polygon res, BoolOpType op)
        {
            Debugging.Debug.Assert(subject != res && clip != res);
            res.Clear();
            this.subject = subject;
            this.clipping = clip;
            this.result = res;
            this.operation = op;
            /*Console.WriteLine("Run");
            Console.WriteLine(subject.ToGeogebra());
            Console.WriteLine(clip.ToGeogebra());*/
            /*run = subject.ToGeogebra().Contains("Polygon((1215, -2495), (1312, -2495), (1312, -2496), (1216, -2592), (1215, -2592))");
            if (run)
            {
                Console.WriteLine("Start");
                Console.WriteLine(subject.ToGeogebra());
                Console.WriteLine(clip.ToGeogebra());
            }*/
            curSeId = 0;
            BboxD subjectBB = subject.Bbox();     // for optimizations 1 and 2
            BboxD clippingBB = clipping.Bbox();   // for optimizations 1 and 2
            double minMax = Math.Min(subjectBB.xmax, clippingBB.xmax); // for optimization 2
            if (TrivialOperation(subjectBB, clippingBB)) // trivial cases can be quickly resolved without sweeping the plane
                return;
            for (int i = 0; i < subject.Ncontours; i++)
                for (int j = 0; j < subject.Contour(i).Nvertices; j++)
                    ProcessSegment(subject.Contour(i).Segment(j), PolygonType.Subject);
            for (int i = 0; i < clipping.Ncontours; i++)
                for (int j = 0; j < clipping.Contour(i).Nvertices; j++)
                    ProcessSegment(clipping.Contour(i).Segment(j), PolygonType.Clipping);

            BSTIterator<SweepEvent> it, prev, next;
            while (eq.Count > 0)
            {
                var se = eq.Peek();
                // optimization 2
                if ((operation == BoolOpType.Intersection && se.point.x > minMax) ||
                    (operation == BoolOpType.Difference && se.point.x > subjectBB.xmax))
                {
                    ConnectEdges();
                    return;
                }
                sortedEvents.Add(se);
                eq.Dequeue();
                if (se.left)
                { // the line segment must be inserted into sl
                    se.posSL = it = sl.Insert(se);
                    prev = it.NextLower();
                    next = it.NextHigher();
                    ComputeFields(se, prev);
                    // Process a possible intersection between "se" and its next neighbor in sl
                    if (!next.IsNull())
                    {
                        if (PossibleIntersection(se, next.Value) == 2)
                        {
                            ComputeFields(se, prev);
                            ComputeFields(next.Value, it);
                        }
                    }
                    // Process a possible intersection between "se" and its previous neighbor in sl
                    if (!prev.IsNull())
                    {
                        if (PossibleIntersection(prev.Value, se) == 2)
                        {
                            var prevprev = prev.NextLower();
                            ComputeFields(prev.Value, prevprev);
                            ComputeFields(se, prev);
                        }
                    }
                }
                else
                { // the line sw_egment must be removed from sl
                    se = se.otherEvent; // we work with the left event
                    it = se.posSL; // se.posSL; is equal to sl.Find(se); but faster
                    prev = it.NextLower();
                    next = it.NextHigher();
                    // delete line segment associated to "se" from sl and check for intersection between the neighbors of "se" in sl
                    sl.Delete(it);
                    if (!next.IsNull() && !prev.IsNull())
                        PossibleIntersection(prev.Value, next.Value);
                }
            }
            ConnectEdges();
            res.RemoveCollinearEdges();
            // Cleanup for next run (eq is already empty)
            sl.Clear();
            sortedEvents.Clear();
            //Console.WriteLine(res);
        }

        private bool TrivialOperation(BboxD subjectBB, BboxD clippingBB)
        {
            // Test 1 for trivial result case
            if (subject.Ncontours * clipping.Ncontours == 0)
            { // At least one of the polygons is empty
                if (operation == BoolOpType.Difference)
                    result.CopyFrom(subject);
                if (operation == BoolOpType.Union || operation == BoolOpType.Xor)
                    result.CopyFrom((subject.Ncontours == 0) ? clipping : subject);
                return true;
            }
            // Test 2 for trivial result case
            if (subjectBB.xmin > clippingBB.xmax || clippingBB.xmin > subjectBB.xmax ||
                subjectBB.ymin > clippingBB.ymax || clippingBB.ymin > subjectBB.ymax)
            {
                // the bounding boxes do not overlap
                if (operation == BoolOpType.Difference)
                    result.CopyFrom(subject);
                if (operation == BoolOpType.Union || operation == BoolOpType.Xor)
                {
                    result.CopyFrom(subject);
                    result.Join(clipping);
                }
                return true;
            }
            return false;
        }

        // Compute the events associated to segment s, and insert them into pq and eq 
        private void ProcessSegment(Segment s, PolygonType pt)
        {
            /*	if (s.degenerate ()) // if the two edge endpoints are equal the segment is dicarded
                    return;          // This can be done as preprocessing to avoid "polygons" with less than 3 edges */
            var e1 = new SweepEvent(curSeId++, true, s.source, null, pt);
            var e2 = new SweepEvent(curSeId++, true, s.target, e1, pt);
            e1.otherEvent = e2;

            if (s.Min() == s.source)
                e2.left = false;
            else
                e1.left = false;
            eq.Enqueue(e1);
            eq.Enqueue(e2);
        }

        // Compute several fields of left event le
        private void ComputeFields(SweepEvent le, BSTIterator<SweepEvent> prev)
        {
            // compute inOut and otherInOut fields
            if (prev.IsNull())
            {
                le.inOut = false;
                le.otherInOut = true;
            }
            else if (le.pol == prev.Value.pol)
            { // previous line segment in sl belongs to the same polygon that "se" belongs to
                le.inOut = !prev.Value.inOut;
                le.otherInOut = prev.Value.otherInOut;
            }
            else
            { // previous line segment in sl belongs to a different polygon that "se" belongs to
                le.inOut = !prev.Value.otherInOut;
                le.otherInOut = prev.Value.Vertical() ? !prev.Value.inOut : prev.Value.inOut;
            }
            // compute prevInResult field
            if (!prev.IsNull())
                le.prevInResult = (!InResult(prev.Value) || prev.Value.Vertical()) ? prev.Value.prevInResult : prev.Value;
            // check if the line segment belongs to the Boolean operation
            le.inResult = InResult(le);
        }

        // Return if the left event le belongs to the result of the Boolean operation
        private bool InResult(SweepEvent le)
        {
            switch (le.type)
            {
                case EdgeType.Normal:
                    switch (operation)
                    {
                        case BoolOpType.Intersection:
                            return !le.otherInOut;
                        case BoolOpType.Union:
                            return le.otherInOut;
                        case BoolOpType.Difference:
                            return (le.pol == PolygonType.Subject && le.otherInOut) ||
                                (le.pol == PolygonType.Clipping && !le.otherInOut);
                        case BoolOpType.Xor:
                            return true;
                    }
                    break;
                case EdgeType.SameTransition:
                    return operation == BoolOpType.Intersection || operation == BoolOpType.Union;
                case EdgeType.DifferentTransition:
                    return operation == BoolOpType.Difference;
                case EdgeType.NonContributing:
                    return false;
            }
            return false; // just to avoid the compiler warning
        }

        // Process a posible intersection between the edges associated to the left events le1 and le2
        private int PossibleIntersection(SweepEvent le1, SweepEvent le2)
        {
            //	if (e1->pol == e2->pol) // you can uncomment these two lines if self-intersecting polygons are not allowed
            //		return 0;

            Vector2D ip1, ip2;  // intersection points
            int nintersections = BoolOpUtils.FindIntersection(le1.Segment(), le2.Segment(), out ip1, out ip2);
            if (nintersections == 0)
                return 0;  // no intersection

            if ((nintersections == 1) && (BoolOpUtils.Approx(le1.point, le2.point) ||
                BoolOpUtils.Approx(le1.otherEvent.point, le2.otherEvent.point)))
                return 0; // the line segments intersect at an endpoint of both line segments

            if (nintersections == 2 && le1.pol == le2.pol)
            {
                // the line segments overlap, but they belong to the same polygon
                Console.WriteLine(le1.Segment() + " " + le2.Segment());
                throw new ArgumentException("Sorry, edges of the same polygon overlap");
            }
            
            // The line segments associated to le1 and le2 intersect
            if (nintersections == 1)
            {
                // if the intersection point is not an endpoint of le1.segment
                if (!BoolOpUtils.Approx(le1.point, ip1) &&
                    !BoolOpUtils.Approx(le1.otherEvent.point, ip1))
                    DivideSegment(le1, ip1);
                // if the intersection point is not an endpoint of le2.segment
                if (!BoolOpUtils.Approx(le2.point, ip1) &&
                    !BoolOpUtils.Approx(le2.otherEvent.point, ip1))
                    DivideSegment(le2, ip1);
                return 1;
            }
            // The line segments associated to le1 and le2 overlap
            tempSortedEvents.Clear();
            if (BoolOpUtils.Approx(le1.point, le2.point))
            {
                tempSortedEvents.Add(null);
            }
            else if (SweepEventComp.ComesAfter(le1, le2))
            {
                tempSortedEvents.Add(le2);
                tempSortedEvents.Add(le1);
            }
            else
            {
                tempSortedEvents.Add(le1);
                tempSortedEvents.Add(le2);
            }
            if (BoolOpUtils.Approx(le1.otherEvent.point, le2.otherEvent.point))
            {
                tempSortedEvents.Add(null);
            }
            else if (SweepEventComp.ComesAfter(le1.otherEvent, le2.otherEvent))
            {
                tempSortedEvents.Add(le2.otherEvent);
                tempSortedEvents.Add(le1.otherEvent);
            }
            else
            {
                tempSortedEvents.Add(le1.otherEvent);
                tempSortedEvents.Add(le2.otherEvent);
            }

            if ((tempSortedEvents.Count == 2) || (tempSortedEvents.Count == 3 && (tempSortedEvents[2] != null)))
            {
                // both line segments are equal or share the left endpoint
                le1.type = EdgeType.NonContributing;
                le2.type = (le1.inOut == le2.inOut) ? EdgeType.SameTransition : EdgeType.DifferentTransition;
                if (tempSortedEvents.Count == 3)
                    DivideSegment(tempSortedEvents[2].otherEvent, tempSortedEvents[1].point);
                return 2;
            }
            if (tempSortedEvents.Count == 3)
            { // the line segments share the right endpoint
                DivideSegment(tempSortedEvents[0], tempSortedEvents[1].point);
                return 3;
            }
            if (tempSortedEvents[0] != tempSortedEvents[3].otherEvent)
            { // no line segment includes totally the other one
                DivideSegment(tempSortedEvents[0], tempSortedEvents[1].point);
                DivideSegment(tempSortedEvents[1], tempSortedEvents[2].point);
                return 3;
            }
            // one line segment includes the other one
            DivideSegment(tempSortedEvents[0], tempSortedEvents[1].point);
            DivideSegment(tempSortedEvents[3].otherEvent, tempSortedEvents[2].point);
            return 3;
        }

        // Divide the segment associated to left event le, updating pq and (implicitly) the status line
        private void DivideSegment(SweepEvent le, Vector2D p)
        {
            //	Debug.Log("YES. INTERSECTION");
            // "Right event" of the "left line segment" resulting from dividing le.segment
            var r = new SweepEvent(curSeId++, false, p, le, le.pol/*, le.type*/);
            // "Left event" of the "right line segment" resulting from dividing le.segment
            var l = new SweepEvent(curSeId++, true, p, le.otherEvent, le.pol/*, le.other.type*/);
            // avoid a rounding error. The left event would be processed after the right event
            if (SweepEventComp.ComesAfter(l, le.otherEvent))
            {
                Debug.Log("Oops");
                le.otherEvent.left = true;
                l.left = false;
            }
            // avoid a rounding error. The left event would be processed after the right event
            if (SweepEventComp.ComesAfter(le, r))
            {
                Debug.Log("Oops2");
            }
            le.otherEvent.otherEvent = l;
            le.otherEvent = r;
            eq.Enqueue(l);
            eq.Enqueue(r);
        }

        // Connect the solution edges to build the result polygon
        private void ConnectEdges()
        {
            // copy the events in the result polygon to resultEvents array
            tempResultEvents.Clear();
            if (tempResultEvents.Capacity < sortedEvents.Count)
                tempResultEvents.Capacity = sortedEvents.Count;
            for (int i = 0; i < sortedEvents.Count; i++)
            {
                var curEvent = sortedEvents[i];
                if ((curEvent.left && curEvent.inResult) || (!curEvent.left && curEvent.otherEvent.inResult))
                    tempResultEvents.Add(curEvent);
            }
            // Due to overlapping edges the resultEvents array can be not wholly sorted
            bool sorted = false;
            while (!sorted)
            {
                sorted = true;
                for (int i = 0; i < tempResultEvents.Count - 1; i++)
                {
                    if (SweepEventComp.ComesAfter(tempResultEvents[i], tempResultEvents[i + 1]))
                    {
                        // swap
                        //resultEvents.Swap(i, i + 1);
                        var tempEvent = tempResultEvents[i];
                        tempResultEvents[i] = tempResultEvents[i + 1];
                        tempResultEvents[i + 1] = tempEvent;
                        sorted = false;
                    }
                }
            }

            for (int i = 0; i < tempResultEvents.Count; i++)
            {
                tempResultEvents[i].pos = i;
                if (!tempResultEvents[i].left)
                    BoolOpUtils.Swap(ref tempResultEvents[i].pos, ref tempResultEvents[i].otherEvent.pos);
            }

            tempProcessed.Clear();
            tempDepth.Clear();
            tempHoleOf.Clear();
            if (tempProcessed.Capacity < tempResultEvents.Count)
                tempProcessed.Capacity = tempResultEvents.Count;
            for (int i = 0; i < tempResultEvents.Count; i++)
                tempProcessed.Add(false);
            for (int i = 0; i < tempResultEvents.Count; i++)
            {
                if (tempProcessed[i])
                    continue;
                result.AddNewContour();
                var contour = result.Last();
                int contourId = result.Ncontours - 1;
                tempDepth.Add(0);
                tempHoleOf.Add(-1);
                if (tempResultEvents[i].prevInResult != null)
                {
                    int lowerContourId = tempResultEvents[i].prevInResult.contourId;
                    if (!tempResultEvents[i].prevInResult.resultInOut)
                    {
                        result.Contour(lowerContourId).AddHole(contourId);
                        tempHoleOf[contourId] = lowerContourId;
                        tempDepth[contourId] = tempDepth[lowerContourId] + 1;
                        contour.External = false;
                    }
                    else if (!result.Contour(lowerContourId).External)
                    {
                        result.Contour(tempHoleOf[lowerContourId]).AddHole(contourId);
                        tempHoleOf[contourId] = tempHoleOf[lowerContourId];
                        tempDepth[contourId] = tempDepth[lowerContourId];
                        contour.External = false;
                    }
                }
                int pos = i;
                var initial = tempResultEvents[i].point;
                contour.Add(initial);
                while (!BoolOpUtils.Approx(tempResultEvents[pos].otherEvent.point, initial))
                {
                    tempProcessed[pos] = true;
                    if (tempResultEvents[pos].left)
                    {
                        tempResultEvents[pos].resultInOut = false;
                        tempResultEvents[pos].contourId = contourId;
                    }
                    else
                    {
                        tempResultEvents[pos].otherEvent.resultInOut = true;
                        tempResultEvents[pos].otherEvent.contourId = contourId;
                    }
                    pos = tempResultEvents[pos].pos;
                    tempProcessed[pos] = true;
                    contour.Add(tempResultEvents[pos].point);
                    pos = NextPos(pos, tempResultEvents, tempProcessed);
                }
                tempProcessed[pos] = tempProcessed[tempResultEvents[pos].pos] = true;
                tempResultEvents[pos].otherEvent.resultInOut = true;
                tempResultEvents[pos].otherEvent.contourId = contourId;
                // If depth is odd, then it is a hole, so change orientation
                if ((tempDepth[contourId] & 1) == 1)
                    contour.ChangeOrientation();
            }
        }

        private int NextPos(int pos, List<SweepEvent> resultEvents, List<bool> processed)
        {
            int newPos = pos + 1;
            while (newPos < resultEvents.Count &&
                BoolOpUtils.Approx(resultEvents[newPos].point, resultEvents[pos].point))
            {
                if (!processed[newPos])
                    return newPos;
                else
                    newPos++;
            }
            newPos = pos - 1;
            while (processed[newPos])
                newPos--;
            return newPos;
        }

        internal static class BoolOpUtils
        {
            private static double EpsilonD = 1e-8, EpsilonD01 = EpsilonD / MathExt.MaxValue;

            internal static void SetEpsilon(double EpsilonD)
            {
                BoolOpUtils.EpsilonD = EpsilonD;
            }

            /* Signed area of the triangle (p0, p1, p2) */
            internal static double SignedArea(Vector2D p0, Vector2D p1, Vector2D p2)
            {
                return (p0.x - p2.x) * (p1.y - p2.y) - (p1.x - p2.x) * (p0.y - p2.y);
            }

            internal static bool Collinear(Vector2D p0, Vector2D p1, Vector2D p2)
            {
                return Approx(SignedArea(p0, p1, p2), 0);
            }

            internal static BboxD Bbox(Vector2D value)
            {
                return new BboxD(value.x, value.y, value.x, value.y);
            }

            internal static void Swap<T>(ref T value1, ref T value2)
            {
                var temp = value1;
                value1 = value2;
                value2 = temp;
            }

            internal static bool Approx(double value1, double value2)
            {
                return MathExt.Approximately(value1, value2, EpsilonD);
            }

            internal static bool Approx(Vector2D value1, Vector2D value2)
            {
                return MathExt.Approximately(value1, value2, EpsilonD);
            }

            internal static int FindIntersection(double u0, double u1, double v0, double v1, 
                out double w0, out double w1)
            {
                if ((u1 < v0) || (u0 > v1))
                {
                    w0 = w1 = double.NaN;
                    return 0;
                }
                if (u1 > v0)
                {
                    if (u0 < v1)
                    {
                        w0 = (u0 < v0) ? v0 : u0;
                        w1 = (u1 > v1) ? v1 : u1;
                        return 2;
                    }
                    else
                    {
                        // u0 == v1
                        w0 = u0;
                        w1 = double.NaN;
                        return 1;
                    }
                }
                else
                {
                    // u1 == v0
                    w0 = u1;
                    w1 = double.NaN;
                    return 1;
                }
            }
            
            internal static int FindIntersection(Segment seg0, Segment seg1, out Vector2D pi0, out Vector2D pi1)
            {
                pi0 = pi1 = new Vector2D(float.NaN, float.NaN);
                var p0 = seg0.source;
                var d0 = new Vector2D(seg0.target.x - p0.x, seg0.target.y - p0.y);
                var p1 = seg1.source;
                var d1 = new Vector2D(seg1.target.x - p1.x, seg1.target.y - p1.y);
                var E = new Vector2D(p1.x - p0.x, p1.y - p0.y);
                double kross = d0.x * d1.y - d0.y * d1.x;
                double sqrKross = kross * kross;
                double sqrLen0 = d0.x * d0.x + d0.y * d0.y;
                double sqrLen1 = d1.x * d1.x + d1.y * d1.y;

                //if (sqrKross > sqrEpsilon * sqrLen0 * sqrLen1)
                if (!Approx(kross, 0))
                {
                    // lines of the segments are not parallel
                    double s = (E.x * d1.y - E.y * d1.x) / kross;
                    if ((s < -EpsilonD01) || (s > 1 + EpsilonD01))
                    {
                        return 0;
                    }
                    double t = (E.x * d0.y - E.y * d0.x) / kross;
                    if ((t < -EpsilonD01) || (t > 1 + EpsilonD01))
                    {
                        return 0;
                    }
                    // intersection of lines is a point an each segment
                    pi0 = new Vector2D(p0.x + s * d0.x, p0.y + s * d0.y);
                    if (Approx(pi0, seg0.source)) pi0 = seg0.source;
                    if (Approx(pi0, seg0.target)) pi0 = seg0.target;
                    if (Approx(pi0, seg1.source)) pi0 = seg1.source;
                    if (Approx(pi0, seg1.target)) pi0 = seg1.target;
                    return 1;
                }

                // lines of the segments are parallel
                /*double sqrLenE = E.x * E.x + E.y * E.y;
                kross = E.x * d0.y - E.y * d0.x;
                sqrKross = kross * kross;
                if (sqrKross > sqrEpsilon * sqrLen0 * sqrLenE)*/
                if (!Collinear(seg0.source, seg0.target, seg1.source))
                {
                    // lines of the segment are different
                    return 0;
                }

                // Lines of the segments are the same. Need to test for overlap of segments.
                double s0 = (d0.x * E.x + d0.y * E.y) / sqrLen0;  // so = Dot (D0, E) * sqrLen0
                double s1 = s0 + (d0.x * d1.x + d0.y * d1.y) / sqrLen0;  // s1 = s0 + Dot (D0, D1) * sqrLen0
                double smin = Math.Min(s0, s1);
                double smax = Math.Max(s0, s1);
                double w0, w1;
                int imax = FindIntersection(0f, 1f, smin, smax, out w0, out w1);

                if (imax > 0)
                {
                    pi0 = new Vector2D(p0.x + w0 * d0.x, p0.y + w0 * d0.y);
                    if (Approx(pi0, seg0.source)) pi0 = seg0.source;
                    if (Approx(pi0, seg0.target)) pi0 = seg0.target;
                    if (Approx(pi0, seg1.source)) pi0 = seg1.source;
                    if (Approx(pi0, seg1.target)) pi0 = seg1.target;
                    if (imax > 1)
                        pi1 = new Vector2D(p0.x + w1 * d0.x, p0.y + w1 * d0.y);
                }
                return imax;
            }
        }
    }
}
