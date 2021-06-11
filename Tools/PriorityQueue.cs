using MathTools;
using System.Reflection;
using System.Runtime.CompilerServices;
using Debugging;

// sourced from: https://github.com/BlueRaja/High-Speed-Priority-Queue-for-C-Sharp
namespace System.Collections.Generic
{
    public struct DefaultPqNode
    {
        public int id;
        public float priority;
        public float secondPriority;

        public DefaultPqNode(int id, float priority, float secondPriority)
        {
            this.id = id;
            this.priority = priority;
            this.secondPriority = secondPriority;
        }
    }

    public class DefaultPqNodeComparer : Comparer<DefaultPqNode>
    {
        public override int Compare(DefaultPqNode higher, DefaultPqNode lower)
        {
            return (MathExt.Approximately(higher.priority, lower.priority) &&
                higher.secondPriority < lower.secondPriority) || higher.priority < lower.priority ? -1 : 1;
        }
    }

    /*public interface IPriorityQueueNode
    {
        int GetID();
    }*/

    /// <summary>
    /// An implementation of a min-Priority Queue using a heap.  Has O(1) .Contains()!
    /// See https://github.com/BlueRaja/High-Speed-Priority-Queue-for-C-Sharp/wiki/Getting-Started for more information
    /// </summary>
    public sealed class PriorityQueue<T> : IEnumerable<T>
    {
        private IComparer<T> comparer;
        private T[] nodes;

        /// <summary>
        /// Instantiate a new Priority Queue
        /// </summary>
        /// <param name="capacity">Initial capacity of the queue</param>
        public PriorityQueue(int capacity, IComparer<T> comparer = null)
        {
            if (!typeof(T).GetTypeInfo().IsValueType && comparer == null)
                throw new ArgumentException("comparer parameter is required when node lookup is enabled and T is not a value type.");
            this.comparer = comparer ?? Comparer<T>.Default;

            Debug.Assert(capacity > 0, "New queue size cannot be smaller than 1");

            Count = 0;
            nodes = new T[capacity + 1];
        }

        /// <summary>
        /// Returns the number of nodes in the queue.
        /// O(1)
        /// </summary>
        public int Count { get; private set; }

        /// <summary>
        /// Returns the capacity of this queue. Once you hit this number (ie. once Count == Capacity),
        /// the queue will double in size.  O(1)
        /// </summary>
        public int Capacity => nodes.Length - 1;

        /// <summary>
        /// Simply sets Count = 0, so we overwrite existing data
        /// O(1)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Reset()
        {
            Count = 0;
        }

        /// <summary>
        /// Removes every node from the queue.
        /// O(n) (So, don't do this often!)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            Array.Clear(nodes, 1, Count);
            Count = 0;
        }

        /// <summary>
        /// Enqueue a node to the priority queue. Lower values are placed in front.
        /// If the node is already enqueued, the result is undefined. 
        /// (There will just be a duplicate in the queue... so no undefined behaviour?)
        /// O(log n)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Enqueue(T node)
        {
            //Assert.IsFalse(Count >= nodes.Length - 1, "Queue is full - node cannot be added: " + node); //id);

            Count++;
            if (Count >= nodes.Length)
                Array.Resize(ref nodes, nodes.Length * 2);
            nodes[Count] = node;
            CascadeUp(Count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Swap(int index1, int index2)
        {
            var node1 = nodes[index1];
            var node2 = nodes[index2];

            //Swap the nodes
            nodes[index1] = node2;
            nodes[index2] = node1;
        }

        //Performance appears to be slightly better when this is NOT inlined
        private void CascadeUp(int index)
        {
            var node = nodes[index];

            //aka Heapify-up
            int parent = index / 2;
            while (parent >= 1)
            {
                var parentNode = nodes[parent];
                if (HasHigherPriority(parentNode, node))
                    break;

                //Node has lower priority value, so move it up the heap
                Swap(index,
                    parent); //For some reason, this is faster with Swap() rather than (less..?) individual operations, like in CascadeDown()
                index = parent;

                parent = parent / 2;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void CascadeDown(int index)
        {
            var node = nodes[index];
            //aka Heapify-down
            T newParent;
            int finalQueueIndex = index;
            while (true)
            {
                int parentIndex = finalQueueIndex;
                newParent = node;
                int childLeftIndex = 2 * finalQueueIndex;

                //Check if the left-child is higher-priority than the current node
                if (childLeftIndex > Count)
                    break;

                var childLeft = nodes[childLeftIndex];
                if (HasHigherPriority(childLeft, newParent))
                {
                    newParent = childLeft;
                    parentIndex = childLeftIndex;
                }

                //Check if the right-child is higher-priority than either the current node or the left child
                int childRightIndex = childLeftIndex + 1;
                if (childRightIndex <= Count)
                {
                    var childRight = nodes[childRightIndex];
                    if (HasHigherPriority(childRight, newParent))
                    {
                        newParent = childRight;
                        parentIndex = childRightIndex;
                    }
                }

                //If either of the children has higher (smaller) priority, swap and continue cascading
                //if (newParent.id != node.id)
                if (finalQueueIndex != parentIndex)
                {
                    //Move new parent to its new index.  node will be moved once, at the end
                    //Doing it this way is one less assignment operation than calling Swap()
                    nodes[finalQueueIndex] = newParent;
                    
                    finalQueueIndex = parentIndex;
                }
                else
                    break;
            }
            nodes[finalQueueIndex] = node;
        }

        /// <summary>
        /// Returns true if 'higher' has higher priority than 'lower', false otherwise.
        /// Note that calling HasHigherPriority(node, node) (ie. both arguments the same node) will return false
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool HasHigherPriority(T higher, T lower)
        {
            return comparer.Compare(higher, lower) == -1;
        }

        /// <summary>
        /// Removes the head of the queue (node with minimum priority), and returns it.
        /// If queue is empty, result is undefined
        /// O(log n)
        /// </summary>
        public T Dequeue()
        {
            Debug.Assert(Count > 0, "Cannot call Dequeue() on an empty queue");

            int firstIndex = 1;
            int lastIndex = Count;
            var minNode = nodes[firstIndex];
            if (lastIndex == firstIndex)
            {
                Count = 0;
                return minNode;
            }

            // Remove first node and put last node in its place
            var lastNode = nodes[lastIndex];
            nodes[firstIndex] = nodes[lastIndex];
            Count--;

            // Last node is out of place, find correct place by cascading down
            CascadeDown(firstIndex);

            return minNode;
        }

        /// <summary>
        /// Returns the head of the queue, without removing it (use Dequeue() for that).
        /// If the queue is empty, behavior is undefined.
        /// O(1)
        /// </summary>
        public T Peek()
        {
            Debug.Assert(Count > 0, "Cannot call .First on an empty queue");
            return nodes[1];
        }

        /*public void DecreasePriority(int id, float newPriority)
        {
            int index = idToQueueIndex[id].queueIndex;
            nodes[index].priority = newPriority;
            CascadeUp(index);
        }*/

        public IEnumerator<T> GetEnumerator()
        {
            for (var i = 1; i <= Count; i++)
                yield return nodes[i];
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }
}
