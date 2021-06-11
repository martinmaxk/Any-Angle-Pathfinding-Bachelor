using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;

// https://github.com/justcoding121/advanced-algorithms/blob/develop/src/Advanced.Algorithms/DataStructures/Tree/RedBlackTree.cs
namespace Advanced.Algorithms.DataStructures
{
    #region Aux
    internal abstract class BSTNodeBase<T>
    {
        // Count of nodes under this node including this node.
        // Used to speed up kth smallest computation.
        internal int Count { get; set; } = 1;

        internal virtual BSTNodeBase<T> Parent { get; set; }

        internal virtual BSTNodeBase<T> Left { get; set; }
        internal virtual BSTNodeBase<T> Right { get; set; }

        internal T Value { get; set; }

        internal bool IsLeftChild => Parent.Left == this;
        internal bool IsRightChild => Parent.Right == this;

        internal bool IsLeaf => Left == null && Right == null;
    }

    internal class BSTHelpers
    {
        internal static void ValidateSortedCollection<T>(IEnumerable<T> sortedCollection, IComparer<T> comparer)
        {
            if (!IsSorted(sortedCollection, comparer))
                throw new ArgumentException("Initial collection should have unique keys and be in sorted order.");
        }

        internal static BSTNodeBase<T> ToBST<T>(BSTNodeBase<T>[] sortedNodes)
        {
            return ToBST(sortedNodes, 0, sortedNodes.Length - 1);
        }

        internal static int AssignCount<T>(BSTNodeBase<T> node)
        {
            if (node == null)
                return 0;

            node.Count = AssignCount(node.Left) + AssignCount(node.Right) + 1;

            return node.Count;
        }

        private static BSTNodeBase<T> ToBST<T>(BSTNodeBase<T>[] sortedNodes, int start, int end)
        {
            if (start > end)
                return null;

            int mid = (start + end) / 2;
            var root = sortedNodes[mid];

            root.Left = ToBST(sortedNodes, start, mid - 1);
            if (root.Left != null)
                root.Left.Parent = root;

            root.Right = ToBST(sortedNodes, mid + 1, end);
            if (root.Right != null)
                root.Right.Parent = root;

            return root;
        }

        private static bool IsSorted<T>(IEnumerable<T> collection, IComparer<T> comparer)
        {
            var enumerator = collection.GetEnumerator();
            if (!enumerator.MoveNext())
                return true;

            var previous = enumerator.Current;

            while (enumerator.MoveNext())
            {
                var current = enumerator.Current;

                if (comparer.Compare(current, previous) <= 0)
                    return false;
            }

            return true;
        }
    }

    internal static class BSTExtensions
    {
        //find the node with the given identifier among descendants of parent and parent
        //uses pre-order traversal
        //O(log(n)) worst O(n) for unbalanced tree
        internal static int Find<T>(this BSTNodeBase<T> current, T value, 
            IComparer<T> comparer, IEqualityComparer<T> equalityComparer, out BSTNodeBase<T> node)
        {
            int position = 0;

            while (true)
            {
                if (current == null)
                {
                    node = null;
                    return -1;
                }

                if (equalityComparer.Equals(current.Value, value))
                {
                    position += (current.Left != null ? current.Left.Count : 0);
                    node = current;
                    return position;
                }

                int compareResult = comparer.Compare(current.Value, value);
                if (compareResult > 0)
                    current = current.Left;
                else
                {
                    position += (current.Left != null ? current.Left.Count : 0) + 1;
                    current = current.Right;
                }
            }
        }

        internal static int FindMax<T>(this BSTNodeBase<T> node, out BSTNodeBase<T> max)
        {
            max = node;
            if (node == null)
                return -1;

            int position = 0;
            while (true)
            {
                if (max.Right == null) return position;
                position += (max.Left != null ? max.Left.Count : 0) + 1;
                max = max.Right;
            }
        }

        internal static int FindMin<T>(this BSTNodeBase<T> node, out BSTNodeBase<T> min)
        {
            min = node;
            if (node == null)
                return -1;

            int position = 0;
            while (true)
            {
                if (min.Left == null) return position;
                position += min.Left.Count;
                min = min.Left;
            }
        }

        internal static BSTNodeBase<T> NextLower<T>(this BSTNodeBase<T> node)
        {
            //root or left child
            if (node.Parent == null || node.IsLeftChild)
            {
                if (node.Left != null)
                {
                    node = node.Left;
                    while (node.Right != null)
                        node = node.Right;

                    return node;
                }
                else
                {
                    while (node.Parent != null && node.IsLeftChild)
                        node = node.Parent;

                    return node?.Parent;
                }
            }
            //right child
            else
            {
                if (node.Left != null)
                {
                    node = node.Left;
                    while (node.Right != null)
                        node = node.Right;

                    return node;
                }
                else
                    return node.Parent;
            }

        }

        internal static BSTNodeBase<T> NextHigher<T>(this BSTNodeBase<T> node)
        {
            //root or left child
            if (node.Parent == null || node.IsLeftChild)
            {
                if (node.Right != null)
                {
                    node = node.Right;
                    while (node.Left != null)
                        node = node.Left;

                    return node;
                }
                else
                    return node?.Parent;
            }
            //right child
            else
            {
                if (node.Right != null)
                {
                    node = node.Right;
                    while (node.Left != null)
                        node = node.Left;

                    return node;
                }
                else
                {
                    while (node.Parent != null && node.IsRightChild)
                        node = node.Parent;

                    return node?.Parent;
                }
            }
        }

        internal static void UpdateCounts<T>(this BSTNodeBase<T> node, bool spiralUp = false)
        {
            while (node != null)
            {
                int leftCount = node.Left?.Count ?? 0;
                var rightCount = node.Right?.Count ?? 0;

                node.Count = leftCount + rightCount + 1;

                node = node.Parent;

                if (!spiralUp)
                    break;
            }
        }

        //get the kth smallest element under given node
        internal static BSTNodeBase<T> KthSmallest<T>(this BSTNodeBase<T> node, int k)
        {
            var leftCount = node.Left != null ? node.Left.Count : 0;

            if (k == leftCount)
                return node;

            if (k < leftCount)
                return KthSmallest(node.Left, k);

            return KthSmallest(node.Right, k - leftCount - 1);
        }

        //get the sorted order position of given item under given node
        internal static int Position<T>(this BSTNodeBase<T> node, BSTPositionQuery<T> query)
        {
            if (node == null)
                return -1;

            var leftCount = node.Left != null ? node.Left.Count : 0;

            if (query.equalityComparer.Equals(node.Value, query.item))
                return leftCount;

            if (query.comparer.Compare(query.item, node.Value) < 0)
                return Position(node.Left, query);

            var position = Position(node.Right, query);

            return position < 0 ? position : position + leftCount + 1;
        }
    }

    // Use to avoid repetition on stack during recursion
    internal class BSTPositionQuery<T>
    {
        public T item;
        public IComparer<T> comparer;
        public IEqualityComparer<T> equalityComparer;

        public BSTPositionQuery(IComparer<T> comparer, IEqualityComparer<T> equalityComparer)
        {
            this.comparer = comparer;
            this.equalityComparer = equalityComparer;
        }
    }

    public struct BSTIterator<T>
    {
        internal BSTNodeBase<T> root;
        public T Value { get { return root.Value; } }
        public int Index { get; private set; }

        internal BSTIterator(BSTNodeBase<T> root, int rootIndex)
        {
            this.root = root;
            Index = rootIndex;
        }

        public BSTIterator<T> NextLower()
        {
            return Next(false);
        }

        public BSTIterator<T> NextHigher()
        {
            return Next(true);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private BSTIterator<T> Next(bool asc)
        {
            if (root == null)
                return new BSTIterator<T>(null, -1);

            return asc ? new BSTIterator<T>(root.NextHigher(), Index + 1) : new BSTIterator<T>(root.NextLower(), Index - 1);
        }

        public bool IsNull()
        {
            return root == null;
        }

        public void Dispose()
        {
            root = null;
            Index = -1;
        }
    }

    public struct BSTEnumerator<T> : IEnumerator<T>
    {
        private readonly bool asc;
        private readonly BSTNodeBase<T> root;
        private BSTIterator<T> iterator;

        public T Current { get { return iterator.Value; } }
        public int CurrentIndex { get { return iterator.Index; } }
        object IEnumerator.Current => Current;

        internal BSTEnumerator(BSTNodeBase<T> root, bool asc = true)
        {
            this.root = root;
            this.asc = asc;
            iterator = new BSTIterator<T>(null, -1);
        }

        public bool MoveNext()
        {
            if (iterator.IsNull())
            {
                BSTNodeBase<T> minMax;
                int position = asc ? root.FindMin(out minMax) : root.FindMax(out minMax);
                iterator = new BSTIterator<T>(minMax, position);
                return true;
            }

            iterator = asc ? iterator.NextHigher() : iterator.NextLower();
            return !iterator.IsNull();
        }

        public void Reset()
        {
            iterator = new BSTIterator<T>(null, -1);
        }

        public void Dispose()
        {
            iterator.Dispose();
        }
    }

    internal static class IEnumerableExtensions
    {
        internal static IEnumerable<T> AsEnumerable<T>(this IEnumerator<T> e)
        {
            while (e.MoveNext())
                yield return e.Current;
        }
    }
    #endregion

    /// <summary>
    /// A red black tree implementation.
    /// </summary>
    public class RedBlackTree<T> : IEnumerable<T>
    {
        // Pool nodes to avoid generating garbage on deletion
        private readonly Stack<RedBlackTreeNode<T>> nodePool = new Stack<RedBlackTreeNode<T>>();
        private readonly BSTPositionQuery<T> posQuery;
        internal RedBlackTreeNode<T> Root { get; set; }

        //if enabled, lookup will fasten deletion/insertion/exists operations. 
        internal readonly Dictionary<T, BSTNodeBase<T>> NodeLookUp;

        public int Count => Root == null ? 0 : Root.Count;

        public RedBlackTree(IComparer<T> comparer = null, IEqualityComparer<T> equalityComparer = null)
        {
            if (!typeof(T).GetTypeInfo().IsValueType && comparer == null)
                throw new ArgumentException("comparer parameter is required when T is not a value type.");
            posQuery = new BSTPositionQuery<T>(comparer ?? Comparer<T>.Default,
                equalityComparer ?? EqualityComparer<T>.Default);
        }

        /// <param name="enableNodeLookUp">Enabling lookup will fasten deletion/insertion/exists operations
        /// at the cost of additional space.</param>
        /// <param name="equalityComparer">Provide equality comparer for node lookup if enabled (required when T is not a value type).</param>
        public RedBlackTree(bool enableNodeLookUp, IComparer <T> comparer = null, IEqualityComparer<T> equalityComparer = null) : this(comparer, equalityComparer)
        {
            if (enableNodeLookUp)
                NodeLookUp = new Dictionary<T, BSTNodeBase<T>>(equalityComparer);
        }

        /// <summary>
        /// Initialize the BST with given sorted keys optionally.
        /// Time complexity: O(n).
        /// </summary>
        /// <param name="sortedCollection">The sorted initial collection.</param>
        /// <param name="enableNodeLookUp">Enabling lookup will fasten deletion/insertion/exists operations
        /// at the cost of additional space.</param>
        /// <param name="equalityComparer">Provide equality comparer for node lookup if enabled (required when T is not a value type).</param>
        public RedBlackTree(IEnumerable<T> sortedCollection, IComparer<T> comparer = null,
            IEqualityComparer<T> equalityComparer = null, bool enableNodeLookUp = false) : this(comparer, equalityComparer)
        {
            BSTHelpers.ValidateSortedCollection(sortedCollection, posQuery.comparer);
            var nodes = sortedCollection.Select(x => CreateNode(null, x)).ToArray();
            Root = (RedBlackTreeNode<T>)BSTHelpers.ToBST(nodes);
            AssignColors(Root);
            BSTHelpers.AssignCount(Root);

            if (enableNodeLookUp)
            {
                if (!typeof(T).GetTypeInfo().IsValueType && equalityComparer == null)
                {
                    throw new ArgumentException("equalityComparer parameter is required when node lookup is enabled and T is not a value type.");
                }

                NodeLookUp = nodes.ToDictionary(x => x.Value, x => x as BSTNodeBase<T>, equalityComparer ?? EqualityComparer<T>.Default);
            }
        }

        public BSTIterator<T> NullIter() 
        { 
            return new BSTIterator<T>(null, -1); 
        }

        /// <summary>
        ///  Time complexity: O(log(n))
        /// </summary>
        public bool HasItem(T value)
        {
            if (Root == null)
                return false;

            if (NodeLookUp != null)
                return NodeLookUp.ContainsKey(value);

            return FindInternal(value).node != null;
        }

        private RedBlackTreeNode<T> CreateNode(RedBlackTreeNode<T> parent, T value, RedBlackTreeNodeColor color = RedBlackTreeNodeColor.Red)
        {
            RedBlackTreeNode<T> node;
            if (nodePool.Count == 0)
                node = new RedBlackTreeNode<T>();
            else
            {
                node = nodePool.Pop();
                node.Count = 1;
                node.Parent = node.Left = node.Right = null;
            }
            node.Reset(parent, value, color);
            return node;
        }

        /// <summary>
        ///  Time complexity: O(n)
        /// </summary>
        public void Clear()
        {
            UnlinkNodeRec(Root);
            Root = null;
        }

        // Iterate tree rooted at node and unlink all nodes
        private void UnlinkNodeRec(RedBlackTreeNode<T> node)
        {
            if (node == null)
                return;
            UnlinkNodeRec(node.Left);
            UnlinkNodeRec(node.Right);
            UnlinkNode(node);
        }

        // Unlink node from tree
        private void UnlinkNode(RedBlackTreeNode<T> node)
        {
            node.Value = default(T);
            /*node.Count = 1;
            node.Parent = node.Left = node.Right = null;*/
            nodePool.Push(node);
        }

        /// <summary>
        ///  Time complexity: O(log(n))
        /// </summary>
        public BSTIterator<T> FindMax()
        {
            BSTNodeBase<T> max;
            int index = Root.FindMax(out max);
            return new BSTIterator<T>(max, index);
        }

        /// <summary>
        ///  Time complexity: O(log(n))
        /// </summary>
        public BSTIterator<T> FindMin()
        {
            BSTNodeBase<T> min;
            int index = Root.FindMin(out min);
            return new BSTIterator<T>(min, index);
        }

        /// <summary>
        ///  Time complexity: O(log(n))
        /// </summary>
        public int IndexOf(T item)
        {
            posQuery.item = item;
            return Root.Position(posQuery);
        }

        /// <summary>
        ///  Time complexity: O(log(n))
        /// </summary>
        public T ElementAt(int index)
        {
            if (index < 0 || index >= Count)
                throw new IndexOutOfRangeException();

            return Root.KthSmallest(index).Value;
        }

        internal RedBlackTreeNode<T> FindNode(T value)
        {
            return Root == null ? null : FindInternal(value).node;
        }

        internal bool Exists(T value)
        {
            return FindNode(value) != null;
        }

        public BSTIterator<T> Find(T value)
        {
            return FindInternal(value).ToIterator();
        }

        //find the node with the given identifier among descendants of parent and parent
        //uses pre-order traversal
        internal RedBlackTreeNodeIndex<T> FindInternal(T value)
        {
            if (NodeLookUp != null)
            {
                if (NodeLookUp.ContainsKey(value))
                {
                    var node = (NodeLookUp[value] as RedBlackTreeNode<T>);
                    return new RedBlackTreeNodeIndex<T>(node, IndexOf(value));
                }

                return new RedBlackTreeNodeIndex<T>(null, -1);
            }

            BSTNodeBase<T> baseNode;
            int index = Root.Find(value, posQuery.comparer, posQuery.equalityComparer, out baseNode);
            return new RedBlackTreeNodeIndex<T>(baseNode as RedBlackTreeNode<T>, index);
        }

        /// <summary>
        ///  Time complexity: O(log(n)).
        ///  Returns the node and the position (index) of the value in sorted order of this BST.
        /// </summary>
        public BSTIterator<T> Insert(T value)
        {
            return InsertInternal(value).ToIterator();
        }

        /// <summary>
        ///  Time complexity: O(log(n))
        /// </summary>
        internal RedBlackTreeNodeIndex<T> InsertInternal(T value)
        {
            //empty tree
            if (Root == null)
            {
                Root = CreateNode(null, value, RedBlackTreeNodeColor.Black);
                if (NodeLookUp != null)
                    NodeLookUp[value] = Root;

                return new RedBlackTreeNodeIndex<T>(Root, 0);
            }

            var newNode = InsertFrom(Root, value);

            if (NodeLookUp != null)
                NodeLookUp[value] = newNode.node;

            return newNode;
        }

        //O(log(n)) always
        private RedBlackTreeNodeIndex<T> InsertFrom(RedBlackTreeNode<T> root, T newNodeValue)
        {
            RedBlackTreeNodeIndex<T> parentAndIndex;
            bool isLeftChild = IsLeftChildIfInserted(root, newNodeValue, out parentAndIndex);
            var node = CreateNode(parentAndIndex.node, newNodeValue);
            if (isLeftChild)
                parentAndIndex.node.Left = node;
            else
                parentAndIndex.node.Right = node;
            BalanceInsertion(node);
            parentAndIndex.node = node;
            return parentAndIndex;
        }

        public void FindPredSucc(T queryValue, out BSTIterator<T> pred, out BSTIterator<T> succ)
        {
            if (IsLeftChildIfInserted(Root, queryValue, out succ))
                pred = succ.NextLower();
            else
            {
                pred = succ;
                succ = pred.NextHigher();
            }
        }

        // Find predecessor/greatest value less than queryValue
        public BSTIterator<T> FindPred(T queryValue)
        {
            BSTIterator<T> parentIter;
            if (IsLeftChildIfInserted(Root, queryValue, out parentIter))
                return parentIter.NextLower();
            return parentIter;
        }

        // Find successor/least value greater than queryValue
        public BSTIterator<T> FindSucc(T queryValue)
        {
            BSTIterator<T> parentIter;
            if (!IsLeftChildIfInserted(Root, queryValue, out parentIter))
                return parentIter.NextHigher();
            return parentIter;
        }

        private bool IsLeftChildIfInserted(RedBlackTreeNode<T> currentNode, T value,
            out BSTIterator<T> parentIter, bool allowDuplicate = true)
        {
            RedBlackTreeNodeIndex<T> parentAndIndex;
            bool isLeftChild = IsLeftChildIfInserted(currentNode, value, out parentAndIndex, allowDuplicate);
            parentIter = parentAndIndex.ToIterator();
            return isLeftChild;
        }

        // O(log(n)) always
        // parentAndIndex is parent node and the position of the query value in the tree
        private bool IsLeftChildIfInserted(RedBlackTreeNode<T> currentNode, T value,
            out RedBlackTreeNodeIndex<T> parentAndIndex, bool allowDuplicate = false)
        {
            int insertionPosition = 0;

            while (true)
            {
                int compareResult = posQuery.comparer.Compare(currentNode.Value, value);

                //current node is less than new item
                if (compareResult < 0)
                {
                    insertionPosition += (currentNode.Left != null ? currentNode.Left.Count : 0) + 1;

                    //no right child
                    if (currentNode.Right == null)
                    {
                        parentAndIndex = new RedBlackTreeNodeIndex<T>(currentNode, insertionPosition);
                        return false;
                    }

                    currentNode = currentNode.Right;
                }
                //current node is greater than new node
                else if (compareResult > 0)
                {
                    if (currentNode.Left == null)
                    {
                        parentAndIndex = new RedBlackTreeNodeIndex<T>(currentNode, insertionPosition);
                        return true;
                    }

                    currentNode = currentNode.Left;
                }
                else if (allowDuplicate)
                {
                    parentAndIndex = new RedBlackTreeNodeIndex<T>(currentNode, insertionPosition);
                    return true;
                }
                else
                {
                    //duplicate
                    throw new Exception("Item with same key exists " + currentNode.Value);
                }
            }
        }

        private void BalanceInsertion(RedBlackTreeNode<T> nodeToBalance)
        {

            while (true)
            {
                if (nodeToBalance == Root)
                {
                    nodeToBalance.NodeColor = RedBlackTreeNodeColor.Black;
                    break;
                }

                //if node to balance is red
                if (nodeToBalance.NodeColor == RedBlackTreeNodeColor.Red)
                {
                    //red-red relation; fix it!
                    if (nodeToBalance.Parent.NodeColor == RedBlackTreeNodeColor.Red)
                    {
                        //red sibling
                        if (nodeToBalance.Parent.Sibling != null && nodeToBalance.Parent.Sibling.NodeColor == RedBlackTreeNodeColor.Red)
                        {
                            //mark both children of parent as black and move up balancing 
                            nodeToBalance.Parent.Sibling.NodeColor = RedBlackTreeNodeColor.Black;
                            nodeToBalance.Parent.NodeColor = RedBlackTreeNodeColor.Black;

                            //root is always black
                            if (nodeToBalance.Parent.Parent != Root)
                                nodeToBalance.Parent.Parent.NodeColor = RedBlackTreeNodeColor.Red;

                            nodeToBalance.UpdateCounts();
                            nodeToBalance.Parent.UpdateCounts();
                            nodeToBalance = nodeToBalance.Parent.Parent;
                        }
                        //absent sibling or black sibling
                        else if (nodeToBalance.Parent.Sibling == null || nodeToBalance.Parent.Sibling.NodeColor == RedBlackTreeNodeColor.Black)
                        {
                            if (nodeToBalance.IsLeftChild && nodeToBalance.Parent.IsLeftChild)
                            {
                                var newRoot = nodeToBalance.Parent;
                                SwapColors(nodeToBalance.Parent, nodeToBalance.Parent.Parent);
                                RightRotate(nodeToBalance.Parent.Parent);

                                if (newRoot == Root)
                                    Root.NodeColor = RedBlackTreeNodeColor.Black;

                                nodeToBalance.UpdateCounts();
                                nodeToBalance = newRoot;
                            }
                            else if (nodeToBalance.IsLeftChild && nodeToBalance.Parent.IsRightChild)
                            {
                                RightRotate(nodeToBalance.Parent);

                                var newRoot = nodeToBalance;

                                SwapColors(nodeToBalance.Parent, nodeToBalance);
                                LeftRotate(nodeToBalance.Parent);

                                if (newRoot == Root)
                                    Root.NodeColor = RedBlackTreeNodeColor.Black;

                                nodeToBalance.UpdateCounts();
                                nodeToBalance = newRoot;
                            }
                            else if (nodeToBalance.IsRightChild && nodeToBalance.Parent.IsRightChild)
                            {
                                var newRoot = nodeToBalance.Parent;
                                SwapColors(nodeToBalance.Parent, nodeToBalance.Parent.Parent);
                                LeftRotate(nodeToBalance.Parent.Parent);

                                if (newRoot == Root)
                                    Root.NodeColor = RedBlackTreeNodeColor.Black;

                                nodeToBalance.UpdateCounts();
                                nodeToBalance = newRoot;
                            }
                            else if (nodeToBalance.IsRightChild && nodeToBalance.Parent.IsLeftChild)
                            {
                                LeftRotate(nodeToBalance.Parent);

                                var newRoot = nodeToBalance;

                                SwapColors(nodeToBalance.Parent, nodeToBalance);
                                RightRotate(nodeToBalance.Parent);

                                if (newRoot == Root)
                                    Root.NodeColor = RedBlackTreeNodeColor.Black;

                                nodeToBalance.UpdateCounts();
                                nodeToBalance = newRoot;
                            }
                        }
                    }
                }

                if (nodeToBalance.Parent != null)
                {
                    nodeToBalance.UpdateCounts();
                    nodeToBalance = nodeToBalance.Parent;
                    continue;
                }

                break;
            }

            nodeToBalance.UpdateCounts(true);
        }

        private void SwapColors(RedBlackTreeNode<T> node1, RedBlackTreeNode<T> node2)
        {
            var tmpColor = node2.NodeColor;
            node2.NodeColor = node1.NodeColor;
            node1.NodeColor = tmpColor;
        }

        /// <summary>
        ///  Delete if value exists. 
        ///  Time complexity: O(log(n))
        ///  Returns the position (index) of the item if deleted; otherwise returns -1
        /// </summary>
        public int Delete(T value)
        {
            if (Root == null)
                return -1;

            var node = FindInternal(value);

            if (node.node == null)
                return -1;

            var position = node.index;

            Delete(node.node);

            if (NodeLookUp != null)
                NodeLookUp.Remove(value);

            return position;
        }

        public void Delete(BSTIterator<T> iterator)
        {
            if (iterator.IsNull())
                throw new ArgumentException("Iterator is invalid");
            Delete(iterator.root as RedBlackTreeNode<T>);
        }

        /// <summary>
        ///  Time complexity: O(log(n))
        /// </summary>
        public T RemoveAt(int index)
        {
            if (index < 0 || index >= Count)
                throw new IndexOutOfRangeException();

            var node = Root.KthSmallest(index) as RedBlackTreeNode<T>;

            var deletedValue = node.Value;

            Delete(node);

            if (NodeLookUp != null)
                NodeLookUp.Remove(deletedValue);

            return node.Value;
        }
        
        //O(log(n)) always
        private void Delete(RedBlackTreeNode<T> node)
        {
            //node is a leaf node
            if (node.IsLeaf)
            {
                //if color is red, we are good; no need to balance
                if (node.NodeColor == RedBlackTreeNodeColor.Red)
                {
                    DeleteLeaf(node);
                    node.Parent?.UpdateCounts(true);
                    return;
                }

                DeleteLeaf(node);
                BalanceNode(node.Parent);
            }
            else
            {
                // case one - right tree is null (move sub tree up)
                if (node.Left != null && node.Right == null)
                {
                    DeleteLeftNode(node);
                    BalanceNode(node.Left);
                }
                // case two - left tree is null  (move sub tree up)
                else if (node.Right != null && node.Left == null)
                {
                    DeleteRightNode(node);
                    BalanceNode(node.Right);
                }
                // case three - two child trees 
                // Relink node's successor in place of node and delete node 
                // (do this instead of swapping values to maintain references and not invalidate iterators)
                // https://gcc.gnu.org/git/?p=gcc.git;a=blob;f=libstdc%2B%2B-v3/src/c%2B%2B98/tree.cc
                else
                {
                    // We know node has a right child
                    var succ = node.Right;
                    while (succ.Left != null)
                        succ = succ.Left;

                    if (node == Root)
                        Root = succ;
                    else if (node.IsLeftChild)
                        node.Parent.Left = succ;
                    else
                        node.Parent.Right = succ;

                    // We know succ.Left = null
                    node.Left.Parent = succ;
                    succ.Left = node.Left;
                    node.Left = null;
                    if (succ.Right != null)
                        succ.Right.Parent = node;
                    var temp = node.Right;
                    node.Right = succ.Right;
                    if (succ == temp)
                    {
                        // After swap, node will be right child of succ
                        succ.Right = node;
                        temp = succ;
                    }
                    else
                    {
                        // We now know succ was a left child
                        succ.Parent.Left = node;
                        succ.Right = temp;
                        temp.Parent = succ;
                        temp = succ.Parent;
                    }
                    succ.Parent = node.Parent;
                    node.Parent = temp;
                    SwapColors(node, succ);
                    
                    // Delete node (now where successor was)
                    Delete(node);
                }
            }
        }

        private void BalanceNode(RedBlackTreeNode<T> nodeToBalance)
        {
            //handle six cases
            while (nodeToBalance != null)
            {
                nodeToBalance.UpdateCounts();
                nodeToBalance = HandleDoubleBlack(nodeToBalance);
            }
        }

        private void DeleteLeaf(RedBlackTreeNode<T> node)
        {
            //if node is root
            if (node.Parent == null)
                Root = null;
            //assign nodes parent.left/right to null
            else if (node.IsLeftChild)
                node.Parent.Left = null;
            else
                node.Parent.Right = null;
            UnlinkNode(node);
        }

        private void DeleteRightNode(RedBlackTreeNode<T> node)
        {
            //root
            if (node.Parent == null)
            {
                Root.Right.Parent = null;
                Root = Root.Right;
                Root.NodeColor = RedBlackTreeNodeColor.Black;
                UnlinkNode(node);
                return;
            }

            //node is left child of parent
            if (node.IsLeftChild)
                node.Parent.Left = node.Right;
            //node is right child of parent
            else
                node.Parent.Right = node.Right;

            node.Right.Parent = node.Parent;

            if (node.Right.NodeColor != RedBlackTreeNodeColor.Red)
            {
                UnlinkNode(node);
                return;
            }

            //black deletion! But we can take its red child and recolor it to black
            //and we are done!
            node.Right.NodeColor = RedBlackTreeNodeColor.Black;
            UnlinkNode(node);
        }

        private void DeleteLeftNode(RedBlackTreeNode<T> node)
        {
            //root
            if (node.Parent == null)
            {
                Root.Left.Parent = null;
                Root = Root.Left;
                Root.NodeColor = RedBlackTreeNodeColor.Black;
                UnlinkNode(node);
                return;
            }

            //node is left child of parent
            if (node.IsLeftChild)
                node.Parent.Left = node.Left;
            //node is right child of parent
            else
                node.Parent.Right = node.Left;

            node.Left.Parent = node.Parent;

            if (node.Left.NodeColor != RedBlackTreeNodeColor.Red)
            {
                UnlinkNode(node);
                return;
            }

            //black deletion! But we can take its red child and recolor it to black
            //and we are done!
            node.Left.NodeColor = RedBlackTreeNodeColor.Black;
            UnlinkNode(node);
        }

        private void RightRotate(RedBlackTreeNode<T> node)
        {
            var prevRoot = node;
            var leftRightChild = prevRoot.Left.Right;

            var newRoot = node.Left;

            //make left child as root
            prevRoot.Left.Parent = prevRoot.Parent;

            if (prevRoot.Parent != null)
            {
                if (prevRoot.Parent.Left == prevRoot)
                    prevRoot.Parent.Left = prevRoot.Left;
                else
                    prevRoot.Parent.Right = prevRoot.Left;
            }

            //move prev root as right child of current root
            newRoot.Right = prevRoot;
            prevRoot.Parent = newRoot;

            //move right child of left child of prev root to left child of right child of new root
            newRoot.Right.Left = leftRightChild;
            if (newRoot.Right.Left != null)
                newRoot.Right.Left.Parent = newRoot.Right;

            if (prevRoot == Root)
                Root = newRoot;

            newRoot.Left.UpdateCounts();
            newRoot.Right.UpdateCounts();
            newRoot.UpdateCounts();
        }

        private void LeftRotate(RedBlackTreeNode<T> node)
        {
            var prevRoot = node;
            var rightLeftChild = prevRoot.Right.Left;

            var newRoot = node.Right;

            //make right child as root
            prevRoot.Right.Parent = prevRoot.Parent;

            if (prevRoot.Parent != null)
            {
                if (prevRoot.Parent.Left == prevRoot)
                    prevRoot.Parent.Left = prevRoot.Right;
                else
                    prevRoot.Parent.Right = prevRoot.Right;
            }

            //move prev root as left child of current root
            newRoot.Left = prevRoot;
            prevRoot.Parent = newRoot;

            //move left child of right child of prev root to right child of left child of new root
            newRoot.Left.Right = rightLeftChild;
            if (newRoot.Left.Right != null)
                newRoot.Left.Right.Parent = newRoot.Left;

            if (prevRoot == Root)
                Root = newRoot;

            newRoot.Left.UpdateCounts();
            newRoot.Right.UpdateCounts();
            newRoot.UpdateCounts();
        }

        private RedBlackTreeNode<T> HandleDoubleBlack(RedBlackTreeNode<T> node)
        {
            //case 1
            if (node == Root)
            {
                node.NodeColor = RedBlackTreeNodeColor.Black;
                return null;
            }

            //case 2
            if (node.Parent != null
                 && node.Parent.NodeColor == RedBlackTreeNodeColor.Black
                 && node.Sibling != null
                 && node.Sibling.NodeColor == RedBlackTreeNodeColor.Red
                 && ((node.Sibling.Left == null && node.Sibling.Right == null)
                 || (node.Sibling.Left != null && node.Sibling.Right != null
                   && node.Sibling.Left.NodeColor == RedBlackTreeNodeColor.Black
                   && node.Sibling.Right.NodeColor == RedBlackTreeNodeColor.Black)))
            {
                node.Parent.NodeColor = RedBlackTreeNodeColor.Red;
                node.Sibling.NodeColor = RedBlackTreeNodeColor.Black;

                if (node.Sibling.IsRightChild)
                    LeftRotate(node.Parent);
                else
                    RightRotate(node.Parent);

                return node;
            }
            //case 3
            if (node.Parent != null
             && node.Parent.NodeColor == RedBlackTreeNodeColor.Black
             && node.Sibling != null
             && node.Sibling.NodeColor == RedBlackTreeNodeColor.Black
             && (node.Sibling.Left == null && node.Sibling.Right == null
             || node.Sibling.Left != null && node.Sibling.Right != null
                                          && node.Sibling.Left.NodeColor == RedBlackTreeNodeColor.Black
                                          && node.Sibling.Right.NodeColor == RedBlackTreeNodeColor.Black))
            {
                //pushed up the double black problem up to parent
                //so now it needs to be fixed
                node.Sibling.NodeColor = RedBlackTreeNodeColor.Red;

                return node.Parent;
            }


            //case 4
            if (node.Parent != null
                 && node.Parent.NodeColor == RedBlackTreeNodeColor.Red
                 && node.Sibling != null
                 && node.Sibling.NodeColor == RedBlackTreeNodeColor.Black
                 && (node.Sibling.Left == null && node.Sibling.Right == null
                 || node.Sibling.Left != null && node.Sibling.Right != null
                                              && node.Sibling.Left.NodeColor == RedBlackTreeNodeColor.Black
                                              && node.Sibling.Right.NodeColor == RedBlackTreeNodeColor.Black))
            {
                //just swap the color of parent and sibling
                //which will compensate the loss of black count 
                node.Parent.NodeColor = RedBlackTreeNodeColor.Black;
                node.Sibling.NodeColor = RedBlackTreeNodeColor.Red;
                node.UpdateCounts(true);
                return null;
            }


            //case 5
            if (node.Parent != null
                && node.Parent.NodeColor == RedBlackTreeNodeColor.Black
                && node.Sibling != null
                && node.Sibling.IsRightChild
                && node.Sibling.NodeColor == RedBlackTreeNodeColor.Black
                && node.Sibling.Left != null
                && node.Sibling.Left.NodeColor == RedBlackTreeNodeColor.Red
                && node.Sibling.Right != null
                && node.Sibling.Right.NodeColor == RedBlackTreeNodeColor.Black)
            {
                node.Sibling.NodeColor = RedBlackTreeNodeColor.Red;
                node.Sibling.Left.NodeColor = RedBlackTreeNodeColor.Black;
                RightRotate(node.Sibling);

                return node;
            }

            //case 5 mirror
            if (node.Parent != null
               && node.Parent.NodeColor == RedBlackTreeNodeColor.Black
               && node.Sibling != null
               && node.Sibling.IsLeftChild
               && node.Sibling.NodeColor == RedBlackTreeNodeColor.Black
               && node.Sibling.Left != null
               && node.Sibling.Left.NodeColor == RedBlackTreeNodeColor.Black
               && node.Sibling.Right != null
               && node.Sibling.Right.NodeColor == RedBlackTreeNodeColor.Red)
            {
                node.Sibling.NodeColor = RedBlackTreeNodeColor.Red;
                node.Sibling.Right.NodeColor = RedBlackTreeNodeColor.Black;
                LeftRotate(node.Sibling);

                return node;
            }

            //case 6
            if (node.Parent != null
                && node.Parent.NodeColor == RedBlackTreeNodeColor.Black
                && node.Sibling != null
                && node.Sibling.IsRightChild
                && node.Sibling.NodeColor == RedBlackTreeNodeColor.Black
                && node.Sibling.Right != null
                && node.Sibling.Right.NodeColor == RedBlackTreeNodeColor.Red)
            {
                //left rotate to increase the black count on left side by one
                //and mark the red right child of sibling to black 
                //to compensate the loss of Black on right side of parent
                node.Sibling.Right.NodeColor = RedBlackTreeNodeColor.Black;
                LeftRotate(node.Parent);
                node.UpdateCounts(true);
                return null;
            }

            //case 6 mirror
            if (node.Parent != null
              && node.Parent.NodeColor == RedBlackTreeNodeColor.Black
              && node.Sibling != null
              && node.Sibling.IsLeftChild
              && node.Sibling.NodeColor == RedBlackTreeNodeColor.Black
              && node.Sibling.Left != null
              && node.Sibling.Left.NodeColor == RedBlackTreeNodeColor.Red)
            {
                //right rotate to increase the black count on right side by one
                //and mark the red left child of sibling to black
                //to compensate the loss of Black on right side of parent
                node.Sibling.Left.NodeColor = RedBlackTreeNodeColor.Black;
                RightRotate(node.Parent);
                node.UpdateCounts(true);
                return null;
            }

            node.UpdateCounts(true);
            return null;
        }

        //assign valid colors assuming the given tree node and its children are in balanced state.
        private void AssignColors(RedBlackTreeNode<T> current)
        {
            if (current == null)
                return;

            AssignColors(current.Left);
            AssignColors(current.Right);

            if (current.IsLeaf)
                current.NodeColor = RedBlackTreeNodeColor.Red;
            else
                current.NodeColor = RedBlackTreeNodeColor.Black;
        }

        /// <summary>
        ///     Get the next lower value to given value in this BST.
        /// </summary>
        public T NextLower(T value)
        {
            var node = FindNode(value);
            if (node == null)
                return default(T);

            var next = (node as BSTNodeBase<T>).NextLower();
            return next != null ? next.Value : default(T);
        }

        /// <summary>
        ///     Get the next higher to given value in this BST.
        /// </summary>
        public T NextHigher(T value)
        {
            var node = FindNode(value);
            if (node == null)
                return default(T);

            var next = (node as BSTNodeBase<T>).NextHigher();
            return next != null ? next.Value : default(T);
        }

        /// <summary>
        /// Descending enumerable.
        /// </summary>
        public IEnumerable<T> AsEnumerableDesc()
        {
            return GetEnumeratorDesc().AsEnumerable();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public BSTEnumerator<T> GetEnumerator()
        {
            return new BSTEnumerator<T>(Root);
        }

        IEnumerator<T> IEnumerable<T>.GetEnumerator()
        {
            return GetEnumerator();
        }

        public BSTEnumerator<T> GetEnumeratorDesc()
        {
            return new BSTEnumerator<T>(Root, false);
        }
    }

    internal enum RedBlackTreeNodeColor
    {
        Black,
        Red
    }

    /// <summary>
    /// Red black tree node
    /// </summary>
    internal class RedBlackTreeNode<T> : BSTNodeBase<T>
    {
        internal new RedBlackTreeNode<T> Parent
        {
            get { return (RedBlackTreeNode<T>)base.Parent; }
            set { base.Parent = value; }
        }

        internal new RedBlackTreeNode<T> Left
        {
            get { return (RedBlackTreeNode<T>)base.Left; }
            set { base.Left = value; }
        }

        internal new RedBlackTreeNode<T> Right
        {
            get { return (RedBlackTreeNode<T>)base.Right; }
            set { base.Right = value; }
        }

        internal RedBlackTreeNodeColor NodeColor { get; set; }

        internal RedBlackTreeNode<T> Sibling => Parent.Left == this ?
                                                Parent.Right : Parent.Left;

        internal void Reset(RedBlackTreeNode<T> parent, T value, RedBlackTreeNodeColor color = RedBlackTreeNodeColor.Red)
        {
            Value = value;
            Parent = parent;
            NodeColor = color;
        }
    }

    internal struct RedBlackTreeNodeIndex<T>
    {
        public RedBlackTreeNode<T> node;
        public int index;

        internal RedBlackTreeNodeIndex(RedBlackTreeNode<T> node, int index)
        {
            this.node = node;
            this.index = index;
        }

        public BSTIterator<T> ToIterator()
        {
            return new BSTIterator<T>(node, index);
        }
    }
}
