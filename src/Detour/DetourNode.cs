// MIT License - Copyright (C) ryancheung and the FelCore team
// This file is subject to the terms and conditions defined in
// file 'LICENSE', which is part of this source code package.

using System;
using dtPolyRef = System.UInt64;
using dtNodeIndex = System.UInt16;
using static RecastSharp.DetourCommon;

namespace RecastSharp
{
    public enum dtNodeFlags
    {
        DT_NODE_OPEN = 0x01,
        DT_NODE_CLOSED = 0x02,
        DT_NODE_PARENT_DETACHED = 0x04 // parent of the node is not adjacent. Found using raycast.
    }
    public unsafe struct dtNode
    {
        public fixed float pos[3]; ///< Position of the node.
        public float cost; ///< Cost from previous node to current node.
        public float total; ///< Cost up to the node.
        public uint pidx; ///< Index to parent node.
        public byte state; ///< extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE
        public int flags; ///< Node flags. A combination of dtNodeFlags.
        public dtPolyRef id; ///< Polygon ref the node corresponds to.
    }

    public unsafe class dtNodePool : System.IDisposable
    {
        //////////////////////////////////////////////////////////////////////////////////////////
        public dtNodePool(int maxNodes, int hashSize)
        {
            dtAssert(dtNextPow2((uint)m_hashSize) == (uint)m_hashSize);
            // pidx is special as 0 means "none" and 1 is the first node. For that reason
            // we have 1 fewer nodes available than the number of values it can contain.
            dtAssert(m_maxNodes > 0 && m_maxNodes <= DT_NULL_IDX && m_maxNodes <= (1 << DT_NODE_PARENT_BITS) - 1);


            this.m_maxNodes = maxNodes;
            this.m_hashSize = hashSize;
            this.m_nodeCount = 0;

            m_nodes = (dtNode*)dtAlloc(sizeof(dtNode) * m_maxNodes);
            m_next = (dtNodeIndex*)dtAlloc(sizeof(dtNodeIndex) * m_maxNodes);
            m_first = (dtNodeIndex*)dtAlloc(sizeof(dtNodeIndex) * hashSize);

            dtAssert(m_nodes != default);
            dtAssert(m_next != default);
            dtAssert(m_first != default);

            new Span<byte>(m_first, sizeof(dtNodeIndex) * m_hashSize).Fill(0xff);
            new Span<byte>(m_next, sizeof(dtNodeIndex) * m_maxNodes).Fill(0xff);
        }

        ~dtNodePool()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected void Dispose(bool disposing)
        {
            dtFree((IntPtr)m_nodes);
            dtFree((IntPtr)m_next);
            dtFree((IntPtr)m_first);
        }

        public void clear()
        {
            new Span<byte>(m_first, sizeof(dtNodeIndex) * m_hashSize).Fill(0xff);
            m_nodeCount = 0;
        }

        // Get a dtNode by ref and extra state information. If there is none then - allocate
        // There can be more than one node for the same polyRef but with different extra state information
        public dtNode* getNode(dtPolyRef id, byte state = 0)
        {
            uint bucket = (uint)(dtHashRef(id) & (m_hashSize - 1));
            dtNodeIndex i = m_first[bucket];
            dtNode* node = default;
            while (i != DT_NULL_IDX)
            {
                if (m_nodes[i].id == id && m_nodes[i].state == state)
                    return &m_nodes[i];
                i = m_next[i];
            }

            if (m_nodeCount >= m_maxNodes)
                return default;

            i = (dtNodeIndex)m_nodeCount;
            m_nodeCount++;

            // Init node
            node = &m_nodes[i];
            node->pidx = 0;
            node->cost = 0F;
            node->total = 0F;
            node->id = id;
            node->state = state;
            node->flags = 0;

            m_next[i] = m_first[bucket];
            m_first[bucket] = i;

            return node;
        }

        public dtNode* findNode(dtPolyRef id, byte state)
        {
            uint bucket = (uint)(dtHashRef(id) & (m_hashSize - 1));
            dtNodeIndex i = m_first[bucket];
            while (i != DT_NULL_IDX)
            {
                if (m_nodes[i].id == id && m_nodes[i].state == state)
                    return &m_nodes[i];
                i = m_next[i];
            }
            return default;
        }

        public uint findNodes(dtPolyRef id, dtNode** nodes, int maxNodes)
        {
            int n = 0;
            uint bucket = (uint)(dtHashRef(id) & (m_hashSize - 1));
            dtNodeIndex i = m_first[bucket];
            while (i != DT_NULL_IDX)
            {
                if (m_nodes[i].id == id)
                {
                    if (n >= maxNodes)
                    {
                        return (uint)n;
                    }
                    nodes[n++] = &m_nodes[i];
                }
                i = m_next[i];
            }

            return (uint)n;
        }

        public uint getNodeIdx(dtNode* node)
        {
            if (node == default)
                return 0;

            return (uint)(node - m_nodes) + 1;
        }

        public dtNode* getNodeAtIdx(uint idx)
        {
            if (idx == 0)
                return default;

            return &m_nodes[idx - 1];
        }

        public int getMemUsed()
        {
            return sizeof(int) * 3 + sizeof(void*) * 3 + sizeof(dtNode) * m_maxNodes + sizeof(dtNodeIndex) * m_maxNodes + sizeof(dtNodeIndex) * m_hashSize;
        }

        public int getMaxNodes()
        {
            return m_maxNodes;
        }

        public int getHashSize()
        {
            return m_hashSize;
        }
        public dtNodeIndex getFirst(int bucket)
        {
            return m_first[bucket];
        }
        public dtNodeIndex getNext(int i)
        {
            return m_next[i];
        }
        public int getNodeCount()
        {
            return m_nodeCount;
        }

        private dtNode* m_nodes;
        private dtNodeIndex* m_first;
        private dtNodeIndex* m_next;
        private readonly int m_maxNodes;
        private readonly int m_hashSize;
        private int m_nodeCount;
    }

    public unsafe class dtNodeQueue : System.IDisposable
    {

        //////////////////////////////////////////////////////////////////////////////////////////
        public dtNodeQueue(int n)
        {
            m_capacity = n;
            dtAssert(m_capacity > 0);
            this.m_size = 0;

            m_heap = (dtNode**)dtAlloc(sizeof(dtNode*) * (m_capacity + 1));
            dtAssert(m_heap != default);
        }

        ~dtNodeQueue()
        {
            Dispose(false);
        }

        bool _disposed;

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected void Dispose(bool disposing)
        {
            if (_disposed) return;

            dtFree((IntPtr)m_heap);
            _disposed = true;
        }

        public void clear()
        {
            m_size = 0;
        }

        public dtNode* top()
        {
            return m_heap[0];
        }

        public dtNode* pop()
        {
            dtNode* result = m_heap[0];
            m_size--;
            trickleDown(0, m_heap[m_size]);
            return result;
        }

        public void push(dtNode* node)
        {
            m_size++;
            bubbleUp(m_size - 1, node);
        }

        public void modify(dtNode* node)
        {
            for (int i = 0; i < m_size; ++i)
            {
                if (m_heap[i] == node)
                {
                    bubbleUp(i, node);
                    return;
                }
            }
        }

        public bool empty()
        {
            return m_size == 0;
        }

        public int getMemUsed()
        {
            return sizeof(dtNode**) + sizeof(int) * 2 + sizeof(dtNode*) * (m_capacity + 1);
        }

        public int getCapacity()
        {
            return m_capacity;
        }

        private void bubbleUp(int i, dtNode* node)
        {
            int parent = (i - 1) / 2;
            // note: (index > 0) means there is a parent
            while ((i > 0) && (m_heap[parent]->total > node->total))
            {
                m_heap[i] = m_heap[parent];
                i = parent;
                parent = (i - 1) / 2;
            }
            m_heap[i] = node;
        }

        private void trickleDown(int i, dtNode* node)
        {
            int child = (i * 2) + 1;
            while (child < m_size)
            {
                if (((child + 1) < m_size) && (m_heap[child]->total > m_heap[child + 1]->total))
                {
                    child++;
                }
                m_heap[i] = m_heap[child];
                i = child;
                child = (i * 2) + 1;
            }
            bubbleUp(i, node);
        }

        private dtNode** m_heap;
        private readonly int m_capacity;
        private int m_size;
    }

}
