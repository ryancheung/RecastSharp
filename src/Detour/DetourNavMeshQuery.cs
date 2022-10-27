// MIT License - Copyright (C) ryancheung and the FelCore team
// This file is subject to the terms and conditions defined in
// file 'LICENSE', which is part of this source code package.

/// Defines polygon filtering and traversal costs for navigation mesh query operations.
/// @ingroup detour

using System;
using dtPolyRef = System.UInt64;
using dtStatus = System.UInt32;

namespace RecastSharp
{
    using static DetourCommon;
    using static dtNodeFlags;
    using static dtPolyTypes;
    using static dtStraightPathFlags;
    using static dtStraightPathOptions;
    using static dtFindPathOptions;
    using static dtRaycastOptions;

    public unsafe class dtQueryFilter
    {
        private float[] m_areaCost = new float[DT_MAX_AREAS]; ///< Cost per area type. (Used by default implementation.)
        private ushort m_includeFlags; ///< Flags for polygons that can be visited. (Used by default implementation.)
        private ushort m_excludeFlags; ///< Flags for polygons that should not be visted. (Used by default implementation.)


        /// @class dtQueryFilter
        ///
        /// <b>The Default Implementation</b>
        /// 
        /// At construction: All area costs default to 1.0.  All flags are included
        /// and none are excluded.
        /// 
        /// If a polygon has both an include and an exclude flag, it will be excluded.
        /// 
        /// The way filtering works, a navigation mesh polygon must have at least one flag 
        /// set to ever be considered by a query. So a polygon with no flags will never
        /// be considered.
        ///
        /// Setting the include flags to 0 will result in all polygons being excluded.
        ///
        /// <b>Custom Implementations</b>
        /// 
        /// Implement a custom query filter by overriding the virtual passFilter() 
        /// and getCost() functions. If this is done, both functions should be as 
        /// fast as possible. Use cached local copies of data rather than accessing 
        /// your own objects where possible.
        /// 
        /// Custom implementations do not need to adhere to the flags or cost logic 
        /// used by the default implementation.  
        /// 
        /// In order for A* searches to work properly, the cost should be proportional to
        /// the travel distance. Implementing a cost modifier less than 1.0 is likely 
        /// to lead to problems during pathfinding.
        ///
        /// @see dtNavMeshQuery

        public dtQueryFilter()
        {
            m_includeFlags = 0xffff;
            m_excludeFlags = 0;
            for (int i = 0; i < DT_MAX_AREAS; ++i)
                m_areaCost[i] = 1.0f;
        }

        public bool passFilter(dtPolyRef @ref, dtMeshTile* tile, dtPoly* poly)
        {
            return (poly->flags & m_includeFlags) != 0 && (poly->flags & m_excludeFlags) == 0;
        }

        /// Returns cost to move from the beginning to the end of a line segment
        /// that is fully contained within a polygon.
        ///  @param[in]        pa            The start position on the edge of the previous and current polygon. [(x, y, z)]
        ///  @param[in]        pb            The end position on the edge of the current and next polygon. [(x, y, z)]
        ///  @param[in]        prevRef        The reference id of the previous polygon. [opt]
        ///  @param[in]        prevTile    The tile containing the previous polygon. [opt]
        ///  @param[in]        prevPoly    The previous polygon. [opt]
        ///  @param[in]        curRef        The reference id of the current polygon.
        ///  @param[in]        curTile        The tile containing the current polygon.
        ///  @param[in]        curPoly        The current polygon.
        ///  @param[in]        nextRef        The refernece id of the next polygon. [opt]
        ///  @param[in]        nextTile    The tile containing the next polygon. [opt]
        ///  @param[in]        nextPoly    The next polygon. [opt]
        public float getCost(float* pa, float* pb,
            dtPolyRef prevRef, dtMeshTile* prevTile, dtPoly* prevPoly,
            dtPolyRef curRef, dtMeshTile* curTile, dtPoly* curPoly,
            dtPolyRef nextRef, dtMeshTile* nextTile, dtPoly* nextPoly)
        {
            return dtVdist(pa, pb) * m_areaCost[curPoly->getArea()];
        }

        /// Returns the traversal cost of the area.
        ///  @param[in]        i        The id of the area.
        /// @returns The traversal cost of the area.
        public float getAreaCost(int i)
        {
            return m_areaCost[i];
        }

        /// Sets the traversal cost of the area.
        ///  @param[in]        i        The id of the area.
        ///  @param[in]        cost    The new cost of traversing the area.
        public void setAreaCost(int i, float cost)
        {
            m_areaCost[i] = cost;
        }

        /// Returns the include flags for the filter.
        /// Any polygons that include one or more of these flags will be
        /// included in the operation.
        public ushort getIncludeFlags()
        {
            return m_includeFlags;
        }

        /// Sets the include flags for the filter.
        /// @param[in]        flags    The new flags.
        public void setIncludeFlags(ushort flags)
        {
            m_includeFlags = flags;
        }

        /// Returns the exclude flags for the filter.
        /// Any polygons that include one ore more of these flags will be
        /// excluded from the operation.
        public ushort getExcludeFlags()
        {
            return m_excludeFlags;
        }

        /// Sets the exclude flags for the filter.
        /// @param[in]        flags        The new flags.
        public void setExcludeFlags(ushort flags)
        {
            m_excludeFlags = flags;
        }
    }

    /// Provides information about raycast hit
    /// filled by dtNavMeshQuery::raycast
    /// @ingroup detour
    public unsafe struct dtRaycastHit
    {
        /// The hit parameter. (FLT_MAX if no wall hit.)
        public float t;

        /// hitNormal    The normal of the nearest wall hit. [(x, y, z)]
        public fixed float hitNormal[3];

        /// The index of the edge on the final polygon where the wall was hit.
        public int hitEdgeIndex;

        /// Pointer to an array of reference ids of the visited polygons. [opt]
        public dtPolyRef* path;

        /// The number of visited polygons. [opt]
        public int pathCount;

        /// The maximum number of polygons the @p path array can hold.
        public int maxPath;

        ///  The cost of the path until hit.
        public float pathCost;
    }

    /// Provides custom polygon query behavior.
    /// Used by dtNavMeshQuery::queryPolygons.
    /// @ingroup detour
    public unsafe interface dtPolyQuery
    {

        /// Called for each batch of unique polygons touched by the search area in dtNavMeshQuery::queryPolygons.
        /// This can be called multiple times for a single query.
        void process(dtMeshTile* tile, dtPoly** polys, dtPolyRef* refs, int count);
    }

    /// Provides the ability to perform pathfinding related queries against
    /// a navigation mesh.
    /// @ingroup detour
    public unsafe class dtNavMeshQuery : System.IDisposable
    {

        //////////////////////////////////////////////////////////////////////////////////////////

        /// @class dtNavMeshQuery
        ///
        /// For methods that support undersized buffers, if the buffer is too small 
        /// to hold the entire result set the return status of the method will include 
        /// the #DT_BUFFER_TOO_SMALL flag.
        ///
        /// Constant member functions can be used by multiple clients without side
        /// effects. (E.g. No change to the closed list. No impact on an in-progress
        /// sliced path query. Etc.)
        /// 
        /// Walls and portals: A @e wall is a polygon segment that is 
        /// considered impassable. A @e portal is a passable segment between polygons.
        /// A portal may be treated as a wall based on the dtQueryFilter used for a query.
        ///
        /// @see dtNavMesh, dtQueryFilter, #dtAllocNavMeshQuery(), #dtAllocNavMeshQuery()

        public dtNavMeshQuery()
        {
        }

        ~dtNavMeshQuery()
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
            if (m_tinyNodePool != null)
                m_tinyNodePool.Dispose();
            if (m_nodePool != null)
                m_nodePool.Dispose();
            if (m_openList != null)
                m_openList.Dispose();

            m_tinyNodePool = null;
            m_nodePool = null;
            m_openList = null;
        }

        /// Initializes the query object.
        ///  @param[in]        nav            Pointer to the dtNavMesh object to use for all queries.
        ///  @param[in]        maxNodes    Maximum number of search nodes. [Limits: 0 < value <= 65535]
        /// @returns The status flags for the query.

        /// @par 
        ///
        /// Must be the first function called after construction, before other
        /// functions are used.
        ///
        /// This function can be used multiple times.
        public dtStatus init(dtNavMesh nav, int maxNodes)
        {
            if (maxNodes > DT_NULL_IDX || maxNodes > (1 << DT_NODE_PARENT_BITS) - 1)
                return DT_FAILURE | DT_INVALID_PARAM;

            m_nav = nav;

            if (m_nodePool == null || m_nodePool.getMaxNodes() < maxNodes)
            {
                if (m_nodePool != null)
                {
                    m_nodePool.Dispose();
                    m_nodePool = null;
                }
                m_nodePool = new dtNodePool(maxNodes, (int)dtNextPow2((uint)(maxNodes / 4)));
                if (m_nodePool == null)
                    return DT_FAILURE | DT_OUT_OF_MEMORY;
            }
            else
            {
                m_nodePool.clear();
            }

            if (m_tinyNodePool == null)
            {
                m_tinyNodePool = new dtNodePool(64, 32);
                if (m_tinyNodePool == null)
                    return DT_FAILURE | DT_OUT_OF_MEMORY;
            }
            else
            {
                m_tinyNodePool.clear();
            }

            if (m_openList == null || m_openList.getCapacity() < maxNodes)
            {
                if (m_openList != null)
                {
                    m_openList.Dispose();
                    m_openList = null;
                }
                m_openList = new dtNodeQueue(maxNodes);
                if (m_openList == null)
                    return DT_FAILURE | DT_OUT_OF_MEMORY;
            }
            else
            {
                m_openList.clear();
            }

            return DT_SUCCESS;
        }

        /// @name Standard Pathfinding Functions

        /// Finds a path from the start polygon to the end polygon.
        ///  @param[in]        startRef    The refrence id of the start polygon.
        ///  @param[in]        endRef        The reference id of the end polygon.
        ///  @param[in]        startPos    A position within the start polygon. [(x, y, z)]
        ///  @param[in]        endPos        A position within the end polygon. [(x, y, z)]
        ///  @param[in]        filter        The polygon filter to apply to the query.
        ///  @param[out]    path        An ordered list of polygon references representing the path. (Start to end.) 
        ///                              [(polyRef) * @p pathCount]
        ///  @param[out]    pathCount    The number of polygons returned in the @p path array.
        ///  @param[in]        maxPath        The maximum number of polygons the @p path array can hold. [Limit: >= 1]

        /// @par
        ///
        /// If the end polygon cannot be reached through the navigation graph,
        /// the last polygon in the path will be the nearest the end polygon.
        ///
        /// If the path array is to small to hold the full result, it will be filled as 
        /// far as possible from the start polygon toward the end polygon.
        ///
        /// The start and end positions are used to calculate traversal costs. 
        /// (The y-values impact the result.)
        ///
        public dtStatus findPath(dtPolyRef startRef, dtPolyRef endRef, float* startPos, float* endPos, ref dtQueryFilter filter, dtPolyRef* path, out int pathCount, int maxPath)
        {
            dtAssert(m_nav != null);
            dtAssert(m_nodePool != null);
            dtAssert(m_openList != null);

            pathCount = 0;

            // Validate input
            if (!m_nav!.isValidPolyRef(startRef) || !m_nav.isValidPolyRef(endRef) || startPos == null || !dtVisfinite(startPos) || endPos == null || !dtVisfinite(endPos) || path == null || maxPath <= 0)
                return DT_FAILURE | DT_INVALID_PARAM;

            if (startRef == endRef)
            {
                path[0] = startRef;
                pathCount = 1;
                return DT_SUCCESS;
            }

            m_nodePool!.clear();
            m_openList!.clear();

            dtNode* startNode = m_nodePool.getNode(startRef);
            dtVcopy(startNode->pos, startPos);
            startNode->pidx = 0;
            startNode->cost = 0F;
            startNode->total = dtVdist(startPos, endPos) * H_SCALE;
            startNode->id = startRef;
            startNode->flags = (int)DT_NODE_OPEN;
            m_openList.push(startNode);

            dtNode* lastBestNode = startNode;
            float lastBestNodeCost = startNode->total;

            bool outOfNodes = false;

            while (!m_openList.empty())
            {
                // Remove node from open list and put it in closed list.
                dtNode* bestNode = m_openList.pop();
                bestNode->flags &= ~(int)DT_NODE_OPEN;
                bestNode->flags |= (int)DT_NODE_CLOSED;

                // Reached the goal, stop searching.
                if (bestNode->id == endRef)
                {
                    lastBestNode = bestNode;
                    break;
                }

                // Get current poly and tile.
                // The API input has been cheked already, skip checking internal data.
                dtPolyRef bestRef = bestNode->id;
                dtMeshTile* bestTile = null;
                dtPoly* bestPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(bestRef, out bestTile, out bestPoly);

                // Get parent poly and tile.
                dtPolyRef parentRef = 0;
                dtMeshTile* parentTile = null;
                dtPoly* parentPoly = null;
                if (bestNode->pidx != 0)
                    parentRef = m_nodePool.getNodeAtIdx(bestNode->pidx)->id;
                if (parentRef != 0)
                    m_nav.getTileAndPolyByRefUnsafe(parentRef, out parentTile, out parentPoly);

                for (uint i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
                {
                    dtPolyRef neighbourRef = bestTile->links[i].@ref;

                    // Skip invalid ids and do not expand back to where we came from.
                    if (neighbourRef == 0 || neighbourRef == parentRef)
                        continue;

                    // Get neighbour poly and tile.
                    // The API input has been cheked already, skip checking internal data.
                    dtMeshTile* neighbourTile = null;
                    dtPoly* neighbourPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, out neighbourTile, out neighbourPoly);

                    if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
                        continue;

                    // deal explicitly with crossing tile boundaries
                    byte crossSide = 0;
                    if (bestTile->links[i].side != 0xff)
                        crossSide = (byte)(bestTile->links[i].side >> 1);

                    // get the node
                    dtNode* neighbourNode = m_nodePool.getNode(neighbourRef, crossSide);
                    if (neighbourNode == null)
                    {
                        outOfNodes = true;
                        continue;
                    }

                    // If the node is visited the first time, calculate node position.
                    if (neighbourNode->flags == 0)
                    {
                        getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, neighbourNode->pos);
                    }

                    // Calculate cost and heuristic.
                    float cost = 0F;
                    float heuristic = 0F;

                    // Special case for last node.
                    if (neighbourRef == endRef)
                    {
                        // Cost
                        float curCost = filter.getCost(bestNode->pos, neighbourNode->pos,
                            parentRef, parentTile, parentPoly,
                            bestRef, bestTile, bestPoly,
                            neighbourRef, neighbourTile, neighbourPoly);
                        float endCost = filter.getCost(neighbourNode->pos, endPos,
                            bestRef, bestTile, bestPoly,
                            neighbourRef, neighbourTile, neighbourPoly,
                            0, null, null);

                        cost = bestNode->cost + curCost + endCost;
                        heuristic = 0F;
                    }
                    else
                    {
                        // Cost
                        float curCost = filter.getCost(bestNode->pos, neighbourNode->pos,
                            parentRef, parentTile, parentPoly,
                            bestRef, bestTile, bestPoly,
                            neighbourRef, neighbourTile, neighbourPoly);
                        cost = bestNode->cost + curCost;
                        heuristic = dtVdist(neighbourNode->pos, endPos) * H_SCALE;
                    }

                    float total = cost + heuristic;

                    // The node is already in open list and the new result is worse, skip.
                    if ((neighbourNode->flags & (int)DT_NODE_OPEN) != 0 && total >= neighbourNode->total)
                        continue;
                    // The node is already visited and process, and the new result is worse, skip.
                    if ((neighbourNode->flags & (int)DT_NODE_CLOSED) != 0 && total >= neighbourNode->total)
                        continue;

                    // Add or update the node.
                    neighbourNode->pidx = m_nodePool.getNodeIdx(bestNode);
                    neighbourNode->id = neighbourRef;
                    neighbourNode->flags = neighbourNode->flags & ~(int)DT_NODE_CLOSED;
                    neighbourNode->cost = cost;
                    neighbourNode->total = total;

                    if ((neighbourNode->flags & (int)DT_NODE_OPEN) != 0)
                    {
                        // Already in open, update node location.
                        m_openList.modify(neighbourNode);
                    }
                    else
                    {
                        // Put the node in open list.
                        neighbourNode->flags |= (int)DT_NODE_OPEN;
                        m_openList.push(neighbourNode);
                    }

                    // Update nearest node to target so far.
                    if (heuristic < lastBestNodeCost)
                    {
                        lastBestNodeCost = heuristic;
                        lastBestNode = neighbourNode;
                    }
                }
            }

            dtStatus status = getPathToNode(lastBestNode, path, out pathCount, maxPath);

            if (lastBestNode->id != endRef)
                status |= DT_PARTIAL_RESULT;

            if (outOfNodes)
                status |= DT_OUT_OF_NODES;

            return status;
        }

        /// Finds the straight path from the start to the end position within the polygon corridor.
        ///  @param[in]        startPos            Path start position. [(x, y, z)]
        ///  @param[in]        endPos                Path end position. [(x, y, z)]
        ///  @param[in]        path                An array of polygon references that represent the path corridor.
        ///  @param[in]        pathSize            The number of polygons in the @p path array.
        ///  @param[out]    straightPath        Points describing the straight path. [(x, y, z) * @p straightPathCount].
        ///  @param[out]    straightPathFlags    Flags describing each point. (See: #dtStraightPathFlags) [opt]
        ///  @param[out]    straightPathRefs    The reference id of the polygon that is being entered at each point. [opt]
        ///  @param[out]    straightPathCount    The number of points in the straight path.
        ///  @param[in]        maxStraightPath        The maximum number of points the straight path arrays can hold.  [Limit: > 0]
        ///  @param[in]        options                Query options. (see: #dtStraightPathOptions)
        /// @returns The status flags for the query.

        /// @par
        /// 
        /// This method peforms what is often called 'string pulling'.
        ///
        /// The start position is clamped to the first polygon in the path, and the 
        /// end position is clamped to the last. So the start and end positions should 
        /// normally be within or very near the first and last polygons respectively.
        ///
        /// The returned polygon references represent the reference id of the polygon 
        /// that is entered at the associated path position. The reference id associated 
        /// with the end point will always be zero.  This allows, for example, matching 
        /// off-mesh link points to their representative polygons.
        ///
        /// If the provided result buffers are too small for the entire result set, 
        /// they will be filled as far as possible from the start toward the end 
        /// position.
        ///
        public dtStatus findStraightPath(float* startPos, float* endPos,
            dtPolyRef* path, int pathSize,
            float* straightPath, byte* straightPathFlags, dtPolyRef* straightPathRefs,
            out int straightPathCount, int maxStraightPath, int options = 0)
        {
            dtAssert(m_nav != null);

            straightPathCount = 0;

            if (startPos == null || !dtVisfinite(startPos) ||
                endPos == null || !dtVisfinite(endPos) ||
                path == null || pathSize <= 0 || path[0] == 0 ||
                maxStraightPath <= 0)
            {
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            dtStatus stat = 0;

            // TODO: Should this be callers responsibility?
            float* closestStartPos = stackalloc float[3];
            if (dtStatusFailed(closestPointOnPolyBoundary(path[0], startPos, closestStartPos)))
                return DT_FAILURE | DT_INVALID_PARAM;

            float* closestEndPos = stackalloc float[3];
            if (dtStatusFailed(closestPointOnPolyBoundary(path[pathSize - 1], endPos, closestEndPos)))
                return DT_FAILURE | DT_INVALID_PARAM;

            // Add start point.
            stat = appendVertex(closestStartPos, (byte)DT_STRAIGHTPATH_START, path[0],
                straightPath, straightPathFlags, straightPathRefs,
                ref straightPathCount, maxStraightPath);
            if (stat != DT_IN_PROGRESS)
                return stat;

            if (pathSize > 1)
            {
                float* portalApex = stackalloc float[3];
                float* portalLeft = stackalloc float[3];
                float* portalRight = stackalloc float[3];
                dtVcopy(portalApex, closestStartPos);
                dtVcopy(portalLeft, portalApex);
                dtVcopy(portalRight, portalApex);
                int apexIndex = 0;
                int leftIndex = 0;
                int rightIndex = 0;

                byte leftPolyType = 0;
                byte rightPolyType = 0;

                dtPolyRef leftPolyRef = path[0];
                dtPolyRef rightPolyRef = path[0];

                float* left = stackalloc float[3];
                float* right = stackalloc float[3];
                for (int i = 0; i < pathSize; ++i)
                {
                    byte toType = 0;

                    if (i + 1 < pathSize)
                    {
                        byte fromType = 0; // fromType is ignored.

                        // Next portal.
                        if (dtStatusFailed(getPortalPoints(path[i], path[i + 1], left, right, ref fromType, ref toType)))
                        {
                            // Failed to get portal points, in practice this means that path[i+1] is invalid polygon.
                            // Clamp the end point to path[i], and return the path so far.

                            if (dtStatusFailed(closestPointOnPolyBoundary(path[i], endPos, closestEndPos)))
                            {
                                // This should only happen when the first polygon is invalid.
                                return DT_FAILURE | DT_INVALID_PARAM;
                            }

                            // Apeend portals along the current straight path segment.
                            if ((options & (int)(DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0)
                            {
                                // Ignore status return value as we're just about to return anyway.
                                appendPortals(apexIndex, i, closestEndPos, path,
                                    straightPath, straightPathFlags, straightPathRefs,
                                    ref straightPathCount, maxStraightPath, options);
                            }

                            // Ignore status return value as we're just about to return anyway.
                            appendVertex(closestEndPos, 0, path[i],
                                straightPath, straightPathFlags, straightPathRefs,
                                ref straightPathCount, maxStraightPath);

                            return DT_SUCCESS | DT_PARTIAL_RESULT | ((straightPathCount >= maxStraightPath) ? DT_BUFFER_TOO_SMALL : 0);
                        }

                        // If starting really close the portal, advance.
                        if (i == 0)
                        {
                            float t = 0;
                            if (dtDistancePtSegSqr2D(portalApex, left, right, ref t) < (0.001f) * (0.001f))
                                continue;
                        }
                    }
                    else
                    {
                        // End of the path.
                        dtVcopy(left, closestEndPos);
                        dtVcopy(right, closestEndPos);

                        toType = (byte)dtPolyTypes.DT_POLYTYPE_GROUND;
                    }

                    // Right vertex.
                    if (dtTriArea2D(portalApex, portalRight, right) <= 0.0f)
                    {
                        if (dtVequal(portalApex, portalRight) || dtTriArea2D(portalApex, portalLeft, right) > 0.0f)
                        {
                            dtVcopy(portalRight, right);
                            rightPolyRef = (i + 1 < pathSize) ? path[i + 1] : 0;
                            rightPolyType = toType;
                            rightIndex = i;
                        }
                        else
                        {
                            // Append portals along the current straight path segment.
                            if ((options & (int)(DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0)
                            {
                                stat = appendPortals(apexIndex, leftIndex, portalLeft, path,
                                    straightPath, straightPathFlags, straightPathRefs,
                                    ref straightPathCount, maxStraightPath, options);
                                if (stat != DT_IN_PROGRESS)
                                    return stat;
                            }

                            dtVcopy(portalApex, portalLeft);
                            apexIndex = leftIndex;

                            byte flags = 0;
                            if (leftPolyRef == 0)
                                flags = (byte)DT_STRAIGHTPATH_END;
                            else if (leftPolyType == (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
                                flags = (byte)DT_STRAIGHTPATH_OFFMESH_CONNECTION;
                            dtPolyRef @ref = leftPolyRef;

                            // Append or update vertex
                            stat = appendVertex(portalApex, flags, @ref,
                                straightPath, straightPathFlags, straightPathRefs,
                                ref straightPathCount, maxStraightPath);
                            if (stat != DT_IN_PROGRESS)
                                return stat;

                            dtVcopy(portalLeft, portalApex);
                            dtVcopy(portalRight, portalApex);
                            leftIndex = apexIndex;
                            rightIndex = apexIndex;

                            // Restart
                            i = apexIndex;

                            continue;
                        }
                    }

                    // Left vertex.
                    if (dtTriArea2D(portalApex, portalLeft, left) >= 0.0f)
                    {
                        if (dtVequal(portalApex, portalLeft) || dtTriArea2D(portalApex, portalRight, left) < 0.0f)
                        {
                            dtVcopy(portalLeft, left);
                            leftPolyRef = (i + 1 < pathSize) ? path[i + 1] : 0;
                            leftPolyType = toType;
                            leftIndex = i;
                        }
                        else
                        {
                            // Append portals along the current straight path segment.
                            if ((options & (int)(DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0)
                            {
                                stat = appendPortals(apexIndex, rightIndex, portalRight, path,
                                    straightPath, straightPathFlags, straightPathRefs,
                                    ref straightPathCount, maxStraightPath, options);
                                if (stat != DT_IN_PROGRESS)
                                    return stat;
                            }

                            dtVcopy(portalApex, portalRight);
                            apexIndex = rightIndex;

                            byte flags = 0;
                            if (rightPolyRef == 0)
                                flags = (byte)DT_STRAIGHTPATH_END;
                            else if (rightPolyType == (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
                                flags = (byte)DT_STRAIGHTPATH_OFFMESH_CONNECTION;
                            dtPolyRef @ref = rightPolyRef;

                            // Append or update vertex
                            stat = appendVertex(portalApex, flags, @ref,
                                straightPath, straightPathFlags, straightPathRefs,
                                ref straightPathCount, maxStraightPath);
                            if (stat != DT_IN_PROGRESS)
                                return stat;

                            dtVcopy(portalLeft, portalApex);
                            dtVcopy(portalRight, portalApex);
                            leftIndex = apexIndex;
                            rightIndex = apexIndex;

                            // Restart
                            i = apexIndex;

                            continue;
                        }
                    }
                }

                // Append portals along the current straight path segment.
                if ((options & (int)(DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0)
                {
                    stat = appendPortals(apexIndex, pathSize - 1, closestEndPos, path,
                        straightPath, straightPathFlags, straightPathRefs,
                        ref straightPathCount, maxStraightPath, options);
                    if (stat != DT_IN_PROGRESS)
                        return stat;
                }
            }

            // Ignore status return value as we're just about to return anyway.
            appendVertex(closestEndPos, (byte)DT_STRAIGHTPATH_END, 0,
                straightPath, straightPathFlags, straightPathRefs,
                ref straightPathCount, maxStraightPath);

            return DT_SUCCESS | ((straightPathCount >= maxStraightPath) ? DT_BUFFER_TOO_SMALL : 0);
        }

        /// @name Sliced Pathfinding Functions
        /// Common use case:
        ///    -# Call initSlicedFindPath() to initialize the sliced path query.
        ///    -# Call updateSlicedFindPath() until it returns complete.
        ///    -# Call finalizeSlicedFindPath() to get the path.

        /// Intializes a sliced path query.
        ///  @param[in]        startRef    The refrence id of the start polygon.
        ///  @param[in]        endRef        The reference id of the end polygon.
        ///  @param[in]        startPos    A position within the start polygon. [(x, y, z)]
        ///  @param[in]        endPos        A position within the end polygon. [(x, y, z)]
        ///  @param[in]        filter        The polygon filter to apply to the query.
        ///  @param[in]        options        query options (see: #dtFindPathOptions)
        /// @returns The status flags for the query.

        /// @par
        ///
        /// @warning Calling any non-slice methods before calling finalizeSlicedFindPath() 
        /// or finalizeSlicedFindPathPartial() may result in corrupted data!
        ///
        /// The @p filter pointer is stored and used for the duration of the sliced
        /// path query.
        ///
        public dtStatus initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
            float* startPos, float* endPos,
            dtQueryFilter filter, uint options = 0)
        {
            dtAssert(m_nav != null);
            dtAssert(m_nodePool != null);
            dtAssert(m_openList != null);

            // Init path state.
            m_query.status = DT_FAILURE;
            m_query.startRef = startRef;
            m_query.endRef = endRef;
            if (startPos != null)
            {
                fixed (float* queryStartPos = m_query.startPos)
                    dtVcopy(queryStartPos, startPos);
            }
            if (endPos != null)
            {
                fixed (float* queryEndPos = m_query.endPos)
                    dtVcopy(queryEndPos, endPos);
            }
            m_query.filter = filter;
            m_query.options = options;
            m_query.raycastLimitSqr = float.MaxValue;

            // Validate input
            if (!m_nav!.isValidPolyRef(startRef) || !m_nav.isValidPolyRef(endRef) ||
                startPos == null || !dtVisfinite(startPos) ||
                endPos == null || !dtVisfinite(endPos))
            {
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            // trade quality with performance?
            if ((options & (uint)DT_FINDPATH_ANY_ANGLE) != 0)
            {
                // limiting to several times the character radius yields nice results. It is not sensitive
                // so it is enough to compute it from the first tile.
                dtMeshTile* tile = m_nav.getTileByRef(startRef);
                float agentRadius = tile->header->walkableRadius;
                m_query.raycastLimitSqr = (agentRadius * DT_RAY_CAST_LIMIT_PROPORTIONS) * (agentRadius * DT_RAY_CAST_LIMIT_PROPORTIONS);
            }

            if (startRef == endRef)
            {
                m_query.status = DT_SUCCESS;
                return DT_SUCCESS;
            }

            m_nodePool!.clear();
            m_openList!.clear();

            dtNode* startNode = m_nodePool.getNode(startRef);
            dtVcopy(startNode->pos, startPos);
            startNode->pidx = 0;
            startNode->cost = 0F;
            startNode->total = dtVdist(startPos, endPos) * H_SCALE;
            startNode->id = startRef;
            startNode->flags = (int)DT_NODE_OPEN;
            m_openList.push(startNode);

            m_query.status = DT_IN_PROGRESS;
            m_query.lastBestNode = startNode;
            m_query.lastBestNodeCost = startNode->total;

            return m_query.status;
        }

        /// Updates an in-progress sliced path query.
        ///  @param[in]        maxIter        The maximum number of iterations to perform.
        ///  @param[out]    doneIters    The actual number of iterations completed. [opt]
        /// @returns The status flags for the query.
        public dtStatus updateSlicedFindPath(int maxIter, out int doneIters)
        {
            doneIters = 0;

            if (!dtStatusInProgress(m_query.status))
                return m_query.status;

            // Make sure the request is still valid.
            if (!m_nav!.isValidPolyRef(m_query.startRef) || !m_nav.isValidPolyRef(m_query.endRef))
            {
                m_query.status = DT_FAILURE;
                return DT_FAILURE;
            }

            dtRaycastHit rayHit = new dtRaycastHit();
            rayHit.maxPath = 0;

            int iter = 0;
            while (iter < maxIter && !m_openList!.empty())
            {
                iter++;

                // Remove node from open list and put it in closed list.
                dtNode* bestNode = m_openList.pop();
                bestNode->flags &= (int)~DT_NODE_OPEN;
                bestNode->flags |= (int)DT_NODE_CLOSED;

                // Reached the goal, stop searching.
                if (bestNode->id == m_query.endRef)
                {
                    m_query.lastBestNode = bestNode;
                    dtStatus details = m_query.status & DT_STATUS_DETAIL_MASK;
                    m_query.status = DT_SUCCESS | details;
                    doneIters = iter;
                    return m_query.status;
                }

                // Get current poly and tile.
                // The API input has been cheked already, skip checking internal data.
                dtPolyRef bestRef = bestNode->id;
                dtMeshTile* bestTile = null;
                dtPoly* bestPoly = null;
                if (dtStatusFailed(m_nav.getTileAndPolyByRef(bestRef, out bestTile, out bestPoly)))
                {
                    // The polygon has disappeared during the sliced query, fail.
                    m_query.status = DT_FAILURE;
                    doneIters = iter;
                    return m_query.status;
                }

                // Get parent and grand parent poly and tile.
                dtPolyRef parentRef = 0;
                dtPolyRef grandpaRef = 0;
                dtMeshTile* parentTile = null;
                dtPoly* parentPoly = null;
                dtNode* parentNode = null;
                if (bestNode->pidx != 0)
                {
                    parentNode = m_nodePool!.getNodeAtIdx(bestNode->pidx);
                    parentRef = parentNode->id;
                    if (parentNode->pidx != 0)
                        grandpaRef = m_nodePool.getNodeAtIdx(parentNode->pidx)->id;
                }
                if (parentRef != 0)
                {
                    bool invalidParent = dtStatusFailed(m_nav.getTileAndPolyByRef(parentRef, out parentTile, out parentPoly));
                    if (invalidParent || (grandpaRef != 0 && !m_nav.isValidPolyRef(grandpaRef)))
                    {
                        // The polygon has disappeared during the sliced query, fail.
                        m_query.status = DT_FAILURE;
                        doneIters = iter;
                        return m_query.status;
                    }
                }

                // decide whether to test raycast to previous nodes
                bool tryLOS = false;
                if ((m_query.options & (uint)DT_FINDPATH_ANY_ANGLE) != 0)
                {
                    if ((parentRef != 0) && (dtVdistSqr(parentNode->pos, bestNode->pos) < m_query.raycastLimitSqr))
                        tryLOS = true;
                }

                for (uint i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
                {
                    dtPolyRef neighbourRef = bestTile->links[i].@ref;

                    // Skip invalid ids and do not expand back to where we came from.
                    if (neighbourRef == 0 || neighbourRef == parentRef)
                        continue;

                    // Get neighbour poly and tile.
                    // The API input has been cheked already, skip checking internal data.
                    dtMeshTile* neighbourTile = null;
                    dtPoly* neighbourPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, out neighbourTile, out neighbourPoly);

                    if (!m_query.filter!.passFilter(neighbourRef, neighbourTile, neighbourPoly))
                        continue;

                    // get the neighbor node
                    dtNode* neighbourNode = m_nodePool!.getNode(neighbourRef, 0);
                    if (neighbourNode == null)
                    {
                        m_query.status |= DT_OUT_OF_NODES;
                        continue;
                    }

                    // do not expand to nodes that were already visited from the same parent
                    if (neighbourNode->pidx != 0 && neighbourNode->pidx == bestNode->pidx)
                        continue;

                    // If the node is visited the first time, calculate node position.
                    if (neighbourNode->flags == 0)
                    {
                        getEdgeMidPoint(bestRef, bestPoly, bestTile,
                            neighbourRef, neighbourPoly, neighbourTile,
                            neighbourNode->pos);
                    }

                    // Calculate cost and heuristic.
                    float cost = 0F;
                    float heuristic = 0F;

                    // raycast parent
                    bool foundShortCut = false;
                    rayHit.pathCost = rayHit.t = 0F;
                    if (tryLOS)
                    {
                        raycast(parentRef, parentNode->pos, neighbourNode->pos, m_query.filter, (uint)DT_RAYCAST_USE_COSTS, &rayHit, grandpaRef);
                        foundShortCut = rayHit.t >= 1.0f;
                    }

                    // update move cost
                    if (foundShortCut)
                    {
                        // shortcut found using raycast. Using shorter cost instead
                        cost = parentNode->cost + rayHit.pathCost;
                    }
                    else
                    {
                        // No shortcut found.
                        float curCost = m_query.filter.getCost(bestNode->pos, neighbourNode->pos,
                            parentRef, parentTile, parentPoly,
                            bestRef, bestTile, bestPoly,
                            neighbourRef, neighbourTile, neighbourPoly);
                        cost = bestNode->cost + curCost;
                    }

                    fixed (float* queryEndPos = m_query.endPos)
                    {
                        // Special case for last node.
                        if (neighbourRef == m_query.endRef)
                        {
                            float endCost = m_query.filter.getCost(neighbourNode->pos, queryEndPos,
                                bestRef, bestTile, bestPoly,
                                neighbourRef, neighbourTile, neighbourPoly,
                                0, null, null);

                            cost = cost + endCost;
                            heuristic = 0F;
                        }
                        else
                        {
                            heuristic = dtVdist(neighbourNode->pos, queryEndPos) * H_SCALE;
                        }
                    }

                    float total = cost + heuristic;

                    // The node is already in open list and the new result is worse, skip.
                    if ((neighbourNode->flags & (int)DT_NODE_OPEN) != 0 && total >= neighbourNode->total)
                        continue;
                    // The node is already visited and process, and the new result is worse, skip.
                    if ((neighbourNode->flags & (int)DT_NODE_CLOSED) != 0 && total >= neighbourNode->total)
                        continue;

                    // Add or update the node.
                    neighbourNode->pidx = foundShortCut ? bestNode->pidx : m_nodePool.getNodeIdx(bestNode);
                    neighbourNode->id = neighbourRef;
                    neighbourNode->flags = neighbourNode->flags & (int)~(DT_NODE_CLOSED | DT_NODE_PARENT_DETACHED);
                    neighbourNode->cost = cost;
                    neighbourNode->total = total;
                    if (foundShortCut)
                    {
                        neighbourNode->flags = neighbourNode->flags | (int)DT_NODE_PARENT_DETACHED;
                    }

                    if ((neighbourNode->flags & (int)DT_NODE_OPEN) != 0)
                    {
                        // Already in open, update node location.
                        m_openList.modify(neighbourNode);
                    }
                    else
                    {
                        // Put the node in open list.
                        neighbourNode->flags |= (int)DT_NODE_OPEN;
                        m_openList.push(neighbourNode);
                    }

                    // Update nearest node to target so far.
                    if (heuristic < m_query.lastBestNodeCost)
                    {
                        m_query.lastBestNodeCost = heuristic;
                        m_query.lastBestNode = neighbourNode;
                    }
                }
            }

            // Exhausted all nodes, but could not find path.
            if (m_openList!.empty())
            {
                uint details = m_query.status & DT_STATUS_DETAIL_MASK;
                m_query.status = DT_SUCCESS | details;
            }

            doneIters = iter;

            return m_query.status;
        }

        /// Finalizes and returns the results of a sliced path query.
        ///  @param[out]    path        An ordered list of polygon references representing the path. (Start to end.) 
        ///                              [(polyRef) * @p pathCount]
        ///  @param[out]    pathCount    The number of polygons returned in the @p path array.
        ///  @param[in]        maxPath        The max number of polygons the path array can hold. [Limit: >= 1]
        /// @returns The status flags for the query.
        public uint finalizeSlicedFindPath(dtPolyRef* path, out int pathCount, int maxPath)
        {
            pathCount = 0;

            if (path == null || maxPath <= 0)
                return DT_FAILURE | DT_INVALID_PARAM;

            if (dtStatusFailed(m_query.status))
            {
                // Reset query.
                m_query.Reset();
                return DT_FAILURE;
            }

            int n = 0;

            if (m_query.startRef == m_query.endRef)
            {
                // Special case: the search starts and ends at same poly.
                path[n++] = m_query.startRef;
            }
            else
            {
                // Reverse the path.
                dtAssert(m_query.lastBestNode != null);

                if (m_query.lastBestNode->id != m_query.endRef)
                    m_query.status |= DT_PARTIAL_RESULT;

                dtNode* prev = null;
                dtNode* node = m_query.lastBestNode;
                int prevRay = 0;
                do
                {
                    dtNode* next = m_nodePool!.getNodeAtIdx(node->pidx);
                    node->pidx = m_nodePool.getNodeIdx(prev);
                    prev = node;
                    int nextRay = node->flags & (int)DT_NODE_PARENT_DETACHED; // keep track of whether parent is not adjacent (i.e. due to raycast shortcut)
                    node->flags = (node->flags & (int)~DT_NODE_PARENT_DETACHED) | prevRay; // and store it in the reversed path's node
                    prevRay = nextRay;
                    node = next;
                } while (node != null);

                // Store path
                node = prev;
                float* normal = stackalloc float[3];
                do
                {
                    dtNode* next = m_nodePool.getNodeAtIdx(node->pidx);
                    uint status = 0;
                    if ((node->flags & (int)DT_NODE_PARENT_DETACHED) != 0)
                    {
                        float t;
                        int m;
                        status = raycast(node->id, node->pos, next->pos, m_query.filter, out t, normal, path + n, out m, maxPath - n);
                        n += m;
                        // raycast ends on poly boundary and the path might include the next poly boundary.
                        if (path[n - 1] == next->id)
                            n--; // remove to avoid duplicates
                    }
                    else
                    {
                        path[n++] = node->id;
                        if (n >= maxPath)
                            status = DT_BUFFER_TOO_SMALL;
                    }

                    if ((status & DT_STATUS_DETAIL_MASK) != 0)
                    {
                        m_query.status |= status & DT_STATUS_DETAIL_MASK;
                        break;
                    }
                    node = next;
                } while (node != null);
            }

            dtStatus details = m_query.status & DT_STATUS_DETAIL_MASK;

            // Reset query.
            m_query.Reset();

            pathCount = n;

            return DT_SUCCESS | details;
        }

        /// Finalizes and returns the results of an incomplete sliced path query, returning the path to the furthest
        /// polygon on the existing path that was visited during the search.
        ///  @param[in]        existing        An array of polygon references for the existing path.
        ///  @param[in]        existingSize    The number of polygon in the @p existing array.
        ///  @param[out]    path            An ordered list of polygon references representing the path. (Start to end.) 
        ///                                  [(polyRef) * @p pathCount]
        ///  @param[out]    pathCount        The number of polygons returned in the @p path array.
        ///  @param[in]        maxPath            The max number of polygons the @p path array can hold. [Limit: >= 1]
        /// @returns The status flags for the query.
        public dtStatus finalizeSlicedFindPathPartial(dtPolyRef* existing, int existingSize,
            dtPolyRef* path, out int pathCount, int maxPath)
        {
            pathCount = 0;

            if (existing == null || existingSize <= 0 || path == null || maxPath <= 0)
                return DT_FAILURE | DT_INVALID_PARAM;

            if (dtStatusFailed(m_query.status))
            {
                // Reset query.
                m_query.Reset();
                return DT_FAILURE;
            }

            int n = 0;

            if (m_query.startRef == m_query.endRef)
            {
                // Special case: the search starts and ends at same poly.
                path[n++] = m_query.startRef;
            }
            else
            {
                // Find furthest existing node that was visited.
                dtNode* prev = null;
                dtNode* node = null;
                for (int i = existingSize - 1; i >= 0; --i)
                {
                    m_nodePool!.findNodes(existing[i], &node, 1);
                    if (node != null)
                        break;
                }

                if (node == null)
                {
                    m_query.status |= DT_PARTIAL_RESULT;
                    dtAssert(m_query.lastBestNode != null);
                    node = m_query.lastBestNode;
                }

                // Reverse the path.
                int prevRay = 0;
                do
                {
                    dtNode* next = m_nodePool!.getNodeAtIdx(node->pidx);
                    node->pidx = m_nodePool.getNodeIdx(prev);
                    prev = node;
                    int nextRay = node->flags & (int)DT_NODE_PARENT_DETACHED; // keep track of whether parent is not adjacent (i.e. due to raycast shortcut)
                    node->flags = (node->flags & (int)~DT_NODE_PARENT_DETACHED) | prevRay; // and store it in the reversed path's node
                    prevRay = nextRay;
                    node = next;
                } while (node != null);

                // Store path
                node = prev;
                float* normal = stackalloc float[3];
                do
                {
                    dtNode* next = m_nodePool.getNodeAtIdx(node->pidx);
                    uint status = 0;
                    if ((node->flags & (int)DT_NODE_PARENT_DETACHED) != 0)
                    {
                        float t;
                        int m;
                        status = raycast(node->id, node->pos, next->pos, m_query.filter, out t, normal, path + n, out m, maxPath - n);
                        n += m;
                        // raycast ends on poly boundary and the path might include the next poly boundary.
                        if (path[n - 1] == next->id)
                            n--; // remove to avoid duplicates
                    }
                    else
                    {
                        path[n++] = node->id;
                        if (n >= maxPath)
                            status = DT_BUFFER_TOO_SMALL;
                    }

                    if ((status & DT_STATUS_DETAIL_MASK) != 0)
                    {
                        m_query.status |= status & DT_STATUS_DETAIL_MASK;
                        break;
                    }
                    node = next;
                } while (node != null);
            }

            dtStatus details = m_query.status & DT_STATUS_DETAIL_MASK;

            // Reset query.
            m_query.Reset();

            pathCount = n;

            return DT_SUCCESS | details;
        }

        /// @name Dijkstra Search Functions

        /// Finds the polygons along the navigation graph that touch the specified circle.
        ///  @param[in]        startRef        The reference id of the polygon where the search starts.
        ///  @param[in]        centerPos        The center of the search circle. [(x, y, z)]
        ///  @param[in]        radius            The radius of the search circle.
        ///  @param[in]        filter            The polygon filter to apply to the query.
        ///  @param[out]    resultRef        The reference ids of the polygons touched by the circle. [opt]
        ///  @param[out]    resultParent    The reference ids of the parent polygons for each result. 
        ///                                  Zero if a result polygon has no parent. [opt]
        ///  @param[out]    resultCost        The search cost from @p centerPos to the polygon. [opt]
        ///  @param[out]    resultCount        The number of polygons found. [opt]
        ///  @param[in]        maxResult        The maximum number of polygons the result arrays can hold.
        /// @returns The status flags for the query.

        /// @par
        ///
        /// At least one result array must be provided.
        ///
        /// The order of the result set is from least to highest cost to reach the polygon.
        ///
        /// A common use case for this method is to perform Dijkstra searches. 
        /// Candidate polygons are found by searching the graph beginning at the start polygon.
        ///
        /// If a polygon is not found via the graph search, even if it intersects the 
        /// search circle, it will not be included in the result set. For example:
        ///
        /// polyA is the start polygon.
        /// polyB shares an edge with polyA. (Is adjacent.)
        /// polyC shares an edge with polyB, but not with polyA
        /// Even if the search circle overlaps polyC, it will not be included in the 
        /// result set unless polyB is also in the set.
        /// 
        /// The value of the center point is used as the start position for cost 
        /// calculations. It is not projected onto the surface of the mesh, so its 
        /// y-value will effect the costs.
        ///
        /// Intersection tests occur in 2D. All polygons and the search circle are 
        /// projected onto the xz-plane. So the y-value of the center point does not 
        /// effect intersection tests.
        ///
        /// If the result arrays are to small to hold the entire result set, they will be 
        /// filled to capacity.
        /// 
        public dtStatus findPolysAroundCircle(dtPolyRef startRef, float* centerPos, float radius,
            dtQueryFilter filter,
            dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
            out int resultCount, int maxResult)
        {
            dtAssert(m_nav != null);
            dtAssert(m_nodePool != null);
            dtAssert(m_openList != null);

            resultCount = 0;

            if (!m_nav!.isValidPolyRef(startRef) ||
                centerPos == null || !dtVisfinite(centerPos) ||
                radius < 0F || !float.IsFinite(radius) ||
                filter == null || maxResult < 0)
            {
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            m_nodePool!.clear();
            m_openList!.clear();

            dtNode* startNode = m_nodePool.getNode(startRef);
            DetourCommon.dtVcopy(startNode->pos, centerPos);
            startNode->pidx = 0;
            startNode->cost = 0F;
            startNode->total = 0F;
            startNode->id = startRef;
            startNode->flags = (int)DT_NODE_OPEN;
            m_openList.push(startNode);

            dtStatus status = DT_SUCCESS;

            int n = 0;

            float radiusSqr = radius * radius;

            float* va = stackalloc float[3];
            float* vb = stackalloc float[3];
            while (!m_openList.empty())
            {
                dtNode* bestNode = m_openList.pop();
                bestNode->flags &= (int)~DT_NODE_OPEN;
                bestNode->flags |= (int)DT_NODE_CLOSED;

                // Get poly and tile.
                // The API input has been cheked already, skip checking internal data.
                dtPolyRef bestRef = bestNode->id;
                dtMeshTile* bestTile = null;
                dtPoly* bestPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(bestRef, out bestTile, out bestPoly);

                // Get parent poly and tile.
                dtPolyRef parentRef = 0;
                dtMeshTile* parentTile = null;
                dtPoly* parentPoly = null;
                if (bestNode->pidx != 0)
                    parentRef = m_nodePool.getNodeAtIdx(bestNode->pidx)->id;
                if (parentRef != 0)
                    m_nav.getTileAndPolyByRefUnsafe(parentRef, out parentTile, out parentPoly);

                if (n < maxResult)
                {
                    if (resultRef != null)
                        resultRef[n] = bestRef;
                    if (resultParent != null)
                        resultParent[n] = parentRef;
                    if (resultCost != null)
                        resultCost[n] = bestNode->total;
                    ++n;
                }
                else
                {
                    status |= DT_BUFFER_TOO_SMALL;
                }

                for (uint i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
                {
                    dtLink* link = &bestTile->links[i];
                    dtPolyRef neighbourRef = link->@ref;
                    // Skip invalid neighbours and do not follow back to parent.
                    if (neighbourRef == 0 || neighbourRef == parentRef)
                        continue;

                    // Expand to neighbour
                    dtMeshTile* neighbourTile = null;
                    dtPoly* neighbourPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, out neighbourTile, out neighbourPoly);

                    // Do not advance if the polygon is excluded by the filter.
                    if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
                        continue;

                    // Find edge and calc distance to the edge.
                    if (getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb) == 0)
                        continue;

                    // If the circle is not touching the next polygon, skip it.
                    float tseg = 0;
                    float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, ref tseg);
                    if (distSqr > radiusSqr)
                        continue;

                    dtNode* neighbourNode = m_nodePool.getNode(neighbourRef);
                    if (neighbourNode == null)
                    {
                        status |= DT_OUT_OF_NODES;
                        continue;
                    }

                    if ((neighbourNode->flags & (int)DT_NODE_CLOSED) != 0)
                        continue;

                    // Cost
                    if (neighbourNode->flags == 0)
                        dtVlerp(neighbourNode->pos, va, vb, 0.5f);

                    float cost = filter.getCost(
                        bestNode->pos, neighbourNode->pos,
                        parentRef, parentTile, parentPoly,
                        bestRef, bestTile, bestPoly,
                        neighbourRef, neighbourTile, neighbourPoly);

                    float total = bestNode->total + cost;

                    // The node is already in open list and the new result is worse, skip.
                    if ((neighbourNode->flags & (int)DT_NODE_OPEN) != 0 && total >= neighbourNode->total)
                        continue;

                    neighbourNode->id = neighbourRef;
                    neighbourNode->pidx = m_nodePool.getNodeIdx(bestNode);
                    neighbourNode->total = total;

                    if ((neighbourNode->flags & (int)DT_NODE_OPEN) != 0)
                    {
                        m_openList.modify(neighbourNode);
                    }
                    else
                    {
                        neighbourNode->flags = (int)DT_NODE_OPEN;
                        m_openList.push(neighbourNode);
                    }
                }
            }

            resultCount = n;

            return status;
        }

        /// Finds the polygons along the naviation graph that touch the specified convex polygon.
        ///  @param[in]        startRef        The reference id of the polygon where the search starts.
        ///  @param[in]        verts            The vertices describing the convex polygon. (CCW) 
        ///                                  [(x, y, z) * @p nverts]
        ///  @param[in]        nverts            The number of vertices in the polygon.
        ///  @param[in]        filter            The polygon filter to apply to the query.
        ///  @param[out]    resultRef        The reference ids of the polygons touched by the search polygon. [opt]
        ///  @param[out]    resultParent    The reference ids of the parent polygons for each result. Zero if a 
        ///                                  result polygon has no parent. [opt]
        ///  @param[out]    resultCost        The search cost from the centroid point to the polygon. [opt]
        ///  @param[out]    resultCount        The number of polygons found.
        ///  @param[in]        maxResult        The maximum number of polygons the result arrays can hold.
        /// @returns The status flags for the query.

        /// @par
        ///
        /// The order of the result set is from least to highest cost.
        /// 
        /// At least one result array must be provided.
        ///
        /// A common use case for this method is to perform Dijkstra searches. 
        /// Candidate polygons are found by searching the graph beginning at the start 
        /// polygon.
        /// 
        /// The same intersection test restrictions that apply to findPolysAroundCircle()
        /// method apply to this method.
        /// 
        /// The 3D centroid of the search polygon is used as the start position for cost 
        /// calculations.
        /// 
        /// Intersection tests occur in 2D. All polygons are projected onto the 
        /// xz-plane. So the y-values of the vertices do not effect intersection tests.
        /// 
        /// If the result arrays are is too small to hold the entire result set, they will 
        /// be filled to capacity.
        ///
        public uint findPolysAroundShape(dtPolyRef startRef, float* verts, int nverts,
            dtQueryFilter filter,
            dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
            out int resultCount, int maxResult)
        {
            dtAssert(m_nav != null);
            dtAssert(m_nodePool != null);
            dtAssert(m_openList != null);

            resultCount = 0;

            if (!m_nav!.isValidPolyRef(startRef) ||
                verts == null || nverts < 3 ||
                filter == null || maxResult < 0)
            {
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            // Validate input
            if (startRef == 0 || !m_nav.isValidPolyRef(startRef))
                return DT_FAILURE | DT_INVALID_PARAM;

            m_nodePool!.clear();
            m_openList!.clear();

            float* centerPos = stackalloc float[3] { 0F, 0F, 0F };
            for (int i = 0; i < nverts; ++i)
                dtVadd(centerPos, centerPos, &verts[i * 3]);
            dtVscale(centerPos, centerPos, 1.0f / nverts);

            dtNode* startNode = m_nodePool.getNode(startRef);
            dtVcopy(startNode->pos, centerPos);
            startNode->pidx = 0;
            startNode->cost = 0F;
            startNode->total = 0F;
            startNode->id = startRef;
            startNode->flags = (int)DT_NODE_OPEN;
            m_openList.push(startNode);

            dtStatus status = DT_SUCCESS;

            int n = 0;

            float* va = stackalloc float[3];
            float* vb = stackalloc float[3];
            while (!m_openList.empty())
            {
                dtNode* bestNode = m_openList.pop();
                bestNode->flags &= (int)~DT_NODE_OPEN;
                bestNode->flags |= (int)DT_NODE_CLOSED;

                // Get poly and tile.
                // The API input has been cheked already, skip checking internal data.
                dtPolyRef bestRef = bestNode->id;
                dtMeshTile* bestTile = null;
                dtPoly* bestPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(bestRef, out bestTile, out bestPoly);

                // Get parent poly and tile.
                dtPolyRef parentRef = 0;
                dtMeshTile* parentTile = null;
                dtPoly* parentPoly = null;
                if (bestNode->pidx != 0)
                    parentRef = m_nodePool.getNodeAtIdx(bestNode->pidx)->id;
                if (parentRef != 0)
                    m_nav.getTileAndPolyByRefUnsafe(parentRef, out parentTile, out parentPoly);

                if (n < maxResult)
                {
                    if (resultRef != null)
                        resultRef[n] = bestRef;
                    if (resultParent != null)
                        resultParent[n] = parentRef;
                    if (resultCost != null)
                        resultCost[n] = bestNode->total;

                    ++n;
                }
                else
                {
                    status |= DT_BUFFER_TOO_SMALL;
                }

                for (uint i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
                {
                    dtLink* link = &bestTile->links[i];
                    dtPolyRef neighbourRef = link->@ref;
                    // Skip invalid neighbours and do not follow back to parent.
                    if (neighbourRef == 0 || neighbourRef == parentRef)
                        continue;

                    // Expand to neighbour
                    dtMeshTile* neighbourTile = null;
                    dtPoly* neighbourPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, out neighbourTile, out neighbourPoly);

                    // Do not advance if the polygon is excluded by the filter.
                    if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
                        continue;

                    // Find edge and calc distance to the edge.
                    if (getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb) == 0)
                        continue;

                    // If the poly is not touching the edge to the next polygon, skip the connection it.
                    float tmin = 0;
                    float tmax = 0;
                    int segMin = 0;
                    int segMax = 0;
                    if (!dtIntersectSegmentPoly2D(va, vb, verts, nverts, ref tmin, ref tmax, ref segMin, ref segMax))
                        continue;
                    if (tmin > 1.0f || tmax < 0.0f)
                        continue;

                    dtNode* neighbourNode = m_nodePool.getNode(neighbourRef);
                    if (neighbourNode == null)
                    {
                        status |= DT_OUT_OF_NODES;
                        continue;
                    }

                    if ((neighbourNode->flags & (int)DT_NODE_CLOSED) != 0)
                        continue;

                    // Cost
                    if (neighbourNode->flags == 0)
                        dtVlerp(neighbourNode->pos, va, vb, 0.5f);

                    float cost = filter.getCost(
                        bestNode->pos, neighbourNode->pos,
                        parentRef, parentTile, parentPoly,
                        bestRef, bestTile, bestPoly,
                        neighbourRef, neighbourTile, neighbourPoly);

                    float total = bestNode->total + cost;

                    // The node is already in open list and the new result is worse, skip.
                    if ((neighbourNode->flags & (int)DT_NODE_OPEN) != 0 && total >= neighbourNode->total)
                        continue;

                    neighbourNode->id = neighbourRef;
                    neighbourNode->pidx = m_nodePool.getNodeIdx(bestNode);
                    neighbourNode->total = total;

                    if ((neighbourNode->flags & (int)DT_NODE_OPEN) != 0)
                    {
                        m_openList.modify(neighbourNode);
                    }
                    else
                    {
                        neighbourNode->flags = (int)DT_NODE_OPEN;
                        m_openList.push(neighbourNode);
                    }
                }
            }

            resultCount = n;

            return status;
        }

        /// Gets a path from the explored nodes in the previous search.
        ///  @param[in]        endRef        The reference id of the end polygon.
        ///  @param[out]    path        An ordered list of polygon references representing the path. (Start to end.)
        ///                              [(polyRef) * @p pathCount]
        ///  @param[out]    pathCount    The number of polygons returned in the @p path array.
        ///  @param[in]        maxPath        The maximum number of polygons the @p path array can hold. [Limit: >= 0]
        ///  @returns        The status flags. Returns DT_FAILURE | DT_INVALID_PARAM if any parameter is wrong, or if
        ///                  @p endRef was not explored in the previous search. Returns DT_SUCCESS | DT_BUFFER_TOO_SMALL
        ///                  if @p path cannot contain the entire path. In this case it is filled to capacity with a partial path.
        ///                  Otherwise returns DT_SUCCESS.
        ///  @remarks        The result of this function depends on the state of the query object. For that reason it should only
        ///                  be used immediately after one of the two Dijkstra searches, findPolysAroundCircle or findPolysAroundShape.
        public dtStatus getPathFromDijkstraSearch(dtPolyRef endRef, dtPolyRef* path, out int pathCount, int maxPath)
        {
            pathCount = 0;

            if (!m_nav!.isValidPolyRef(endRef) || path == null || maxPath < 0)
                return DT_FAILURE | DT_INVALID_PARAM;

            dtNode* endNode;
            if (m_nodePool!.findNodes(endRef, &endNode, 1) != 1 ||
                (endNode->flags & (int)DT_NODE_CLOSED) == 0)
            {
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            return getPathToNode(endNode, path, out pathCount, maxPath);
        }

        /// @name Local Query Functions

        /// Finds the polygon nearest to the specified center point.
        /// [opt] means the specified parameter can be a null pointer, in that case the output parameter will not be set.
        ///
        ///  @param[in]        center        The center of the search box. [(x, y, z)]
        ///  @param[in]        halfExtents    The search distance along each axis. [(x, y, z)]
        ///  @param[in]        filter        The polygon filter to apply to the query.
        ///  @param[out]    nearestRef    The reference id of the nearest polygon. Will be set to 0 if no polygon is found.
        ///  @param[out]    nearestPt    The nearest point on the polygon. Unchanged if no polygon is found. [opt] [(x, y, z)]
        /// @returns The status flags for the query.

        /// @par 
        ///
        /// @note If the search box does not intersect any polygons the search will 
        /// return #DT_SUCCESS, but @p nearestRef will be zero. So if in doubt, check 
        /// @p nearestRef before using @p nearestPt.
        ///
        public dtStatus findNearestPoly(float* center, float* halfExtents,
            dtQueryFilter filter,
            out dtPolyRef nearestRef, float* nearestPt)
        {
            return findNearestPoly(center, halfExtents, filter, out nearestRef, nearestPt, out _);
        }

        /// Finds the polygon nearest to the specified center point.
        /// [opt] means the specified parameter can be a null pointer, in that case the output parameter will not be set.
        /// 
        ///  @param[in]        center        The center of the search box. [(x, y, z)]
        ///  @param[in]        halfExtents    The search distance along each axis. [(x, y, z)]
        ///  @param[in]        filter        The polygon filter to apply to the query.
        ///  @param[out]    nearestRef    The reference id of the nearest polygon. Will be set to 0 if no polygon is found.
        ///  @param[out]    nearestPt    The nearest point on the polygon. Unchanged if no polygon is found. [opt] [(x, y, z)]
        ///  @param[out]    isOverPoly     Set to true if the point's X/Z coordinate lies inside the polygon, false otherwise. Unchanged if no polygon is found. [opt]
        /// @returns The status flags for the query.

        // If center and nearestPt point to an equal position, isOverPoly will be true;
        // however there's also a special case of climb height inside the polygon (see dtFindNearestPolyQuery)
        public dtStatus findNearestPoly(float* center, float* halfExtents,
            dtQueryFilter filter,
            out dtPolyRef nearestRef, float* nearestPt, out bool isOverPoly)
        {
            dtAssert(m_nav != null);

            nearestRef = 0;
            isOverPoly = false;
            // queryPolygons below will check rest of params

            dtFindNearestPolyQuery query = new dtFindNearestPolyQuery(this, center);

            dtStatus status = queryPolygons(center, halfExtents, filter, query);
            if (dtStatusFailed(status))
                return status;

            nearestRef = query.nearestRef();
            // Only override nearestPt if we actually found a poly so the nearest point
            // is valid.
            if (nearestPt != null && nearestRef != 0)
            {
                dtVcopy(new Span<float>(nearestPt, 3), query.nearestPoint());
                isOverPoly = query.isOverPoly();
            }

            return DT_SUCCESS;
        }

        /// Finds polygons that overlap the search box.
        ///  @param[in]        center        The center of the search box. [(x, y, z)]
        ///  @param[in]        halfExtents        The search distance along each axis. [(x, y, z)]
        ///  @param[in]        filter        The polygon filter to apply to the query.
        ///  @param[out]    polys        The reference ids of the polygons that overlap the query box.
        ///  @param[out]    polyCount    The number of polygons in the search result.
        ///  @param[in]        maxPolys    The maximum number of polygons the search result can hold.
        /// @returns The status flags for the query.

        /// @par 
        ///
        /// If no polygons are found, the function will return #DT_SUCCESS with a
        /// @p polyCount of zero.
        ///
        /// If @p polys is too small to hold the entire result set, then the array will 
        /// be filled to capacity. The method of choosing which polygons from the 
        /// full set are included in the partial result set is undefined.
        ///
        public dtStatus queryPolygons(float* center, float* halfExtents,
            dtQueryFilter filter,
            dtPolyRef* polys, out int polyCount, int maxPolys)
        {
            polyCount = 0;

            if (polys == null || maxPolys < 0)
                return DT_FAILURE | DT_INVALID_PARAM;

            dtCollectPolysQuery collector = new dtCollectPolysQuery(polys, maxPolys);

            uint status = queryPolygons(center, halfExtents, filter, collector);
            if (dtStatusFailed(status))
                return status;

            polyCount = collector.numCollected();
            return collector.overflowed() ? DT_SUCCESS | DT_BUFFER_TOO_SMALL : DT_SUCCESS;
        }

        /// Finds polygons that overlap the search box.
        ///  @param[in]        center        The center of the search box. [(x, y, z)]
        ///  @param[in]        halfExtents        The search distance along each axis. [(x, y, z)]
        ///  @param[in]        filter        The polygon filter to apply to the query.
        ///  @param[in]        query        The query. Polygons found will be batched together and passed to this query.

        /// @par 
        ///
        /// The query will be invoked with batches of polygons. Polygons passed
        /// to the query have bounding boxes that overlap with the center and halfExtents
        /// passed to this function. The dtPolyQuery::process function is invoked multiple
        /// times until all overlapping polygons have been processed.
        ///
        public dtStatus queryPolygons(float* center, float* halfExtents,
            dtQueryFilter filter, dtPolyQuery query)
        {
            dtAssert(m_nav != null);

            if (center == null || !dtVisfinite(center) ||
                halfExtents == null || !dtVisfinite(halfExtents) ||
                filter == null || query == null)
            {
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            float* bmin = stackalloc float[3];
            float* bmax = stackalloc float[3];
            dtVsub(bmin, center, halfExtents);
            dtVadd(bmax, center, halfExtents);

            // Find tiles the query touches.
            int minx = 0;
            int miny = 0;
            int maxx = 0;
            int maxy = 0;
            m_nav!.calcTileLoc(bmin, ref minx, ref miny);
            m_nav.calcTileLoc(bmax, ref maxx, ref maxy);

            const int MAX_NEIS = 32;
            dtMeshTile** neis = stackalloc dtMeshTile*[MAX_NEIS];

            for (int y = miny; y <= maxy; ++y)
            {
                for (int x = minx; x <= maxx; ++x)
                {
                    int nneis = m_nav.getTilesAt(x, y, neis, MAX_NEIS);
                    for (int j = 0; j < nneis; ++j)
                    {
                        queryPolygonsInTile(neis[j], bmin, bmax, filter, query);
                    }
                }
            }

            return DT_SUCCESS;
        }

        /// Finds the non-overlapping navigation polygons in the local neighbourhood around the center position.
        ///  @param[in]        startRef        The reference id of the polygon where the search starts.
        ///  @param[in]        centerPos        The center of the query circle. [(x, y, z)]
        ///  @param[in]        radius            The radius of the query circle.
        ///  @param[in]        filter            The polygon filter to apply to the query.
        ///  @param[out]    resultRef        The reference ids of the polygons touched by the circle.
        ///  @param[out]    resultParent    The reference ids of the parent polygons for each result. 
        ///                                  Zero if a result polygon has no parent. [opt]
        ///  @param[out]    resultCount        The number of polygons found.
        ///  @param[in]        maxResult        The maximum number of polygons the result arrays can hold.
        /// @returns The status flags for the query.

        /// @par
        ///
        /// This method is optimized for a small search radius and small number of result 
        /// polygons.
        ///
        /// Candidate polygons are found by searching the navigation graph beginning at 
        /// the start polygon.
        ///
        /// The same intersection test restrictions that apply to the findPolysAroundCircle 
        /// mehtod applies to this method.
        ///
        /// The value of the center point is used as the start point for cost calculations. 
        /// It is not projected onto the surface of the mesh, so its y-value will effect 
        /// the costs.
        /// 
        /// Intersection tests occur in 2D. All polygons and the search circle are 
        /// projected onto the xz-plane. So the y-value of the center point does not 
        /// effect intersection tests.
        /// 
        /// If the result arrays are is too small to hold the entire result set, they will 
        /// be filled to capacity.
        /// 
        public uint findLocalNeighbourhood(dtPolyRef startRef, float* centerPos, float radius,
            dtQueryFilter? filter,
            dtPolyRef* resultRef, dtPolyRef* resultParent, out int resultCount, int maxResult)
        {
            dtAssert(m_nav != null);
            dtAssert(m_tinyNodePool != null);

            resultCount = 0;

            if (!m_nav!.isValidPolyRef(startRef) ||
                centerPos == null || !dtVisfinite(centerPos) ||
                radius < 0F || !float.IsFinite(radius) ||
                filter == null || maxResult < 0)
            {
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            const int MAX_STACK = 48;
            dtNode** stack = stackalloc dtNode*[MAX_STACK];
            int nstack = 0;

            m_tinyNodePool!.clear();

            dtNode* startNode = m_tinyNodePool.getNode(startRef);
            startNode->pidx = 0;
            startNode->id = startRef;
            startNode->flags = (int)DT_NODE_CLOSED;
            stack[nstack++] = startNode;

            float radiusSqr = radius * radius;

            float* pa = stackalloc float[DT_VERTS_PER_POLYGON * 3];
            float* pb = stackalloc float[DT_VERTS_PER_POLYGON * 3];

            uint status = DT_SUCCESS;

            int n = 0;
            if (n < maxResult)
            {
                resultRef[n] = startNode->id;
                if (resultParent != null)
                    resultParent[n] = 0;
                ++n;
            }
            else
            {
                status |= DT_BUFFER_TOO_SMALL;
            }

            float* va = stackalloc float[3];
            float* vb = stackalloc float[3];

            while (nstack != 0)
            {
                // Pop front.
                dtNode* curNode = stack[0];
                for (int i = 0; i < nstack - 1; ++i)
                    stack[i] = stack[i + 1];
                nstack--;

                // Get poly and tile.
                // The API input has been cheked already, skip checking internal data.
                dtPolyRef curRef = curNode->id;
                dtMeshTile* curTile = null;
                dtPoly* curPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(curRef, out curTile, out curPoly);

                for (uint i = curPoly->firstLink; i != DT_NULL_LINK; i = curTile->links[i].next)
                {
                    dtLink* link = &curTile->links[i];
                    dtPolyRef neighbourRef = link->@ref;
                    // Skip invalid neighbours.
                    if (neighbourRef == 0)
                        continue;

                    // Skip if cannot alloca more nodes.
                    dtNode* neighbourNode = m_tinyNodePool.getNode(neighbourRef);
                    if (neighbourNode == null)
                        continue;
                    // Skip visited.
                    if ((neighbourNode->flags & (int)DT_NODE_CLOSED) != 0)
                        continue;

                    // Expand to neighbour
                    dtMeshTile* neighbourTile = null;
                    dtPoly* neighbourPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, out neighbourTile, out neighbourPoly);

                    // Skip off-mesh connections.
                    if (neighbourPoly->getType() == (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
                        continue;

                    // Do not advance if the polygon is excluded by the filter.
                    if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
                        continue;

                    // Find edge and calc distance to the edge.
                    if (getPortalPoints(curRef, curPoly, curTile, neighbourRef, neighbourPoly, neighbourTile, va, vb) == 0)
                        continue;

                    // If the circle is not touching the next polygon, skip it.
                    float tseg = 0;
                    float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, ref tseg);
                    if (distSqr > radiusSqr)
                        continue;

                    // Mark node visited, this is done before the overlap test so that
                    // we will not visit the poly again if the test fails.
                    neighbourNode->flags |= (int)DT_NODE_CLOSED;
                    neighbourNode->pidx = m_tinyNodePool.getNodeIdx(curNode);

                    // Check that the polygon does not collide with existing polygons.

                    // Collect vertices of the neighbour poly.
                    int npa = neighbourPoly->vertCount;
                    for (int k = 0; k < npa; ++k)
                        dtVcopy(&pa[k * 3], &neighbourTile->verts[neighbourPoly->verts[k] * 3]);

                    bool overlap = false;
                    for (int j = 0; j < n; ++j)
                    {
                        dtPolyRef pastRef = resultRef[j];

                        // Connected polys do not overlap.
                        bool connected = false;
                        for (uint k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
                        {
                            if (curTile->links[k].@ref == pastRef)
                            {
                                connected = true;
                                break;
                            }
                        }
                        if (connected)
                            continue;

                        // Potentially overlapping.
                        dtMeshTile* pastTile = null;
                        dtPoly* pastPoly = null;
                        m_nav.getTileAndPolyByRefUnsafe(pastRef, out pastTile, out pastPoly);

                        // Get vertices and test overlap
                        int npb = pastPoly->vertCount;
                        for (int k = 0; k < npb; ++k)
                            dtVcopy(&pb[k * 3], &pastTile->verts[pastPoly->verts[k] * 3]);

                        if (dtOverlapPolyPoly2D(pa, npa, pb, npb))
                        {
                            overlap = true;
                            break;
                        }
                    }
                    if (overlap)
                        continue;

                    // This poly is fine, store and advance to the poly.
                    if (n < maxResult)
                    {
                        resultRef[n] = neighbourRef;
                        if (resultParent != null)
                            resultParent[n] = curRef;
                        ++n;
                    }
                    else
                    {
                        status |= DT_BUFFER_TOO_SMALL;
                    }

                    if (nstack < MAX_STACK)
                    {
                        stack[nstack++] = neighbourNode;
                    }
                }
            }

            resultCount = n;

            return status;
        }

        /// Moves from the start to the end position constrained to the navigation mesh.
        ///  @param[in]        startRef        The reference id of the start polygon.
        ///  @param[in]        startPos        A position of the mover within the start polygon. [(x, y, x)]
        ///  @param[in]        endPos            The desired end position of the mover. [(x, y, z)]
        ///  @param[in]        filter            The polygon filter to apply to the query.
        ///  @param[out]    resultPos        The result position of the mover. [(x, y, z)]
        ///  @param[out]    visited            The reference ids of the polygons visited during the move.
        ///  @param[out]    visitedCount    The number of polygons visited during the move.
        ///  @param[in]        maxVisitedSize    The maximum number of polygons the @p visited array can hold.
        /// @returns The status flags for the query.

        /// @par
        ///
        /// This method is optimized for small delta movement and a small number of 
        /// polygons. If used for too great a distance, the result set will form an 
        /// incomplete path.
        ///
        /// @p resultPos will equal the @p endPos if the end is reached. 
        /// Otherwise the closest reachable position will be returned.
        /// 
        /// @p resultPos is not projected onto the surface of the navigation 
        /// mesh. Use #getPolyHeight if this is needed.
        ///
        /// This method treats the end position in the same manner as 
        /// the #raycast method. (As a 2D point.) See that method's documentation 
        /// for details.
        /// 
        /// If the @p visited array is too small to hold the entire result set, it will 
        /// be filled as far as possible from the start position toward the end 
        /// position.
        ///
        public dtStatus moveAlongSurface(dtPolyRef startRef, float* startPos, float* endPos,
            dtQueryFilter? filter,
            float* resultPos, dtPolyRef* visited, out int visitedCount, int maxVisitedSize)
        {
            dtAssert(m_nav != null);
            dtAssert(m_tinyNodePool != null);

            visitedCount = 0;

            if (!m_nav!.isValidPolyRef(startRef) ||
                startPos == null || !dtVisfinite(startPos) ||
                endPos == null || !dtVisfinite(endPos) ||
                filter == null || resultPos == null || visited == null ||
                maxVisitedSize <= 0)
            {
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            dtStatus status = DT_SUCCESS;

            const int MAX_STACK = 48;
            dtNode** stack = stackalloc dtNode*[MAX_STACK];
            int nstack = 0;

            m_tinyNodePool!.clear();

            dtNode* startNode = m_tinyNodePool.getNode(startRef);
            startNode->pidx = 0;
            startNode->cost = 0F;
            startNode->total = 0F;
            startNode->id = startRef;
            startNode->flags = (int)DT_NODE_CLOSED;
            stack[nstack++] = startNode;

            float* bestPos = stackalloc float[3];
            float bestDist = float.MaxValue;
            dtNode* bestNode = null;
            dtVcopy(bestPos, startPos);

            // Search constraints
            float* searchPos = stackalloc float[3];
            float searchRadSqr;
            dtVlerp(searchPos, startPos, endPos, 0.5f);
            searchRadSqr = (dtVdist(startPos, endPos) / 2.0f + 0.001f) * (dtVdist(startPos, endPos) / 2.0f + 0.001f);

            float* verts = stackalloc float[DT_VERTS_PER_POLYGON * 3];
            const int MAX_NEIS = 8;
            dtPolyRef* neis = stackalloc dtPolyRef[MAX_NEIS];

            while (nstack != 0)
            {
                // Pop front.
                dtNode* curNode = stack[0];
                for (int i = 0; i < nstack - 1; ++i)
                    stack[i] = stack[i + 1];
                nstack--;

                // Get poly and tile.
                // The API input has been cheked already, skip checking internal data.
                dtPolyRef curRef = curNode->id;
                dtMeshTile* curTile = null;
                dtPoly* curPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(curRef, out curTile, out curPoly);

                // Collect vertices.
                int nverts = curPoly->vertCount;
                for (int i = 0; i < nverts; ++i)
                    dtVcopy(&verts[i * 3], &curTile->verts[curPoly->verts[i] * 3]);

                // If target is inside the poly, stop search.
                if (dtPointInPolygon(endPos, verts, nverts))
                {
                    bestNode = curNode;
                    dtVcopy(bestPos, endPos);
                    break;
                }

                // Find wall edges and find nearest point inside the walls.
                for (int i = 0, j = (int)curPoly->vertCount - 1; i < (int)curPoly->vertCount; j = i++)
                {
                    // Find links to neighbours.
                    int nneis = 0;

                    if ((curPoly->neis[j] & DT_EXT_LINK) != 0)
                    {
                        // Tile border.
                        for (uint k = curPoly->firstLink; k != DetourCommon.DT_NULL_LINK; k = curTile->links[k].next)
                        {
                            dtLink* link = &curTile->links[k];
                            if (link->edge == j)
                            {
                                if (link->@ref != 0)
                                {
                                    dtMeshTile* neiTile = null;
                                    dtPoly* neiPoly = null;
                                    m_nav.getTileAndPolyByRefUnsafe(link->@ref, out neiTile, out neiPoly);
                                    if (filter.passFilter(link->@ref, neiTile, neiPoly))
                                    {
                                        if (nneis < MAX_NEIS)
                                            neis[nneis++] = link->@ref;
                                    }
                                }
                            }
                        }
                    }
                    else if (curPoly->neis[j] != 0)
                    {
                        uint idx = (uint)(curPoly->neis[j] - 1);
                        dtPolyRef @ref = m_nav.getPolyRefBase(curTile) | idx;
                        if (filter.passFilter(@ref, curTile, &curTile->polys[idx]))
                        {
                            // Internal edge, encode id.
                            neis[nneis++] = @ref;
                        }
                    }

                    if (nneis == 0)
                    {
                        // Wall edge, calc distance.
                        float* vj = &verts[j * 3];
                        float* vi = &verts[i * 3];
                        float tseg = 0;
                        float distSqr = dtDistancePtSegSqr2D(endPos, vj, vi, ref tseg);
                        if (distSqr < bestDist)
                        {
                            // Update nearest distance.
                            dtVlerp(bestPos, vj, vi, tseg);
                            bestDist = distSqr;
                            bestNode = curNode;
                        }
                    }
                    else
                    {
                        for (int k = 0; k < nneis; ++k)
                        {
                            // Skip if no node can be allocated.
                            dtNode* neighbourNode = m_tinyNodePool.getNode(neis[k]);
                            if (neighbourNode == null)
                                continue;
                            // Skip if already visited.
                            if ((neighbourNode->flags & (int)DT_NODE_CLOSED) != 0)
                                continue;

                            // Skip the link if it is too far from search constraint.
                            // TODO: Maybe should use getPortalPoints(), but this one is way faster.
                            float* vj = &verts[j * 3];
                            float* vi = &verts[i * 3];
                            float tseg = 0;
                            float distSqr = dtDistancePtSegSqr2D(searchPos, vj, vi, ref tseg);
                            if (distSqr > searchRadSqr)
                                continue;

                            // Mark as the node as visited and push to queue.
                            if (nstack < MAX_STACK)
                            {
                                neighbourNode->pidx = m_tinyNodePool.getNodeIdx(curNode);
                                neighbourNode->flags |= (int)DT_NODE_CLOSED;
                                stack[nstack++] = neighbourNode;
                            }
                        }
                    }
                }
            }

            int n = 0;
            if (bestNode != null)
            {
                // Reverse the path.
                dtNode* prev = null;
                dtNode* node = bestNode;
                do
                {
                    dtNode* next = m_tinyNodePool.getNodeAtIdx(node->pidx);
                    node->pidx = m_tinyNodePool.getNodeIdx(prev);
                    prev = node;
                    node = next;
                } while (node != null);

                // Store result
                node = prev;
                do
                {
                    visited[n++] = node->id;
                    if (n >= maxVisitedSize)
                    {
                        status |= DT_BUFFER_TOO_SMALL;
                        break;
                    }
                    node = m_tinyNodePool.getNodeAtIdx(node->pidx);
                } while (node != null);
            }

            dtVcopy(resultPos, bestPos);

            visitedCount = n;

            return status;
        }

        /// Casts a 'walkability' ray along the surface of the navigation mesh from 
        /// the start position toward the end position.
        /// @note A wrapper around raycast(..., RaycastHit*). Retained for backward compatibility.
        ///  @param[in]        startRef    The reference id of the start polygon.
        ///  @param[in]        startPos    A position within the start polygon representing 
        ///                              the start of the ray. [(x, y, z)]
        ///  @param[in]        endPos        The position to cast the ray toward. [(x, y, z)]
        ///  @param[out]    t            The hit parameter. (FLT_MAX if no wall hit.)
        ///  @param[out]    hitNormal    The normal of the nearest wall hit. [(x, y, z)]
        ///  @param[in]        filter        The polygon filter to apply to the query.
        ///  @param[out]    path        The reference ids of the visited polygons. [opt]
        ///  @param[out]    pathCount    The number of visited polygons. [opt]
        ///  @param[in]        maxPath        The maximum number of polygons the @p path array can hold.
        /// @returns The status flags for the query.

        /// @par
        ///
        /// This method is meant to be used for quick, short distance checks.
        ///
        /// If the path array is too small to hold the result, it will be filled as 
        /// far as possible from the start postion toward the end position.
        ///
        /// <b>Using the Hit Parameter (t)</b>
        /// 
        /// If the hit parameter is a very high value (FLT_MAX), then the ray has hit 
        /// the end position. In this case the path represents a valid corridor to the 
        /// end position and the value of @p hitNormal is undefined.
        ///
        /// If the hit parameter is zero, then the start position is on the wall that 
        /// was hit and the value of @p hitNormal is undefined.
        ///
        /// If 0 < t < 1.0 then the following applies:
        ///
        /// @code
        /// distanceToHitBorder = distanceToEndPosition * t
        /// hitPoint = startPos + (endPos - startPos) * t
        /// @endcode
        ///
        /// <b>Use Case Restriction</b>
        ///
        /// The raycast ignores the y-value of the end position. (2D check.) This 
        /// places significant limits on how it can be used. For example:
        ///
        /// Consider a scene where there is a main floor with a second floor balcony 
        /// that hangs over the main floor. So the first floor mesh extends below the 
        /// balcony mesh. The start position is somewhere on the first floor. The end 
        /// position is on the balcony.
        ///
        /// The raycast will search toward the end position along the first floor mesh. 
        /// If it reaches the end position's xz-coordinates it will indicate FLT_MAX
        /// (no wall hit), meaning it reached the end position. This is one example of why
        /// this method is meant for short distance checks.
        ///
        public dtStatus raycast(dtPolyRef startRef, float* startPos, float* endPos,
            dtQueryFilter? filter,
            out float t, float* hitNormal, dtPolyRef* path, out int pathCount, int maxPath)
        {
            dtRaycastHit hit = new dtRaycastHit();
            hit.path = path;
            hit.maxPath = maxPath;

            dtStatus status = raycast(startRef, startPos, endPos, filter, 0, &hit);

            t = hit.t;
            if (hitNormal != null)
                dtVcopy(hitNormal, hit.hitNormal);
            pathCount = hit.pathCount;

            return status;
        }

        /// Casts a 'walkability' ray along the surface of the navigation mesh from 
        /// the start position toward the end position.
        ///  @param[in]        startRef    The reference id of the start polygon.
        ///  @param[in]        startPos    A position within the start polygon representing 
        ///                              the start of the ray. [(x, y, z)]
        ///  @param[in]        endPos        The position to cast the ray toward. [(x, y, z)]
        ///  @param[in]        filter        The polygon filter to apply to the query.
        ///  @param[in]        flags        govern how the raycast behaves. See dtRaycastOptions
        ///  @param[out]    hit            Pointer to a raycast hit structure which will be filled by the results.
        ///  @param[in]        prevRef        parent of start ref. Used during for cost calculation [opt]
        /// @returns The status flags for the query.

        /// @par
        ///
        /// This method is meant to be used for quick, short distance checks.
        ///
        /// If the path array is too small to hold the result, it will be filled as 
        /// far as possible from the start postion toward the end position.
        ///
        /// <b>Using the Hit Parameter t of RaycastHit</b>
        /// 
        /// If the hit parameter is a very high value (FLT_MAX), then the ray has hit 
        /// the end position. In this case the path represents a valid corridor to the 
        /// end position and the value of @p hitNormal is undefined.
        ///
        /// If the hit parameter is zero, then the start position is on the wall that 
        /// was hit and the value of @p hitNormal is undefined.
        ///
        /// If 0 < t < 1.0 then the following applies:
        ///
        /// @code
        /// distanceToHitBorder = distanceToEndPosition * t
        /// hitPoint = startPos + (endPos - startPos) * t
        /// @endcode
        ///
        /// <b>Use Case Restriction</b>
        ///
        /// The raycast ignores the y-value of the end position. (2D check.) This 
        /// places significant limits on how it can be used. For example:
        ///
        /// Consider a scene where there is a main floor with a second floor balcony 
        /// that hangs over the main floor. So the first floor mesh extends below the 
        /// balcony mesh. The start position is somewhere on the first floor. The end 
        /// position is on the balcony.
        ///
        /// The raycast will search toward the end position along the first floor mesh. 
        /// If it reaches the end position's xz-coordinates it will indicate FLT_MAX
        /// (no wall hit), meaning it reached the end position. This is one example of why
        /// this method is meant for short distance checks.
        ///
        public dtStatus raycast(dtPolyRef startRef, float* startPos, float* endPos,
            dtQueryFilter? filter, uint options, dtRaycastHit* hit, dtPolyRef prevRef = 0)
        {
            dtAssert(m_nav != null);

            if (hit == null)
                return DT_FAILURE | DT_INVALID_PARAM;

            hit->t = 0F;
            hit->pathCount = 0;
            hit->pathCost = 0F;

            // Validate input
            if (!m_nav!.isValidPolyRef(startRef) ||
                startPos == null || !dtVisfinite(startPos) ||
                endPos == null || !dtVisfinite(endPos) ||
                filter == null ||
                (prevRef != 0 && !m_nav.isValidPolyRef(prevRef)))
            {
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            float* dir = stackalloc float[3];
            float* curPos = stackalloc float[3];
            float* lastPos = stackalloc float[3];
            float* verts = stackalloc float[DT_VERTS_PER_POLYGON * 3 + 3];
            int n = 0;

            dtVcopy(curPos, startPos);
            dtVsub(dir, endPos, startPos);
            dtVset(hit->hitNormal, 0F, 0F, 0F);

            uint status = DT_SUCCESS;

            dtMeshTile* prevTile;
            dtMeshTile* tile;
            dtMeshTile* nextTile;
            dtPoly* prevPoly;
            dtPoly* poly;
            dtPoly* nextPoly;
            dtPolyRef curRef = 0;

            // The API input has been checked already, skip checking internal data.
            curRef = startRef;
            tile = null;
            poly = null;
            m_nav.getTileAndPolyByRefUnsafe(curRef, out tile, out poly);
            nextTile = prevTile = tile;
            nextPoly = prevPoly = poly;
            if (prevRef != 0)
            {
                m_nav.getTileAndPolyByRefUnsafe(prevRef, out prevTile, out prevPoly);
            }

            float* eDir = stackalloc float[3];
            float* diff = stackalloc float[3];

            while (curRef != 0)
            {
                // Cast ray against current polygon.

                // Collect vertices.
                int nv = 0;
                for (int i = 0; i < (int)poly->vertCount; ++i)
                {
                    dtVcopy(&verts[nv * 3], &tile->verts[poly->verts[i] * 3]);
                    nv++;
                }

                float tmin = 0;
                float tmax = 0;
                int segMin = 0;
                int segMax = 0;
                if (!dtIntersectSegmentPoly2D(startPos, endPos, verts, nv, ref tmin, ref tmax, ref segMin, ref segMax))
                {
                    // Could not hit the polygon, keep the old t and report hit.
                    hit->pathCount = n;
                    return status;
                }

                hit->hitEdgeIndex = segMax;

                // Keep track of furthest t so far.
                if (tmax > hit->t)
                    hit->t = tmax;

                // Store visited polygons.
                if (n < hit->maxPath)
                    hit->path[n++] = curRef;
                else
                    status |= DT_BUFFER_TOO_SMALL;

                // Ray end is completely inside the polygon.
                if (segMax == -1)
                {
                    hit->t = float.MaxValue;
                    hit->pathCount = n;

                    // add the cost
                    if ((options & (uint)DT_RAYCAST_USE_COSTS) != 0)
                        hit->pathCost += filter.getCost(curPos, endPos, prevRef, prevTile, prevPoly, curRef, tile, poly, curRef, tile, poly);
                    return status;
                }

                // Follow neighbours.
                dtPolyRef nextRef = 0;

                for (uint i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
                {
                    dtLink* link = &tile->links[i];

                    // Find link which contains this edge.
                    if ((int)link->edge != segMax)
                        continue;

                    // Get pointer to the next polygon.
                    nextTile = null;
                    nextPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(link->@ref, out nextTile, out nextPoly);

                    // Skip off-mesh connections.
                    if (nextPoly->getType() == (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
                        continue;

                    // Skip links based on filter.
                    if (!filter.passFilter(link->@ref, nextTile, nextPoly))
                        continue;

                    // If the link is internal, just return the ref.
                    if (link->side == (byte)0xff)
                    {
                        nextRef = link->@ref;
                        break;
                    }

                    // If the link is at tile boundary,

                    // Check if the link spans the whole edge, and accept.
                    if (link->bmin == 0 && link->bmax == 255)
                    {
                        nextRef = link->@ref;
                        break;
                    }

                    // Check for partial edge links.
                    int v0 = poly->verts[link->edge];
                    int v1 = poly->verts[(link->edge + 1) % poly->vertCount];
                    float* left = &tile->verts[v0 * 3];
                    float* right = &tile->verts[v1 * 3];

                    // Check that the intersection lies inside the link portal.
                    if (link->side == 0 || link->side == 4)
                    {
                        // Calculate link size.
                        float s = 1.0f / 255.0f;
                        float lmin = left[2] + (right[2] - left[2]) * (link->bmin * s);
                        float lmax = left[2] + (right[2] - left[2]) * (link->bmax * s);
                        if (lmin > lmax) dtSwap(ref lmin, ref lmax);

                        // Find Z intersection.
                        float z = startPos[2] + (endPos[2] - startPos[2]) * tmax;
                        if (z >= lmin && z <= lmax)
                        {
                            nextRef = link->@ref;
                            break;
                        }
                    }
                    else if (link->side == 2 || link->side == 6)
                    {
                        // Calculate link size.
                        float s = 1.0f / 255.0f;
                        float lmin = left[0] + (right[0] - left[0]) * (link->bmin * s);
                        float lmax = left[0] + (right[0] - left[0]) * (link->bmax * s);
                        if (lmin > lmax) dtSwap(ref lmin, ref lmax);

                        // Find X intersection.
                        float x = startPos[0] + (endPos[0] - startPos[0]) * tmax;
                        if (x >= lmin && x <= lmax)
                        {
                            nextRef = link->@ref;
                            break;
                        }
                    }
                }

                // add the cost
                if ((options & (uint)DT_RAYCAST_USE_COSTS) != 0)
                {
                    // compute the intersection point at the furthest end of the polygon
                    // and correct the height (since the raycast moves in 2d)
                    dtVcopy(lastPos, curPos);
                    dtVmad(curPos, startPos, dir, hit->t);
                    float* e1 = &verts[segMax * 3];
                    float* e2 = &verts[((segMax + 1) % nv) * 3];
                    dtVsub(eDir, e2, e1);
                    dtVsub(diff, curPos, e1);
                    float s = (eDir[0]) * (eDir[0]) > (eDir[2]) * (eDir[2]) ? diff[0] / eDir[0] : diff[2] / eDir[2];
                    curPos[1] = e1[1] + eDir[1] * s;

                    hit->pathCost += filter.getCost(lastPos, curPos, prevRef, prevTile, prevPoly, curRef, tile, poly, nextRef, nextTile, nextPoly);
                }

                if (nextRef == 0)
                {
                    // No neighbour, we hit a wall.

                    // Calculate hit normal.
                    int a = segMax;
                    int b = segMax + 1 < nv ? segMax + 1 : 0;
                    float* va = &verts[a * 3];
                    float* vb = &verts[b * 3];
                    float dx = vb[0] - va[0];
                    float dz = vb[2] - va[2];
                    hit->hitNormal[0] = dz;
                    hit->hitNormal[1] = 0F;
                    hit->hitNormal[2] = -dx;
                    dtVnormalize(hit->hitNormal);

                    hit->pathCount = n;
                    return status;
                }

                // No hit, advance to neighbour polygon.
                prevRef = curRef;
                curRef = nextRef;
                prevTile = tile;
                tile = nextTile;
                prevPoly = poly;
                poly = nextPoly;
            }

            hit->pathCount = n;

            return status;
        }


        /// Finds the distance from the specified position to the nearest polygon wall.
        ///  @param[in]        startRef        The reference id of the polygon containing @p centerPos.
        ///  @param[in]        centerPos        The center of the search circle. [(x, y, z)]
        ///  @param[in]        maxRadius        The radius of the search circle.
        ///  @param[in]        filter            The polygon filter to apply to the query.
        ///  @param[out]    hitDist            The distance to the nearest wall from @p centerPos.
        ///  @param[out]    hitPos            The nearest position on the wall that was hit. [(x, y, z)]
        ///  @param[out]    hitNormal        The normalized ray formed from the wall point to the 
        ///                                  source point. [(x, y, z)]
        /// @returns The status flags for the query.

        /// @par
        ///
        /// @p hitPos is not adjusted using the height detail data.
        ///
        /// @p hitDist will equal the search radius if there is no wall within the 
        /// radius. In this case the values of @p hitPos and @p hitNormal are
        /// undefined.
        ///
        /// The normal will become unpredicable if @p hitDist is a very small number.
        ///
        public dtStatus findDistanceToWall(dtPolyRef startRef, float* centerPos, float maxRadius,
            dtQueryFilter? filter,
            out float hitDist, float* hitPos, float* hitNormal)
        {
            dtAssert(m_nav != null);
            dtAssert(m_nodePool != null);
            dtAssert(m_openList != null);

            hitDist = 0;

            // Validate input
            if (!m_nav!.isValidPolyRef(startRef) ||
                centerPos == null || !dtVisfinite(centerPos) ||
                maxRadius < 0F || !float.IsFinite(maxRadius) ||
                filter == null || hitPos == null || hitNormal == null)
            {
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            m_nodePool!.clear();
            m_openList!.clear();

            dtNode* startNode = m_nodePool.getNode(startRef);
            dtVcopy(startNode->pos, centerPos);
            startNode->pidx = 0;
            startNode->cost = 0F;
            startNode->total = 0F;
            startNode->id = startRef;
            startNode->flags = (int)DT_NODE_OPEN;
            m_openList.push(startNode);

            float radiusSqr = maxRadius * maxRadius;

            dtStatus status = DT_SUCCESS;

            while (!m_openList.empty())
            {
                dtNode* bestNode = m_openList.pop();
                bestNode->flags &= (int)~DT_NODE_OPEN;
                bestNode->flags |= (int)DT_NODE_CLOSED;

                // Get poly and tile.
                // The API input has been cheked already, skip checking internal data.
                dtPolyRef bestRef = bestNode->id;
                dtMeshTile* bestTile = null;
                dtPoly* bestPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(bestRef, out bestTile, out bestPoly);

                // Get parent poly and tile.
                dtPolyRef parentRef = 0;
                dtMeshTile* parentTile = null;
                dtPoly* parentPoly = null;
                if (bestNode->pidx != 0)
                    parentRef = m_nodePool.getNodeAtIdx(bestNode->pidx)->id;
                if (parentRef != 0)
                    m_nav.getTileAndPolyByRefUnsafe(parentRef, out parentTile, out parentPoly);

                // Hit test walls.
                for (int i = 0, j = (int)bestPoly->vertCount - 1; i < (int)bestPoly->vertCount; j = i++)
                {
                    // Skip non-solid edges.
                    if ((bestPoly->neis[j] & DT_EXT_LINK) != 0)
                    {
                        // Tile border.
                        bool solid = true;
                        for (uint k = bestPoly->firstLink; k != DT_NULL_LINK; k = bestTile->links[k].next)
                        {
                            dtLink* link = &bestTile->links[k];
                            if (link->edge == j)
                            {
                                if (link->@ref != 0)
                                {
                                    dtMeshTile* neiTile = null;
                                    dtPoly* neiPoly = null;
                                    m_nav.getTileAndPolyByRefUnsafe(link->@ref, out neiTile, out neiPoly);
                                    if (filter.passFilter(link->@ref, neiTile, neiPoly))
                                        solid = false;
                                }
                                break;
                            }
                        }
                        if (!solid) continue;
                    }
                    else if (bestPoly->neis[j] != 0)
                    {
                        // Internal edge
                        uint idx = (uint)(bestPoly->neis[j] - 1);
                        dtPolyRef @ref = m_nav.getPolyRefBase(bestTile) | idx;
                        if (filter.passFilter(@ref, bestTile, &bestTile->polys[idx]))
                            continue;
                    }

                    // Calc distance to the edge.
                    float* vj = &bestTile->verts[bestPoly->verts[j] * 3];
                    float* vi = &bestTile->verts[bestPoly->verts[i] * 3];
                    float tseg = 0;
                    float distSqr = dtDistancePtSegSqr2D(centerPos, vj, vi, ref tseg);

                    // Edge is too far, skip.
                    if (distSqr > radiusSqr)
                        continue;

                    // Hit wall, update radius.
                    radiusSqr = distSqr;
                    // Calculate hit pos.
                    hitPos[0] = vj[0] + (vi[0] - vj[0]) * tseg;
                    hitPos[1] = vj[1] + (vi[1] - vj[1]) * tseg;
                    hitPos[2] = vj[2] + (vi[2] - vj[2]) * tseg;
                }

                for (uint i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
                {
                    dtLink* link = &bestTile->links[i];
                    dtPolyRef neighbourRef = link->@ref;
                    // Skip invalid neighbours and do not follow back to parent.
                    if (neighbourRef == 0 || neighbourRef == parentRef)
                        continue;

                    // Expand to neighbour.
                    dtMeshTile* neighbourTile = null;
                    dtPoly* neighbourPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, out neighbourTile, out neighbourPoly);

                    // Skip off-mesh connections.
                    if (neighbourPoly->getType() == (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
                        continue;

                    // Calc distance to the edge.
                    float* va = &bestTile->verts[bestPoly->verts[link->edge] * 3];
                    float* vb = &bestTile->verts[bestPoly->verts[(link->edge + 1) % bestPoly->vertCount] * 3];
                    float tseg = 0;
                    float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, ref tseg);

                    // If the circle is not touching the next polygon, skip it.
                    if (distSqr > radiusSqr)
                        continue;

                    if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
                        continue;

                    dtNode* neighbourNode = m_nodePool.getNode(neighbourRef);
                    if (neighbourNode == null)
                    {
                        status |= DT_OUT_OF_NODES;
                        continue;
                    }

                    if ((neighbourNode->flags & (int)DT_NODE_CLOSED) != 0)
                        continue;

                    // Cost
                    if (neighbourNode->flags == 0)
                    {
                        getEdgeMidPoint(bestRef, bestPoly, bestTile,
                            neighbourRef, neighbourPoly, neighbourTile, neighbourNode->pos);
                    }

                    float total = bestNode->total + dtVdist(bestNode->pos, neighbourNode->pos);

                    // The node is already in open list and the new result is worse, skip.
                    if ((neighbourNode->flags & (int)DT_NODE_OPEN) != 0 && total >= neighbourNode->total)
                        continue;

                    neighbourNode->id = neighbourRef;
                    neighbourNode->flags = (neighbourNode->flags & (int)~DT_NODE_CLOSED);
                    neighbourNode->pidx = m_nodePool.getNodeIdx(bestNode);
                    neighbourNode->total = total;

                    if ((neighbourNode->flags & (int)DT_NODE_OPEN) != 0)
                    {
                        m_openList.modify(neighbourNode);
                    }
                    else
                    {
                        neighbourNode->flags |= (int)DT_NODE_OPEN;
                        m_openList.push(neighbourNode);
                    }
                }
            }

            // Calc hit normal.
            dtVsub(hitNormal, centerPos, hitPos);
            dtVnormalize(hitNormal);

            hitDist = (float)Math.Sqrt(radiusSqr);

            return status;
        }

        /// Returns the segments for the specified polygon, optionally including portals.
        ///  @param[in]		ref				The reference id of the polygon.
        ///  @param[in]		filter			The polygon filter to apply to the query.
        ///  @param[out]	segmentVerts	The segments. [(ax, ay, az, bx, by, bz) * segmentCount]
        ///  @param[out]	segmentRefs		The reference ids of each segment's neighbor polygon. 
        ///  								Or zero if the segment is a wall. [opt] [(parentRef) * @p segmentCount] 
        ///  @param[out]	segmentCount	The number of segments returned.
        ///  @param[in]		maxSegments		The maximum number of segments the result arrays can hold.
        /// @returns The status flags for the query.

        /// @par
        ///
        /// If the @p segmentRefs parameter is provided, then all polygon segments will be returned. 
        /// Otherwise only the wall segments are returned.
        /// 
        /// A segment that is normally a portal will be included in the result set as a 
        /// wall if the @p filter results in the neighbor polygon becoomming impassable.
        /// 
        /// The @p segmentVerts and @p segmentRefs buffers should normally be sized for the 
        /// maximum segments per polygon of the source navigation mesh.
        /// 
        public dtStatus getPolyWallSegments(dtPolyRef @ref, dtQueryFilter? filter,
            float* segmentVerts, dtPolyRef* segmentRefs, out int segmentCount, int maxSegments)
        {
            dtAssert(m_nav != null);

            segmentCount = 0;

            dtMeshTile* tile = null;
            dtPoly* poly = null;
            if (dtStatusFailed(m_nav!.getTileAndPolyByRef(@ref, out tile, out poly)))
                return DT_FAILURE | DT_INVALID_PARAM;

            if (filter == null || segmentVerts == null || maxSegments < 0)
                return DT_FAILURE | DT_INVALID_PARAM;

            int n = 0;
            const int MAX_INTERVAL = 16;
            dtSegInterval* ints = stackalloc dtSegInterval[MAX_INTERVAL];
            int nints;

            bool storePortals = segmentRefs != null;

            dtStatus status = DT_SUCCESS;

            for (int i = 0, j = (int)poly->vertCount - 1; i < (int)poly->vertCount; j = i++)
            {
                // Skip non-solid edges.
                nints = 0;
                if ((poly->neis[j] & DT_EXT_LINK) != 0)
                {
                    // Tile border.
                    for (uint k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
                    {
                        dtLink* link = &tile->links[k];
                        if (link->edge == j)
                        {
                            if (link->@ref != 0)
                            {
                                dtMeshTile* neiTile = null;
                                dtPoly* neiPoly = null;
                                m_nav.getTileAndPolyByRefUnsafe(link->@ref, out neiTile, out neiPoly);
                                if (filter.passFilter(link->@ref, neiTile, neiPoly))
                                {
                                    insertInterval(ints, ref nints, MAX_INTERVAL, link->bmin, link->bmax, link->@ref);
                                }
                            }
                        }
                    }
                }
                else
                {
                    // Internal edge
                    dtPolyRef neiRef = 0;
                    if (poly->neis[j] != 0)
                    {
                        uint idx = (uint)(poly->neis[j] - 1);
                        neiRef = m_nav.getPolyRefBase(tile) | idx;
                        if (!filter.passFilter(neiRef, tile, &tile->polys[idx]))
                            neiRef = 0;
                    }

                    // If the edge leads to another polygon and portals are not stored, skip.
                    if (neiRef != 0 && !storePortals)
                        continue;

                    if (n < maxSegments)
                    {
                        float* _vj = &tile->verts[poly->verts[j] * 3];
                        float* _vi = &tile->verts[poly->verts[i] * 3];
                        float* seg = &segmentVerts[n * 6];
                        dtVcopy(seg + 0, _vj);
                        dtVcopy(seg + 3, _vi);
                        if (segmentRefs != null)
                            segmentRefs[n] = neiRef;
                        n++;
                    }
                    else
                    {
                        status |= DT_BUFFER_TOO_SMALL;
                    }

                    continue;
                }

                // Add sentinels
                insertInterval(ints, ref nints, MAX_INTERVAL, -1, 0, 0);
                insertInterval(ints, ref nints, MAX_INTERVAL, 255, 256, 0);

                // Store segments.
                float* vj = &tile->verts[poly->verts[j] * 3];
                float* vi = &tile->verts[poly->verts[i] * 3];
                for (int k = 1; k < nints; ++k)
                {
                    // Portal segment.
                    if (storePortals && ints[k].@ref != 0)
                    {
                        float tmin = ints[k].tmin / 255.0f;
                        float tmax = ints[k].tmax / 255.0f;
                        if (n < maxSegments)
                        {
                            float* seg = &segmentVerts[n * 6];
                            dtVlerp(seg + 0, vj, vi, tmin);
                            dtVlerp(seg + 3, vj, vi, tmax);
                            if (segmentRefs != null)
                                segmentRefs[n] = ints[k].@ref;
                            n++;
                        }
                        else
                        {
                            status |= DT_BUFFER_TOO_SMALL;
                        }
                    }

                    // Wall segment.
                    int imin = ints[k - 1].tmax;
                    int imax = ints[k].tmin;
                    if (imin != imax)
                    {
                        float tmin = imin / 255.0f;
                        float tmax = imax / 255.0f;
                        if (n < maxSegments)
                        {
                            float* seg = &segmentVerts[n * 6];
                            dtVlerp(seg + 0, vj, vi, tmin);
                            dtVlerp(seg + 3, vj, vi, tmax);
                            if (segmentRefs != null)
                                segmentRefs[n] = 0;
                            n++;
                        }
                        else
                        {
                            status |= DT_BUFFER_TOO_SMALL;
                        }
                    }
                }
            }

            segmentCount = n;

            return status;
        }

        /// Returns random location on navmesh.
        /// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
        ///  @param[in]		filter			The polygon filter to apply to the query.
        ///  @param[in]		frand			Function returning a random number [0..1).
        ///  @param[out]	randomRef		The reference id of the random location.
        ///  @param[out]	randomPt		The random location. 
        /// @returns The status flags for the query.
        public delegate float frandDelegate();

        public dtStatus findRandomPoint(dtQueryFilter? filter, frandDelegate frand,
            out dtPolyRef randomRef, float* randomPt)
        {
            dtAssert(m_nav != null);

            randomRef = 0;

            if (filter == null || frand == null || randomPt == null)
                return DT_FAILURE | DT_INVALID_PARAM;

            // Randomly pick one tile. Assume that all tiles cover roughly the same area.
            dtMeshTile* tile = null;
            float tsum = 0.0f;
            for (int i = 0; i < m_nav!.getMaxTiles(); i++)
            {
                dtMeshTile* _t = m_nav.getTile(i);
                if (_t == null || _t->header == null) continue;

                // Choose random tile using reservoi sampling.
                const float area = 1.0f; // Could be tile area too.
                tsum += area;
                float u = frand();
                if (u * tsum <= area)
                    tile = _t;
            }
            if (tile == null)
                return DT_FAILURE;

            // Randomly pick one polygon weighted by polygon area.
            dtPoly* poly = null;
            dtPolyRef polyRef = 0;
            dtPolyRef @base = m_nav.getPolyRefBase(tile);

            float areaSum = 0.0f;
            for (int i = 0; i < tile->header->polyCount; ++i)
            {
                dtPoly* p = &tile->polys[i];
                // Do not return off-mesh connection polygons.
                if (p->getType() != (byte)DT_POLYTYPE_GROUND)
                    continue;
                // Must pass filter
                dtPolyRef @ref = @base | (uint)i;
                if (!filter.passFilter(@ref, tile, p))
                    continue;

                // Calc area of the polygon.
                float polyArea = 0.0f;
                for (int j = 2; j < p->vertCount; ++j)
                {
                    float* va = &tile->verts[p->verts[0] * 3];
                    float* vb = &tile->verts[p->verts[j - 1] * 3];
                    float* vc = &tile->verts[p->verts[j] * 3];
                    polyArea += dtTriArea2D(va, vb, vc);
                }

                // Choose random polygon weighted by area, using reservoi sampling.
                areaSum += polyArea;
                float u = frand();
                if (u * areaSum <= polyArea)
                {
                    poly = p;
                    polyRef = @ref;
                }
            }

            if (poly == null)
                return DT_FAILURE;

            // Randomly pick point on polygon.
            float* v = &tile->verts[poly->verts[0] * 3];
            float* verts = stackalloc float[3 * DT_VERTS_PER_POLYGON];
            float* areas = stackalloc float[DT_VERTS_PER_POLYGON];
            dtVcopy(&verts[0 * 3], v);
            for (int j = 1; j < poly->vertCount; ++j)
            {
                v = &tile->verts[poly->verts[j] * 3];
                dtVcopy(&verts[j * 3], v);
            }

            float s = frand();
            float t = frand();

            float* pt = stackalloc float[3];
            dtRandomPointInConvexPoly(verts, poly->vertCount, areas, s, t, pt);

            closestPointOnPoly(polyRef, pt, pt, out bool _);

            dtVcopy(randomPt, pt);
            randomRef = polyRef;

            return DetourCommon.DT_SUCCESS;
        }

        /// Returns random location on navmesh within the reach of specified location.
        /// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
        /// The location is not exactly constrained by the circle, but it limits the visited polygons.
        ///  @param[in]		startRef		The reference id of the polygon where the search starts.
        ///  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
        ///  @param[in]		filter			The polygon filter to apply to the query.
        ///  @param[in]		frand			Function returning a random number [0..1).
        ///  @param[out]	randomRef		The reference id of the random location.
        ///  @param[out]	randomPt		The random location. [(x, y, z)]
        /// @returns The status flags for the query.
        public dtStatus findRandomPointAroundCircle(dtPolyRef startRef, float* centerPos, float maxRadius,
            dtQueryFilter? filter, frandDelegate frand,
            out dtPolyRef randomRef, float* randomPt)
        {
            dtAssert(m_nav != null);
            dtAssert(m_nodePool != null);
            dtAssert(m_openList != null);

            randomRef = 0;

            // Validate input
            if (!m_nav!.isValidPolyRef(startRef) ||
                centerPos == null || !dtVisfinite(centerPos) ||
                maxRadius < 0F || !float.IsFinite(maxRadius) ||
                filter == null || frand == null || randomPt == null)
            {
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            dtMeshTile* startTile = null;
            dtPoly* startPoly = null;
            m_nav.getTileAndPolyByRefUnsafe(startRef, out startTile, out startPoly);
            if (!filter.passFilter(startRef, startTile, startPoly))
                return DT_FAILURE | DT_INVALID_PARAM;

            m_nodePool!.clear();
            m_openList!.clear();

            dtNode* startNode = m_nodePool.getNode(startRef);
            dtVcopy(startNode->pos, centerPos);
            startNode->pidx = 0;
            startNode->cost = 0F;
            startNode->total = 0F;
            startNode->id = startRef;
            startNode->flags = (int)DT_NODE_OPEN;
            m_openList.push(startNode);

            dtStatus status = DT_SUCCESS;

            float radiusSqr = maxRadius * maxRadius;
            float areaSum = 0.0f;

            dtMeshTile* randomTile = null;
            dtPoly* randomPoly = null;
            dtPolyRef randomPolyRef = 0;

            float* va = stackalloc float[3];
            float* vb = stackalloc float[3];
            while (!m_openList.empty())
            {
                dtNode* bestNode = m_openList.pop();
                bestNode->flags &= (int)~DT_NODE_OPEN;
                bestNode->flags |= (int)DT_NODE_CLOSED;

                // Get poly and tile.
                // The API input has been cheked already, skip checking internal data.
                dtPolyRef bestRef = bestNode->id;
                dtMeshTile* bestTile = null;
                dtPoly* bestPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(bestRef, out bestTile, out bestPoly);

                // Place random locations on on ground.
                if (bestPoly->getType() == (byte)DT_POLYTYPE_GROUND)
                {
                    // Calc area of the polygon.
                    float polyArea = 0.0f;
                    for (int j = 2; j < bestPoly->vertCount; ++j)
                    {
                        float* _va = &bestTile->verts[bestPoly->verts[0] * 3];
                        float* _vb = &bestTile->verts[bestPoly->verts[j - 1] * 3];
                        float* _vc = &bestTile->verts[bestPoly->verts[j] * 3];
                        polyArea += dtTriArea2D(_va, _vb, _vc);
                    }
                    // Choose random polygon weighted by area, using reservoi sampling.
                    areaSum += polyArea;
                    float u = frand();
                    if (u * areaSum <= polyArea)
                    {
                        randomTile = bestTile;
                        randomPoly = bestPoly;
                        randomPolyRef = bestRef;
                    }
                }


                // Get parent poly and tile.
                dtPolyRef parentRef = 0;
                dtMeshTile* parentTile = null;
                dtPoly* parentPoly = null;
                if (bestNode->pidx != 0)
                    parentRef = m_nodePool.getNodeAtIdx(bestNode->pidx)->id;
                if (parentRef != 0)
                    m_nav.getTileAndPolyByRefUnsafe(parentRef, out parentTile, out parentPoly);

                for (uint i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
                {
                    dtLink* link = &bestTile->links[i];
                    dtPolyRef neighbourRef = link->@ref;
                    // Skip invalid neighbours and do not follow back to parent.
                    if (neighbourRef == 0 || neighbourRef == parentRef)
                        continue;

                    // Expand to neighbour
                    dtMeshTile* neighbourTile = null;
                    dtPoly* neighbourPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, out neighbourTile, out neighbourPoly);

                    // Do not advance if the polygon is excluded by the filter.
                    if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
                        continue;

                    // Find edge and calc distance to the edge.
                    if (getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb) == 0)
                        continue;

                    // If the circle is not touching the next polygon, skip it.
                    float tseg = 0;
                    float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, ref tseg);
                    if (distSqr > radiusSqr)
                        continue;

                    dtNode* neighbourNode = m_nodePool.getNode(neighbourRef);
                    if (neighbourNode == null)
                    {
                        status |= DT_OUT_OF_NODES;
                        continue;
                    }

                    if ((neighbourNode->flags & (int)DT_NODE_CLOSED) != 0)
                        continue;

                    // Cost
                    if (neighbourNode->flags == 0)
                        dtVlerp(neighbourNode->pos, va, vb, 0.5f);

                    float total = bestNode->total + dtVdist(bestNode->pos, neighbourNode->pos);

                    // The node is already in open list and the new result is worse, skip.
                    if ((neighbourNode->flags & (int)DT_NODE_OPEN) != 0 && total >= neighbourNode->total)
                        continue;

                    neighbourNode->id = neighbourRef;
                    neighbourNode->flags = (neighbourNode->flags & (int)~DT_NODE_CLOSED);
                    neighbourNode->pidx = m_nodePool.getNodeIdx(bestNode);
                    neighbourNode->total = total;

                    if ((neighbourNode->flags & (int)DT_NODE_OPEN) != 0)
                    {
                        m_openList.modify(neighbourNode);
                    }
                    else
                    {
                        neighbourNode->flags = (int)DT_NODE_OPEN;
                        m_openList.push(neighbourNode);
                    }
                }
            }

            if (randomPoly == null)
                return DT_FAILURE;

            // Randomly pick point on polygon.
            float* v = &randomTile->verts[randomPoly->verts[0] * 3];
            float* verts = stackalloc float[3 * DT_VERTS_PER_POLYGON];
            float* areas = stackalloc float[DT_VERTS_PER_POLYGON];
            dtVcopy(&verts[0 * 3], v);
            for (int j = 1; j < randomPoly->vertCount; ++j)
            {
                v = &randomTile->verts[randomPoly->verts[j] * 3];
                dtVcopy(&verts[j * 3], v);
            }

            float s = frand();
            float t = frand();

            float* pt = stackalloc float[3];
            dtRandomPointInConvexPoly(verts, randomPoly->vertCount, areas, s, t, pt);

            closestPointOnPoly(randomPolyRef, pt, pt, out bool _);

            dtVcopy(randomPt, pt);
            randomRef = randomPolyRef;

            return status;
        }

        /// Finds the closest point on the specified polygon.
        ///  @param[in]		ref			The reference id of the polygon.
        ///  @param[in]		pos			The position to check. [(x, y, z)]
        ///  @param[out]	closest		The closest point on the polygon. [(x, y, z)]
        ///  @param[out]	posOverPoly	True of the position is over the polygon.
        /// @returns The status flags for the query.

        //////////////////////////////////////////////////////////////////////////////////////////

        /// @par
        ///
        /// Uses the detail polygons to find the surface height. (Most accurate.)
        ///
        /// @p pos does not have to be within the bounds of the polygon or navigation mesh.
        ///
        /// See closestPointOnPolyBoundary() for a limited but faster option.
        ///
        public dtStatus closestPointOnPoly(dtPolyRef @ref, float* pos, float* closest, out bool posOverPoly)
        {
            dtAssert(m_nav != null);
            posOverPoly = false;

            if (!m_nav!.isValidPolyRef(@ref) ||
                pos == null || !dtVisfinite(pos) ||
                closest == null)
            {
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            m_nav.closestPointOnPoly(@ref, pos, closest, out posOverPoly);
            return DT_SUCCESS;
        }

        /// Returns a point on the boundary closest to the source point if the source point is outside the 
        /// polygon's xz-bounds.
        ///  @param[in]		ref			The reference id to the polygon.
        ///  @param[in]		pos			The position to check. [(x, y, z)]
        ///  @param[out]	closest		The closest point. [(x, y, z)]
        /// @returns The status flags for the query.

        /// @par
        ///
        /// Much faster than closestPointOnPoly().
        ///
        /// If the provided position lies within the polygon's xz-bounds (above or below), 
        /// then @p pos and @p closest will be equal.
        ///
        /// The height of @p closest will be the polygon boundary.  The height detail is not used.
        /// 
        /// @p pos does not have to be within the bounds of the polybon or the navigation mesh.
        /// 
        public uint closestPointOnPolyBoundary(dtPolyRef @ref, float* pos, float* closest)
        {
            dtAssert(m_nav != null);

            dtMeshTile* tile = null;
            dtPoly* poly = null;
            if (dtStatusFailed(m_nav!.getTileAndPolyByRef(@ref, out tile, out poly)))
                return DT_FAILURE | DT_INVALID_PARAM;

            if (pos == null || !dtVisfinite(pos) || closest == null)
                return DT_FAILURE | DT_INVALID_PARAM;

            // Collect vertices.
            float* verts = stackalloc float[DetourCommon.DT_VERTS_PER_POLYGON * 3];
            float* edged = stackalloc float[DetourCommon.DT_VERTS_PER_POLYGON];
            float* edget = stackalloc float[DetourCommon.DT_VERTS_PER_POLYGON];
            int nv = 0;
            for (int i = 0; i < (int)poly->vertCount; ++i)
            {
                dtVcopy(&verts[nv * 3], &tile->verts[poly->verts[i] * 3]);
                nv++;
            }

            bool inside = dtDistancePtPolyEdgesSqr(pos, verts, nv, edged, edget);
            if (inside)
            {
                // Point is inside the polygon, return the point.
                dtVcopy(closest, pos);
            }
            else
            {
                // Point is outside the polygon, dtClamp to nearest edge.
                float dmin = edged[0];
                int imin = 0;
                for (int i = 1; i < nv; ++i)
                {
                    if (edged[i] < dmin)
                    {
                        dmin = edged[i];
                        imin = i;
                    }
                }
                float* va = &verts[imin * 3];
                float* vb = &verts[((imin + 1) % nv) * 3];
                dtVlerp(closest, va, vb, edget[imin]);
            }

            return DT_SUCCESS;
        }

        /// Gets the height of the polygon at the provided position using the height detail. (Most accurate.)
        ///  @param[in]		ref			The reference id of the polygon.
        ///  @param[in]		pos			A position within the xz-bounds of the polygon. [(x, y, z)]
        ///  @param[out]	height		The height at the surface of the polygon.
        /// @returns The status flags for the query.

        /// @par
        ///
        /// Will return #DT_FAILURE | DT_INVALID_PARAM if the provided position is outside the xz-bounds 
        /// of the polygon.
        /// 
        public uint getPolyHeight(dtPolyRef @ref, float* pos, out float height)
        {
            dtAssert(m_nav != null);

            height = 0;

            dtMeshTile* tile = null;
            dtPoly* poly = null;
            if (dtStatusFailed(m_nav!.getTileAndPolyByRef(@ref, out tile, out poly)))
                return DT_FAILURE | DT_INVALID_PARAM;

            if (pos == null || !dtVisfinite2D(pos))
                return DT_FAILURE | DT_INVALID_PARAM;

            // We used to return success for offmesh connections, but the
            // getPolyHeight in DetourNavMesh does not do this, so special
            // case it here.
            if (poly->getType() == (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
            {
                float* v0 = &tile->verts[poly->verts[0] * 3];
                float* v1 = &tile->verts[poly->verts[1] * 3];
                float t = 0;
                dtDistancePtSegSqr2D(pos, v0, v1, ref t);
                height = v0[1] + (v1[1] - v0[1]) * t;

                return DT_SUCCESS;
            }

            return m_nav.getPolyHeight(tile, poly, pos, ref height) ? DT_SUCCESS : DT_FAILURE | DT_INVALID_PARAM;
        }

        /// @name Miscellaneous Functions

        /// Returns true if the polygon reference is valid and passes the filter restrictions.
        ///  @param[in]		ref			The polygon reference to check.
        ///  @param[in]		filter		The filter to apply.
        public bool isValidPolyRef(dtPolyRef @ref, dtQueryFilter filter)
        {
            dtMeshTile* tile = null;
            dtPoly* poly = null;
            dtStatus status = m_nav!.getTileAndPolyByRef(@ref, out tile, out poly);
            // If cannot get polygon, assume it does not exists and boundary is invalid.
            if (dtStatusFailed(status))
                return false;
            // If cannot pass filter, assume flags has changed and boundary is invalid.
            if (!filter.passFilter(@ref, tile, poly))
                return false;
            return true;
        }

        /// Returns true if the polygon reference is in the closed list. 
        ///  @param[in]		ref		The reference id of the polygon to check.
        /// @returns True if the polygon is in closed list.

        /// @par
        ///
        /// The closed list is the list of polygons that were fully evaluated during 
        /// the last navigation graph search. (A* or Dijkstra)
        /// 
        public bool isInClosedList(dtPolyRef @ref)
        {
            if (m_nodePool == null) return false;

            dtNode** nodes = stackalloc dtNode*[DT_MAX_STATES_PER_NODE];
            int n = (int)m_nodePool.findNodes(@ref, nodes, DT_MAX_STATES_PER_NODE);

            for (int i = 0; i < n; i++)
            {
                if ((nodes[i]->flags & (int)DT_NODE_CLOSED) != 0)
                    return true;
            }

            return false;
        }

        /// Gets the node pool.
        /// @returns The node pool.
        public dtNodePool? getNodePool()
        {
            return m_nodePool;
        }

        /// Gets the navigation mesh the query object is using.
        /// @return The navigation mesh the query object is using.
        public dtNavMesh? getAttachedNavMesh()
        {
            return m_nav;
        }

        /// Queries polygons within a tile.
        private void queryPolygonsInTile(dtMeshTile* tile, float* qmin, float* qmax,
            dtQueryFilter filter, dtPolyQuery query)
        {
            dtAssert(m_nav != null);
            const int batchSize = 32;
            dtPolyRef* polyRefs = stackalloc dtPolyRef[batchSize];
            dtPoly** polys = stackalloc dtPoly*[batchSize];
            int n = 0;

            if (tile->bvTree != null)
            {
                dtBVNode* node = &tile->bvTree[0];
                dtBVNode* end = &tile->bvTree[tile->header->bvNodeCount];
                float* tbmin = tile->header->bmin;
                float* tbmax = tile->header->bmax;
                float qfac = tile->header->bvQuantFactor;

                // Calculate quantized box
                ushort* bmin = stackalloc ushort[3];
                ushort* bmax = stackalloc ushort[3];
                // dtClamp query box to world box.
                float minx = dtClamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
                float miny = dtClamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
                float minz = dtClamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
                float maxx = dtClamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
                float maxy = dtClamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
                float maxz = dtClamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
                // Quantize
                bmin[0] = (ushort)((int)(qfac * minx) & 0xfffe);
                bmin[1] = (ushort)((int)(qfac * miny) & 0xfffe);
                bmin[2] = (ushort)((int)(qfac * minz) & 0xfffe);
                bmax[0] = (ushort)((int)(qfac * maxx + 1) | 1);
                bmax[1] = (ushort)((int)(qfac * maxy + 1) | 1);
                bmax[2] = (ushort)((int)(qfac * maxz + 1) | 1);

                // Traverse tree
                dtPolyRef @base = m_nav!.getPolyRefBase(tile);
                while (node < end)
                {
                    bool overlap = dtOverlapQuantBounds(bmin, bmax, node->bmin, node->bmax);
                    bool isLeafNode = node->i >= 0;

                    if (isLeafNode && overlap)
                    {
                        dtPolyRef @ref = @base | (uint)node->i;
                        if (filter.passFilter(@ref, tile, &tile->polys[node->i]))
                        {
                            polyRefs[n] = @ref;
                            polys[n] = &tile->polys[node->i];

                            if (n == batchSize - 1)
                            {
                                query.process(tile, polys, polyRefs, batchSize);
                                n = 0;
                            }
                            else
                            {
                                n++;
                            }
                        }
                    }

                    if (overlap || isLeafNode)
                    {
                        node++;
                    }
                    else
                    {
                        int escapeIndex = -node->i;
                        node += escapeIndex;
                    }
                }
            }
            else
            {
                float* bmin = stackalloc float[3];
                float* bmax = stackalloc float[3];
                dtPolyRef @base = m_nav!.getPolyRefBase(tile);
                for (int i = 0; i < tile->header->polyCount; ++i)
                {
                    dtPoly* p = &tile->polys[i];
                    // Do not return off-mesh connection polygons.
                    if (p->getType() == (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
                        continue;
                    // Must pass filter
                    dtPolyRef @ref = @base | (uint)i;
                    if (!filter.passFilter(@ref, tile, p))
                        continue;
                    // Calc polygon bounds.
                    float* v = &tile->verts[p->verts[0] * 3];
                    dtVcopy(bmin, v);
                    dtVcopy(bmax, v);
                    for (int j = 1; j < p->vertCount; ++j)
                    {
                        v = &tile->verts[p->verts[j] * 3];
                        dtVmin(bmin, v);
                        dtVmax(bmax, v);
                    }
                    if (dtOverlapBounds(qmin, qmax, bmin, bmax))
                    {
                        polyRefs[n] = @ref;
                        polys[n] = p;

                        if (n == batchSize - 1)
                        {
                            query.process(tile, polys, polyRefs, batchSize);
                            n = 0;
                        }
                        else
                        {
                            n++;
                        }
                    }
                }
            }

            // Process the last polygons that didn't make a full batch.
            if (n > 0)
                query.process(tile, polys, polyRefs, n);
        }

        /// Returns portal points between two polygons.
        private uint getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right,
            ref byte fromType, ref byte toType)
        {
            dtAssert(m_nav != null);

            dtMeshTile* fromTile = null;
            dtPoly* fromPoly = null;
            if (dtStatusFailed(m_nav!.getTileAndPolyByRef(from, out fromTile, out fromPoly)))
                return DT_FAILURE | DT_INVALID_PARAM;
            fromType = fromPoly->getType();

            dtMeshTile* toTile = null;
            dtPoly* toPoly = null;
            if (dtStatusFailed(m_nav.getTileAndPolyByRef(to, out toTile, out toPoly)))
                return DT_FAILURE | DT_INVALID_PARAM;
            toType = toPoly->getType();

            return getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right);
        }


        // Returns portal points between two polygons.
        private dtStatus getPortalPoints(dtPolyRef from, dtPoly* fromPoly, dtMeshTile* fromTile,
            dtPolyRef to, dtPoly* toPoly, dtMeshTile* toTile,
            float* left, float* right)
        {
            // Find the link that points to the 'to' polygon.
            dtLink* link = null;
            for (uint i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next)
            {
                if (fromTile->links[i].@ref == to)
                {
                    link = &fromTile->links[i];
                    break;
                }
            }
            if (link == null)
                return DT_FAILURE | DT_INVALID_PARAM;

            // Handle off-mesh connections.
            if (fromPoly->getType() == (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
            {
                // Find link that points to first vertex.
                for (uint i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next)
                {
                    if (fromTile->links[i].@ref == to)
                    {
                        int v = fromTile->links[i].edge;
                        dtVcopy(left, &fromTile->verts[fromPoly->verts[v] * 3]);
                        dtVcopy(right, &fromTile->verts[fromPoly->verts[v] * 3]);
                        return DT_SUCCESS;
                    }
                }
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            if (toPoly->getType() == (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
            {
                for (uint i = toPoly->firstLink; i != DT_NULL_LINK; i = toTile->links[i].next)
                {
                    if (toTile->links[i].@ref == from)
                    {
                        int v = toTile->links[i].edge;
                        dtVcopy(left, &toTile->verts[toPoly->verts[v] * 3]);
                        dtVcopy(right, &toTile->verts[toPoly->verts[v] * 3]);
                        return DT_SUCCESS;
                    }
                }
                return DT_FAILURE | DT_INVALID_PARAM;
            }

            // Find portal vertices.
            int v0 = fromPoly->verts[link->edge];
            int v1 = fromPoly->verts[(link->edge + 1) % (int)fromPoly->vertCount];
            dtVcopy(left, &fromTile->verts[v0 * 3]);
            dtVcopy(right, &fromTile->verts[v1 * 3]);

            // If the link is at tile boundary, dtClamp the vertices to
            // the link width.
            if (link->side != 0xff)
            {
                // Unpack portal limits.
                if (link->bmin != 0 || link->bmax != 255)
                {
                    float s = 1.0f / 255.0f;
                    float tmin = link->bmin * s;
                    float tmax = link->bmax * s;
                    dtVlerp(left, &fromTile->verts[v0 * 3], &fromTile->verts[v1 * 3], tmin);
                    dtVlerp(right, &fromTile->verts[v0 * 3], &fromTile->verts[v1 * 3], tmax);
                }
            }

            return DT_SUCCESS;
        }

        /// Returns edge mid point between two polygons.

        // Returns edge mid point between two polygons.
        private dtStatus getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid)
        {
            float* left = stackalloc float[3];
            float* right = stackalloc float[3];
            byte fromType = 0;
            byte toType = 0;
            if (dtStatusFailed(getPortalPoints(from, to, left, right, ref fromType, ref toType)))
                return DT_FAILURE | DT_INVALID_PARAM;
            mid[0] = (left[0] + right[0]) * 0.5f;
            mid[1] = (left[1] + right[1]) * 0.5f;
            mid[2] = (left[2] + right[2]) * 0.5f;
            return DT_SUCCESS;
        }

        private uint getEdgeMidPoint(dtPolyRef from, dtPoly* fromPoly, dtMeshTile* fromTile,
            dtPolyRef to, dtPoly* toPoly, dtMeshTile* toTile, float* mid)
        {
            float* left = stackalloc float[3];
            float* right = stackalloc float[3];
            if (dtStatusFailed(getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right)))
                return DT_FAILURE | DT_INVALID_PARAM;
            mid[0] = (left[0] + right[0]) * 0.5f;
            mid[1] = (left[1] + right[1]) * 0.5f;
            mid[2] = (left[2] + right[2]) * 0.5f;
            return DT_SUCCESS;
        }

        // Appends vertex to a straight path
        private dtStatus appendVertex(float* pos, byte flags, dtPolyRef @ref,
            float* straightPath, byte* straightPathFlags, dtPolyRef* straightPathRefs,
            ref int straightPathCount, int maxStraightPath)
        {
            if (straightPathCount > 0 && dtVequal(&straightPath[(straightPathCount - 1) * 3], pos))
            {
                // The vertices are equal, update flags and poly.
                if (straightPathFlags != null)
                    straightPathFlags[straightPathCount - 1] = flags;
                if (straightPathRefs != null)
                    straightPathRefs[straightPathCount - 1] = @ref;
            }
            else
            {
                // Append new vertex.
                dtVcopy(&straightPath[straightPathCount * 3], pos);
                if (straightPathFlags != null)
                    straightPathFlags[straightPathCount] = flags;
                if (straightPathRefs != null)
                    straightPathRefs[straightPathCount] = @ref;
                straightPathCount++;

                // If there is no space to append more vertices, return.
                if (straightPathCount >= maxStraightPath)
                    return DT_SUCCESS | DT_BUFFER_TOO_SMALL;

                // If reached end of path, return.
                if (flags == (byte)DT_STRAIGHTPATH_END)
                    return DT_SUCCESS;
            }
            return DT_IN_PROGRESS;
        }

        // Appends intermediate portal points to a straight path.
        private dtStatus appendPortals(int startIdx, int endIdx, float* endPos, dtPolyRef* path,
            float* straightPath, byte* straightPathFlags, dtPolyRef* straightPathRefs,
            ref int straightPathCount, int maxStraightPath, int options)
        {
            float* startPos = &straightPath[(straightPathCount - 1) * 3];
            // Append or update last vertex
            dtStatus stat = 0;
            float* pt = stackalloc float[3];
            float* left = stackalloc float[3];
            float* right = stackalloc float[3];
            for (int i = startIdx; i < endIdx; i++)
            {
                // Calculate portal
                dtPolyRef from = path[i];
                dtMeshTile* fromTile = null;
                dtPoly* fromPoly = null;
                if (dtStatusFailed(m_nav!.getTileAndPolyByRef(from, out fromTile, out fromPoly)))
                    return DT_FAILURE | DT_INVALID_PARAM;

                dtPolyRef to = path[i + 1];
                dtMeshTile* toTile = null;
                dtPoly* toPoly = null;
                if (dtStatusFailed(m_nav.getTileAndPolyByRef(to, out toTile, out toPoly)))
                    return DT_FAILURE | DT_INVALID_PARAM;

                if (dtStatusFailed(getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right)))
                    break;

                if ((options & (int)DT_STRAIGHTPATH_AREA_CROSSINGS) != 0)
                {
                    // Skip intersection if only area crossings are requested.
                    if (fromPoly->getArea() == toPoly->getArea())
                        continue;
                }

                // Append intersection
                float s = 0;
                float t = 0;
                if (dtIntersectSegSeg2D(startPos, endPos, left, right, ref s, ref t))
                {
                    dtVlerp(pt, left, right, t);

                    stat = appendVertex(pt, 0, path[i + 1], straightPath, straightPathFlags, straightPathRefs, ref straightPathCount, maxStraightPath);
                    if (stat != DT_IN_PROGRESS)
                        return stat;
                }
            }
            return DT_IN_PROGRESS;
        }

        // Gets the path leading to the specified end node.
        dtStatus getPathToNode(dtNode* endNode, dtPolyRef* path, out int pathCount, int maxPath)
        {
            // Find the length of the entire path.
            dtNode* curNode = endNode;
            int length = 0;
            do
            {
                length++;
                curNode = m_nodePool!.getNodeAtIdx(curNode->pidx);
            } while (curNode != null);

            // If the path cannot be fully stored then advance to the last node we will be able to store.
            curNode = endNode;
            int writeCount;
            for (writeCount = length; writeCount > maxPath; writeCount--)
            {
                dtAssert(curNode != null);

                curNode = m_nodePool.getNodeAtIdx(curNode->pidx);
            }

            // Write path
            for (int i = writeCount - 1; i >= 0; i--)
            {
                dtAssert(curNode != null);

                path[i] = curNode->id;
                curNode = m_nodePool.getNodeAtIdx(curNode->pidx);
            }

            dtAssert(curNode == null);

            pathCount = Math.Min(length, maxPath);

            if (length > maxPath)
                return DT_SUCCESS | DT_BUFFER_TOO_SMALL;

            return DT_SUCCESS;
        }

        static void insertInterval(dtSegInterval* ints, ref int nints, int maxInts,
                                short tmin, short tmax, dtPolyRef @ref)
        {
            if (nints + 1 > maxInts) return;
            // Find insertion point.
            int idx = 0;
            while (idx < nints)
            {
                if (tmax <= ints[idx].tmin)
                    break;
                idx++;
            }
            // Move current results.
            if ((nints - idx) != 0)
            {
                var size = sizeof(dtSegInterval) * (nints - idx);
                Buffer.MemoryCopy(ints + idx, ints + idx + 1, size, size);
            }

            // Store
            ints[idx].@ref = @ref;
            ints[idx].tmin = tmin;
            ints[idx].tmax = tmax;
            nints++;
        }

        private dtNavMesh? m_nav; ///< Pointer to navmesh data.

        private unsafe struct dtQueryData
        {
            public dtStatus status;
            public dtNode* lastBestNode;
            public float lastBestNodeCost;
            public dtPolyRef startRef;
            public dtPolyRef endRef;
            public fixed float startPos[3];
            public fixed float endPos[3];
            public dtQueryFilter? filter;
            public uint options;
            public float raycastLimitSqr;

            public void Reset()
            {
                status = 0;
                lastBestNode = null;
                lastBestNodeCost = 0;
                startRef = 0;
                endRef = 0;
                startPos[0] = 0;
                startPos[1] = 0;
                startPos[2] = 0;
                endPos[0] = 0;
                endPos[1] = 0;
                endPos[2] = 0;
                filter = null;
                options = 0;
                raycastLimitSqr = 0;
            }
        }
        private dtQueryData m_query; ///< Sliced query state.

        private dtNodePool? m_tinyNodePool; ///< Pointer to small node pool.
		private dtNodePool? m_nodePool; ///< Pointer to node pool.
		private dtNodeQueue? m_openList; ///< Pointer to open list queue.
	}
    public unsafe class dtFindNearestPolyQuery : dtPolyQuery
    {
        private dtNavMeshQuery m_query;
        private float* m_center;
        private float m_nearestDistanceSqr;
        private dtPolyRef m_nearestRef;
        private float[] m_nearestPoint = new float[3];
        private bool m_overPoly;

        public dtFindNearestPolyQuery(dtNavMeshQuery query, float* center)
        {
            this.m_query = query;
            this.m_center = center;
            this.m_nearestDistanceSqr = float.MaxValue;
        }

        public dtPolyRef nearestRef()
        {
            return m_nearestRef;
        }
        public ReadOnlySpan<float> nearestPoint()
        {
            return new ReadOnlySpan<float>(m_nearestPoint);
        }
        public bool isOverPoly()
        {
            return m_overPoly;
        }

        public void process(dtMeshTile* tile, dtPoly** polys, dtPolyRef* refs, int count)
        {
            float* closestPtPoly = stackalloc float[3];
            float* diff = stackalloc float[3];
            for (int i = 0; i < count; ++i)
            {
                dtPolyRef @ref = refs[i];
                bool posOverPoly = false;
                float d;
                m_query.closestPointOnPoly(@ref, m_center, closestPtPoly, out posOverPoly);

                // If a point is directly over a polygon and closer than
                // climb height, favor that instead of straight line nearest point.
                dtVsub(diff, m_center, closestPtPoly);
                if (posOverPoly)
                {
                    d = Math.Abs(diff[1]) - tile->header->walkableClimb;
                    d = d > 0F ? d * d : 0F;
                }
                else
                {
                    d = dtVlenSqr(diff);
                }

                if (d < m_nearestDistanceSqr)
                {
                    dtVcopy(m_nearestPoint, new ReadOnlySpan<float>(closestPtPoly, 3));

                    m_nearestDistanceSqr = d;
                    m_nearestRef = @ref;
                    m_overPoly = posOverPoly;
                }
            }
        }
    }

    public unsafe class dtCollectPolysQuery : dtPolyQuery
    {
        private dtPolyRef* m_polys;
        private int m_maxPolys;
        private int m_numCollected;
        private bool m_overflow;

        public dtCollectPolysQuery(dtPolyRef* polys, int maxPolys)
        {
            this.m_polys = polys;
            this.m_maxPolys = maxPolys;
        }

        public int numCollected()
        {
            return m_numCollected;
        }
        public bool overflowed()
        {
            return m_overflow;
        }

        public void process(dtMeshTile* tile, dtPoly** polys, dtPolyRef* refs, int count)
        {
            int numLeft = m_maxPolys - m_numCollected;
            int toCopy = count;
            if (toCopy > numLeft)
            {
                m_overflow = true;
                toCopy = numLeft;
            }

            var size = toCopy * sizeof(dtPolyRef);
            Buffer.MemoryCopy(refs, m_polys + m_numCollected, size, size);
            m_numCollected += toCopy;
        }
    }

    public struct dtSegInterval
    {
        public dtPolyRef @ref;
        public short tmin;
        public short tmax;
    }
}
