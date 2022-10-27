// MIT License - Copyright (C) ryancheung and the FelCore team
// This file is subject to the terms and conditions defined in
// file 'LICENSE', which is part of this source code package.

using System;
using dtPolyRef = System.UInt64;
using dtTileRef = System.UInt64;
using static RecastSharp.DetourCommon;
using dtStatus = System.UInt32;

namespace RecastSharp
{
    using static dtTileFlags;
    using static dtPolyTypes;

    /// Tile flags used for various functions and fields.
    /// For an example, see dtNavMesh::addTile().
    public enum dtTileFlags
    {
        /// The navigation mesh owns the tile memory and is responsible for freeing it.
        DT_TILE_FREE_DATA = 0x01
    }

    /// Vertex flags returned by dtNavMeshQuery::findStraightPath.
    public enum dtStraightPathFlags
    {
        DT_STRAIGHTPATH_START = 0x01, ///< The vertex is the start position in the path.
        DT_STRAIGHTPATH_END = 0x02, ///< The vertex is the end position in the path.
        DT_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04 ///< The vertex is the start of an off-mesh connection.
    }

    /// Options for dtNavMeshQuery::findStraightPath.
    public enum dtStraightPathOptions
    {
        DT_STRAIGHTPATH_AREA_CROSSINGS = 0x01, ///< Add a vertex at every polygon edge crossing where area changes.
        DT_STRAIGHTPATH_ALL_CROSSINGS = 0x02 ///< Add a vertex at every polygon edge crossing.
    }


    /// Options for dtNavMeshQuery::initSlicedFindPath and updateSlicedFindPath
    public enum dtFindPathOptions
    {
        DT_FINDPATH_ANY_ANGLE = 0x02 ///< use raycasts during pathfind to "shortcut" (raycast still consider costs)
    }

    /// Options for dtNavMeshQuery::raycast
    public enum dtRaycastOptions
    {
        DT_RAYCAST_USE_COSTS = 0x01 ///< Raycast should calculate movement cost along the ray and fill RaycastHit::cost
    }

    public enum dtDetailTriEdgeFlags
    {
        DT_DETAIL_EDGE_BOUNDARY = 0x01 ///< Detail triangle edge is part of the poly boundary
    }

    /// Flags representing the type of a navigation mesh polygon.
    public enum dtPolyTypes
    {
        /// The polygon is a standard convex polygon that is part of the surface of the mesh.
        DT_POLYTYPE_GROUND = 0,
        /// The polygon is an off-mesh connection consisting of two vertices.
        DT_POLYTYPE_OFFMESH_CONNECTION = 1
    }


    /// Defines a polygon within a dtMeshTile object.
    /// @ingroup detour
    public unsafe struct dtPoly
    {
        /// Index to first link in linked list. (Or #DT_NULL_LINK if there is no link.)
        public uint firstLink;

        /// The indices of the polygon's vertices.
        /// The actual vertices are located in dtMeshTile::verts.
        public fixed ushort verts[DT_VERTS_PER_POLYGON];

        /// Packed data representing neighbor polygons references and flags for each edge.
        public fixed ushort neis[DT_VERTS_PER_POLYGON];

        /// The user defined polygon flags.
        public ushort flags;

        /// The number of vertices in the polygon.
        public byte vertCount;

        /// The bit packed area id and polygon type.
        /// @note Use the structure's set and get methods to acess this value.
        public byte areaAndtype;

        /// Sets the user defined area id. [Limit: < #DT_MAX_AREAS]
        public void setArea(byte a)
        {
            areaAndtype = (byte)((areaAndtype & 0xc0) | (a & 0x3f));
        }

        /// Sets the polygon type. (See: #dtPolyTypes.)
        public void setType(byte t)
        {
            areaAndtype = (byte)((areaAndtype & 0x3f) | (t << 6));
        }

        /// Gets the user defined area id.
        public byte getArea()
        {
            return (byte)(areaAndtype & 0x3f);
        }

        /// Gets the polygon type. (See: #dtPolyTypes)
        public byte getType()
        {
            return (byte)(areaAndtype >> 6);
        }
    }

    /// Defines the location of detail sub-mesh data within a dtMeshTile.
    public struct dtPolyDetail
    {
        public uint vertBase; ///< The offset of the vertices in the dtMeshTile::detailVerts array.
        public uint triBase; ///< The offset of the triangles in the dtMeshTile::detailTris array.
        public byte vertCount; ///< The number of vertices in the sub-mesh.
        public byte triCount; ///< The number of triangles in the sub-mesh.
    }

    /// Defines a link between polygons.
    /// @note This structure is rarely if ever used by the end user.
    /// @see dtMeshTile
    public struct dtLink
    {
        public dtPolyRef @ref; ///< Neighbour reference. (The neighbor that is linked to.)
        public uint next; ///< Index of the next link.
        public byte edge; ///< Index of the polygon edge that owns this link.
        public byte side; ///< If a boundary link, defines on which side the link is.
        public byte bmin; ///< If a boundary link, defines the minimum sub-edge area.
        public byte bmax; ///< If a boundary link, defines the maximum sub-edge area.
    }

    /// Bounding volume node.
    /// @note This structure is rarely if ever used by the end user.
    /// @see dtMeshTile
    public unsafe struct dtBVNode
    {
        public fixed ushort bmin[3]; ///< Minimum bounds of the node's AABB. [(x, y, z)]
        public fixed ushort bmax[3]; ///< Maximum bounds of the node's AABB. [(x, y, z)]
        public int i; ///< The node's index. (Negative for escape sequence.)
    }

    /// Defines an navigation mesh off-mesh connection within a dtMeshTile object.
    /// An off-mesh connection is a user defined traversable connection made up to two vertices.
    public unsafe struct dtOffMeshConnection
    {
        /// The endpoints of the connection. [(ax, ay, az, bx, by, bz)]
        public fixed float pos[6];

        /// The radius of the endpoints. [Limit: >= 0]
        public float rad;

        /// The polygon reference of the connection within the tile.
        public ushort poly;

        /// Link flags. 
        /// @note These are not the connection's user defined flags. Those are assigned via the 
        /// connection's dtPoly definition. These are link flags used for internal purposes.
        public byte flags;

        /// End point side.
        public byte side;

        /// The id of the offmesh connection. (User assigned when the navigation mesh is built.)
        public uint userId;
    }

    /// Provides high level information related to a dtMeshTile object.
    /// @ingroup detour
    public unsafe struct dtMeshHeader
    {
        public int magic; ///< Tile magic number. (Used to identify the data format.)
        public int version; ///< Tile data format version number.
        public int x; ///< The x-position of the tile within the dtNavMesh tile grid. (x, y, layer)
        public int y; ///< The y-position of the tile within the dtNavMesh tile grid. (x, y, layer)
        public int layer; ///< The layer of the tile within the dtNavMesh tile grid. (x, y, layer)
        public uint userId; ///< The user defined id of the tile.
        public int polyCount; ///< The number of polygons in the tile.
        public int vertCount; ///< The number of vertices in the tile.
        public int maxLinkCount; ///< The number of allocated links.
        public int detailMeshCount; ///< The number of sub-meshes in the detail mesh.

        /// The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
        public int detailVertCount;

        public int detailTriCount; ///< The number of triangles in the detail mesh.
        public int bvNodeCount; ///< The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
        public int offMeshConCount; ///< The number of off-mesh connections.
        public int offMeshBase; ///< The index of the first polygon which is an off-mesh connection.
        public float walkableHeight; ///< The height of the agents using the tile.
        public float walkableRadius; ///< The radius of the agents using the tile.
        public float walkableClimb; ///< The maximum climb height of the agents using the tile.
        public fixed float bmin[3]; ///< The minimum bounds of the tile's AABB. [(x, y, z)]
        public fixed float bmax[3]; ///< The maximum bounds of the tile's AABB. [(x, y, z)]

        /// The bounding volume quantization factor. 
        public float bvQuantFactor;
    }

    /// Defines a navigation mesh tile.
    /// @ingroup detour
    public unsafe struct dtMeshTile
    {
        public uint salt; ///< Counter describing modifications to the tile.

        public uint linksFreeList; ///< Index to the next free link.
        public dtMeshHeader* header; ///< The tile header.
        public dtPoly* polys; ///< The tile polygons. [Size: dtMeshHeader::polyCount]
        public float* verts; ///< The tile vertices. [(x, y, z) * dtMeshHeader::vertCount]
        public dtLink* links; ///< The tile links. [Size: dtMeshHeader::maxLinkCount]
        public dtPolyDetail* detailMeshes; ///< The tile's detail sub-meshes. [Size: dtMeshHeader::detailMeshCount]

        /// The detail mesh's unique vertices. [(x, y, z) * dtMeshHeader::detailVertCount]
        public float* detailVerts;

        /// The detail mesh's triangles. [(vertA, vertB, vertC, triFlags) * dtMeshHeader::detailTriCount].
        /// See dtDetailTriEdgeFlags and dtGetDetailTriEdgeFlags.
        public byte* detailTris;

        /// The tile bounding volume nodes. [Size: dtMeshHeader::bvNodeCount]
        /// (Will be null if bounding volumes are disabled.)
        public dtBVNode* bvTree;

        public dtOffMeshConnection* offMeshCons; ///< The tile off-mesh connections. [Size: dtMeshHeader::offMeshConCount]

        public byte* data; ///< The tile data. (Not directly accessed under normal situations.)
        public int dataSize; ///< Size of the tile data.
        public int flags; ///< Tile flags. (See: #dtTileFlags)
        public dtMeshTile* next; ///< The next free tile, or the next tile in the spatial grid.
    }

    /// Configuration parameters used to define multi-tile navigation meshes.
    /// The values are used to allocate space during the initialization of a navigation mesh.
    /// @see dtNavMesh::init()
    /// @ingroup detour
    public unsafe struct dtNavMeshParams
    {
        public fixed float orig[3]; ///< The world space origin of the navigation mesh's tile space. [(x, y, z)]
        public float tileWidth; ///< The width of each tile. (Along the x-axis.)
        public float tileHeight; ///< The height of each tile. (Along the z-axis.)
        public int maxTiles; ///< The maximum number of tiles the navigation mesh can contain. This and maxPolys are used to calculate how many bits are needed to identify tiles and polygons uniquely.
        public int maxPolys; ///< The maximum number of polygons each tile can contain. This and maxTiles are used to calculate how many bits are needed to identify tiles and polygons uniquely.
    }

    /// A navigation mesh based on tiles of convex polygons.
    /// @ingroup detour
    public unsafe class dtNavMesh : System.IDisposable
    {

        //////////////////////////////////////////////////////////////////////////////////////////

        /**
        @class dtNavMesh
        
        The navigation mesh consists of one or more tiles defining three primary types of structural data:
        
        A polygon mesh which defines most of the navigation graph. (See rcPolyMesh for its structure.)
        A detail mesh used for determining surface height on the polygon mesh. (See rcPolyMeshDetail for its structure.)
        Off-mesh connections, which define custom point-to-point edges within the navigation graph.
        
        The general build process is as follows:
        
        -# Create rcPolyMesh and rcPolyMeshDetail data using the Recast build pipeline.
        -# Optionally, create off-mesh connection data.
        -# Combine the source data into a dtNavMeshCreateParams structure.
        -# Create a tile data array using dtCreateNavMeshData().
        -# Allocate at dtNavMesh object and initialize it. (For single tile navigation meshes,
        the tile data is loaded during this step.)
        -# For multi-tile navigation meshes, load the tile data using dtNavMesh::addTile().
        
        Notes:
        
        - This class is usually used in conjunction with the dtNavMeshQuery class for pathfinding.
        - Technically, all navigation meshes are tiled. A 'solo' mesh is simply a navigation mesh initialized 
        to have only a single tile.
        - This class does not implement any asynchronous methods. So the ::dtStatus result of all methods will 
        always contain either a success or failure flag.
        
        @see dtNavMeshQuery, dtCreateNavMeshData, dtNavMeshCreateParams, #dtAllocNavMesh, #dtFreeNavMesh
        */

        public dtNavMesh()
        {
            this.m_tileWidth = 0F;
            this.m_tileHeight = 0F;
            this.m_maxTiles = 0;
            this.m_tileLutSize = 0;
            this.m_tileLutMask = 0;

            m_orig[0] = 0F;
            m_orig[1] = 0F;
            m_orig[2] = 0F;
        }

        ~dtNavMesh()
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

            for (int i = 0; i < m_maxTiles; ++i)
            {
                if ((m_tiles[i].flags & (int)DT_TILE_FREE_DATA) != 0)
                {
                    dtFree(m_tiles[i].data);
                    m_tiles[i].data = null;
                    m_tiles[i].dataSize = 0;
                }
            }
            dtFree(m_posLookup);
            dtFree(m_tiles);

            _disposed = true;
        }

        /// @name Initialization and Tile Management

        /// Initializes the navigation mesh for tiled use.
        ///  @param[in]    params        Initialization parameters.
        /// @return The status flags for the operation.
        public uint init(dtNavMeshParams* @params)
        {
            m_params = *@params;
            dtVcopy(m_orig, new ReadOnlySpan<float>(@params->orig, 3));
            m_tileWidth = @params->tileWidth;
            m_tileHeight = @params->tileHeight;

            // Init tiles
            m_maxTiles = @params->maxTiles;
            m_tileLutSize = (int)dtNextPow2((uint)(@params->maxTiles / 4));
            if (m_tileLutSize == 0)
                m_tileLutSize = 1;
            m_tileLutMask = m_tileLutSize - 1;

            m_tiles = (dtMeshTile*)dtAlloc(sizeof(dtMeshTile) * m_maxTiles);
            if (m_tiles == null)
                return DT_FAILURE | DT_OUT_OF_MEMORY;
            m_posLookup = (dtMeshTile**)dtAlloc(sizeof(dtMeshTile*) * m_tileLutSize);
            if (m_posLookup == null)
                return DT_FAILURE | DT_OUT_OF_MEMORY;

            new Span<byte>(m_tiles, sizeof(dtMeshTile) * m_maxTiles).Fill(0);
            new Span<byte>(m_posLookup, sizeof(dtMeshTile*) * m_tileLutSize).Fill(0);
            m_nextFree = null;
            for (int i = m_maxTiles - 1; i >= 0; --i)
            {
                m_tiles[i].salt = 1;
                m_tiles[i].next = m_nextFree;
                m_nextFree = &m_tiles[i];
            }

            return DT_SUCCESS;
        }

        /// Initializes the navigation mesh for single tile use.
        ///  @param[in]    data        Data of the new tile. (See: #dtCreateNavMeshData)
        ///  @param[in]    dataSize    The data size of the new tile.
        ///  @param[in]    flags        The tile flags. (See: #dtTileFlags)
        /// @return The status flags for the operation.
        ///  @see dtCreateNavMeshData
        public uint init(byte* data, int dataSize, int flags)
        {
            // Make sure the data is in right format.
            dtMeshHeader* header = (dtMeshHeader*)data;
            if (header->magic != DT_NAVMESH_MAGIC)
                return DT_FAILURE | DT_WRONG_MAGIC;
            if (header->version != DT_NAVMESH_VERSION)
                return DT_FAILURE | DT_WRONG_VERSION;

            dtNavMeshParams @params = new dtNavMeshParams();
            dtVcopy(@params.orig, header->bmin);
            @params.tileWidth = header->bmax[0] - header->bmin[0];
            @params.tileHeight = header->bmax[2] - header->bmin[2];
            @params.maxTiles = 1;
            @params.maxPolys = header->polyCount;

            uint status = init(&@params);
            if (dtStatusFailed(status))
                return status;

            dtPolyRef result = 0;
            return addTile(data, dataSize, flags, 0, ref result);
        }

        /// The navigation mesh initialization params.

        /// @par
        ///
        /// @note The parameters are created automatically when the single tile
        /// initialization is performed.
        public dtNavMeshParams getParams()
        {
            return m_params;
        }

        /// Adds a tile to the navigation mesh.
        ///  @param[in]        data        Data for the new tile mesh. (See: #dtCreateNavMeshData)
        ///  @param[in]        dataSize    Data size of the new tile mesh.
        ///  @param[in]        flags        Tile flags. (See: #dtTileFlags)
        ///  @param[in]        lastRef        The desired reference for the tile. (When reloading a tile.) [opt] [Default: 0]
        ///  @param[out]    result        The tile reference. (If the tile was succesfully added.) [opt]
        /// @return The status flags for the operation.

        /// @par
        ///
        /// The add operation will fail if the data is in the wrong format, the allocated tile
        /// space is full, or there is a tile already at the specified reference.
        ///
        /// The lastRef parameter is used to restore a tile with the same tile
        /// reference it had previously used.  In this case the #dtPolyRef's for the
        /// tile will be restored to the same values they were before the tile was 
        /// removed.
        ///
        /// The nav mesh assumes exclusive access to the data passed and will make
        /// changes to the dynamic portion of the data. For that reason the data
        /// should not be reused in other nav meshes until the tile has been successfully
        /// removed from this nav mesh.
        ///
        /// @see dtCreateNavMeshData, #removeTile
        public dtStatus addTile(byte* data, int dataSize, int flags, dtTileRef lastRef, ref dtTileRef result)
        {
            // Make sure the data is in right format.
            dtMeshHeader* header = (dtMeshHeader*)data;
            if (header->magic != DT_NAVMESH_MAGIC)
                return DT_FAILURE | DT_WRONG_MAGIC;
            if (header->version != DT_NAVMESH_VERSION)
                return DT_FAILURE | DT_WRONG_VERSION;

            // Make sure the location is free.
            if (getTileAt(header->x, header->y, header->layer) != null)
                return DT_FAILURE | DT_ALREADY_OCCUPIED;

            // Allocate a tile.
            dtMeshTile* tile = null;
            if (lastRef == 0)
            {
                if (m_nextFree != null)
                {
                    tile = m_nextFree;
                    m_nextFree = tile->next;
                    tile->next = null;
                }
            }
            else
            {
                // Try to relocate the tile to specific index with same salt.
                int tileIndex = (int)decodePolyIdTile((dtPolyRef)lastRef);
                if (tileIndex >= m_maxTiles)
                    return DT_FAILURE | DT_OUT_OF_MEMORY;
                // Try to find the specific tile id from the free list.
                dtMeshTile* target = &m_tiles[tileIndex];
                dtMeshTile* prev = null;
                tile = m_nextFree;
                while (tile != null && tile != target)
                {
                    prev = tile;
                    tile = tile->next;
                }
                // Could not find the correct location.
                if (tile != target)
                    return DT_FAILURE | DT_OUT_OF_MEMORY;
                // Remove from freelist
                if (prev == null)
                    m_nextFree = tile->next;
                else
                    prev->next = tile->next;

                // Restore salt.
                tile->salt = decodePolyIdSalt((dtPolyRef)lastRef);
            }

            // Make sure we could allocate a tile.
            if (tile == null)
                return DT_FAILURE | DT_OUT_OF_MEMORY;

            // Insert tile into the position lut.
            int h = DetourCommon.computeTileHash(header->x, header->y, m_tileLutMask);
            tile->next = m_posLookup[h];
            m_posLookup[h] = tile;

            // Patch header pointers.
            int headerSize = dtAlign4(sizeof(dtMeshHeader));
            int vertsSize = dtAlign4(sizeof(float) * 3 * header->vertCount);
            int polysSize = dtAlign4(sizeof(dtPoly) * header->polyCount);
            int linksSize = dtAlign4(sizeof(dtLink) * (header->maxLinkCount));
            int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail) * header->detailMeshCount);
            int detailVertsSize = dtAlign4(sizeof(float) * 3 * header->detailVertCount);
            int detailTrisSize = dtAlign4(sizeof(byte) * 4 * header->detailTriCount);
            int bvtreeSize = dtAlign4(sizeof(dtBVNode) * header->bvNodeCount);
            int offMeshLinksSize = dtAlign4(sizeof(dtOffMeshConnection) * header->offMeshConCount);

            byte* d = data + headerSize;
            tile->verts = dtGetThenAdvanceBufferPointer<float>(ref d, vertsSize);
            tile->polys = dtGetThenAdvanceBufferPointer<dtPoly>(ref d, polysSize);
            tile->links = dtGetThenAdvanceBufferPointer<dtLink>(ref d, linksSize);
            tile->detailMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(ref d, detailMeshesSize);
            tile->detailVerts = dtGetThenAdvanceBufferPointer<float>(ref d, detailVertsSize);
            tile->detailTris = dtGetThenAdvanceBufferPointer<byte>(ref d, detailTrisSize);
            tile->bvTree = dtGetThenAdvanceBufferPointer<dtBVNode>(ref d, bvtreeSize);
            tile->offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(ref d, offMeshLinksSize);

            // If there are no items in the bvtree, reset the tree pointer.
            if (bvtreeSize == 0)
                tile->bvTree = null;

            // Build links freelist
            tile->linksFreeList = 0;
            tile->links[header->maxLinkCount - 1].next = DT_NULL_LINK;
            for (int i = 0; i < header->maxLinkCount - 1; ++i)
                tile->links[i].next = (uint)(i + 1);

            // Init tile.
            tile->header = header;
            tile->data = data;
            tile->dataSize = dataSize;
            tile->flags = flags;

            connectIntLinks(tile);

            // Base off-mesh connections to their starting polygons and connect connections inside the tile.
            baseOffMeshLinks(tile);
            connectExtOffMeshLinks(tile, tile, -1);

            // Create connections with neighbour tiles.
            const int MAX_NEIS = 32;
            dtMeshTile** neis = stackalloc dtMeshTile*[MAX_NEIS];
            int nneis;

            // Connect with layers in current tile.
            nneis = getTilesAt(header->x, header->y, neis, MAX_NEIS);
            for (int j = 0; j < nneis; ++j)
            {
                if (neis[j] == tile)
                    continue;

                connectExtLinks(tile, neis[j], -1);
                connectExtLinks(neis[j], tile, -1);
                connectExtOffMeshLinks(tile, neis[j], -1);
                connectExtOffMeshLinks(neis[j], tile, -1);
            }

            // Connect with neighbour tiles.
            for (int i = 0; i < 8; ++i)
            {
                nneis = getNeighbourTilesAt(header->x, header->y, i, neis, MAX_NEIS);
                for (int j = 0; j < nneis; ++j)
                {
                    connectExtLinks(tile, neis[j], i);
                    connectExtLinks(neis[j], tile, dtOppositeTile(i));
                    connectExtOffMeshLinks(tile, neis[j], i);
                    connectExtOffMeshLinks(neis[j], tile, dtOppositeTile(i));
                }
            }

            result = getTileRef(tile);

            return DT_SUCCESS;
        }

        /// Removes the specified tile from the navigation mesh.
        ///  @param[in]        ref            The reference of the tile to remove.
        ///  @param[out]    data        Data associated with deleted tile.
        ///  @param[out]    dataSize    Size of the data associated with deleted tile.
        /// @return The status flags for the operation.

        /// @par
        ///
        /// This function returns the data for the tile so that, if desired,
        /// it can be added back to the navigation mesh at a later point.
        ///
        /// @see #addTile
        public dtStatus removeTile(dtTileRef @ref, byte** data, out int dataSize)
        {
            dataSize = 0;

            if (@ref == 0)
                return DT_FAILURE | DT_INVALID_PARAM;
            uint tileIndex = decodePolyIdTile((dtPolyRef)@ref);
            uint tileSalt = decodePolyIdSalt((dtPolyRef)@ref);
            if ((int)tileIndex >= m_maxTiles)
                return DT_FAILURE | DT_INVALID_PARAM;
            dtMeshTile* tile = &m_tiles[tileIndex];
            if (tile->salt != tileSalt)
                return DT_FAILURE | DT_INVALID_PARAM;

            // Remove tile from hash lookup.
            int h = DetourCommon.computeTileHash(tile->header->x, tile->header->y, m_tileLutMask);
            dtMeshTile* prev = null;
            dtMeshTile* cur = m_posLookup[h];
            while (cur != null)
            {
                if (cur == tile)
                {
                    if (prev != null)
                        prev->next = cur->next;
                    else
                        m_posLookup[h] = cur->next;
                    break;
                }
                prev = cur;
                cur = cur->next;
            }

            // Remove connections to neighbour tiles.
            const int MAX_NEIS = 32;
            dtMeshTile** neis = stackalloc dtMeshTile*[MAX_NEIS];
            int nneis;

            // Disconnect from other layers in current tile.
            nneis = getTilesAt(tile->header->x, tile->header->y, neis, MAX_NEIS);
            for (int j = 0; j < nneis; ++j)
            {
                if (neis[j] == tile) continue;
                unconnectLinks(neis[j], tile);
            }

            // Disconnect from neighbour tiles.
            for (int i = 0; i < 8; ++i)
            {
                nneis = getNeighbourTilesAt(tile->header->x, tile->header->y, i, neis, MAX_NEIS);
                for (int j = 0; j < nneis; ++j)
                    unconnectLinks(neis[j], tile);
            }

            // Reset tile.
            if ((tile->flags & (int)DT_TILE_FREE_DATA) != 0)
            {
                // Owns data
                dtFree(tile->data);
                tile->data = null;
                tile->dataSize = 0;
                if (data != null)
                    data[0] = null;
                dataSize = 0;
            }
            else
            {
                if (data != null)
                    data[0] = tile->data;
                dataSize = tile->dataSize;
            }

            tile->header = null;
            tile->flags = 0;
            tile->linksFreeList = 0;
            tile->polys = null;
            tile->verts = null;
            tile->links = null;
            tile->detailMeshes = null;
            tile->detailVerts = null;
            tile->detailTris = null;
            tile->bvTree = null;
            tile->offMeshCons = null;

            // Update salt, salt should never be zero.
            tile->salt = (tile->salt + 1) & ((1 << (int)DT_SALT_BITS) - 1);
            if (tile->salt == 0)
                tile->salt++;

            // Add to free list.
            tile->next = m_nextFree;
            m_nextFree = tile;

            return DT_SUCCESS;
        }

        ///
        /// @name Query Functions

        /// Calculates the tile grid location for the specified world position.
        ///  @param[in]    pos  The world position for the query. [(x, y, z)]
        ///  @param[out]    tx        The tile's x-location. (x, y)
        ///  @param[out]    ty        The tile's y-location. (x, y)
        public void calcTileLoc(float* pos, ref int tx, ref int ty)
        {
            tx = (int)Math.Floor((pos[0] - m_orig[0]) / m_tileWidth);
            ty = (int)Math.Floor((pos[2] - m_orig[2]) / m_tileHeight);
        }

        /// Gets the tile at the specified grid location.
        ///  @param[in]    x        The tile's x-location. (x, y, layer)
        ///  @param[in]    y        The tile's y-location. (x, y, layer)
        ///  @param[in]    layer    The tile's layer. (x, y, layer)
        /// @return The tile, or null if the tile does not exist.
        public dtMeshTile* getTileAt(int x, int y, int layer)
        {
            // Find tile based on hash.
            int h = DetourCommon.computeTileHash(x, y, m_tileLutMask);
            dtMeshTile* tile = m_posLookup[h];
            while (tile != null)
            {
                if (tile->header != null &&
                    tile->header->x == x &&
                    tile->header->y == y &&
                    tile->header->layer == layer)
                {
                    return tile;
                }
                tile = tile->next;
            }
            return null;
        }

        /// Gets all tiles at the specified grid location. (All layers.)
        ///  @param[in]        x            The tile's x-location. (x, y)
        ///  @param[in]        y            The tile's y-location. (x, y)
        ///  @param[out]    tiles        A pointer to an array of tiles that will hold the result.
        ///  @param[in]        maxTiles    The maximum tiles the tiles parameter can hold.
        /// @return The number of tiles returned in the tiles array.
        public int getTilesAt(int x, int y, dtMeshTile** tiles, int maxTiles)
        {
            int n = 0;

            // Find tile based on hash.
            int h = computeTileHash(x, y, m_tileLutMask);
            dtMeshTile* tile = m_posLookup[h];
            while (tile != null)
            {
                if (tile->header != null &&
                    tile->header->x == x &&
                    tile->header->y == y)
                {
                    if (n < maxTiles)
                        tiles[n++] = tile;
                }
                tile = tile->next;
            }

            return n;
        }

        /// Gets the tile reference for the tile at specified grid location.
        ///  @param[in]    x        The tile's x-location. (x, y, layer)
        ///  @param[in]    y        The tile's y-location. (x, y, layer)
        ///  @param[in]    layer    The tile's layer. (x, y, layer)
        /// @return The tile reference of the tile, or 0 if there is none.
        public dtTileRef getTileRefAt(int x, int y, int layer)
        {
            // Find tile based on hash.
            int h = computeTileHash(x, y, m_tileLutMask);
            dtMeshTile* tile = m_posLookup[h];
            while (tile != null)
            {
                if (tile->header != null &&
                    tile->header->x == x &&
                    tile->header->y == y &&
                    tile->header->layer == layer)
                {
                    return getTileRef(tile);
                }
                tile = tile->next;
            }
            return 0;
        }

        /// Gets the tile reference for the specified tile.
        ///  @param[in]    tile    The tile.
        /// @return The tile reference of the tile.
        public dtTileRef getTileRef(dtMeshTile* tile)
        {
            if (tile == null)
                return 0;
            uint it = (uint)(tile - m_tiles);
            return (dtTileRef)encodePolyId(tile->salt, it, 0);
        }

        /// Gets the tile for the specified tile reference.
        ///  @param[in]    ref        The tile reference of the tile to retrieve.
        /// @return The tile for the specified reference, or null if the 
        ///        reference is invalid.
        public dtMeshTile* getTileByRef(dtTileRef @ref)
        {
            if (@ref == 0) return null;
            uint tileIndex = decodePolyIdTile((dtPolyRef)@ref);
            uint tileSalt = decodePolyIdSalt((dtPolyRef)@ref);
            if ((int)tileIndex >= m_maxTiles)
                return null;
            dtMeshTile* tile = &m_tiles[tileIndex];
            if (tile->salt != tileSalt)
                return null;
            return tile;
        }

        /// The maximum number of tiles supported by the navigation mesh.
        /// @return The maximum number of tiles supported by the navigation mesh.
        public int getMaxTiles()
        {
            return m_maxTiles;
        }

        /// Gets the tile at the specified index.
        ///  @param[in]    i        The tile index. [Limit: 0 >= index < #getMaxTiles()]
        /// @return The tile at the specified index.
        public dtMeshTile* getTile(int i)
        {
            return &m_tiles[i];
        }

        /// Gets the tile and polygon for the specified polygon reference.
        ///  @param[in]        ref        The reference for the a polygon.
        ///  @param[out]    tile    The tile containing the polygon.
        ///  @param[out]    poly    The polygon.
        /// @return The status flags for the operation.
        public dtStatus getTileAndPolyByRef(dtPolyRef @ref, out dtMeshTile* tile, out dtPoly* poly)
        {
            tile = null;
            poly = null;
            if (@ref == 0) return DT_FAILURE;
            uint salt = 0;
            uint it = 0;
            uint ip = 0;
            decodePolyId(@ref, ref salt, ref it, ref ip);
            if (it >= (uint)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
            if (m_tiles[it].salt != salt || m_tiles[it].header == null) return DT_FAILURE | DT_INVALID_PARAM;
            if (ip >= (uint)m_tiles[it].header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
            tile = &m_tiles[it];
            poly = &m_tiles[it].polys[ip];
            return DT_SUCCESS;
        }

        /// Returns the tile and polygon for the specified polygon reference.
        ///  @param[in]        ref        A known valid reference for a polygon.
        ///  @param[out]    tile    The tile containing the polygon.
        ///  @param[out]    poly    The polygon.

        /// @par
        ///
        /// @warning Only use this function if it is known that the provided polygon
        /// reference is valid. This function is faster than #getTileAndPolyByRef, but
        /// it does not validate the reference.
        public void getTileAndPolyByRefUnsafe(dtPolyRef @ref, out dtMeshTile* tile, out dtPoly* poly)
        {
            uint salt = 0;
            uint it = 0;
            uint ip = 0;
            decodePolyId(@ref, ref salt, ref it, ref ip);
            tile = &m_tiles[it];
            poly = &m_tiles[it].polys[ip];
        }

        /// Checks the validity of a polygon reference.
        ///  @param[in]    ref        The polygon reference to check.
        /// @return True if polygon reference is valid for the navigation mesh.
        public bool isValidPolyRef(dtPolyRef @ref)
        {
            if (@ref == 0) return false;
            uint salt = 0;
            uint it = 0;
            uint ip = 0;
            decodePolyId(@ref, ref salt, ref it, ref ip);
            if (it >= (uint)m_maxTiles) return false;
            if (m_tiles[it].salt != salt || m_tiles[it].header == null) return false;
            if (ip >= (uint)m_tiles[it].header->polyCount) return false;
            return true;
        }

        /// Gets the polygon reference for the tile's base polygon.
        ///  @param[in]    tile        The tile.
        /// @return The polygon reference for the base polygon in the specified tile.

        /// @par
        ///
        /// Example use case:
        /// @code
        ///
        /// const dtPolyRef base = navmesh->getPolyRefBase(tile);
        /// for (int i = 0; i < tile->header->polyCount; ++i)
        /// {
        ///     const dtPoly* p = &tile->polys[i];
        ///     const dtPolyRef ref = base | (dtPolyRef)i;
        ///     
        ///     // Use the reference to access the polygon data.
        /// }
        /// @endcode
        public dtPolyRef getPolyRefBase(dtMeshTile* tile)
        {
            if (tile == null) return 0;
            uint it = (uint)(tile - m_tiles);
            return encodePolyId(tile->salt, it, 0);
        }

        /// Gets the endpoints for an off-mesh connection, ordered by "direction of travel".
        ///  @param[in]        prevRef        The reference of the polygon before the connection.
        ///  @param[in]        polyRef        The reference of the off-mesh connection polygon.
        ///  @param[out]    startPos    The start position of the off-mesh connection. [(x, y, z)]
        ///  @param[out]    endPos        The end position of the off-mesh connection. [(x, y, z)]
        /// @return The status flags for the operation.

        /// @par
        ///
        /// Off-mesh connections are stored in the navigation mesh as special 2-vertex 
        /// polygons with a single edge. At least one of the vertices is expected to be 
        /// inside a normal polygon. So an off-mesh connection is "entered" from a 
        /// normal polygon at one of its endpoints. This is the polygon identified by 
        /// the prevRef parameter.
        public dtStatus getOffMeshConnectionPolyEndPoints(dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos)
        {
            uint salt = 0;
            uint it = 0;
            uint ip = 0;

            if (polyRef == 0)
                return DT_FAILURE;

            // Get current polygon
            decodePolyId(polyRef, ref salt, ref it, ref ip);
            if (it >= (uint)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
            if (m_tiles[it].salt != salt || m_tiles[it].header == null) return DT_FAILURE | DT_INVALID_PARAM;
            dtMeshTile* tile = &m_tiles[it];
            if (ip >= (uint)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
            dtPoly* poly = &tile->polys[ip];

            // Make sure that the current poly is indeed off-mesh link.
            if (poly->getType() != (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
                return DT_FAILURE;

            // Figure out which way to hand out the vertices.
            int idx0 = 0;
            int idx1 = 1;

            // Find link that points to first vertex.
            for (uint i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
            {
                if (tile->links[i].edge == 0)
                {
                    if (tile->links[i].@ref != prevRef)
                    {
                        idx0 = 1;
                        idx1 = 0;
                    }
                    break;
                }
            }

            dtVcopy(startPos, &tile->verts[poly->verts[idx0] * 3]);
            dtVcopy(endPos, &tile->verts[poly->verts[idx1] * 3]);

            return DT_SUCCESS;
        }

        /// Gets the specified off-mesh connection.
        ///  @param[in]    ref        The polygon reference of the off-mesh connection.
        /// @return The specified off-mesh connection, or null if the polygon reference is not valid.
        public dtOffMeshConnection* getOffMeshConnectionByRef(dtPolyRef @ref)
        {
            uint salt = 0;
            uint it = 0;
            uint ip = 0;

            if (@ref == 0) return null;

            // Get current polygon
            decodePolyId(@ref, ref salt, ref it, ref ip);
            if (it >= (uint)m_maxTiles) return null;
            if (m_tiles[it].salt != salt || m_tiles[it].header == null) return null;
            dtMeshTile* tile = &m_tiles[it];
            if (ip >= (uint)tile->header->polyCount) return null;
            dtPoly* poly = &tile->polys[ip];

            // Make sure that the current poly is indeed off-mesh link.
            if (poly->getType() != (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
                return null;

            int idx = (int)ip - tile->header->offMeshBase;
            dtAssert(idx < tile->header->offMeshConCount);
            return &tile->offMeshCons[idx];
        }

        /// @name State Management
        /// These functions do not effect #dtTileRef or #dtPolyRef's. 

        /// Sets the user defined flags for the specified polygon.
        ///  @param[in]    ref        The polygon reference.
        ///  @param[in]    flags    The new flags for the polygon.
        /// @return The status flags for the operation.
        public dtStatus setPolyFlags(dtPolyRef @ref, ushort flags)
        {
            if (@ref == 0) return DT_FAILURE;
            uint salt = 0;
            uint it = 0;
            uint ip = 0;
            decodePolyId(@ref, ref salt, ref it, ref ip);
            if (it >= (uint)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
            if (m_tiles[it].salt != salt || m_tiles[it].header == null) return DT_FAILURE | DT_INVALID_PARAM;
            dtMeshTile* tile = &m_tiles[it];
            if (ip >= (uint)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
            dtPoly* poly = &tile->polys[ip];

            // Change flags.
            poly->flags = flags;

            return DT_SUCCESS;
        }

        /// Gets the user defined flags for the specified polygon.
        ///  @param[in]        ref                The polygon reference.
        ///  @param[out]    resultFlags        The polygon flags.
        /// @return The status flags for the operation.
        public dtStatus getPolyFlags(dtPolyRef @ref, out ushort resultFlags)
        {
            resultFlags = 0;

            if (@ref == 0) return DT_FAILURE;
            uint salt = 0;
            uint it = 0;
            uint ip = 0;
            decodePolyId(@ref, ref salt, ref it, ref ip);
            if (it >= (uint)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
            if (m_tiles[it].salt != salt || m_tiles[it].header == null) return DT_FAILURE | DT_INVALID_PARAM;
            dtMeshTile* tile = &m_tiles[it];
            if (ip >= (uint)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
            dtPoly* poly = &tile->polys[ip];

            resultFlags = poly->flags;

            return DT_SUCCESS;
        }

        /// Sets the user defined area for the specified polygon.
        ///  @param[in]    ref        The polygon reference.
        ///  @param[in]    area    The new area id for the polygon. [Limit: < #DT_MAX_AREAS]
        /// @return The status flags for the operation.
        public dtStatus setPolyArea(dtPolyRef @ref, byte area)
        {
            if (@ref == 0) return DT_FAILURE;
            uint salt = 0;
            uint it = 0;
            uint ip = 0;
            decodePolyId(@ref, ref salt, ref it, ref ip);
            if (it >= (uint)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
            if (m_tiles[it].salt != salt || m_tiles[it].header == null) return DT_FAILURE | DT_INVALID_PARAM;
            dtMeshTile* tile = &m_tiles[it];
            if (ip >= (uint)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
            dtPoly* poly = &tile->polys[ip];

            poly->setArea(area);

            return DT_SUCCESS;
        }

        /// Gets the user defined area for the specified polygon.
        ///  @param[in]        ref            The polygon reference.
        ///  @param[out]    resultArea    The area id for the polygon.
        /// @return The status flags for the operation.
        public dtStatus getPolyArea(dtPolyRef @ref, out byte resultArea)
        {
            resultArea = 0;

            if (@ref == 0) return DT_FAILURE;
            uint salt = 0;
            uint it = 0;
            uint ip = 0;
            decodePolyId(@ref, ref salt, ref it, ref ip);
            if (it >= (uint)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
            if (m_tiles[it].salt != salt || m_tiles[it].header == null) return DT_FAILURE | DT_INVALID_PARAM;
            dtMeshTile* tile = &m_tiles[it];
            if (ip >= (uint)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
            dtPoly* poly = &tile->polys[ip];

            resultArea = poly->getArea();

            return DT_SUCCESS;
        }

        /// Gets the size of the buffer required by #storeTileState to store the specified tile's state.
        ///  @param[in]    tile    The tile.
        /// @return The size of the buffer required to store the state.

        ///  @see #storeTileState
        public int getTileStateSize(dtMeshTile* tile)
        {
            if (tile == null) return 0;
            int headerSize = dtAlign4(sizeof(dtTileState));
            int polyStateSize = dtAlign4(sizeof(dtPolyState) * tile->header->polyCount);
            return headerSize + polyStateSize;
        }

        /// Stores the non-structural state of the tile in the specified buffer. (Flags, area ids, etc.)
        ///  @param[in]        tile            The tile.
        ///  @param[out]    data            The buffer to store the tile's state in.
        ///  @param[in]        maxDataSize        The size of the data buffer. [Limit: >= #getTileStateSize]
        /// @return The status flags for the operation.

        /// @par
        ///
        /// Tile state includes non-structural data such as polygon flags, area ids, etc.
        /// @note The state data is only valid until the tile reference changes.
        /// @see #getTileStateSize, #restoreTileState
        public dtStatus storeTileState(dtMeshTile* tile, byte* data, int maxDataSize)
        {
            // Make sure there is enough space to store the state.
            int sizeReq = getTileStateSize(tile);
            if (maxDataSize < sizeReq)
                return DT_FAILURE | DT_BUFFER_TOO_SMALL;

            dtTileState* tileState = dtGetThenAdvanceBufferPointer<dtTileState>(ref data, dtAlign4(sizeof(dtTileState)));
            dtPolyState* polyStates = dtGetThenAdvanceBufferPointer<dtPolyState>(ref data, dtAlign4(sizeof(dtPolyState) * tile->header->polyCount));

            // Store tile state.
            tileState->magic = DT_NAVMESH_STATE_MAGIC;
            tileState->version = DT_NAVMESH_STATE_VERSION;
            tileState->@ref = getTileRef(tile);

            // Store per poly state.
            for (int i = 0; i < tile->header->polyCount; ++i)
            {
                dtPoly* p = &tile->polys[i];
                dtPolyState* s = &polyStates[i];
                s->flags = p->flags;
                s->area = p->getArea();
            }

            return DT_SUCCESS;
        }

        /// Restores the state of the tile.
        ///  @param[in]    tile            The tile.
        ///  @param[in]    data            The new state. (Obtained from #storeTileState.)
        ///  @param[in]    maxDataSize        The size of the state within the data buffer.
        /// @return The status flags for the operation.

        /// @par
        ///
        /// Tile state includes non-structural data such as polygon flags, area ids, etc.
        /// @note This function does not impact the tile's #dtTileRef and #dtPolyRef's.
        /// @see #storeTileState
        public uint restoreTileState(dtMeshTile* tile, byte* data, int maxDataSize)
        {
            // Make sure there is enough space to store the state.
            int sizeReq = getTileStateSize(tile);
            if (maxDataSize < sizeReq)
                return DT_FAILURE | DT_INVALID_PARAM;

            dtTileState* tileState = dtGetThenAdvanceBufferPointer<dtTileState>(ref data, dtAlign4(sizeof(dtTileState)));
            dtPolyState* polyStates = dtGetThenAdvanceBufferPointer<dtPolyState>(ref data, dtAlign4(sizeof(dtPolyState) * tile->header->polyCount));

            // Check that the restore is possible.
            if (tileState->magic != DT_NAVMESH_STATE_MAGIC)
                return DT_FAILURE | DT_WRONG_MAGIC;
            if (tileState->version != DT_NAVMESH_STATE_VERSION)
                return DT_FAILURE | DT_WRONG_VERSION;
            if (tileState->@ref != getTileRef(tile))
                return DT_FAILURE | DT_INVALID_PARAM;

            // Restore per poly state.
            for (int i = 0; i < tile->header->polyCount; ++i)
            {
                dtPoly* p = &tile->polys[i];
                dtPolyState* s = &polyStates[i];
                p->flags = s->flags;
                p->setArea(s->area);
            }

            return DT_SUCCESS;
        }

        /// @name Encoding and Decoding
        /// These functions are generally meant for internal use only.

        /// Derives a standard polygon reference.
        ///  @note This function is generally meant for internal use only.
        ///  @param[in]    salt    The tile's salt value.
        ///  @param[in]    it        The index of the tile.
        ///  @param[in]    ip        The index of the polygon within the tile.
        public dtPolyRef encodePolyId(uint salt, uint it, uint ip)
        {
            return ((dtPolyRef)salt << (int)(DT_POLY_BITS + DT_TILE_BITS)) | ((dtPolyRef)it << (int)DT_POLY_BITS) | (dtPolyRef)ip;
        }

        /// Decodes a standard polygon reference.
        ///  @note This function is generally meant for internal use only.
        ///  @param[in]    ref   The polygon reference to decode.
        ///  @param[out]    salt    The tile's salt value.
        ///  @param[out]    it        The index of the tile.
        ///  @param[out]    ip        The index of the polygon within the tile.
        ///  @see #encodePolyId
        public void decodePolyId(dtPolyRef @ref, ref uint salt, ref uint it, ref uint ip)
        {
            dtPolyRef saltMask = ((dtPolyRef)1 << (int)DT_SALT_BITS) - 1;
            dtPolyRef tileMask = ((dtPolyRef)1 << (int)DT_TILE_BITS) - 1;
            dtPolyRef polyMask = ((dtPolyRef)1 << (int)DT_POLY_BITS) - 1;

            salt = (uint)((@ref >> (int)(DT_POLY_BITS + DT_TILE_BITS)) & saltMask);
            it = (uint)((@ref >> (int)DT_POLY_BITS) & tileMask);
            ip = (uint)(@ref & polyMask);
        }

        /// Extracts a tile's salt value from the specified polygon reference.
        ///  @note This function is generally meant for internal use only.
        ///  @param[in]    ref        The polygon reference.
        ///  @see #encodePolyId
        public uint decodePolyIdSalt(dtPolyRef @ref)
        {
            dtPolyRef saltMask = ((dtPolyRef)1 << (int)DT_SALT_BITS) - 1;
            return (uint)((@ref >> (int)(DT_POLY_BITS + DT_TILE_BITS)) & saltMask);
        }

        /// Extracts the tile's index from the specified polygon reference.
        ///  @note This function is generally meant for internal use only.
        ///  @param[in]    ref        The polygon reference.
        ///  @see #encodePolyId
        public uint decodePolyIdTile(dtPolyRef @ref)
        {
            dtPolyRef tileMask = ((dtPolyRef)1 << (int)DetourCommon.DT_TILE_BITS) - 1;
            return (uint)((@ref >> (int)DetourCommon.DT_POLY_BITS) & tileMask);
        }

        /// Extracts the polygon's index (within its tile) from the specified polygon reference.
        ///  @note This function is generally meant for internal use only.
        ///  @param[in]    ref        The polygon reference.
        ///  @see #encodePolyId
        public uint decodePolyIdPoly(dtPolyRef @ref)
        {
            dtPolyRef polyMask = ((dtPolyRef)1 << (int)DetourCommon.DT_POLY_BITS) - 1;
            return (uint)(@ref & polyMask);
        }

        /// Returns neighbour tile based on side.
        private int getNeighbourTilesAt(int x, int y, int side, dtMeshTile** tiles, int maxTiles)
        {
            int nx = x;
            int ny = y;
            switch (side)
            {
                case 0:
                    nx++;
                    break;
                case 1:
                    nx++;
                    ny++;
                    break;
                case 2:
                    ny++;
                    break;
                case 3:
                    nx--;
                    ny++;
                    break;
                case 4:
                    nx--;
                    break;
                case 5:
                    nx--;
                    ny--;
                    break;
                case 6:
                    ny--;
                    break;
                case 7:
                    nx++;
                    ny--;
                    break;
            };

            return getTilesAt(nx, ny, tiles, maxTiles);
        }

        private int findConnectingPolys(float* va, float* vb, dtMeshTile* tile, int side, dtPolyRef* con, float* conarea, int maxcon)
        {
            if (tile == null) return 0;

            float* amin = stackalloc float[2];
            float* amax = stackalloc float[2];
            calcSlabEndPoints(va, vb, amin, amax, side);
            float apos = getSlabCoord(va, side);

            // Remove links pointing to 'side' and compact the links array.
            float* bmin = stackalloc float[2];
            float* bmax = stackalloc float[2];
            ushort m = (ushort)(DT_EXT_LINK | (ushort)side);
            int n = 0;

            dtPolyRef @base = getPolyRefBase(tile);

            for (int i = 0; i < tile->header->polyCount; ++i)
            {
                dtPoly* poly = &tile->polys[i];
                int nv = poly->vertCount;
                for (int j = 0; j < nv; ++j)
                {
                    // Skip edges which do not point to the right side.
                    if (poly->neis[j] != m) continue;

                    float* vc = &tile->verts[poly->verts[j] * 3];
                    float* vd = &tile->verts[poly->verts[(j + 1) % nv] * 3];
                    float bpos = getSlabCoord(vc, side);

                    // Segments are not close enough.
                    if (Math.Abs(apos - bpos) > 0.01f)
                        continue;

                    // Check if the segments touch.
                    calcSlabEndPoints(vc, vd, bmin, bmax, side);

                    if (!overlapSlabs(amin, amax, bmin, bmax, 0.01f, tile->header->walkableClimb)) continue;

                    // Add return value.
                    if (n < maxcon)
                    {
                        conarea[n * 2 + 0] = Math.Max(amin[0], bmin[0]);
                        conarea[n * 2 + 1] = Math.Min(amax[0], bmax[0]);
                        con[n] = @base | (uint)i;
                        n++;
                    }
                    break;
                }
            }
            return n;
        }

        /// Builds internal polygons links for a tile.
        private void connectIntLinks(dtMeshTile* tile)
        {
            if (tile == null)
                return;

            dtPolyRef @base = getPolyRefBase(tile);

            for (int i = 0; i < tile->header->polyCount; ++i)
            {
                dtPoly* poly = &tile->polys[i];
                poly->firstLink = DT_NULL_LINK;

                if (poly->getType() == (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
                    continue;

                // Build edge links backwards so that the links will be
                // in the linked list from lowest index to highest.
                for (int j = poly->vertCount - 1; j >= 0; --j)
                {
                    // Skip hard and non-internal edges.
                    if (poly->neis[j] == 0 || (poly->neis[j] & DT_EXT_LINK) != 1)
                        continue;

                    uint idx = allocLink(tile);
                    if (idx != DT_NULL_LINK)
                    {
                        dtLink* link = &tile->links[idx];
                        link->@ref = @base | (uint)(poly->neis[j] - 1);
                        link->edge = (byte)j;
                        link->side = 0xff;
                        link->bmin = link->bmax = 0;
                        // Add to linked list.
                        link->next = poly->firstLink;
                        poly->firstLink = idx;
                    }
                }
            }
        }

        /// Builds internal polygons links for a tile.
        private void baseOffMeshLinks(dtMeshTile* tile)
        {
            if (tile == null) return;

            dtPolyRef @base = getPolyRefBase(tile);

            float* halfExtents = stackalloc float[3];
            float* nearestPt = stackalloc float[3];

            // Base off-mesh connection start points.
            for (int i = 0; i < tile->header->offMeshConCount; ++i)
            {
                dtOffMeshConnection* con = &tile->offMeshCons[i];
                dtPoly* poly = &tile->polys[con->poly];

                halfExtents[0] = con->rad;
                halfExtents[1] = tile->header->walkableClimb;
                halfExtents[2] = con->rad;

                // Find polygon to connect to.
                float* p = &con->pos[0]; // First vertex
                dtPolyRef @ref = findNearestPolyInTile(tile, p, halfExtents, nearestPt);
                if (@ref == 0) continue;
                // findNearestPoly may return too optimistic results, further check to make sure.
                if ((nearestPt[0] - p[0]) * (nearestPt[0] - p[0]) + (nearestPt[2] - p[2]) * (nearestPt[2] - p[2]) > (con->rad) * (con->rad))
                    continue;
                // Make sure the location is on current mesh.
                float* v = &tile->verts[poly->verts[0] * 3];
                dtVcopy(v, nearestPt);

                // Link off-mesh connection to target poly.
                uint idx = allocLink(tile);
                if (idx != DT_NULL_LINK)
                {
                    dtLink* link = &tile->links[idx];
                    link->@ref = @ref;
                    link->edge = (byte)0;
                    link->side = 0xff;
                    link->bmin = link->bmax = 0;
                    // Add to linked list.
                    link->next = poly->firstLink;
                    poly->firstLink = idx;
                }

                // Start end-point is always connect back to off-mesh connection.
                uint tidx = allocLink(tile);
                if (tidx != DT_NULL_LINK)
                {
                    ushort landPolyIdx = (ushort)decodePolyIdPoly(@ref);
                    dtPoly* landPoly = &tile->polys[landPolyIdx];
                    dtLink* link = &tile->links[tidx];
                    link->@ref = @base | (con->poly);
                    link->edge = 0xff;
                    link->side = 0xff;
                    link->bmin = link->bmax = 0;
                    // Add to linked list.
                    link->next = landPoly->firstLink;
                    landPoly->firstLink = tidx;
                }
            }
        }

        /// Builds external polygon links for a tile.
        private void connectExtLinks(dtMeshTile* tile, dtMeshTile* target, int side)
        {
            if (tile == null) return;

            // Connect border links.
            dtPolyRef* nei = stackalloc dtPolyRef[4];
            float* neia = stackalloc float[4 * 2];
            for (int i = 0; i < tile->header->polyCount; ++i)
            {
                dtPoly* poly = &tile->polys[i];

                // Create new links.
                //        unsigned short m = DT_EXT_LINK | (unsigned short)side;

                int nv = poly->vertCount;
                for (int j = 0; j < nv; ++j)
                {
                    // Skip non-portal edges.
                    if ((poly->neis[j] & DT_EXT_LINK) == 0)
                        continue;

                    int dir = (int)(poly->neis[j] & 0xff);
                    if (side != -1 && dir != side)
                        continue;

                    // Create new links
                    float* va = &tile->verts[poly->verts[j] * 3];
                    float* vb = &tile->verts[poly->verts[(j + 1) % nv] * 3];
                    int nnei = findConnectingPolys(va, vb, target, dtOppositeTile(dir), nei, neia, 4);
                    for (int k = 0; k < nnei; ++k)
                    {
                        uint idx = allocLink(tile);
                        if (idx != DT_NULL_LINK)
                        {
                            dtLink* link = &tile->links[idx];
                            link->@ref = nei[k];
                            link->edge = (byte)j;
                            link->side = (byte)dir;

                            link->next = poly->firstLink;
                            poly->firstLink = idx;

                            // Compress portal limits to a byte value.
                            if (dir == 0 || dir == 4)
                            {
                                float tmin = (neia[k * 2 + 0] - va[2]) / (vb[2] - va[2]);
                                float tmax = (neia[k * 2 + 1] - va[2]) / (vb[2] - va[2]);
                                if (tmin > tmax)
                                    dtSwap(ref tmin, ref tmax);
                                link->bmin = (byte)(dtClamp(tmin, 0.0f, 1.0f) * 255.0f);
                                link->bmax = (byte)(dtClamp(tmax, 0.0f, 1.0f) * 255.0f);
                            }
                            else if (dir == 2 || dir == 6)
                            {
                                float tmin = (neia[k * 2 + 0] - va[0]) / (vb[0] - va[0]);
                                float tmax = (neia[k * 2 + 1] - va[0]) / (vb[0] - va[0]);
                                if (tmin > tmax)
                                    dtSwap(ref tmin, ref tmax);
                                link->bmin = (byte)(dtClamp(tmin, 0.0f, 1.0f) * 255.0f);
                                link->bmax = (byte)(dtClamp(tmax, 0.0f, 1.0f) * 255.0f);
                            }
                        }
                    }
                }
            }
        }

        /// Builds external polygon links for a tile.
        private void connectExtOffMeshLinks(dtMeshTile* tile, dtMeshTile* target, int side)
        {
            if (tile == null) return;

            // Connect off-mesh links.
            // We are interested on links which land from target tile to this tile.
            byte oppositeSide = (byte)((side == -1) ? 0xff : (byte)dtOppositeTile(side));

            float* halfExtents = stackalloc float[3];
            float* nearestPt = stackalloc float[3];
            for (int i = 0; i < target->header->offMeshConCount; ++i)
            {
                dtOffMeshConnection* targetCon = &target->offMeshCons[i];
                if (targetCon->side != oppositeSide)
                    continue;

                dtPoly* targetPoly = &target->polys[targetCon->poly];
                // Skip off-mesh connections which start location could not be connected at all.
                if (targetPoly->firstLink == DT_NULL_LINK)
                    continue;

                halfExtents[0] = targetCon->rad;
                halfExtents[1] = target->header->walkableClimb;
                halfExtents[2] = targetCon->rad;

                // Find polygon to connect to.
                float* p = &targetCon->pos[3];
                dtPolyRef @ref = findNearestPolyInTile(tile, p, halfExtents, nearestPt);
                if (@ref == 0)
                    continue;
                // findNearestPoly may return too optimistic results, further check to make sure.
                if ((nearestPt[0] - p[0]) * (nearestPt[0] - p[0]) + (nearestPt[2] - p[2]) * (nearestPt[2] - p[2]) > (targetCon->rad) * (targetCon->rad))
                    continue;
                // Make sure the location is on current mesh.
                float* v = &target->verts[targetPoly->verts[1] * 3];
                dtVcopy(v, nearestPt);

                // Link off-mesh connection to target poly.
                uint idx = allocLink(target);
                if (idx != DT_NULL_LINK)
                {
                    dtLink* link = &target->links[idx];
                    link->@ref = @ref;
                    link->edge = (byte)1;
                    link->side = oppositeSide;
                    link->bmin = link->bmax = 0;
                    // Add to linked list.
                    link->next = targetPoly->firstLink;
                    targetPoly->firstLink = idx;
                }

                // Link target poly to off-mesh connection.
                if ((targetCon->flags & DT_OFFMESH_CON_BIDIR) != 0)
                {
                    uint tidx = allocLink(tile);
                    if (tidx != DT_NULL_LINK)
                    {
                        ushort landPolyIdx = (ushort)decodePolyIdPoly(@ref);
                        dtPoly* landPoly = &tile->polys[landPolyIdx];
                        dtLink* link = &tile->links[tidx];
                        link->@ref = getPolyRefBase(target) | (targetCon->poly);
                        link->edge = 0xff;
                        link->side = (byte)(side == -1 ? 0xff : side);
                        link->bmin = link->bmax = 0;
                        // Add to linked list.
                        link->next = landPoly->firstLink;
                        landPoly->firstLink = tidx;
                    }
                }
            }

        }

        /// Removes external links at specified side.
        private void unconnectLinks(dtMeshTile* tile, dtMeshTile* target)
        {
            if (tile == null || target == null) return;

            uint targetNum = decodePolyIdTile(getTileRef(target));

            for (int i = 0; i < tile->header->polyCount; ++i)
            {
                dtPoly* poly = &tile->polys[i];
                uint j = poly->firstLink;
                uint pj = DT_NULL_LINK;
                while (j != DT_NULL_LINK)
                {
                    if (decodePolyIdTile(tile->links[j].@ref) == targetNum)
                    {
                        // Remove link.
                        uint nj = tile->links[j].next;
                        if (pj == DT_NULL_LINK)
                            poly->firstLink = nj;
                        else
                            tile->links[pj].next = nj;
                        freeLink(tile, j);
                        j = nj;
                    }
                    else
                    {
                        // Advance
                        pj = j;
                        j = tile->links[j].next;
                    }
                }
            }
        }


        // TODO: These methods are duplicates from dtNavMeshQuery, but are needed for off-mesh connection finding.

        /// Queries polygons within a tile.
        private int queryPolygonsInTile(dtMeshTile* tile, float* qmin, float* qmax, dtPolyRef* polys, int maxPolys)
        {
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
                bmin[0] = (ushort)((ushort)(qfac * minx) & 0xfffe);
                bmin[1] = (ushort)((ushort)(qfac * miny) & 0xfffe);
                bmin[2] = (ushort)((ushort)(qfac * minz) & 0xfffe);
                bmax[0] = (ushort)((ushort)(qfac * maxx + 1) | 1);
                bmax[1] = (ushort)((ushort)(qfac * maxy + 1) | 1);
                bmax[2] = (ushort)((ushort)(qfac * maxz + 1) | 1);

                // Traverse tree
                dtPolyRef @base = getPolyRefBase(tile);
                int n = 0;
                while (node < end)
                {
                    bool overlap = dtOverlapQuantBounds(bmin, bmax, node->bmin, node->bmax);
                    bool isLeafNode = node->i >= 0;

                    if (isLeafNode && overlap)
                    {
                        if (n < maxPolys)
                            polys[n++] = @base | (uint)node->i;
                    }

                    if (overlap || isLeafNode)
                        node++;
                    else
                    {
                        int escapeIndex = -node->i;
                        node += escapeIndex;
                    }
                }

                return n;
            }
            else
            {
                float* bmin = stackalloc float[3];
                float* bmax = stackalloc float[3];
                int n = 0;
                dtPolyRef @base = getPolyRefBase(tile);
                for (int i = 0; i < tile->header->polyCount; ++i)
                {
                    dtPoly* p = &tile->polys[i];
                    // Do not return off-mesh connection polygons.
                    if (p->getType() == (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
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
                        if (n < maxPolys)
                            polys[n++] = @base | (uint)i;
                    }
                }
                return n;
            }
        }

        /// Find nearest polygon within a tile.
        private dtPolyRef findNearestPolyInTile(dtMeshTile* tile, float* center, float* halfExtents, float* nearestPt)
        {
            float* bmin = stackalloc float[3];
            float* bmax = stackalloc float[3];
            dtVsub(bmin, center, halfExtents);
            dtVadd(bmax, center, halfExtents);

            // Get nearby polygons from proximity grid.
            dtPolyRef* polys = stackalloc dtPolyRef[128];
            int polyCount = queryPolygonsInTile(tile, bmin, bmax, polys, 128);

            float* closestPtPoly = stackalloc float[3];
            float* diff = stackalloc float[3];

            // Find nearest polygon amongst the nearby polygons.
            dtPolyRef nearest = 0;
            float nearestDistanceSqr = float.MaxValue;
            for (int i = 0; i < polyCount; ++i)
            {
                dtPolyRef @ref = polys[i];
                bool posOverPoly = false;
                float d;
                closestPointOnPoly(@ref, center, closestPtPoly, out posOverPoly);

                // If a point is directly over a polygon and closer than
                // climb height, favor that instead of straight line nearest point.
                dtVsub(diff, center, closestPtPoly);
                if (posOverPoly)
                {
                    d = Math.Abs(diff[1]) - tile->header->walkableClimb;
                    d = d > 0F ? d * d : 0F;
                }
                else
                {
                    d = dtVlenSqr(diff);
                }

                if (d < nearestDistanceSqr)
                {
                    dtVcopy(nearestPt, closestPtPoly);
                    nearestDistanceSqr = d;
                    nearest = @ref;
                }
            }

            return nearest;
        }

        /// Returns whether position is over the poly and the height at the position if so.
        internal bool getPolyHeight(dtMeshTile* tile, dtPoly* poly, float* pos, ref float height)
        {
            // Off-mesh connections do not have detail polys and getting height
            // over them does not make sense.
            if (poly->getType() == (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
                return false;

            uint ip = (uint)(poly - tile->polys);
            dtPolyDetail* pd = &tile->detailMeshes[ip];

            float* verts = stackalloc float[DT_VERTS_PER_POLYGON * 3];
            int nv = poly->vertCount;
            for (int i = 0; i < nv; ++i)
                dtVcopy(&verts[i * 3], &tile->verts[poly->verts[i] * 3]);

            if (!dtPointInPolygon(pos, verts, nv))
                return false;

            float** v = stackalloc float*[3];
            // Find height at the location.
            for (int j = 0; j < pd->triCount; ++j)
            {
                byte* t = &tile->detailTris[(pd->triBase + j) * 4];
                for (int k = 0; k < 3; ++k)
                {
                    if (t[k] < poly->vertCount)
                        v[k] = &tile->verts[poly->verts[t[k]] * 3];
                    else
                        v[k] = &tile->detailVerts[(pd->vertBase + (t[k] - poly->vertCount)) * 3];
                }
                float h = 0;
                if (dtClosestHeightPointTriangle(pos, v[0], v[1], v[2], ref h))
                {
                    height = h;
                    return true;
                }
            }

            // If all triangle checks failed above (can happen with degenerate triangles
            // or larger floating point values) the point is on an edge, so just select
            // closest. This should almost never happen so the extra iteration here is
            // ok.
            float* closest = stackalloc float[3];
            closestPointOnDetailEdges(false, tile, poly, pos, closest);
            height = closest[1];
            return true;
        }

        /// Returns closest point on polygon.
        internal void closestPointOnPoly(dtPolyRef @ref, float* pos, float* closest, out bool posOverPoly)
        {
            dtMeshTile* tile = null;
            dtPoly* poly = null;
            getTileAndPolyByRefUnsafe(@ref, out tile, out poly);

            dtVcopy(closest, pos);
            if (getPolyHeight(tile, poly, pos, ref closest[1]))
            {
                posOverPoly = true;
                return;
            }

            posOverPoly = false;

            // Off-mesh connections don't have detail polygons.
            if (poly->getType() == (byte)DT_POLYTYPE_OFFMESH_CONNECTION)
            {
                float* v0 = &tile->verts[poly->verts[0] * 3];
                float* v1 = &tile->verts[poly->verts[1] * 3];
                float t = 0;
                dtDistancePtSegSqr2D(pos, v0, v1, ref t);
                dtVlerp(closest, v0, v1, t);
                return;
            }

            // Outside poly that is not an offmesh connection.
            closestPointOnDetailEdges(true, tile, poly, pos, closest);
        }

        private dtNavMeshParams m_params; ///< Current initialization params. TODO: do not store this info twice.
        private float[] m_orig = new float[3]; ///< Origin of the tile (0,0)
        private float m_tileWidth; ///< Dimensions of each tile.
        private float m_tileHeight;
        private int m_maxTiles; ///< Max number of tiles.
        private int m_tileLutSize; ///< Tile hash lookup size (must be pot).
        private int m_tileLutMask; ///< Tile hash lookup mask.

        private dtMeshTile** m_posLookup; ///< Tile hash lookup.
        private dtMeshTile* m_nextFree; ///< Freelist of tiles.
        private dtMeshTile* m_tiles; ///< List of tiles.
    }

    public struct dtTileState
    {
        public int magic; // Magic number, used to identify the data.
        public int version; // Data version number.
        public dtTileRef @ref; // Tile ref at the time of storing the data.
    }

    public struct dtPolyState
    {
        public ushort flags; // Flags (see dtPolyFlags).
        public byte area; // Area ID of the polygon.
    }
}
