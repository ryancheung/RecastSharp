// MIT License - Copyright (C) ryancheung and the FelCore team
// This file is subject to the terms and conditions defined in
// file 'LICENSE', which is part of this source code package.

using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using size_t = System.Int32;
using dtPolyRef = System.UInt64;
using dtStatus = System.UInt32;

namespace RecastSharp
{
    using static dtDetailTriEdgeFlags;
    using static QuickSort;
    public static unsafe class DetourCommon
    {
        /// Sets the base custom allocation functions to be used by Detour.
        ///  @param[in]        allocFunc    The memory allocation function to be used by #dtAlloc
        ///  @param[in]        freeFunc    The memory de-allocation function to be used by #dtFree
        public static void dtAllocSetCustom(delegate* managed<int, IntPtr> allocFunc, delegate* managed<IntPtr, void> freeFunc)
        {
            sAllocFunc = allocFunc != default ? allocFunc : &dtAllocDefault;
            sFreeFunc = freeFunc != default ? freeFunc : &dtFreeDefault;
        }

        /// Allocates a memory block.
        ///  @param[in]        size    The size, in bytes of memory, to allocate.
        ///  @param[in]        hint    A hint to the allocator on how long the memory is expected to be in use.
        ///  @return A pointer to the beginning of the allocated memory block, or null if the allocation failed.
        /// @see dtFree
        public static IntPtr dtAlloc(size_t size)
        {
            return sAllocFunc(size);
        }

        /// Deallocates a memory block.
        ///  @param[in]        ptr        A pointer to a memory block previously allocated using #dtAlloc.
        /// @see dtAlloc
        public static void dtFree(IntPtr ptr)
        {
            sFreeFunc(ptr);
        }

        public static void dtFree(void* ptr)
        {
            sFreeFunc((IntPtr)ptr);
        }


        internal static IntPtr dtAllocDefault(size_t size)
        {
            return Marshal.AllocHGlobal(size);
        }

        internal static void dtFreeDefault(IntPtr ptr)
        {
            Marshal.FreeHGlobal(ptr);
        }

        internal static delegate* managed<int, IntPtr> sAllocFunc = &dtAllocDefault;
        internal static delegate* managed<IntPtr, void> sFreeFunc = &dtFreeDefault;


        public static void dtAssert(bool condition)
        {
            Debug.Assert(condition);
        }

        public static void dtAssert(bool condition, string? message)
        {
            Debug.Assert(condition, message);
        }


        /// Swaps the values of the two parameters.
        ///  @param[in,out]    a    Value A
        ///  @param[in,out]    b    Value B
        public static void dtSwap<T>(ref T a, ref T b)
        {
            T t = a;
            a = b;
            b = t;
        }

        /// Clamps the value to the specified range.
        ///  @param[in]        v    The value to clamp.
        ///  @param[in]        mn    The minimum permitted return value.
        ///  @param[in]        mx    The maximum permitted return value.
        ///  @return The value, clamped to the specified range.
        public static int dtClamp(int v, int mn, int mx)
        {
            return v < mn ? mn : (v > mx ? mx : v);
        }
        public static uint dtClamp(uint v, uint mn, uint mx)
        {
            return v < mn ? mn : (v > mx ? mx : v);
        }
        public static byte dtClamp(byte v, byte mn, byte mx)
        {
            return v < mn ? mn : (v > mx ? mx : v);
        }
        public static ushort dtClamp(ushort v, ushort mn, ushort mx)
        {
            return v < mn ? mn : (v > mx ? mx : v);
        }
        public static float dtClamp(float v, float mn, float mx)
        {
            return v < mn ? mn : (v > mx ? mx : v);
        }

        ///
        /// @name Vector helper functions.
        ///

        /// Derives the cross product of two vectors. (@p v1 x @p v2)
        ///  @param[out]    dest    The cross product. [(x, y, z)]
        ///  @param[in]        v1        A Vector [(x, y, z)]
        ///  @param[in]        v2        A vector [(x, y, z)]
        public static void dtVcross(float* dest, float* v1, float* v2)
        {
            dtVcross(new Span<float>(dest, 3), new ReadOnlySpan<float>(v1, 3), new ReadOnlySpan<float>(v2, 3));
        }
        public static void dtVcross(Span<float> dest, ReadOnlySpan<float> v1, ReadOnlySpan<float> v2)
        {
            dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
            dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
            dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
        }

        /// Derives the dot product of two vectors. (@p v1 . @p v2)
        ///  @param[in]        v1    A Vector [(x, y, z)]
        ///  @param[in]        v2    A vector [(x, y, z)]
        /// @return The dot product.
        public static float dtVdot(float* v1, float* v2)
        {
            return dtVdot(new ReadOnlySpan<float>(v1, 3), new ReadOnlySpan<float>(v2, 3));
        }
        public static float dtVdot(ReadOnlySpan<float> v1, ReadOnlySpan<float> v2)
        {
            return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
        }

        /// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
        ///  @param[out]    dest    The result vector. [(x, y, z)]
        ///  @param[in]        v1        The base vector. [(x, y, z)]
        ///  @param[in]        v2        The vector to scale and add to @p v1. [(x, y, z)]
        ///  @param[in]        s        The amount to scale @p v2 by before adding to @p v1.
        public static void dtVmad(float* dest, float* v1, float* v2, float s)
        {
            dtVmad(new Span<float>(dest, 3), new ReadOnlySpan<float>(v1, 3), new ReadOnlySpan<float>(v2, 3), s);
        }
        public static void dtVmad(Span<float> dest, ReadOnlySpan<float> v1, ReadOnlySpan<float> v2, float s)
        {
            dest[0] = v1[0] + v2[0] * s;
            dest[1] = v1[1] + v2[1] * s;
            dest[2] = v1[2] + v2[2] * s;
        }

        /// Performs a linear interpolation between two vectors. (@p v1 toward @p v2)
        ///  @param[out]    dest    The result vector. [(x, y, x)]
        ///  @param[in]        v1        The starting vector.
        ///  @param[in]        v2        The destination vector.
        ///     @param[in]        t        The interpolation factor. [Limits: 0 <= value <= 1.0]
        public static void dtVlerp(float* dest, float* v1, float* v2, float t)
        {
            dtVlerp(new Span<float>(dest, 3), new ReadOnlySpan<float>(v1, 3), new ReadOnlySpan<float>(v2, 3), t);
        }
        public static void dtVlerp(Span<float> dest, ReadOnlySpan<float> v1, ReadOnlySpan<float> v2, float t)
        {
            dest[0] = v1[0] + (v2[0] - v1[0]) * t;
            dest[1] = v1[1] + (v2[1] - v1[1]) * t;
            dest[2] = v1[2] + (v2[2] - v1[2]) * t;
        }

        /// Performs a vector addition. (@p v1 + @p v2)
        ///  @param[out]    dest    The result vector. [(x, y, z)]
        ///  @param[in]        v1        The base vector. [(x, y, z)]
        ///  @param[in]        v2        The vector to add to @p v1. [(x, y, z)]
        public static void dtVadd(float* dest, float* v1, float* v2)
        {
            dtVadd(new Span<float>(dest, 3), new ReadOnlySpan<float>(v1, 3), new ReadOnlySpan<float>(v2, 3));
        }
        public static void dtVadd(Span<float> dest, ReadOnlySpan<float> v1, ReadOnlySpan<float> v2)
        {
            dest[0] = v1[0] + v2[0];
            dest[1] = v1[1] + v2[1];
            dest[2] = v1[2] + v2[2];
        }

        /// Performs a vector subtraction. (@p v1 - @p v2)
        ///  @param[out]    dest    The result vector. [(x, y, z)]
        ///  @param[in]        v1        The base vector. [(x, y, z)]
        ///  @param[in]        v2        The vector to subtract from @p v1. [(x, y, z)]
        public static void dtVsub(float* dest, float* v1, float* v2)
        {
            dtVsub(new Span<float>(dest, 3), new ReadOnlySpan<float>(v1, 3), new ReadOnlySpan<float>(v2, 3));
        }
        public static void dtVsub(Span<float> dest, ReadOnlySpan<float> v1, ReadOnlySpan<float> v2)
        {
            dest[0] = v1[0] - v2[0];
            dest[1] = v1[1] - v2[1];
            dest[2] = v1[2] - v2[2];
        }

        /// Scales the vector by the specified value. (@p v * @p t)
        ///  @param[out]    dest    The result vector. [(x, y, z)]
        ///  @param[in]        v        The vector to scale. [(x, y, z)]
        ///  @param[in]        t        The scaling factor.
        public static void dtVscale(float* dest, float* v, float t)
        {
            dtVscale(new Span<float>(dest, 3), new ReadOnlySpan<float>(v, 3), t);
        }
        public static void dtVscale(Span<float> dest, ReadOnlySpan<float> v, float t)
        {
            dest[0] = v[0] * t;
            dest[1] = v[1] * t;
            dest[2] = v[2] * t;
        }

        /// Selects the minimum value of each element from the specified vectors.
        ///  @param[in,out]    mn    A vector.  (Will be updated with the result.) [(x, y, z)]
        ///  @param[in]    v    A vector. [(x, y, z)]
        public static void dtVmin(float* mn, float* v)
        {
            dtVmin(new Span<float>(mn, 3), new ReadOnlySpan<float>(v, 3));
        }
        public static void dtVmin(Span<float> mn, ReadOnlySpan<float> v)
        {
            mn[0] = Math.Min(mn[0], v[0]);
            mn[1] = Math.Min(mn[1], v[1]);
            mn[2] = Math.Min(mn[2], v[2]);
        }

        /// Selects the maximum value of each element from the specified vectors.
        ///  @param[in,out]    mx    A vector.  (Will be updated with the result.) [(x, y, z)]
        ///  @param[in]        v    A vector. [(x, y, z)]
        public static void dtVmax(float* mx, float* v)
        {
            dtVmax(new Span<float>(mx, 3), new ReadOnlySpan<float>(v, 3));
        }
        public static void dtVmax(Span<float> mx, ReadOnlySpan<float> v)
        {
            mx[0] = Math.Max(mx[0], v[0]);
            mx[1] = Math.Max(mx[1], v[1]);
            mx[2] = Math.Max(mx[2], v[2]);
        }

        /// Sets the vector elements to the specified values.
        ///  @param[out]    dest    The result vector. [(x, y, z)]
        ///  @param[in]        x        The x-value of the vector.
        ///  @param[in]        y        The y-value of the vector.
        ///  @param[in]        z        The z-value of the vector.
        public static void dtVset(float* dest, float x, float y, float z)
        {
            dtVset(new Span<float>(dest, 3), x, y, z);
        }
        public static void dtVset(Span<float> dest, float x, float y, float z)
        {
            dest[0] = x;
            dest[1] = y;
            dest[2] = z;
        }

        /// Performs a vector copy.
        ///  @param[out]    dest    The result. [(x, y, z)]
        ///  @param[in]        a        The vector to copy. [(x, y, z)]
        public static void dtVcopy(float* dest, float* a)
        {
            dtVcopy(new Span<float>(dest, 3), new ReadOnlySpan<float>(a, 3));
        }
        public static void dtVcopy(Span<float> dest, ReadOnlySpan<float> a)
        {
            dest[0] = a[0];
            dest[1] = a[1];
            dest[2] = a[2];
        }

        /// Derives the scalar length of the vector.
        ///  @param[in]        v The vector. [(x, y, z)]
        /// @return The scalar length of the vector.
        public static float dtVlen(float* v)
        {
            return dtVlen(new ReadOnlySpan<float>(v, 3));
        }
        public static float dtVlen(ReadOnlySpan<float> v)
        {
            return (float)Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        }

        /// Derives the square of the scalar length of the vector. (len * len)
        ///  @param[in]        v The vector. [(x, y, z)]
        /// @return The square of the scalar length of the vector.
        public static float dtVlenSqr(float* v)
        {
            return dtVlenSqr(new ReadOnlySpan<float>(v, 3));
        }
        public static float dtVlenSqr(ReadOnlySpan<float> v)
        {
            return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
        }

        /// Returns the distance between two points.
        ///  @param[in]        v1    A point. [(x, y, z)]
        ///  @param[in]        v2    A point. [(x, y, z)]
        /// @return The distance between the two points.
        public static float dtVdist(float* v1, float* v2)
        {
            return dtVdist(new ReadOnlySpan<float>(v1, 3), new ReadOnlySpan<float>(v2, 3));
        }
        public static float dtVdist(ReadOnlySpan<float> v1, ReadOnlySpan<float> v2)
        {
            float dx = v2[0] - v1[0];
            float dy = v2[1] - v1[1];
            float dz = v2[2] - v1[2];
            return (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// Returns the square of the distance between two points.
        ///  @param[in]        v1    A point. [(x, y, z)]
        ///  @param[in]        v2    A point. [(x, y, z)]
        /// @return The square of the distance between the two points.
        public static float dtVdistSqr(float* v1, float* v2)
        {
            return dtVdistSqr(new ReadOnlySpan<float>(v1, 3), new ReadOnlySpan<float>(v2, 3));
        }
        public static float dtVdistSqr(ReadOnlySpan<float> v1, ReadOnlySpan<float> v2)
        {
            float dx = v2[0] - v1[0];
            float dy = v2[1] - v1[1];
            float dz = v2[2] - v1[2];
            return dx * dx + dy * dy + dz * dz;
        }

        /// Derives the distance between the specified points on the xz-plane.
        ///  @param[in]        v1    A point. [(x, y, z)]
        ///  @param[in]        v2    A point. [(x, y, z)]
        /// @return The distance between the point on the xz-plane.
        ///
        /// The vectors are projected onto the xz-plane, so the y-values are ignored.
        public static float dtVdist2D(float* v1, float* v2)
        {
            return dtVdist2D(new ReadOnlySpan<float>(v1, 3), new ReadOnlySpan<float>(v2, 3));
        }
        public static float dtVdist2D(ReadOnlySpan<float> v1, ReadOnlySpan<float> v2)
        {
            float dx = v2[0] - v1[0];
            float dz = v2[2] - v1[2];
            return (float)Math.Sqrt(dx * dx + dz * dz);
        }

        /// Derives the square of the distance between the specified points on the xz-plane.
        ///  @param[in]        v1    A point. [(x, y, z)]
        ///  @param[in]        v2    A point. [(x, y, z)]
        /// @return The square of the distance between the point on the xz-plane.
        public static float dtVdist2DSqr(float* v1, float* v2)
        {
            return dtVdist2DSqr(new ReadOnlySpan<float>(v1, 3), new ReadOnlySpan<float>(v2, 3));
        }
        public static float dtVdist2DSqr(ReadOnlySpan<float> v1, ReadOnlySpan<float> v2)
        {
            float dx = v2[0] - v1[0];
            float dz = v2[2] - v1[2];
            return dx * dx + dz * dz;
        }

        /// Normalizes the vector.
        ///  @param[in,out]    v    The vector to normalize. [(x, y, z)]
        public static void dtVnormalize(float* v)
        {
            dtVnormalize(new Span<float>(v, 3));
        }
        public static void dtVnormalize(Span<float> v)
        {
            float d = 1.0f / (float)Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            v[0] *= d;
            v[1] *= d;
            v[2] *= d;
        }

        /// Performs a 'sloppy' colocation check of the specified points.
        ///  @param[in]        p0    A point. [(x, y, z)]
        ///  @param[in]        p1    A point. [(x, y, z)]
        /// @return True if the points are considered to be at the same location.
        ///
        /// Basically, this function will return true if the specified points are 
        /// close enough to eachother to be considered colocated.
        public static bool dtVequal(float* p0, float* p1)
        {
            return dtVequal(new ReadOnlySpan<float>(p0, 3), new ReadOnlySpan<float>(p1, 3));
        }
        public static bool dtVequal(ReadOnlySpan<float> p0, ReadOnlySpan<float> p1)
        {
            float thr = (1.0f / 16384.0f) * (1.0f / 16384.0f);
            float d = dtVdistSqr(p0, p1);
            return d < thr;
        }

        /// Checks that the specified vector's components are all finite.
        ///  @param[in]        v    A point. [(x, y, z)]
        /// @return True if all of the point's components are finite, i.e. not NaN
        /// or any of the infinities.
        public static bool dtVisfinite(float* v)
        {
            return dtVisfinite(new ReadOnlySpan<float>(v, 3));
        }
        public static bool dtVisfinite(ReadOnlySpan<float> v)
        {
            bool result = float.IsFinite(v[0]) && float.IsFinite(v[1]) && float.IsFinite(v[2]);

            return result;
        }

        /// Checks that the specified vector's 2D components are finite.
        ///  @param[in]        v    A point. [(x, y, z)]
        public static bool dtVisfinite2D(float* v)
        {
            return dtVisfinite2D(new ReadOnlySpan<float>(v, 3));
        }
        public static bool dtVisfinite2D(ReadOnlySpan<float> v)
        {
            bool result = float.IsFinite(v[0]) && float.IsFinite(v[2]);
            return result;
        }

        /// Derives the dot product of two vectors on the xz-plane. (@p u . @p v)
        ///  @param[in]        u        A vector [(x, y, z)]
        ///  @param[in]        v        A vector [(x, y, z)]
        /// @return The dot product on the xz-plane.
        ///
        /// The vectors are projected onto the xz-plane, so the y-values are ignored.
        public static float dtVdot2D(float* u, float* v)
        {
            return dtVdot2D(new ReadOnlySpan<float>(u, 3), new ReadOnlySpan<float>(v, 3));
        }
        public static float dtVdot2D(ReadOnlySpan<float> u, ReadOnlySpan<float> v)
        {
            return u[0] * v[0] + u[2] * v[2];
        }

        /// Derives the xz-plane 2D perp product of the two vectors. (uz*vx - ux*vz)
        ///  @param[in]        u        The LHV vector [(x, y, z)]
        ///  @param[in]        v        The RHV vector [(x, y, z)]
        /// @return The dot product on the xz-plane.
        ///
        /// The vectors are projected onto the xz-plane, so the y-values are ignored.
        public static float dtVperp2D(float* u, float* v)
        {
            return dtVperp2D(new ReadOnlySpan<float>(u, 3), new ReadOnlySpan<float>(v, 3));
        }
        public static float dtVperp2D(ReadOnlySpan<float> u, ReadOnlySpan<float> v)
        {
            return u[2] * v[0] - u[0] * v[2];
        }

        ///
        /// @name Computational geometry helper functions.
        ///

        /// Derives the signed xz-plane area of the triangle ABC, or the relationship of line AB to point C.
        ///  @param[in]        a        Vertex A. [(x, y, z)]
        ///  @param[in]        b        Vertex B. [(x, y, z)]
        ///  @param[in]        c        Vertex C. [(x, y, z)]
        /// @return The signed xz-plane area of the triangle.
        public static float dtTriArea2D(float* a, float* b, float* c)
        {
            return dtTriArea2D(new ReadOnlySpan<float>(a, 3), new ReadOnlySpan<float>(b, 3), new ReadOnlySpan<float>(c, 3));
        }
        public static float dtTriArea2D(ReadOnlySpan<float> a, ReadOnlySpan<float> b, ReadOnlySpan<float> c)
        {
            float abx = b[0] - a[0];
            float abz = b[2] - a[2];
            float acx = c[0] - a[0];
            float acz = c[2] - a[2];
            return acx * abz - abx * acz;
        }

        /// Determines if two axis-aligned bounding boxes overlap.
        ///  @param[in]        amin    Minimum bounds of box A. [(x, y, z)]
        ///  @param[in]        amax    Maximum bounds of box A. [(x, y, z)]
        ///  @param[in]        bmin    Minimum bounds of box B. [(x, y, z)]
        ///  @param[in]        bmax    Maximum bounds of box B. [(x, y, z)]
        /// @return True if the two AABB's overlap.
        /// @see dtOverlapBounds
        public static bool dtOverlapQuantBounds(ushort* amin, ushort* amax, ushort* bmin, ushort* bmax)
        {
            return dtOverlapQuantBounds(new ReadOnlySpan<ushort>(amin, 3), new ReadOnlySpan<ushort>(amax, 3), new ReadOnlySpan<ushort>(bmin, 3), new ReadOnlySpan<ushort>(bmax, 3));
        }
        public static bool dtOverlapQuantBounds(ReadOnlySpan<ushort> amin, ReadOnlySpan<ushort> amax, ReadOnlySpan<ushort> bmin, ReadOnlySpan<ushort> bmax)
        {
            bool overlap = true;
            overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
            overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
            overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
            return overlap;
        }

        /// Determines if two axis-aligned bounding boxes overlap.
        ///  @param[in]        amin    Minimum bounds of box A. [(x, y, z)]
        ///  @param[in]        amax    Maximum bounds of box A. [(x, y, z)]
        ///  @param[in]        bmin    Minimum bounds of box B. [(x, y, z)]
        ///  @param[in]        bmax    Maximum bounds of box B. [(x, y, z)]
        /// @return True if the two AABB's overlap.
        /// @see dtOverlapQuantBounds
        public static bool dtOverlapBounds(float* amin, float* amax, float* bmin, float* bmax)
        {
            return dtOverlapBounds(new ReadOnlySpan<float>(amin, 3), new ReadOnlySpan<float>(amax, 3), new ReadOnlySpan<float>(bmin, 3), new ReadOnlySpan<float>(bmax, 3));
        }
        public static bool dtOverlapBounds(ReadOnlySpan<float> amin, ReadOnlySpan<float> amax, ReadOnlySpan<float> bmin, ReadOnlySpan<float> bmax)
        {
            bool overlap = true;
            overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
            overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
            overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
            return overlap;
        }


        ///////////////////////////////////////////////////////////////////////////

        // This section contains detailed documentation for members that don't have
        // a source file. It reduces clutter in the main section of the header.

        /**

        @fn float dtTriArea2D(const float* a, const float* b, const float* c)
        @par

        The vertices are projected onto the xz-plane, so the y-values are ignored.

        This is a low cost function than can be used for various purposes.  Its main purpose
        is for point/line relationship testing.

        In all cases: A value of zero indicates that all vertices are collinear or represent the same point.
        (On the xz-plane.)

        When used for point/line relationship tests, AB usually represents a line against which
        the C point is to be tested.  In this case:

        A positive value indicates that point C is to the left of line AB, looking from A toward B.<br/>
        A negative value indicates that point C is to the right of lineAB, looking from A toward B.

        When used for evaluating a triangle:

        The absolute value of the return value is two times the area of the triangle when it is
        projected onto the xz-plane.

        A positive return value indicates:

        <ul>
        <li>The vertices are wrapped in the normal Detour wrap direction.</li>
        <li>The triangle's 3D face normal is in the general up direction.</li>
        </ul>

        A negative return value indicates:

        <ul>
        <li>The vertices are reverse wrapped. (Wrapped opposite the normal Detour wrap direction.)</li>
        <li>The triangle's 3D face normal is in the general down direction.</li>
        </ul>

        */


        //////////////////////////////////////////////////////////////////////////////////////////


        /// Derives the closest point on a triangle from the specified reference point.
        ///  @param[out]    closest    The closest point on the triangle.    
        ///  @param[in]        p        The reference point from which to test. [(x, y, z)]
        ///  @param[in]        a        Vertex A of triangle ABC. [(x, y, z)]
        ///  @param[in]        b        Vertex B of triangle ABC. [(x, y, z)]
        ///  @param[in]        c        Vertex C of triangle ABC. [(x, y, z)]
        public static void dtClosestPtPointTriangle(float* closest, float* p, float* a, float* b, float* c)
        {
            dtClosestPtPointTriangle(new Span<float>(closest, 3), new ReadOnlySpan<float>(p, 3), new ReadOnlySpan<float>(a, 3), new ReadOnlySpan<float>(b, 3), new ReadOnlySpan<float>(c, 3));
        }
        public static void dtClosestPtPointTriangle(Span<float> closest, ReadOnlySpan<float> p, ReadOnlySpan<float> a, ReadOnlySpan<float> b, ReadOnlySpan<float> c)
        {
            // Check if P in vertex region outside A
            Span<float> ab = stackalloc float[3];
            Span<float> ac = stackalloc float[3];
            Span<float> ap = stackalloc float[3];
            dtVsub(ab, b, a);
            dtVsub(ac, c, a);
            dtVsub(ap, p, a);
            float d1 = dtVdot(ab, ap);
            float d2 = dtVdot(ac, ap);
            if (d1 <= 0.0f && d2 <= 0.0f)
            {
                // barycentric coordinates (1,0,0)
                dtVcopy(closest, a);
                return;
            }

            // Check if P in vertex region outside B
            Span<float> bp = stackalloc float[3];
            dtVsub(bp, p, b);
            float d3 = dtVdot(ab, bp);
            float d4 = dtVdot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3)
            {
                // barycentric coordinates (0,1,0)
                dtVcopy(closest, b);
                return;
            }

            // Check if P in edge region of AB, if so return projection of P onto AB
            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
            {
                // barycentric coordinates (1-v,v,0)
                float v = d1 / (d1 - d3);
                closest[0] = a[0] + v * ab[0];
                closest[1] = a[1] + v * ab[1];
                closest[2] = a[2] + v * ab[2];
                return;
            }

            // Check if P in vertex region outside C
            Span<float> cp = stackalloc float[3];
            dtVsub(cp, p, c);
            float d5 = dtVdot(ab, cp);
            float d6 = dtVdot(ac, cp);
            if (d6 >= 0.0f && d5 <= d6)
            {
                // barycentric coordinates (0,0,1)
                dtVcopy(closest, c);
                return;
            }

            // Check if P in edge region of AC, if so return projection of P onto AC
            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
            {
                // barycentric coordinates (1-w,0,w)
                float w = d2 / (d2 - d6);
                closest[0] = a[0] + w * ac[0];
                closest[1] = a[1] + w * ac[1];
                closest[2] = a[2] + w * ac[2];
                return;
            }

            // Check if P in edge region of BC, if so return projection of P onto BC
            float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
            {
                // barycentric coordinates (0,1-w,w)
                float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                closest[0] = b[0] + w * (c[0] - b[0]);
                closest[1] = b[1] + w * (c[1] - b[1]);
                closest[2] = b[2] + w * (c[2] - b[2]);
                return;
            }

            {
                // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
                float denom = 1.0f / (va + vb + vc);
                float v = vb * denom;
                float w = vc * denom;
                closest[0] = a[0] + ab[0] * v + ac[0] * w;
                closest[1] = a[1] + ab[1] * v + ac[1] * w;
                closest[2] = a[2] + ab[2] * v + ac[2] * w;
            }
        }

        /// Derives the y-axis height of the closest point on the triangle from the specified reference point.
        ///  @param[in]        p        The reference point from which to test. [(x, y, z)]
        ///  @param[in]        a        Vertex A of triangle ABC. [(x, y, z)]
        ///  @param[in]        b        Vertex B of triangle ABC. [(x, y, z)]
        ///  @param[in]        c        Vertex C of triangle ABC. [(x, y, z)]
        ///  @param[out]    h        The resulting height.
        public static bool dtClosestHeightPointTriangle(float* p, float* a, float* b, float* c, ref float h)
        {
            return dtClosestHeightPointTriangle(new ReadOnlySpan<float>(p, 3), new ReadOnlySpan<float>(a, 3), new ReadOnlySpan<float>(b, 3), new ReadOnlySpan<float>(c, 3), ref h);
        }
        public static bool dtClosestHeightPointTriangle(ReadOnlySpan<float> p, ReadOnlySpan<float> a, ReadOnlySpan<float> b, ReadOnlySpan<float> c, ref float h)
        {
            const float EPS = 1e-6f;
            Span<float> v0 = stackalloc float[3];
            Span<float> v1 = stackalloc float[3];
            Span<float> v2 = stackalloc float[3];

            dtVsub(v0, c, a);
            dtVsub(v1, b, a);
            dtVsub(v2, p, a);

            // Compute scaled barycentric coordinates
            float denom = v0[0] * v1[2] - v0[2] * v1[0];
            if (Math.Abs(denom) < EPS)
                return false;

            float u = v1[2] * v2[0] - v1[0] * v2[2];
            float v = v0[0] * v2[2] - v0[2] * v2[0];

            if (denom < 0F)
            {
                denom = -denom;
                u = -u;
                v = -v;
            }

            // If point lies inside the triangle, return interpolated ycoord.
            if (u >= 0.0f && v >= 0.0f && (u + v) <= denom)
            {
                h = a[1] + (v0[1] * u + v1[1] * v) / denom;
                return true;
            }
            return false;
        }

        public static bool dtIntersectSegmentPoly2D(float* p0, float* p1, float* verts, int nverts, ref float tmin, ref float tmax, ref int segMin, ref int segMax)
        {
            return dtIntersectSegmentPoly2D(new ReadOnlySpan<float>(p0, 3), new ReadOnlySpan<float>(p1, 3), new ReadOnlySpan<float>(verts, nverts), nverts, ref tmin, ref tmax, ref segMin, ref segMax);
        }
        public static bool dtIntersectSegmentPoly2D(ReadOnlySpan<float> p0, ReadOnlySpan<float> p1, ReadOnlySpan<float> verts, int nverts, ref float tmin, ref float tmax, ref int segMin, ref int segMax)
        {
            const float EPS = 0.00000001f;

            tmin = 0F;
            tmax = 1F;
            segMin = -1;
            segMax = -1;

            Span<float> dir = stackalloc float[3];
            dtVsub(dir, p1, p0);

            Span<float> edge = stackalloc float[3];
            Span<float> diff = stackalloc float[3];
            for (int i = 0, j = nverts - 1; i < nverts; j = i++)
            {
                dtVsub(edge, verts.Slice(i * 3, 3), verts.Slice(j * 3, 3));
                dtVsub(diff, p0, verts.Slice(j * 3, 3));
                float n = dtVperp2D(edge, diff);
                float d = dtVperp2D(dir, edge);
                if (Math.Abs(d) < EPS)
                {
                    // S is nearly parallel to this edge
                    if (n < 0F)
                        return false;
                    else
                        continue;
                }
                float t = n / d;
                if (d < 0F)
                {
                    // segment S is entering across this edge
                    if (t > tmin)
                    {
                        tmin = t;
                        segMin = j;
                        // S enters after leaving polygon
                        if (tmin > tmax)
                            return false;
                    }
                }
                else
                {
                    // segment S is leaving across this edge
                    if (t < tmax)
                    {
                        tmax = t;
                        segMax = j;
                        // S leaves before entering polygon
                        if (tmax < tmin)
                            return false;
                    }
                }
            }

            return true;
        }

        public static bool dtIntersectSegSeg2D(float* ap, float* aq, float* bp, float* bq, ref float s, ref float t)
        {
            return dtIntersectSegSeg2D(new ReadOnlySpan<float>(ap, 3), new ReadOnlySpan<float>(aq, 3), new ReadOnlySpan<float>(bp, 3), new ReadOnlySpan<float>(bq, 3), ref s, ref t);
        }
        public static bool dtIntersectSegSeg2D(ReadOnlySpan<float> ap, ReadOnlySpan<float> aq, ReadOnlySpan<float> bp, ReadOnlySpan<float> bq, ref float s, ref float t)
        {
            Span<float> u = stackalloc float[3];
            Span<float> v = stackalloc float[3];
            Span<float> w = stackalloc float[3];
            dtVsub(u, aq, ap);
            dtVsub(v, bq, bp);
            dtVsub(w, ap, bp);
            float d = vperpXZ(u, v);
            if (Math.Abs(d) < 1e-6f)
                return false;

            s = vperpXZ(v, w) / d;
            t = vperpXZ(u, w) / d;
            return true;
        }

        /// @par
        ///
        /// All points are projected onto the xz-plane, so the y-values are ignored.

        /// Determines if the specified point is inside the convex polygon on the xz-plane.
        ///  @param[in]        pt        The point to check. [(x, y, z)]
        ///  @param[in]        verts    The polygon vertices. [(x, y, z) * @p nverts]
        ///  @param[in]        nverts    The number of vertices. [Limit: >= 3]
        /// @return True if the point is inside the polygon.
        public static bool dtPointInPolygon(float* pt, float* verts, int nverts)
        {
            return dtPointInPolygon(new ReadOnlySpan<float>(pt, 3), new ReadOnlySpan<float>(verts, nverts), nverts);
        }
        public static bool dtPointInPolygon(ReadOnlySpan<float> pt, ReadOnlySpan<float> verts, int nverts)
        {
            // TODO: Replace pnpoly with triArea2D tests?
            int i;
            int j;
            bool c = false;
            for (i = 0, j = nverts - 1; i < nverts; j = i++)
            {
                var vi = verts.Slice(i * 3, 3);
                var vj = verts.Slice(j * 3, 3);
                if (((vi[2] > pt[2]) != (vj[2] > pt[2])) && (pt[0] < (vj[0] - vi[0]) * (pt[2] - vi[2]) / (vj[2] - vi[2]) + vi[0]))
                    c = !c;
            }
            return c;
        }

        public static bool dtDistancePtPolyEdgesSqr(float* pt, float* verts, int nverts, float* ed, float* et)
        {
            return dtDistancePtPolyEdgesSqr(new ReadOnlySpan<float>(pt, 3), new ReadOnlySpan<float>(verts, nverts), nverts, new Span<float>(ed, nverts), new Span<float>(et, nverts));
        }
        public static bool dtDistancePtPolyEdgesSqr(ReadOnlySpan<float> pt, ReadOnlySpan<float> verts, int nverts, Span<float> ed, Span<float> et)
        {
            // TODO: Replace pnpoly with triArea2D tests?
            int i;
            int j;
            bool c = false;
            for (i = 0, j = nverts - 1; i < nverts; j = i++)
            {
                var vi = verts.Slice(i * 3, 3);
                var vj = verts.Slice(j * 3, 3);
                if (((vi[2] > pt[2]) != (vj[2] > pt[2])) && (pt[0] < (vj[0] - vi[0]) * (pt[2] - vi[2]) / (vj[2] - vi[2]) + vi[0]))
                    c = !c;
                ed[j] = dtDistancePtSegSqr2D(pt, vj, vi, ref et[j]);
            }
            return c;
        }

        public static float dtDistancePtSegSqr2D(float* pt, float* p, float* q, ref float t)
        {
            return dtDistancePtSegSqr2D(new ReadOnlySpan<float>(pt, 3), new ReadOnlySpan<float>(p, 3), new ReadOnlySpan<float>(q, 3), ref t);
        }
        public static float dtDistancePtSegSqr2D(ReadOnlySpan<float> pt, ReadOnlySpan<float> p, ReadOnlySpan<float> q, ref float t)
        {
            float pqx = q[0] - p[0];
            float pqz = q[2] - p[2];
            float dx = pt[0] - p[0];
            float dz = pt[2] - p[2];
            float d = pqx * pqx + pqz * pqz;
            t = pqx * dx + pqz * dz;
            if (d > 0F)
                t /= d;

            if (t < 0F)
                t = 0F;
            else if (t > 1F)
                t = 1F;

            dx = p[0] + t * pqx - pt[0];
            dz = p[2] + t * pqz - pt[2];
            return dx * dx + dz * dz;
        }

        /// Derives the centroid of a convex polygon.
        ///  @param[out]    tc        The centroid of the polgyon. [(x, y, z)]
        ///  @param[in]        idx        The polygon indices. [(vertIndex) * @p nidx]
        ///  @param[in]        nidx    The number of indices in the polygon. [Limit: >= 3]
        ///  @param[in]        verts    The polygon vertices. [(x, y, z) * vertCount]
        public static void dtCalcPolyCenter(float* tc, ushort* idx, int nidx, float* verts)
        {
            dtCalcPolyCenter(new Span<float>(tc, 3), new ReadOnlySpan<ushort>(idx, nidx), nidx, new ReadOnlySpan<float>(verts, int.MaxValue));
        }
        public static void dtCalcPolyCenter(Span<float> tc, ReadOnlySpan<ushort> idx, int nidx, ReadOnlySpan<float> verts)
        {
            tc[0] = 0.0f;
            tc[1] = 0.0f;
            tc[2] = 0.0f;
            for (int j = 0; j < nidx; ++j)
            {
                var v = verts.Slice(idx[j] * 3, 3);
                tc[0] += v[0];
                tc[1] += v[1];
                tc[2] += v[2];
            }
            float s = 1.0f / nidx;
            tc[0] *= s;
            tc[1] *= s;
            tc[2] *= s;
        }

        /// @par
        ///
        /// All vertices are projected onto the xz-plane, so the y-values are ignored.

        /// Determines if the two convex polygons overlap on the xz-plane.
        ///  @param[in]        polya        Polygon A vertices.    [(x, y, z) * @p npolya]
        ///  @param[in]        npolya        The number of vertices in polygon A.
        ///  @param[in]        polyb        Polygon B vertices.    [(x, y, z) * @p npolyb]
        ///  @param[in]        npolyb        The number of vertices in polygon B.
        /// @return True if the two polygons overlap.
        public static bool dtOverlapPolyPoly2D(float* polya, int npolya, float* polyb, int npolyb)
        {
            return dtOverlapPolyPoly2D(new ReadOnlySpan<float>(polya, npolya), npolya, new ReadOnlySpan<float>(polyb, npolyb), npolyb);
        }
        public static bool dtOverlapPolyPoly2D(ReadOnlySpan<float> polya, int npolya, ReadOnlySpan<float> polyb, int npolyb)
        {
            const float eps = 1e-4f;

            Span<float> n = stackalloc float[3];
            for (int i = 0, j = npolya - 1; i < npolya; j = i++)
            {
                var va = polya.Slice(j * 3);
                var vb = polya.Slice(i * 3);
                n[0] = vb[2] - va[2];
                n[1] = 0F;
                n[2] = -(vb[0] - va[0]);
                float amin = default;
                float amax = default;
                float bmin = default;
                float bmax = default;
                projectPoly(n, polya, npolya, ref amin, ref amax);
                projectPoly(n, polyb, npolyb, ref bmin, ref bmax);
                if (!overlapRange(amin, amax, bmin, bmax, eps))
                {
                    // Found separating axis
                    return false;
                }
            }
            for (int i = 0, j = npolyb - 1; i < npolyb; j = i++)
            {
                var va = polyb.Slice(j * 3);
                var vb = polyb.Slice(i * 3);
                n[0] = vb[2] - va[2];
                n[1] = 0F;
                n[2] = -(vb[0] - va[0]);
                float amin = default;
                float amax = default;
                float bmin = default;
                float bmax = default;
                projectPoly(n, polya, npolya, ref amin, ref amax);
                projectPoly(n, polyb, npolyb, ref bmin, ref bmax);
                if (!overlapRange(amin, amax, bmin, bmax, eps))
                {
                    // Found separating axis
                    return false;
                }
            }
            return true;
        }

        ///
        /// @name Miscellanious functions.
        ///

        public static uint dtNextPow2(uint v)
        {
            v--;
            v |= v >> 1;
            v |= v >> 2;
            v |= v >> 4;
            v |= v >> 8;
            v |= v >> 16;
            v++;
            return v;
        }

        public static uint dtIlog2(int v)
        {
            int r;
            int shift;
            r = (Convert.ToInt32(v > 0xffff) << 4);
            v >>= r;
            shift = (Convert.ToInt32(v > 0xff) << 3);
            v >>= shift;
            r |= shift;
            shift = (Convert.ToInt32(v > 0xf) << 2);
            v >>= shift;
            r |= shift;
            shift = (Convert.ToInt32(v > 0x3) << 1);
            v >>= shift;
            r |= shift;
            r |= (v >> 1);
            return (uint)r;
        }

        public static int dtAlign4(int x)
        {
            return (x + 3) & ~3;
        }

        public static int dtOppositeTile(int side)
        {
            return (side + 4) & 0x7;
        }

        public static void dtSwapByte(byte* a, byte* b)
        {
            byte tmp = *a;
            *a = *b;
            *b = tmp;
        }

        public static void dtSwapEndian(ushort* v)
        {
            byte* x = (byte*)v;
            dtSwapByte(x + 0, x + 1);
        }

        public static void dtSwapEndian(short* v)
        {
            byte* x = (byte*)v;
            dtSwapByte(x + 0, x + 1);
        }

        public static void dtSwapEndian(uint* v)
        {
            byte* x = (byte*)v;
            dtSwapByte(x + 0, x + 3); dtSwapByte(x + 1, x + 2);
        }

        public static void dtSwapEndian(int* v)
        {
            byte* x = (byte*)v;
            dtSwapByte(x + 0, x + 3); dtSwapByte(x + 1, x + 2);
        }

        public static void dtSwapEndian(float* v)
        {
            byte* x = (byte*)v;
            dtSwapByte(x + 0, x + 3); dtSwapByte(x + 1, x + 2);
        }

        // Returns a random point in a convex polygon.
        // Adapted from Graphics Gems article.
        public static void dtRandomPointInConvexPoly(float* pts, int npts, float* areas, float s, float t, float* @out)
        {
            dtRandomPointInConvexPoly(new ReadOnlySpan<float>(pts, npts), npts, new Span<float>(areas, npts), s, t, new Span<float>(@out, 3));
        }
        public static void dtRandomPointInConvexPoly(ReadOnlySpan<float> pts, int npts, Span<float> areas, float s, float t, Span<float> @out)
        {
            // Calc triangle araes
            float areasum = 0.0f;
            for (int i = 2; i < npts; i++)
            {
                areas[i] = dtTriArea2D(pts, pts.Slice((i - 1) * 3), pts.Slice(i * 3));
                areasum += Math.Max(0.001f, areas[i]);
            }
            // Find sub triangle weighted by area.
            float thr = s * areasum;
            float acc = 0.0f;
            float u = 1.0f;
            int tri = npts - 1;
            for (int i = 2; i < npts; i++)
            {
                float dacc = areas[i];
                if (thr >= acc && thr < (acc + dacc))
                {
                    u = (thr - acc) / dacc;
                    tri = i;
                    break;
                }
                acc += dacc;
            }

            float v = (float)Math.Sqrt(t);

            float a = 1 - v;
            float b = (1 - u) * v;
            float c = u * v;
            var pa = pts;
            var pb = pts.Slice((tri - 1) * 3);
            var pc = pts.Slice(tri * 3);

            @out[0] = a * pa[0] + b * pb[0] + c * pc[0];
            @out[1] = a * pa[1] + b * pb[1] + c * pc[1];
            @out[2] = a * pa[2] + b * pb[2] + c * pc[2];
        }

        public static TypeToRetrieveAs* dtGetThenAdvanceBufferPointer<TypeToRetrieveAs>(ref byte* buffer, size_t distanceToAdvance) where TypeToRetrieveAs : unmanaged
        {
            var returnPointer = (TypeToRetrieveAs*)buffer;
            buffer += distanceToAdvance;
            return returnPointer;
        }

        internal static void projectPoly(float* axis, float* poly, int npoly, ref float rmin, ref float rmax)
        {
            projectPoly(new ReadOnlySpan<float>(axis, 3), new ReadOnlySpan<float>(poly, npoly), npoly, ref rmin, ref rmax);
        }
        internal static void projectPoly(ReadOnlySpan<float> axis, ReadOnlySpan<float> poly, int npoly, ref float rmin, ref float rmax)
        {
            rmin = rmax = dtVdot2D(axis, poly);
            for (int i = 1; i < npoly; ++i)
            {
                float d = dtVdot2D(axis, poly.Slice(i * 3));
                rmin = Math.Min(rmin, d);
                rmax = Math.Max(rmax, d);
            }
        }

        public static bool overlapRange(float amin, float amax, float bmin, float bmax, float eps)
        {
            return ((amin + eps) > bmax || (amax - eps) < bmin) ? false : true;
        }

        public static float vperpXZ(ReadOnlySpan<float> a, ReadOnlySpan<float> b)
        {
            return a[0] * b[2] - a[2] * b[0];
        }



        // Note: If you want to use 64-bit refs, change the types of both dtPolyRef & dtTileRef.
        // It is also recommended that you change dtHashRef() to a proper 64-bit hash.

        /// A handle to a polygon within a navigation mesh tile.
        /// @ingroup detour
        internal const uint DT_SALT_BITS = 16;
        internal const uint DT_TILE_BITS = 28;
        internal const uint DT_POLY_BITS = 20;

        /// The maximum number of vertices per navigation polygon.
        /// @ingroup detour
        internal const int DT_VERTS_PER_POLYGON = 6;

        /// @name Tile Serialization Constants
        /// These constants are used to detect whether a navigation tile's data
        /// and state format is compatible with the current build.
        ///

        /// A magic number used to detect compatibility of navigation tile data.
        internal const int DT_NAVMESH_MAGIC = 'D' << 24 | 'N' << 16 | 'A' << 8 | 'V';

        /// A version number used to detect compatibility of navigation tile data.
        internal const int DT_NAVMESH_VERSION = 7;

        /// A magic number used to detect the compatibility of navigation tile states.
        internal const int DT_NAVMESH_STATE_MAGIC = 'D' << 24 | 'N' << 16 | 'M' << 8 | 'S';

        /// A version number used to detect compatibility of navigation tile states.
        internal const int DT_NAVMESH_STATE_VERSION = 1;

        /// A flag that indicates that an entity links to an external entity.
        /// (E.g. A polygon edge is a portal that links to another polygon.)
        internal const ushort DT_EXT_LINK = 0x8000;

        /// A value that indicates the entity does not link to anything.
        internal const uint DT_NULL_LINK = 0xffffffff;

        /// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
        internal const uint DT_OFFMESH_CON_BIDIR = 1;

        /// The maximum number of user defined area ids.
        /// @ingroup detour
        internal const int DT_MAX_AREAS = 64;


        /// Limit raycasting during any angle pahfinding
        /// The limit is given as a multiple of the character radius
        internal const float DT_RAY_CAST_LIMIT_PROPORTIONS = 50.0f;

        /// Get flags for edge in detail triangle.
        /// @param    triFlags[in]        The flags for the triangle (last component of detail vertices above).
        /// @param    edgeIndex[in]        The index of the first vertex of the edge. For instance, if 0,
        ///                                returns flags for edge AB.
        public static int dtGetDetailTriEdgeFlags(byte triFlags, int edgeIndex)
        {
            return (triFlags >> (edgeIndex * 2)) & 0x3;
        }

        /// Allocates a navigation mesh object using the Detour allocator.
        /// @return A navigation mesh that is ready for initialization, or null on failure.
        ///  @ingroup detour
        public static dtNavMesh dtAllocNavMesh()
        {
            return new dtNavMesh();
        }

        /// @par
        ///
        /// This function will only free the memory for tiles with the #DT_TILE_FREE_DATA
        /// flag set.

        /// Frees the specified navigation mesh object using the Detour allocator.
        ///  @param[in]    navmesh        A navigation mesh allocated using #dtAllocNavMesh
        ///  @ingroup detour
        public static void dtFreeNavMesh(dtNavMesh navmesh)
        {
            if (navmesh == null)
                return;
            navmesh.Dispose();
        }


        ///////////////////////////////////////////////////////////////////////////

        // This section contains detailed documentation for members that don't have
        // a source file. It reduces clutter in the main section of the header.

        /**
        
        @typedef dtPolyRef
        @par
        
        Polygon references are subject to the same invalidate/preserve/restore 
        rules that apply to #dtTileRef's.  If the #dtTileRef for the polygon's
        tile changes, the polygon reference becomes invalid.
        
        Changing a polygon's flags, area id, etc. does not impact its polygon
        reference.
        
        @typedef dtTileRef
        @par
        
        The following changes will invalidate a tile reference:
        
        - The referenced tile has been removed from the navigation mesh.
        - The navigation mesh has been initialized using a different set
        of #dtNavMeshParams.
        
        A tile reference is preserved/restored if the tile is added to a navigation 
        mesh initialized with the original #dtNavMeshParams and is added at the
        original reference location. (E.g. The lastRef parameter is used with
        dtNavMesh::addTile.)
        
        Basically, if the storage structure of a tile changes, its associated
        tile reference changes.
        
        
        @var unsigned short dtPoly::neis[DT_VERTS_PER_POLYGON]
        @par
        
        Each entry represents data for the edge starting at the vertex of the same index. 
        E.g. The entry at index n represents the edge data for vertex[n] to vertex[n+1].
        
        A value of zero indicates the edge has no polygon connection. (It makes up the 
        border of the navigation mesh.)
        
        The information can be extracted as follows: 
        @code 
        neighborRef = neis[n] & 0xff; // Get the neighbor polygon reference.
        
        if (neis[n] & #DT_EX_LINK)
        {
            // The edge is an external (portal) edge.
        }
        @endcode
        
        @var float dtMeshHeader::bvQuantFactor
        @par
        
        This value is used for converting between world and bounding volume coordinates.
        For example:
        @code
        const float cs = 1.0f / tile->header->bvQuantFactor;
        const dtBVNode* n = &tile->bvTree[i];
        if (n->i >= 0)
        {
            // This is a leaf node.
            float worldMinX = tile->header->bmin[0] + n->bmin[0]*cs;
            float worldMinY = tile->header->bmin[0] + n->bmin[1]*cs;
            // Etc...
        }
        @endcode
        
        @struct dtMeshTile
        @par
        
        Tiles generally only exist within the context of a dtNavMesh object.
        
        Some tile content is optional.  For example, a tile may not contain any
        off-mesh connections.  In this case the associated pointer will be null.
        
        If a detail mesh exists it will share vertices with the base polygon mesh.  
        Only the vertices unique to the detail mesh will be stored in #detailVerts.
        
        @warning Tiles returned by a dtNavMesh object are not guarenteed to be populated.
        For example: The tile at a location might not have been loaded yet, or may have been removed.
        In this case, pointers will be null.  So if in doubt, check the polygon count in the 
        tile's header to determine if a tile has polygons defined.
        
        @var float dtOffMeshConnection::pos[6]
        @par
        
        For a properly built navigation mesh, vertex A will always be within the bounds of the mesh. 
        Vertex B is not required to be within the bounds of the mesh.
        
        */



        public static bool overlapSlabs(float* amin, float* amax, float* bmin, float* bmax, float px, float py)
        {
            return overlapSlabs(new ReadOnlySpan<float>(amin, 3), new ReadOnlySpan<float>(amax, 3),
                new ReadOnlySpan<float>(bmin, 3), new ReadOnlySpan<float>(bmax, 3), px, py);
        }
        public static bool overlapSlabs(ReadOnlySpan<float> amin, ReadOnlySpan<float> amax, ReadOnlySpan<float> bmin, ReadOnlySpan<float> bmax, float px, float py)
        {
            // Check for horizontal overlap.
            // The segment is shrunken a little so that slabs which touch
            // at end points are not connected.
            float minx = Math.Max(amin[0] + px, bmin[0] + px);
            float maxx = Math.Min(amax[0] - px, bmax[0] - px);
            if (minx > maxx)
                return false;

            // Check vertical overlap.
            float ad = (amax[1] - amin[1]) / (amax[0] - amin[0]);
            float ak = amin[1] - ad * amin[0];
            float bd = (bmax[1] - bmin[1]) / (bmax[0] - bmin[0]);
            float bk = bmin[1] - bd * bmin[0];
            float aminy = ad * minx + ak;
            float amaxy = ad * maxx + ak;
            float bminy = bd * minx + bk;
            float bmaxy = bd * maxx + bk;
            float dmin = bminy - aminy;
            float dmax = bmaxy - amaxy;

            // Crossing segments always overlap.
            if (dmin * dmax < 0F)
                return true;

            // Check for overlap at endpoints.
            float thr = (py * 2) * (py * 2);
            if (dmin * dmin <= thr || dmax * dmax <= thr)
                return true;

            return false;
        }

        internal static float getSlabCoord(float* va, int side)
        {
            return getSlabCoord(new ReadOnlySpan<float>(va, 3), side);
        }
        internal static float getSlabCoord(ReadOnlySpan<float> va, int side)
        {
            if (side == 0 || side == 4)
                return va[0];
            else if (side == 2 || side == 6)
                return va[2];
            return 0F;
        }

        internal static void calcSlabEndPoints(float* va, float* vb, float* bmin, float* bmax, int side)
        {
            calcSlabEndPoints(new ReadOnlySpan<float>(va, 3), new ReadOnlySpan<float>(vb, 3), new Span<float>(bmin, 3), new Span<float>(bmax, 3), side);
        }
        internal static void calcSlabEndPoints(ReadOnlySpan<float> va, ReadOnlySpan<float> vb, Span<float> bmin, Span<float> bmax, int side)
        {
            if (side == 0 || side == 4)
            {
                if (va[2] < vb[2])
                {
                    bmin[0] = va[2];
                    bmin[1] = va[1];
                    bmax[0] = vb[2];
                    bmax[1] = vb[1];
                }
                else
                {
                    bmin[0] = vb[2];
                    bmin[1] = vb[1];
                    bmax[0] = va[2];
                    bmax[1] = va[1];
                }
            }
            else if (side == 2 || side == 6)
            {
                if (va[0] < vb[0])
                {
                    bmin[0] = va[0];
                    bmin[1] = va[1];
                    bmax[0] = vb[0];
                    bmax[1] = vb[1];
                }
                else
                {
                    bmin[0] = vb[0];
                    bmin[1] = vb[1];
                    bmax[0] = va[0];
                    bmax[1] = va[1];
                }
            }
        }

        public static int computeTileHash(int x, int y, int mask)
        {
            const uint h1 = 0x8da6b343; // Large multiplicative constants;
            const uint h2 = 0xd8163841; // here arbitrarily chosen primes
            var n = h1 * x + h2 * y;
            return (int)(n & mask);
        }

        public static uint allocLink(dtMeshTile* tile)
        {
            if (tile->linksFreeList == DT_NULL_LINK)
                return DT_NULL_LINK;
            uint link = tile->linksFreeList;
            tile->linksFreeList = tile->links[link].next;
            return link;
        }

        public static void freeLink(dtMeshTile* tile, uint link)
        {
            tile->links[link].next = tile->linksFreeList;
            tile->linksFreeList = link;
        }

        public static void closestPointOnDetailEdges(bool onlyBoundary, dtMeshTile* tile, dtPoly* poly, float* pos, float* closest)
        {
            uint ip = (uint)(poly - tile->polys);
            dtPolyDetail* pd = &tile->detailMeshes[ip];

            float dmin = float.MaxValue;
            float tmin = 0F;
            float* pmin = default;
            float* pmax = default;

            Span<IntPtr> v = stackalloc IntPtr[3];
            for (int i = 0; i < pd->triCount; i++)
            {
                byte* tris = &tile->detailTris[(pd->triBase + i) * 4];
                int ANY_BOUNDARY_EDGE = ((int)DT_DETAIL_EDGE_BOUNDARY << 0) | ((int)DT_DETAIL_EDGE_BOUNDARY << 2) | ((int)DT_DETAIL_EDGE_BOUNDARY << 4);
                if (onlyBoundary && (tris[3] & ANY_BOUNDARY_EDGE) == 0)
                    continue;

                for (int j = 0; j < 3; ++j)
                {
                    if (tris[j] < poly->vertCount)
                        v[j] = (IntPtr)(&tile->verts[poly->verts[tris[j]] * 3]);
                    else
                        v[j] = (IntPtr)(&tile->detailVerts[(pd->vertBase + (tris[j] - poly->vertCount)) * 3]);
                }

                for (int k = 0, j = 2; k < 3; j = k++)
                {
                    if ((dtGetDetailTriEdgeFlags(tris[3], j) & (int)DT_DETAIL_EDGE_BOUNDARY) == 0 && (onlyBoundary || tris[j] < tris[k]))
                    {
                        // Only looking at boundary edges and this is internal, or
                        // this is an inner edge that we will see again or have already seen.
                        continue;
                    }

                    float t = default;
                    float d = dtDistancePtSegSqr2D(pos, (float*)v[j], (float*)v[k], ref t);
                    if (d < dmin)
                    {
                        dmin = d;
                        tmin = t;
                        pmin = (float*)v[j];
                        pmax = (float*)v[k];
                    }
                }
            }

            dtVlerp(closest, pmin, pmax, tmin);
        }

        // TODO: Better error handling.

        /// @par
        /// 
        /// The output data array is allocated using the detour allocator (dtAlloc()).  The method
        /// used to free the memory will be determined by how the tile is added to the navigation
        /// mesh.
        ///
        /// @see dtNavMesh, dtNavMesh::addTile()

        /// Builds navigation mesh tile data from the provided tile creation data.
        /// @ingroup detour
        ///  @param[in]        params        Tile creation data.
        ///  @param[out]    outData        The resulting tile data.
        ///  @param[out]    outDataSize    The size of the tile data array.
        /// @return True if the tile data was successfully created.
        public static bool dtCreateNavMeshData(dtNavMeshCreateParams* @params, byte** outData, ref int outDataSize)
        {
            if (@params->nvp > DT_VERTS_PER_POLYGON)
                return false;
            if (@params->vertCount >= 0xffff)
                return false;
            if (@params->vertCount == 0 || @params->verts == default)
                return false;
            if (@params->polyCount == 0 || @params->polys == default)
                return false;

            int nvp = @params->nvp;

            // Classify off-mesh connection points. We store only the connections
            // whose start point is inside the tile.
            byte* offMeshConClass = default;
            int storedOffMeshConCount = 0;
            int offMeshConLinkCount = 0;

            if (@params->offMeshConCount > 0)
            {
                offMeshConClass = (byte*)dtAlloc(sizeof(byte) * @params->offMeshConCount * 2);
                if (offMeshConClass == default)
                    return false;

                // Find tight heigh bounds, used for culling out off-mesh start locations.
                float hmin = float.MaxValue;
                float hmax = -float.MaxValue;

                if (@params->detailVerts != default && @params->detailVertsCount != 0)
                {
                    for (int i = 0; i < @params->detailVertsCount; ++i)
                    {
                        float h = @params->detailVerts[i * 3 + 1];
                        hmin = Math.Min(hmin, h);
                        hmax = Math.Max(hmax, h);
                    }
                }
                else
                {
                    for (int i = 0; i < @params->vertCount; ++i)
                    {
                        var iv = &@params->verts[i * 3];
                        float h = @params->bmin[1] + iv[1] * @params->ch;
                        hmin = Math.Min(hmin, h);
                        hmax = Math.Max(hmax, h);
                    }
                }
                hmin -= @params->walkableClimb;
                hmax += @params->walkableClimb;
                Span<float> bmin = stackalloc float[3];
                Span<float> bmax = stackalloc float[3];
                dtVcopy(bmin, new ReadOnlySpan<float>(@params->bmin, 3));
                dtVcopy(bmax, new ReadOnlySpan<float>(@params->bmax, 3));
                bmin[1] = hmin;
                bmax[1] = hmax;

                for (int i = 0; i < @params->offMeshConCount; ++i)
                {
                    var p0 = &@params->offMeshConVerts[(i * 2 + 0) * 3];
                    var p1 = &@params->offMeshConVerts[(i * 2 + 1) * 3];
                    offMeshConClass[i * 2 + 0] = classifyOffMeshPoint(p0, bmin, bmax);
                    offMeshConClass[i * 2 + 1] = classifyOffMeshPoint(p1, bmin, bmax);

                    // Zero out off-mesh start positions which are not even potentially touching the mesh.
                    if (offMeshConClass[i * 2 + 0] == 0xff)
                    {
                        if (p0[1] < bmin[1] || p0[1] > bmax[1])
                            offMeshConClass[i * 2 + 0] = 0;
                    }

                    // Cound how many links should be allocated for off-mesh connections.
                    if (offMeshConClass[i * 2 + 0] == 0xff)
                        offMeshConLinkCount++;
                    if (offMeshConClass[i * 2 + 1] == 0xff)
                        offMeshConLinkCount++;

                    if (offMeshConClass[i * 2 + 0] == 0xff)
                        storedOffMeshConCount++;
                }
            }

            // Off-mesh connectionss are stored as polygons, adjust values.
            int totPolyCount = @params->polyCount + storedOffMeshConCount;
            int totVertCount = @params->vertCount + storedOffMeshConCount * 2;

            // Find portal edges which are at tile borders.
            int edgeCount = 0;
            int portalCount = 0;
            for (int i = 0; i < @params->polyCount; ++i)
            {
                var p = &@params->polys[i * 2 * nvp];
                for (int j = 0; j < nvp; ++j)
                {
                    if (p[j] == MESH_NULL_IDX)
                        break;
                    edgeCount++;

                    if ((p[nvp + j] & 0x8000) != 0)
                    {
                        ushort dir = (ushort)(p[nvp + j] & 0xf);
                        if (dir != 0xf)
                            portalCount++;
                    }
                }
            }

            int maxLinkCount = edgeCount + portalCount * 2 + offMeshConLinkCount * 2;

            // Find unique detail vertices.
            int uniqueDetailVertCount = 0;
            int detailTriCount = 0;
            if (@params->detailMeshes != default)
            {
                // Has detail mesh, count unique detail vertex count and use input detail tri count.
                detailTriCount = @params->detailTriCount;
                for (int i = 0; i < @params->polyCount; ++i)
                {
                    var p = &@params->polys[i * nvp * 2];
                    int ndv = (int)@params->detailMeshes[i * 4 + 1];
                    int nv = 0;
                    for (int j = 0; j < nvp; ++j)
                    {
                        if (p[j] == MESH_NULL_IDX)
                            break;
                        nv++;
                    }
                    ndv -= nv;
                    uniqueDetailVertCount += ndv;
                }
            }
            else
            {
                // No input detail mesh, build detail mesh from nav polys.
                uniqueDetailVertCount = 0; // No extra detail verts.
                detailTriCount = 0;
                for (int i = 0; i < @params->polyCount; ++i)
                {
                    var p = &@params->polys[i * nvp * 2];
                    int nv = 0;
                    for (int j = 0; j < nvp; ++j)
                    {
                        if (p[j] == MESH_NULL_IDX)
                            break;
                        nv++;
                    }
                    detailTriCount += nv - 2;
                }
            }

            // Calculate data size
            int headerSize = dtAlign4(sizeof(dtMeshHeader));
            int vertsSize = dtAlign4(sizeof(float) * 3 * totVertCount);
            int polysSize = dtAlign4(sizeof(dtPoly) * totPolyCount);
            int linksSize = dtAlign4(sizeof(dtLink) * maxLinkCount);
            int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail) * @params->polyCount);
            int detailVertsSize = dtAlign4(sizeof(float) * 3 * uniqueDetailVertCount);
            int detailTrisSize = dtAlign4(sizeof(byte) * 4 * detailTriCount);
            int bvTreeSize = @params->buildBvTree ? dtAlign4(sizeof(dtBVNode) * @params->polyCount * 2) : 0;
            int offMeshConsSize = dtAlign4(sizeof(dtOffMeshConnection) * storedOffMeshConCount);

            int dataSize = headerSize + vertsSize + polysSize + linksSize + detailMeshesSize +
                detailVertsSize + detailTrisSize + bvTreeSize + offMeshConsSize;

            byte* data = (byte*)dtAlloc(sizeof(byte) * dataSize);
            if (data == default)
            {
                dtFree(offMeshConClass);
                return false;
            }

            new Span<byte>(data, dataSize).Fill(0);

            byte* d = data;

            dtMeshHeader* header = dtGetThenAdvanceBufferPointer<dtMeshHeader>(ref d, headerSize);
            float* navVerts = dtGetThenAdvanceBufferPointer<float>(ref d, vertsSize);
            dtPoly* navPolys = dtGetThenAdvanceBufferPointer<dtPoly>(ref d, polysSize);
            d += linksSize; // Ignore links; just leave enough space for them. They'll be created on load.
            dtPolyDetail* navDMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(ref d, detailMeshesSize);
            float* navDVerts = dtGetThenAdvanceBufferPointer<float>(ref d, detailVertsSize);
            byte* navDTris = dtGetThenAdvanceBufferPointer<byte>(ref d, detailTrisSize);
            dtBVNode* navBvtree = dtGetThenAdvanceBufferPointer<dtBVNode>(ref d, bvTreeSize);
            dtOffMeshConnection* offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(ref d, offMeshConsSize);


            // Store header
            header->magic = DT_NAVMESH_MAGIC;
            header->version = DT_NAVMESH_VERSION;
            header->x = @params->tileX;
            header->y = @params->tileY;
            header->layer = @params->tileLayer;
            header->userId = @params->userId;
            header->polyCount = totPolyCount;
            header->vertCount = totVertCount;
            header->maxLinkCount = maxLinkCount;
            dtVcopy(new Span<float>(header->bmin, 3), new ReadOnlySpan<float>(@params->bmin, 3));
            dtVcopy(new Span<float>(header->bmax, 3), new ReadOnlySpan<float>(@params->bmax, 3));

            header->detailMeshCount = @params->polyCount;
            header->detailVertCount = uniqueDetailVertCount;
            header->detailTriCount = detailTriCount;
            header->bvQuantFactor = 1.0f / @params->cs;
            header->offMeshBase = @params->polyCount;
            header->walkableHeight = @params->walkableHeight;
            header->walkableRadius = @params->walkableRadius;
            header->walkableClimb = @params->walkableClimb;
            header->offMeshConCount = storedOffMeshConCount;
            header->bvNodeCount = @params->buildBvTree ? @params->polyCount * 2 : 0;

            int offMeshVertsBase = @params->vertCount;
            int offMeshPolyBase = @params->polyCount;

            // Store vertices
            // Mesh vertices
            for (int i = 0; i < @params->vertCount; ++i)
            {
                var iv = &@params->verts[i * 3];
                var v = &navVerts[i * 3];
                v[0] = @params->bmin[0] + iv[0] * @params->cs;
                v[1] = @params->bmin[1] + iv[1] * @params->ch;
                v[2] = @params->bmin[2] + iv[2] * @params->cs;
            }
            // Off-mesh link vertices.
            int n = 0;
            for (int i = 0; i < @params->offMeshConCount; ++i)
            {
                // Only store connections which start from this tile.
                if (offMeshConClass[i * 2 + 0] == 0xff)
                {
                    var linkv = &@params->offMeshConVerts[i * 2 * 3];
                    var v = &navVerts[(offMeshVertsBase + n * 2) * 3];
                    dtVcopy(new Span<float>(&v[0], 3), new ReadOnlySpan<float>(&linkv[0], 3));
                    dtVcopy(new Span<float>(&v[3], 3), new ReadOnlySpan<float>(&linkv[3], 3));
                    n++;
                }
            }

            // Store polygons
            // Mesh polys
            var src = @params->polys;
            for (int i = 0; i < @params->polyCount; ++i)
            {
                dtPoly p = navPolys[i];
                p.vertCount = 0;
                p.flags = @params->polyFlags[i];
                p.setArea(@params->polyAreas[i]);
                p.setType((byte)dtPolyTypes.DT_POLYTYPE_GROUND);
                for (int j = 0; j < nvp; ++j)
                {
                    if (src[j] == MESH_NULL_IDX)
                        break;
                    p.verts[j] = src[j];
                    if ((src[nvp + j] & 0x8000) != 0)
                    {
                        // Border or portal edge.
                        ushort dir = (ushort)(src[nvp + j] & 0xf);
                        if (dir == 0xf) // Border
                            p.neis[j] = 0;
                        else if (dir == 0) // Portal x-
                            p.neis[j] = (ushort)(DT_EXT_LINK | 4);
                        else if (dir == 1) // Portal z+
                            p.neis[j] = (ushort)(DT_EXT_LINK | 2);
                        else if (dir == 2) // Portal x+
                            p.neis[j] = (ushort)(DT_EXT_LINK | 0);
                        else if (dir == 3) // Portal z-
                            p.neis[j] = (ushort)(DT_EXT_LINK | 6);
                    }
                    else
                    {
                        // Normal connection
                        p.neis[j] = (ushort)(src[nvp + j] + 1);
                    }

                    p.vertCount++;
                }
                src += nvp * 2;
            }
            // Off-mesh connection vertices.
            n = 0;
            for (int i = 0; i < @params->offMeshConCount; ++i)
            {
                // Only store connections which start from this tile.
                if (offMeshConClass[i * 2 + 0] == 0xff)
                {
                    dtPoly p = navPolys[offMeshPolyBase + n];
                    p.vertCount = 2;
                    p.verts[0] = (ushort)(offMeshVertsBase + n * 2 + 0);
                    p.verts[1] = (ushort)(offMeshVertsBase + n * 2 + 1);
                    p.flags = @params->offMeshConFlags[i];
                    p.setArea(@params->offMeshConAreas[i]);
                    p.setType((byte)dtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION);
                    n++;
                }
            }

            // Store detail meshes and vertices.
            // The nav polygon vertices are stored as the first vertices on each mesh.
            // We compress the mesh data by skipping them and using the navmesh coordinates.
            if (@params->detailMeshes != default)
            {
                ushort vbase = 0;
                for (int i = 0; i < @params->polyCount; ++i)
                {
                    ref dtPolyDetail dtl = ref navDMeshes[i];
                    int vb = (int)@params->detailMeshes[i * 4 + 0];
                    int ndv = (int)@params->detailMeshes[i * 4 + 1];
                    int nv = navPolys[i].vertCount;
                    dtl.vertBase = (uint)vbase;
                    dtl.vertCount = (byte)(ndv - nv);
                    dtl.triBase = (uint)@params->detailMeshes[i * 4 + 2];
                    dtl.triCount = (byte)@params->detailMeshes[i * 4 + 3];
                    // Copy vertices except the first 'nv' verts which are equal to nav poly verts.
                    if ((ndv - nv) != 0)
                    {
                        Buffer.MemoryCopy(&@params->detailVerts[(vb + nv) * 3], &navDVerts[vbase * 3], sizeof(float) * 3 * (ndv - nv), sizeof(float) * 3 * (ndv - nv));
                        vbase += (ushort)(ndv - nv);
                    }
                }
                // Store triangles.
                Buffer.MemoryCopy(@params->detailTris, navDTris, sizeof(byte) * 4 * @params->detailTriCount, sizeof(byte) * 4 * @params->detailTriCount);
            }
            else
            {
                // Create dummy detail mesh by triangulating polys.
                int tbase = 0;
                for (int i = 0; i < @params->polyCount; ++i)
                {
                    dtPolyDetail dtl = navDMeshes[i];
                    int nv = navPolys[i].vertCount;
                    dtl.vertBase = 0;
                    dtl.vertCount = 0;
                    dtl.triBase = (uint)tbase;
                    dtl.triCount = (byte)(nv - 2);
                    // Triangulate polygon (local indices).
                    for (int j = 2; j < nv; ++j)
                    {
                        var t = &navDTris[tbase * 4];
                        t[0] = 0;
                        t[1] = (byte)(j - 1);
                        t[2] = (byte)j;
                        // Bit for each edge that belongs to poly boundary.
                        t[3] = (byte)(1 << 2);
                        if (j == 2)
                            t[3] |= (1 << 0);
                        if (j == nv - 1)
                            t[3] |= (1 << 4);
                        tbase++;
                    }
                }
            }

            // Store and create BVtree.
            if (@params->buildBvTree)
            {
                createBVTree(@params, navBvtree, 2 * @params->polyCount);
            }

            // Store Off-Mesh connections.
            n = 0;
            for (int i = 0; i < @params->offMeshConCount; ++i)
            {
                // Only store connections which start from this tile.
                if (offMeshConClass[i * 2 + 0] == 0xff)
                {
                    dtOffMeshConnection* con = &offMeshCons[n];
                    con->poly = (ushort)(offMeshPolyBase + n);
                    // Copy connection end-points.
                    var endPts = &@params->offMeshConVerts[i * 2 * 3];
                    dtVcopy(&con->pos[0], &endPts[0]);
                    dtVcopy(&con->pos[3], &endPts[3]);
                    con->rad = @params->offMeshConRad[i];
                    con->flags = @params->offMeshConDir[i] != default ? (byte)DT_OFFMESH_CON_BIDIR : (byte)0;
                    con->side = offMeshConClass[i * 2 + 1];
                    if (@params->offMeshConUserID != default)
                        con->userId = @params->offMeshConUserID[i];
                    n++;
                }
            }

            dtFree(offMeshConClass);

            *outData = data;
            outDataSize = dataSize;

            return true;
        }

        /// Swaps the endianess of the tile data's header (#dtMeshHeader).
        ///  @param[in,out]	data		The tile data array.
        ///  @param[in]		dataSize	The size of the data array.
        public static bool dtNavMeshHeaderSwapEndian(byte* data, int dataSize)
        {
            dtMeshHeader* header = (dtMeshHeader*)data;

            int swappedMagic = DT_NAVMESH_MAGIC;
            int swappedVersion = DT_NAVMESH_VERSION;
            dtSwapEndian(&swappedMagic);
            dtSwapEndian(&swappedVersion);

            if ((header->magic != DT_NAVMESH_MAGIC || header->version != DT_NAVMESH_VERSION) &&
                (header->magic != swappedMagic || header->version != swappedVersion))
            {
                return false;
            }

            dtSwapEndian(&header->magic);
            dtSwapEndian(&header->version);
            dtSwapEndian(&header->x);
            dtSwapEndian(&header->y);
            dtSwapEndian(&header->layer);
            dtSwapEndian(&header->userId);
            dtSwapEndian(&header->polyCount);
            dtSwapEndian(&header->vertCount);
            dtSwapEndian(&header->maxLinkCount);
            dtSwapEndian(&header->detailMeshCount);
            dtSwapEndian(&header->detailVertCount);
            dtSwapEndian(&header->detailTriCount);
            dtSwapEndian(&header->bvNodeCount);
            dtSwapEndian(&header->offMeshConCount);
            dtSwapEndian(&header->offMeshBase);
            dtSwapEndian(&header->walkableHeight);
            dtSwapEndian(&header->walkableRadius);
            dtSwapEndian(&header->walkableClimb);
            dtSwapEndian(&header->bmin[0]);
            dtSwapEndian(&header->bmin[1]);
            dtSwapEndian(&header->bmin[2]);
            dtSwapEndian(&header->bmax[0]);
            dtSwapEndian(&header->bmax[1]);
            dtSwapEndian(&header->bmax[2]);
            dtSwapEndian(&header->bvQuantFactor);

            // Freelist index and pointers are updated when tile is added, no need to swap.

            return true;
        }

        /// @par
        ///
        /// @warning This function assumes that the header is in the correct endianess already. 
        /// Call #dtNavMeshHeaderSwapEndian() first on the data if the data is expected to be in wrong endianess 
        /// to start with. Call #dtNavMeshHeaderSwapEndian() after the data has been swapped if converting from 
        /// native to foreign endianess.

        /// Swaps endianess of the tile data.
        ///  @param[in,out]	data		The tile data array.
        ///  @param[in]		dataSize	The size of the data array.
        public static bool dtNavMeshDataSwapEndian(byte* data, int dataSize)
        {
            // Make sure the data is in right format.
            dtMeshHeader* header = (dtMeshHeader*)data;
            if (header->magic != DT_NAVMESH_MAGIC)
                return false;
            if (header->version != DT_NAVMESH_VERSION)
                return false;

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
            float* verts = dtGetThenAdvanceBufferPointer<float>(ref d, vertsSize);
            dtPoly* polys = dtGetThenAdvanceBufferPointer<dtPoly>(ref d, polysSize);
            d += linksSize; // Ignore links; they technically should be endian-swapped but all their data is overwritten on load anyway.
                            //dtLink* links = dtGetThenAdvanceBufferPointer<dtLink>(d, linksSize);
            dtPolyDetail* detailMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(ref d, detailMeshesSize);
            float* detailVerts = dtGetThenAdvanceBufferPointer<float>(ref d, detailVertsSize);
            d += detailTrisSize; // Ignore detail tris; single bytes can't be endian-swapped.
                                 //unsigned char* detailTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
            dtBVNode* bvTree = dtGetThenAdvanceBufferPointer<dtBVNode>(ref d, bvtreeSize);
            dtOffMeshConnection* offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(ref d, offMeshLinksSize);

            // Vertices
            for (int i = 0; i < header->vertCount * 3; ++i)
            {
                dtSwapEndian(&verts[i]);
            }

            // Polys
            for (int i = 0; i < header->polyCount; ++i)
            {
                dtPoly* p = &polys[i];
                // poly->firstLink is update when tile is added, no need to swap.
                for (int j = 0; j < DT_VERTS_PER_POLYGON; ++j)
                {
                    dtSwapEndian(&p->verts[j]);
                    dtSwapEndian(&p->neis[j]);
                }
                dtSwapEndian(&p->flags);
            }

            // Links are rebuild when tile is added, no need to swap.

            // Detail meshes
            for (int i = 0; i < header->detailMeshCount; ++i)
            {
                dtPolyDetail* pd = &detailMeshes[i];
                dtSwapEndian(&pd->vertBase);
                dtSwapEndian(&pd->triBase);
            }

            // Detail verts
            for (int i = 0; i < header->detailVertCount * 3; ++i)
            {
                dtSwapEndian(&detailVerts[i]);
            }

            // BV-tree
            for (int i = 0; i < header->bvNodeCount; ++i)
            {
                dtBVNode* node = &bvTree[i];
                for (int j = 0; j < 3; ++j)
                {
                    dtSwapEndian(&node->bmin[j]);
                    dtSwapEndian(&node->bmax[j]);
                }
                dtSwapEndian(&node->i);
            }

            // Off-mesh Connections.
            for (int i = 0; i < header->offMeshConCount; ++i)
            {
                dtOffMeshConnection* con = &offMeshCons[i];
                for (int j = 0; j < 6; ++j)
                    dtSwapEndian(&con->pos[j]);
                dtSwapEndian(&con->rad);
                dtSwapEndian(&con->poly);
            }

            return true;
        }


        // This section contains detailed documentation for members that don't have
        // a source file. It reduces clutter in the main section of the header.

        /**
		
		@struct dtNavMeshCreateParams
		@par
		
		This structure is used to marshal data between the Recast mesh generation pipeline and Detour navigation components.
		
		See the rcPolyMesh and rcPolyMeshDetail documentation for detailed information related to mesh structure.
		
		Units are usually in voxels (vx) or world units (wu). The units for voxels, grid size, and cell size 
		are all based on the values of #cs and #ch.
		
		The standard navigation mesh build process is to create tile data using dtCreateNavMeshData, then add the tile 
		to a navigation mesh using either the dtNavMesh single tile <tt>init()</tt> function or the dtNavMesh::addTile()
		function.
		
		@see dtCreateNavMeshData
		
		*/



        internal static ushort MESH_NULL_IDX = 0xffff;

        internal static int compareItemX(void* va, void* vb)
        {
            BVItem* a = (BVItem*)va;
            BVItem* b = (BVItem*)vb;
            if (a->bmin[0] < b->bmin[0])
                return -1;
            if (a->bmin[0] > b->bmin[0])
                return 1;
            return 0;
        }

        internal static int compareItemY(void* va, void* vb)
        {
            BVItem* a = (BVItem*)va;
            BVItem* b = (BVItem*)vb;
            if (a->bmin[1] < b->bmin[1])
                return -1;
            if (a->bmin[1] > b->bmin[1])
                return 1;
            return 0;
        }

        internal static int compareItemZ(void* va, void* vb)
        {
            BVItem* a = (BVItem*)va;
            BVItem* b = (BVItem*)vb;
            if (a->bmin[2] < b->bmin[2])
                return -1;
            if (a->bmin[2] > b->bmin[2])
                return 1;
            return 0;
        }

        internal static void calcExtends(BVItem* items, int nitems, int imin, int imax, ushort* bmin, ushort* bmax)
        {
            bmin[0] = items[imin].bmin[0];
            bmin[1] = items[imin].bmin[1];
            bmin[2] = items[imin].bmin[2];

            bmax[0] = items[imin].bmax[0];
            bmax[1] = items[imin].bmax[1];
            bmax[2] = items[imin].bmax[2];

            for (int i = imin + 1; i < imax; ++i)
            {
                ref BVItem it = ref items[i];
                if (it.bmin[0] < bmin[0]) bmin[0] = it.bmin[0];
                if (it.bmin[1] < bmin[1]) bmin[1] = it.bmin[1];
                if (it.bmin[2] < bmin[2]) bmin[2] = it.bmin[2];

                if (it.bmax[0] > bmax[0]) bmax[0] = it.bmax[0];
                if (it.bmax[1] > bmax[1]) bmax[1] = it.bmax[1];
                if (it.bmax[2] > bmax[2]) bmax[2] = it.bmax[2];
            }
        }

        public static int longestAxis(ushort x, ushort y, ushort z)
        {
            int axis = 0;
            ushort maxVal = x;
            if (y > maxVal)
            {
                axis = 1;
                maxVal = y;
            }
            if (z > maxVal)
            {
                axis = 2;
            }
            return axis;
        }

        internal static void subdivide(BVItem* items, int nitems, int imin, int imax, ref int curNode, dtBVNode* nodes)
        {
            int inum = imax - imin;
            int icur = curNode;

            //ref dtBVNode node = ref nodes[curNode++];
            dtBVNode* node = &nodes[curNode++];

            if (inum == 1)
            {
                // Leaf
                node->bmin[0] = items[imin].bmin[0];
                node->bmin[1] = items[imin].bmin[1];
                node->bmin[2] = items[imin].bmin[2];

                node->bmax[0] = items[imin].bmax[0];
                node->bmax[1] = items[imin].bmax[1];
                node->bmax[2] = items[imin].bmax[2];

                node->i = items[imin].i;
            }
            else
            {
                // Split
                calcExtends(items, nitems, imin, imax, node->bmin, node->bmax);

                int axis = longestAxis((ushort)(node->bmax[0] - node->bmin[0]),
                                       (ushort)(node->bmax[1] - node->bmin[1]),
                                       (ushort)(node->bmax[2] - node->bmin[2]));

                if (axis == 0)
                {
                    // Sort along x-axis
                    qsort(items + imin, inum, sizeof(BVItem), &compareItemX);
                }
                else if (axis == 1)
                {
                    // Sort along y-axis
                    qsort(items + imin, inum, sizeof(BVItem), &compareItemY);
                }
                else
                {
                    // Sort along z-axis
                    qsort(items + imin, inum, sizeof(BVItem), &compareItemZ);
                }

                int isplit = imin + inum / 2;

                // Left
                subdivide(items, nitems, imin, isplit, ref curNode, nodes);
                // Right
                subdivide(items, nitems, isplit, imax, ref curNode, nodes);

                int iescape = curNode - icur;
                // Negative index means escape.
                node->i = -iescape;
            }
        }

        internal static int createBVTree(dtNavMeshCreateParams* @params, dtBVNode* nodes, int UnnamedParameter)
        {
            // Build tree
            float quantFactor = 1 / @params->cs;
            BVItem* items = (BVItem*)dtAlloc(sizeof(BVItem) * @params->polyCount);
            float* bmin = stackalloc float[3];
            float* bmax = stackalloc float[3];
            for (int i = 0; i < @params->polyCount; i++)
            {
                ref BVItem it = ref items[i];
                it.i = i;
                // Calc polygon bounds. Use detail meshes if available.
                if (@params->detailMeshes != default)
                {
                    int vb = (int)@params->detailMeshes[i * 4 + 0];
                    int ndv = (int)@params->detailMeshes[i * 4 + 1];

                    var dv = &@params->detailVerts[vb * 3];
                    dtVcopy(bmin, dv);
                    dtVcopy(bmax, dv);

                    for (int j = 1; j < ndv; j++)
                    {
                        dtVmin(bmin, &dv[j * 3]);
                        dtVmax(bmax, &dv[j * 3]);
                    }

                    // BV-tree uses cs for all dimensions
                    it.bmin[0] = (ushort)dtClamp((int)((bmin[0] - @params->bmin[0]) * quantFactor), 0, 0xffff);
                    it.bmin[1] = (ushort)dtClamp((int)((bmin[1] - @params->bmin[1]) * quantFactor), 0, 0xffff);
                    it.bmin[2] = (ushort)dtClamp((int)((bmin[2] - @params->bmin[2]) * quantFactor), 0, 0xffff);

                    it.bmax[0] = (ushort)dtClamp((int)((bmax[0] - @params->bmin[0]) * quantFactor), 0, 0xffff);
                    it.bmax[1] = (ushort)dtClamp((int)((bmax[1] - @params->bmin[1]) * quantFactor), 0, 0xffff);
                    it.bmax[2] = (ushort)dtClamp((int)((bmax[2] - @params->bmin[2]) * quantFactor), 0, 0xffff);
                }
                else
                {
                    var p = &@params->polys[i * @params->nvp * 2];
                    it.bmin[0] = it.bmax[0] = @params->verts[p[0] * 3 + 0];
                    it.bmin[1] = it.bmax[1] = @params->verts[p[0] * 3 + 1];
                    it.bmin[2] = it.bmax[2] = @params->verts[p[0] * 3 + 2];

                    for (int j = 1; j < @params->nvp; ++j)
                    {
                        if (p[j] == MESH_NULL_IDX)
                            break;
                        ushort x = @params->verts[p[j] * 3 + 0];
                        ushort y = @params->verts[p[j] * 3 + 1];
                        ushort z = @params->verts[p[j] * 3 + 2];

                        if (x < it.bmin[0])
                            it.bmin[0] = x;
                        if (y < it.bmin[1])
                            it.bmin[1] = y;
                        if (z < it.bmin[2])
                            it.bmin[2] = z;

                        if (x > it.bmax[0])
                            it.bmax[0] = x;
                        if (y > it.bmax[1])
                            it.bmax[1] = y;
                        if (z > it.bmax[2])
                            it.bmax[2] = z;
                    }
                    // Remap y
                    it.bmin[1] = (ushort)Math.Floor((float)it.bmin[1] * @params->ch / @params->cs);
                    it.bmax[1] = (ushort)Math.Ceiling((float)it.bmax[1] * @params->ch / @params->cs);
                }
            }

            int curNode = 0;
            subdivide(items, @params->polyCount, 0, @params->polyCount, ref curNode, nodes);

            dtFree(items);

            return curNode;
        }

        internal static byte classifyOffMeshPoint(float* pt, ReadOnlySpan<float> bmin, ReadOnlySpan<float> bmax)
        {
            const byte XP = (byte)(1 << 0);
            const byte ZP = (byte)(1 << 1);
            const byte XM = (byte)(1 << 2);
            const byte ZM = (byte)(1 << 3);

            int outcode = 0;
            outcode |= (pt[0] >= bmax[0]) ? XP : 0;
            outcode |= (pt[2] >= bmax[2]) ? ZP : 0;
            outcode |= (pt[0] < bmin[0]) ? XM : 0;
            outcode |= (pt[2] < bmin[2]) ? ZM : 0;

            switch (outcode)
            {
                case XP:
                    return 0;
                case XP | ZP:
                    return 1;
                case ZP:
                    return 2;
                case XM | ZP:
                    return 3;
                case XM:
                    return 4;
                case XM | ZM:
                    return 5;
                case ZM:
                    return 6;
                case XP | ZM:
                    return 7;
            };

            return 0xff;
        }


        /// Allocates a query object using the Detour allocator.
        /// @return An allocated query object, or null on failure.
        /// @ingroup detour
        public static dtNavMeshQuery dtAllocNavMeshQuery()
        {
            return new dtNavMeshQuery();
        }

        /// Frees the specified query object using the Detour allocator.
        ///  @param[in]		query		A query object allocated using #dtAllocNavMeshQuery
        /// @ingroup detour
        public static void dtFreeNavMeshQuery(dtNavMeshQuery navmesh)
        {
            if (navmesh == null)
            {
                return;
            }
            navmesh.Dispose();
        }

        internal const float H_SCALE = 0.999f; // Search heuristic scale.


        internal const ushort DT_NULL_IDX = unchecked((ushort)~0);

        internal const int DT_NODE_PARENT_BITS = 24;
        internal const int DT_NODE_STATE_BITS = 2;

        internal const int DT_MAX_STATES_PER_NODE = 1 << DT_NODE_STATE_BITS; // number of extra states per node. See dtNode::state


        // From Thomas Wang, https://gist.github.com/badboy/6267743
        public static uint dtHashRef(dtPolyRef a)
        {
            a = (~a) + (a << 18); // a = (a << 18) - a - 1;
            a = a ^ (a >> 31);
            a = a * 21; // a = (a + (a << 2)) + (a << 4);
            a = a ^ (a >> 11);
            a = a + (a << 6);
            a = a ^ (a >> 22);
            return (uint)a;
        }

        public const uint DT_FAILURE = 1u << 31;            // Operation failed.
        public const uint DT_SUCCESS = 1u << 30;            // Operation succeed.
        public const uint DT_IN_PROGRESS = 1u << 29;        // Operation still in progress.

        // Detail information for status.
        public const uint DT_STATUS_DETAIL_MASK = 0x0ffffff;
        public const uint DT_WRONG_MAGIC = 1 << 0;      // Input data is not recognized.
        public const uint DT_WRONG_VERSION = 1 << 1;    // Input data is in wrong version.
        public const uint DT_OUT_OF_MEMORY = 1 << 2;    // Operation ran out of memory.
        public const uint DT_INVALID_PARAM = 1 << 3;    // An input parameter was invalid.
        public const uint DT_BUFFER_TOO_SMALL = 1 << 4; // Result buffer for the query was too small to store all results.
        public const uint DT_OUT_OF_NODES = 1 << 5;     // Query ran out of nodes during search.
        public const uint DT_PARTIAL_RESULT = 1 << 6;   // Query did not reach the end location, returning best guess. 
        public const uint DT_ALREADY_OCCUPIED = 1 << 7; // A tile has already been assigned to the given x,y coordinate

        // Returns true of status is success.
        public static bool dtStatusSucceed(dtStatus status)
        {
            return (status & DT_SUCCESS) != 0;
        }

        // Returns true of status is failure.
        public static bool dtStatusFailed(dtStatus status)
        {
            return (status & DT_FAILURE) != 0;
        }

        // Returns true of status is in progress.
        public static bool dtStatusInProgress(dtStatus status)
        {
            return (status & DT_IN_PROGRESS) != 0;
        }

        // Returns true if specific detail is set.
        public static bool dtStatusDetail(dtStatus status, uint detail)
        {
            return (status & detail) != 0;
        }
    }
}