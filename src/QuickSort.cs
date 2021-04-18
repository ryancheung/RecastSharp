// MIT License - Copyright (C) ryancheung and the FelCore team
// This file is subject to the terms and conditions defined in
// file 'LICENSE', which is part of this source code package.

using System;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace RecastSharp
{
    public static unsafe class QuickSort
    {
        static void swapcode<T>(byte* parmi, byte* parmj, int n) where T : unmanaged
        {
            var i = (n) / sizeof(T);
            T* pi = (T*)parmi;
            T* pj = (T*)parmj;

            do
            {
                T t = *pi;
                *pi++ = *pj;
                *pj++ = t;
            } while (--i > 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void SWAPINIT(byte* a, int es, out int swaptype)
        {
            swaptype = ((a - (byte*)0) % sizeof(CLong) != 0) || (es % sizeof(CLong) != 0) ? 2 : (es == sizeof(CLong) ? 0 : 1);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void swapfunc(byte* a, byte* b, int n, int swaptype)
        {
            if (swaptype <= 1)
                swapcode<CLong>(a, b, n);
            else
                swapcode<byte>(a, b, n);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void swap(byte* a, byte* b, int es, int swaptype)
        {
            if (swaptype == 0)
            {
                var t = *(CLong*)a;
                *(CLong*)a = *(CLong*)b;
                *(CLong*)b = t;
            }
            else
                swapfunc(a, b, es, swaptype);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void vecswap(byte* a, byte* b, int n, int swaptype)
        {
            if (n > 0)
                swapfunc(a, b, n, swaptype);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static byte* med3(byte* a, byte* b, byte* c, delegate* managed<void*, void*, int> cmp)
        {
            return cmp(a, b) < 0 ?
                (cmp(b, c) < 0 ? b : (cmp(a, c) < 0 ? c : a))
                    : (cmp(b, c) > 0 ? b : (cmp(a, c) < 0 ? a : c));
        }

        internal static void qsort(void* aa, int n, int es, delegate* managed<void*, void*, int> cmp)
        {
            byte* pa; byte* pb; byte* pc; byte* pd; byte* pl; byte* pm; byte* pn;
            int d, r, swaptype, swap_cnt;
            byte* a = (byte*)aa;

        loop:
            SWAPINIT(a, es, out swaptype);

            swap_cnt = 0;
            if (n < 7)
            {
                for (pm = a + es; pm < a + n * es; pm += es)
                    for (pl = pm; pl > a && cmp(pl - es, pl) > 0; pl -= es)
                        swap(pl, pl - es, es, swaptype);
                return;
            }
            pm = a + (n / 2) * es;
            if (n > 7)
            {
                pl = a;
                pn = a + (n - 1) * es;
                if (n > 40)
                {
                    d = (n / 8) * es;
                    pl = med3(pl, pl + d, pl + 2 * d, cmp);
                    pm = med3(pm - d, pm, pm + d, cmp);
                    pn = med3(pn - 2 * d, pn - d, pn, cmp);
                }
                pm = med3(pl, pm, pn, cmp);
            }
            swap(a, pm, es, swaptype);
            pa = pb = a + es;

            pc = pd = a + (n - 1) * es;
            for (; ; )
            {
                while (pb <= pc && (r = cmp(pb, a)) <= 0)
                {
                    if (r == 0)
                    {
                        swap_cnt = 1;
                        swap(pa, pb, es, swaptype);
                        pa += es;
                    }
                    pb += es;
                }
                while (pb <= pc && (r = cmp(pc, a)) >= 0)
                {
                    if (r == 0)
                    {
                        swap_cnt = 1;
                        swap(pc, pd, es, swaptype);
                        pd -= es;
                    }
                    pc -= es;
                }
                if (pb > pc)
                    break;
                swap(pb, pc, es, swaptype);
                swap_cnt = 1;
                pb += es;
                pc -= es;
            }
            if (swap_cnt == 0)
            {  /* Switch to insertion sort */
                for (pm = a + es; pm < a + n * es; pm += es)
                    for (pl = pm; pl > a && cmp(pl - es, pl) > 0; pl -= es)
                        swap(pl, pl - es, es, swaptype);
                return;
            }
            pn = a + n * es;
            r = (int)Math.Min(pa - a, pb - pa);
            vecswap(a, pb - r, r, swaptype);
            r = (int)Math.Min(pd - pc, pn - pd - es);
            vecswap(pb, pn - r, r, swaptype);
            r = (int)(pb - pa);
            if (r > es)
                qsort(a, r / es, es, cmp);
            r = (int)(pd - pc);
            if (r > es)
            {
                /* Iterate rather than recurse to save stack space */
                a = pn - r;
                n = r / es;
                goto loop;
            }
        }
    }
}
