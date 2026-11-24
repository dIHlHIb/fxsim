/* Chunk executor for array mode; included twice (TY = i128 / int64_t). No include guard on purpose.
 * Needs TY, SUF, CAST_C(v,p) (instruction cast), CAST_I(v,p,q) (cast of input q), OVF_A(v,p) (accumulator),
 * FW_OK (1: use the branch-free Floor/wrap/signed loops when every cast of the instruction allows it). */
#define ACAT_(a, b) a##b
#define ACAT(a, b) ACAT_(a, b)
static void ACAT(arr_exec, SUF)(const Ins *ins, long ni, const int *order, long no, const int *ds, const int *dsrc,
                                TY *mem, TY *carry, TY *acc, int m, i128 *wacc) {
    for (long o = 0; o < no; o++) {
        int id = order[o];
        if (id >= ni) {   /* Unit Delay: one-sample shift with carried state */
            long k = id - ni;
            TY *y = mem + (size_t)ds[k] * ACHUNK, *x = mem + (size_t)dsrc[k] * ACHUNK;
            y[0] = carry[k];
            for (int j = 1; j < m; j++) y[j] = x[j - 1];
            carry[k] = x[m - 1];
            continue;
        }
        const Ins *p = &ins[id];
        if (p->op >= 10) {   /* instructions: element by element through the 128-bit helper */
            TY *y = mem + (size_t)p->dst * ACHUNK;
            int srcs[1025];
            i128 in[1025];
            int ns_ = ins_srcs(p, srcs);
            for (int j = 0; j < m; j++) {
                for (int q = 0; q < ns_; q++) in[q] = (i128)mem[(size_t)srcs[q] * ACHUNK + j];
                y[j] = (TY)op_new(p, in);
            }
            continue;
        }
#if FW_OK
        if (p->wide) { exec_wide64(p, (int64_t *)mem, wacc, m); continue; }
#else
        (void)wacc;
#endif
        TY *y = mem + (size_t)p->dst * ACHUNK;
        const TY *a = mem + (size_t)p->a * ACHUNK, *b = mem + (size_t)p->b * ACHUNK;
        switch (p->op) {
        case 1: { TY k1 = (TY)p->k1;
            if (FW_OK && p->fw) { const int r = p->f.rsh, l = p->f.lsh, w = p->f.sw;
                for (int j = 0; j < m; j++) y[j] = (TY)fw_cast((int64_t)(a[j] * k1), r, l, w); }
            else for (int j = 0; j < m; j++) y[j] = CAST_C(a[j] * k1, p);
            break; }
        case 2: for (int j = 0; j < m; j++) y[j] = CAST_C(a[j] * b[j], p); break;
        case 3: for (int j = 0; j < m; j++) y[j] = CAST_C(a[j], p); break;
        case 4: { TY lo = (TY)p->k1, hi = (TY)p->k2;
            for (int j = 0; j < m; j++) { TY v = a[j]; if (v < lo) v = lo; if (v > hi) v = hi; y[j] = v; } break; }
        case 5: for (int j = 0; j < m; j++) { TY v = a[j]; v = p->x ? -v : (v < 0 ? -v : v); y[j] = CAST_C(v, p); } break;
        case 6: {
            TY sa = (TY)1 << p->x, sb = (TY)1 << p->y;
            for (int j = 0; j < m; j++) {
                TY u = a[j] * sa, w = b[j] * sb, v;
                switch (p->n) {
                case 0: v = u == w; break; case 1: v = u != w; break; case 2: v = u < w; break;
                case 3: v = u <= w; break; case 4: v = u > w; break; default: v = u >= w; break;
                }
                y[j] = v;
            }
            break;
        }
        case 7: {
            if (p->x >= 0) { const TY *xs = mem + (size_t)p->src[p->x] * ACHUNK;
                for (int j = 0; j < m; j++) acc[j] = CAST_I(xs[j], p, p->x); }
            else for (int j = 0; j < m; j++) acc[j] = 0;
            for (int q = 0; q < p->n; q++) {
                if (q == p->x) continue;
                const TY *xs = mem + (size_t)p->src[q] * ACHUNK;
                if (FW_OK && p->fw) {
                    const int r = p->fs[q].rsh, l = p->fs[q].lsh, w = p->fs[q].sw, aw = p->f2.sw;
                    if (p->sg[q] > 0) for (int j = 0; j < m; j++)
                        acc[j] = (TY)fw_cast((int64_t)((uint64_t)acc[j] + (uint64_t)fw_cast((int64_t)xs[j], r, l, w)), 0, 0, aw);
                    else for (int j = 0; j < m; j++)
                        acc[j] = (TY)fw_cast((int64_t)((uint64_t)acc[j] - (uint64_t)fw_cast((int64_t)xs[j], r, l, w)), 0, 0, aw);
                } else if (p->sg[q] > 0) for (int j = 0; j < m; j++) acc[j] = OVF_A(acc[j] + CAST_I(xs[j], p, q), p);
                else for (int j = 0; j < m; j++) acc[j] = OVF_A(acc[j] - CAST_I(xs[j], p, q), p);
            }
            for (int j = 0; j < m; j++) y[j] = CAST_C(acc[j], p);
            break;
        }
        case 8: {
            const TY *x0 = mem + (size_t)p->src[0] * ACHUNK;
            for (int j = 0; j < m; j++) acc[j] = CAST_I(x0[j], p, 0);
            for (int q = 1; q < p->n; q++) {
                const TY *xs = mem + (size_t)p->src[q] * ACHUNK;
                for (int j = 0; j < m; j++) { TY v = CAST_I(xs[j], p, q); if (p->x ? v > acc[j] : v < acc[j]) acc[j] = v; }
            }
            for (int j = 0; j < m; j++) y[j] = CAST_C(acc[j], p);
            break;
        }
        default: for (int j = 0; j < m; j++) y[j] = a[j]; break;
        }
    }
}
#undef ACAT
#undef ACAT_
