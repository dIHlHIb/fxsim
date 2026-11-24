/*
 * 64-bit fast kernel: used when every intermediate of a candidate fits int64 (checked by the front-end).
 * Loop order is tap-outer / sample-inner over cache-sized chunks, so every per-tap constant is
 * loop-invariant and branches are perfectly predictable. Rounding is branchless via a bias:
 *   q = ((n + b0 + (neg & bneg) + (bit & bbit)) >> rsh) << lsh,   neg = n<0, bit = floor-bit of n/2^rsh
 * which reproduces Floor / Ceiling / Zero / Nearest / Round / Convergent exactly (see fxcore._round_div).
 */
typedef struct {
    int rsh, lsh, sat, sgn, sw;
    int64_t b0, bneg, bbit, lo, hi, mask;
} FC;

static void fc_make(FC *c, const Raw *r) {
    memset(c, 0, sizeof(*c));
    int sh = r->sh;
    c->rsh = sh > 0 ? sh : 0;
    c->lsh = sh < 0 ? -sh : 0;
    c->sat = r->sat;
    c->sgn = r->s;
    c->sw = 64 - r->wl;
    if (sh > 0) {
        int64_t full = ((int64_t)1 << sh) - 1, half = (int64_t)1 << (sh - 1);
        switch (r->rm) {
        case RM_CEIL: c->b0 = full; break;
        case RM_ZERO: c->bneg = full; break;
        case RM_NEAREST: c->b0 = half; break;
        case RM_ROUND: c->b0 = half; c->bneg = -1; break;
        case RM_CONV: c->b0 = half - 1; c->bbit = 1; break;
        default: break; /* floor */
        }
    }
    if (r->s) { c->lo = -((int64_t)1 << (r->wl - 1)); c->hi = ((int64_t)1 << (r->wl - 1)) - 1; }
    else { c->lo = 0; c->hi = (int64_t)(((uint64_t)1 << r->wl) - 1); }
    c->mask = c->hi;
}

static inline int64_t fc_cast(int64_t n, const FC *c) {
    int64_t neg = n >> 63;
    int64_t bit = (n >> c->rsh) & 1;
    int64_t q = ((n + c->b0 + (neg & c->bneg) + (bit & c->bbit)) >> c->rsh) * ((int64_t)1 << c->lsh);
    if (q < c->lo || q > c->hi) {
        if (c->sat) q = q < c->lo ? c->lo : c->hi;
        else if (c->sgn) q = (int64_t)((uint64_t)q << c->sw) >> c->sw;
        else q &= c->mask;
    }
    return q;
}

typedef struct {
    int64_t *npar;
    FC *gain, *tapin;
    int *sign;
    int first;
    FC acc, out;
} StageF;

#define FCHUNK 1024

/* Floor + wrap + signed specialisation: cast(n) = sign-extend_wl((n >> rsh) << lsh), always applied
 * (identity when in range), so the inner loop has no branches. Exact same results as fc_cast for
 * casts with rm = Floor, sat = 0, s = 1 (checked per stage by fc_is_fw). */
static int fc_is_fw(const FC *c) { return c->sgn && !c->sat && !c->b0 && !c->bneg && !c->bbit; }

static inline int64_t fw_cast(int64_t n, int rsh, int lsh, int sw) {
    uint64_t q = (uint64_t)(n >> rsh) << lsh;
    return (int64_t)(q << sw) >> sw;
}

static int stage_is_fw(const StageF *st, int nt) {
    if (!fc_is_fw(&st->acc) || !fc_is_fw(&st->out)) return 0;
    for (int k = 0; k < nt; k++)
        if (!fc_is_fw(&st->gain[k]) || !fc_is_fw(&st->tapin[k])) return 0;
    return 1;
}

static void run_stage_fw(const StageF *st, int nt, const int64_t *u, int64_t *y, long n, int64_t *acc) {
    const int first = st->first;
    const int asw = st->acc.sw;
    for (long c0 = 0; c0 < n; c0 += FCHUNK) {
        long m = n - c0 < FCHUNK ? n - c0 : FCHUNK;
        if (first >= 0) {
            const int64_t *up = u + c0 - first;
            const FC *g = &st->gain[first], *t = &st->tapin[first];
            const int64_t np = st->npar[first];
            for (long i = 0; i < m; i++)
                acc[i] = fw_cast(fw_cast(up[i] * np, g->rsh, g->lsh, g->sw), t->rsh, t->lsh, t->sw);
        } else {
            for (long i = 0; i < m; i++) acc[i] = 0;
        }
        for (int k = 0; k < nt; k++) {
            if (k == first) continue;
            const int64_t *up = u + c0 - k;
            const int64_t np = st->npar[k];
            const int gr = st->gain[k].rsh, gl = st->gain[k].lsh, gs = st->gain[k].sw;
            const int tr = st->tapin[k].rsh, tl = st->tapin[k].lsh, ts = st->tapin[k].sw;
            if (st->sign[k] > 0)
                for (long i = 0; i < m; i++)
                    acc[i] = fw_cast((int64_t)((uint64_t)acc[i] + (uint64_t)fw_cast(fw_cast(up[i] * np, gr, gl, gs), tr, tl, ts)), 0, 0, asw);
            else
                for (long i = 0; i < m; i++)
                    acc[i] = fw_cast((int64_t)((uint64_t)acc[i] - (uint64_t)fw_cast(fw_cast(up[i] * np, gr, gl, gs), tr, tl, ts)), 0, 0, asw);
        }
        const FC *o = &st->out;
        for (long i = 0; i < m; i++) y[c0 + i] = fw_cast(acc[i], o->rsh, o->lsh, o->sw);
    }
}

/* u points at element 0 of a series that has at least nt zero elements before it. */
static int run_stage_fast(const StageF *st, int nt, const int64_t *u, int64_t *y, long n) {
    int64_t *acc = malloc(sizeof(int64_t) * FCHUNK);
    if (!acc) return 0;
    if (!getenv("FXCHAIN_NO_FW") && stage_is_fw(st, nt)) { run_stage_fw(st, nt, u, y, n, acc); free(acc); return 1; }
    const int first = st->first;
    for (long c0 = 0; c0 < n; c0 += FCHUNK) {
        long m = n - c0 < FCHUNK ? n - c0 : FCHUNK;
        if (first >= 0) {
            const int64_t *up = u + c0 - first;
            for (long i = 0; i < m; i++) acc[i] = fc_cast(fc_cast(up[i] * st->npar[first], &st->gain[first]), &st->tapin[first]);
        } else {
            for (long i = 0; i < m; i++) acc[i] = 0;
        }
        for (int k = 0; k < nt; k++) {
            if (k == first) continue;
            const int64_t *up = u + c0 - k;
            const int64_t np = st->npar[k];
            const FC *g = &st->gain[k], *t = &st->tapin[k], *a = &st->acc;
            if (st->sign[k] > 0)
                for (long i = 0; i < m; i++) acc[i] = fc_cast(acc[i] + fc_cast(fc_cast(up[i] * np, g), t), a);
            else
                for (long i = 0; i < m; i++) acc[i] = fc_cast(acc[i] - fc_cast(fc_cast(up[i] * np, g), t), a);
        }
        for (long i = 0; i < m; i++) y[c0 + i] = fc_cast(acc[i], &st->out);
    }
    free(acc);
    return 1;
}
