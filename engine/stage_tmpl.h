/* Type-parameterised stage kernel; included twice (TY=i128, TY=int64_t). No include guard on purpose. */
#define CAT_(a, b) a##b
#define CAT(a, b) CAT_(a, b)
#define CastN CAT(Cast, SUF)
#define StageN CAT(Stage, SUF)

typedef struct {
    int sh, rm, wl, sat;
    TY lo, hi;
    UTY mask;
} CastN;

static void CAT(make_cast, SUF)(CastN *c, const Raw *r) {
    c->sh = r->sh; c->rm = r->rm; c->wl = r->wl; c->sat = r->sat;
    if (r->s) { c->lo = -((TY)1 << (r->wl - 1)); c->hi = ((TY)1 << (r->wl - 1)) - 1; }
    else { c->lo = 0; c->hi = (TY)(((UTY)1 << r->wl) - 1); }
    c->mask = (UTY)(((UTY)1 << r->wl) - 1);
}

#ifndef DIAG_HIT
#define DIAG_HIT(sat) ((void)0)
#endif
static inline TY CAT(ovf, SUF)(TY n, const CastN *c) {
    if (n >= c->lo && n <= c->hi) return n;
    DIAG_HIT(c->sat);   /* diagnostics: a value left this quantisation point's type (128-bit instance only) */
    if (c->sat) return n < c->lo ? c->lo : c->hi;
    UTY m = (UTY)n & c->mask;
    if (c->lo < 0 && (m >> (c->wl - 1))) return (TY)m - ((TY)1 << c->wl);
    return (TY)m;
}

static inline TY CAT(do_cast, SUF)(TY n, const CastN *c) {
    int sh = c->sh;
    if (sh <= 0) return CAT(ovf, SUF)((TY)((UTY)n << (-sh)), c);   /* via unsigned: no UB for negative n */
    TY q = n >> sh;
    UTY r = (UTY)n & (((UTY)1 << sh) - 1);
    if (r != 0) {
        switch (c->rm) {
        case RM_FLOOR: break;
        case RM_CEIL: q += 1; break;
        case RM_ZERO: if (n < 0) q += 1; break;
        default: {
            UTY half = (UTY)1 << (sh - 1);
            if (c->rm == RM_NEAREST) { if (r >= half) q += 1; }
            else if (r > half) q += 1;
            else if (r == half) {
                if (c->rm == RM_ROUND) { if (n > 0) q += 1; }
                else q += (q & 1); /* convergent */
            }
        }
        }
    }
    return CAT(ovf, SUF)(q, c);
}

typedef struct {
    TY *npar;
    CastN *gain;   /* product -> gain output */
    CastN *tapin;  /* gain output -> accumulator type */
    int *sign;
    int first;     /* first '+' tap or -1 */
    CastN acc;     /* accumulator overflow behaviour */
    CastN out;     /* accumulator -> stage output */
} StageN;

static int CAT(stage_alloc, SUF)(StageN *d, int nt) {
    d->npar = malloc(sizeof(TY) * (size_t)nt);
    d->gain = malloc(sizeof(CastN) * (size_t)nt);
    d->tapin = malloc(sizeof(CastN) * (size_t)nt);
    d->sign = malloc(sizeof(int) * (size_t)nt);
    return d->npar && d->gain && d->tapin && d->sign;
}

static void CAT(stage_free, SUF)(StageN *d) {
    free(d->npar); free(d->gain); free(d->tapin); free(d->sign);
}

static void CAT(run_stage, SUF)(const StageN *st, int nt, const TY *u, TY *y, long n) {
    const int first = st->first;
    for (long i = 0; i < n; i++) {
        TY acc = 0;
        if (first >= 0) {
            TY uu = (i - first >= 0) ? u[i - first] : 0;
            acc = CAT(do_cast, SUF)(CAT(do_cast, SUF)(uu * st->npar[first], &st->gain[first]), &st->tapin[first]);
        }
        for (int k = 0; k < nt; k++) {
            if (k == first) continue;
            TY uu = (i - k >= 0) ? u[i - k] : 0;
            TY v = CAT(do_cast, SUF)(CAT(do_cast, SUF)(uu * st->npar[k], &st->gain[k]), &st->tapin[k]);
            acc = CAT(ovf, SUF)(st->sign[k] > 0 ? acc + v : acc - v, &st->acc);
        }
        y[i] = CAT(do_cast, SUF)(acc, &st->out);
    }
}

#undef CastN
#undef StageN
