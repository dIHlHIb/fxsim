#define _POSIX_C_SOURCE 200809L  /* clock_gettime under -std=c99 on Linux */
/*
 * fxchain_core: bit-exact fixed-point cascaded FIR simulator (per-sample hot loop).
 * Reads a whitespace-separated token protocol on stdin (produced by the Python
 * front-end `fxchain`), writes one text record per candidate on stdout.
 * No global mutable state; one process handles any number of candidates.
 *
 * Input protocol (all integers are decimal, 128-bit capable):
 *   X n s wl fl  v0 .. v(n-1)            design input, stored integers
 *   REF n s wl fl v0 .. v(n-1)           optional reference for the final stage
 *   C nstages ntaps outmode final_fl     one candidate; followed by, per stage:
 *       per tap : npar  gsh grm gs gwl gsat          (gain: product shift/cast)
 *       then    : first  and per tap: sign ash   (sum input cast to acc, via cast)
 *     see read_candidate() for the exact order.
 *   END
 * A "cast" is: sh rm s wl sat   with sh = fl_from - fl_to (>0 right shift w/ rounding).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <math.h>

typedef __int128 i128;
typedef unsigned __int128 u128;

enum { RM_FLOOR, RM_CEIL, RM_ZERO, RM_NEAREST, RM_ROUND, RM_CONV };

typedef struct { int sh, rm, s, wl, sat; } Raw;

static char *buf;
static size_t pos, len;

static int fail(const char *code, const char *msg) {
    printf("E %s %s\n", code, msg);
    fflush(stdout);
    return 1;
}

static int next_tok(char **start) {
    while (pos < len && (buf[pos] == ' ' || buf[pos] == '\n' || buf[pos] == '\r' || buf[pos] == '\t')) pos++;
    if (pos >= len) return 0;
    *start = buf + pos;
    while (pos < len && !(buf[pos] == ' ' || buf[pos] == '\n' || buf[pos] == '\r' || buf[pos] == '\t')) pos++;
    buf[pos] = 0; /* NUL-terminate the token in place (buffer has a spare byte) */
    if (pos < len) pos++;
    return 1;
}

static int rd_i128(i128 *out) {
    char *s;
    if (!next_tok(&s)) return 0;
    int neg = 0;
    if (*s == '-') { neg = 1; s++; }
    u128 v = 0;
    while (*s >= '0' && *s <= '9') { v = v * 10 + (unsigned)(*s - '0'); s++; }
    *out = neg ? -(i128)v : (i128)v;
    return 1;
}

static int rd_int(int *out) {
    i128 v;
    if (!rd_i128(&v)) return 0;
    *out = (int)v;
    return 1;
}

static void print_i128(FILE *f, i128 v) {
    char t[48];
    int n = 0;
    int neg = v < 0;
    u128 u = neg ? -(u128)v : (u128)v;
    if (u == 0) t[n++] = '0';
    while (u) { t[n++] = (char)('0' + (int)(u % 10)); u /= 10; }
    if (neg) fputc('-', f);
    while (n) fputc(t[--n], f);
}

static int rd_raw(Raw *r) {
    return rd_int(&r->sh) && rd_int(&r->rm) && rd_int(&r->s) && rd_int(&r->wl) && rd_int(&r->sat);
}

static i128 *x_in, *ref;
static long nx, nref;
static int xs, xwl, xfl, rs, rwl, rfl;

#define TY i128
#define UTY u128
#define SUF 128
#include "stage_tmpl.h"
#undef TY
#undef UTY
#undef SUF
#include "fast_kernel.h"

/* ---- streaming mode (-s): line-at-a-time protocol for search drivers ----
 * Identical arithmetic; additionally a "base" candidate (preceded by the token BASE) keeps its
 * per-stage outputs so later candidates that share a prefix of identical stages reuse them.
 * Reuse is exact (a stage is reused only if all its parameters are identical), so results do not
 * depend on the cache. Output per candidate:
 *   R core_ns nx ns first_recomputed_stage
 *   S s min max                      (per stage)
 *   M maxabsdiff F sumdiff argmax    (only with REF; stored units of 2^-F)
 *   D
 */
static int g_stream, g_want_base;
static struct { int valid, ns, nt; long nx; StageF *st; int64_t **out; } base;

static int stagef_eq(const StageF *a, const StageF *b, int nt) {
    return a->first == b->first && !memcmp(&a->acc, &b->acc, sizeof(FC)) && !memcmp(&a->out, &b->out, sizeof(FC)) &&
           !memcmp(a->npar, b->npar, sizeof(int64_t) * (size_t)nt) && !memcmp(a->sign, b->sign, sizeof(int) * (size_t)nt) &&
           !memcmp(a->gain, b->gain, sizeof(FC) * (size_t)nt) && !memcmp(a->tapin, b->tapin, sizeof(FC) * (size_t)nt);
}

static void stagef_free(StageF *d) { free(d->npar); free(d->gain); free(d->tapin); free(d->sign); }

static void stream_report(long ns_t, int ns, int start, const int64_t *const *o64, i128 *const *o128, int ofl) {
    printf("R %ld %ld %d %d\n", ns_t, nx, ns, start);
    for (int s = 0; s < ns; s++) {
        i128 mn, mx;
        if (o64) { int64_t a = o64[s][0], b = o64[s][0];
            for (long i = 1; i < nx; i++) { int64_t v = o64[s][i]; a = v < a ? v : a; b = v > b ? v : b; }
            mn = a; mx = b;
        } else { mn = mx = o128[s][0];
            for (long i = 1; i < nx; i++) { i128 v = o128[s][i]; if (v < mn) mn = v; if (v > mx) mx = v; }
        }
        printf("S %d ", s); print_i128(stdout, mn); putchar(' '); print_i128(stdout, mx); putchar('\n');
    }
    if (nref > 0) {
        int F = ofl > rfl ? ofl : rfl;
        i128 md = 0, sum = 0;
        long am = 0;
        for (long i = 0; i < nx; i++) {
            i128 y = o64 ? (i128)o64[ns - 1][i] : o128[ns - 1][i];
            i128 d = y * ((i128)1 << (F - ofl)) - ref[i] * ((i128)1 << (F - rfl));
            sum += d;
            if (d < 0) d = -d;
            if (d > md) { md = d; am = i; }
        }
        printf("M "); print_i128(stdout, md); printf(" %d ", F); print_i128(stdout, sum); printf(" %ld\n", am);
    }
    printf("D\n");
    fflush(stdout);
}

static int stream_eval(int ns, int nt, int ofl, int fast, StageF *s64, Stage128 *s128, i128 **ys) {
    struct timespec t0, t1;
    int want_base = g_want_base;
    g_want_base = 0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    if (!fast) {
        for (int s = 0; s < ns; s++) run_stage128(&s128[s], nt, s == 0 ? x_in : ys[s - 1], ys[s], nx);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        stream_report((long)(t1.tv_sec - t0.tv_sec) * 1000000000L + (t1.tv_nsec - t0.tv_nsec), ns, 0, NULL, ys, ofl);
        for (int s = 0; s < ns; s++) stage_free128(&s128[s]);
        return 0;
    }
    size_t pad = (size_t)nt, len64 = pad + (size_t)nx;
    int start = 0;
    int usable = base.valid && base.ns == ns && base.nt == nt && base.nx == nx;
    if (usable) while (start < ns && stagef_eq(&s64[start], &base.st[start], nt)) start++;
    int64_t **out = calloc((size_t)ns, sizeof(int64_t *));
    int64_t *xp = NULL;
    if (!out) return fail("OOM", "alloc");
    if (start == 0) {
        xp = calloc(len64, sizeof(int64_t));
        if (!xp) return fail("OOM", "alloc");
        for (long i = 0; i < nx; i++) xp[pad + (size_t)i] = (int64_t)x_in[i];
    }
    for (int s = 0; s < ns; s++) {
        if (s < start) { out[s] = base.out[s]; continue; }
        out[s] = calloc(len64, sizeof(int64_t));
        const int64_t *in = s == 0 ? xp : out[s - 1];
        if (!out[s] || !run_stage_fast(&s64[s], nt, in + pad, out[s] + pad, nx)) return fail("OOM", "alloc");
    }
    free(xp);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    const int64_t **view = malloc(sizeof(int64_t *) * (size_t)ns);
    if (!view) return fail("OOM", "alloc");
    for (int s = 0; s < ns; s++) view[s] = out[s] + pad;
    stream_report((long)(t1.tv_sec - t0.tv_sec) * 1000000000L + (t1.tv_nsec - t0.tv_nsec), ns, start, view, NULL, ofl);
    free(view);
    if (want_base) {
        if (base.valid && !usable) {
            for (int s = 0; s < base.ns; s++) { stagef_free(&base.st[s]); free(base.out[s]); }
            free(base.st); free(base.out); base.valid = 0;
        }
        if (!base.valid) {
            base.st = calloc((size_t)ns, sizeof(StageF)); base.out = calloc((size_t)ns, sizeof(int64_t *));
            if (!base.st || !base.out) return fail("OOM", "alloc");
            start = 0;
            for (int s = 0; s < ns; s++) base.out[s] = NULL;
        }
        for (int s = 0; s < ns; s++) {
            if (s < start) { stagef_free(&s64[s]); continue; }   /* identical to the stored stage */
            if (base.valid) { stagef_free(&base.st[s]); free(base.out[s]); }
            base.st[s] = s64[s]; base.out[s] = out[s];
        }
        base.valid = 1; base.ns = ns; base.nt = nt; base.nx = nx;
    } else {
        for (int s = 0; s < ns; s++) { stagef_free(&s64[s]); if (s >= start) free(out[s]); }
    }
    free(out);
    return 0;
}

static int read_candidate(void) {
    int ns, nt, mode, ofl, fast;
    if (!rd_int(&ns) || !rd_int(&nt) || !rd_int(&mode) || !rd_int(&ofl) || !rd_int(&fast))
        return fail("PROTOCOL", "truncated candidate header");
    if (ns < 1 || ns > 64 || nt < 1 || nt > 4096) return fail("PROTOCOL", "bad nstages/ntaps");
    size_t tn = (size_t)ns * (size_t)nt;
    i128 *npar = malloc(sizeof(i128) * tn);
    Raw *gr = malloc(sizeof(Raw) * tn), *tr = malloc(sizeof(Raw) * tn);
    int *sg = malloc(sizeof(int) * tn), *firsts = malloc(sizeof(int) * (size_t)ns);
    Raw *accr = malloc(sizeof(Raw) * (size_t)ns), *outr = malloc(sizeof(Raw) * (size_t)ns);
    i128 **ys = calloc((size_t)ns, sizeof(i128 *));
    Stage128 *s128 = calloc((size_t)ns, sizeof(Stage128));
    StageF *s64 = calloc((size_t)ns, sizeof(StageF));
    if (!npar || !gr || !tr || !sg || !firsts || !accr || !outr || !ys || !s128 || !s64) return fail("OOM", "alloc");
    for (int s = 0; s < ns; s++) {
        for (int k = 0; k < nt; k++) {
            size_t j = (size_t)s * (size_t)nt + (size_t)k;
            if (!rd_i128(&npar[j]) || !rd_raw(&gr[j]) || !rd_raw(&tr[j]) || !rd_int(&sg[j]))
                return fail("PROTOCOL", "truncated stage");
        }
        if (!rd_int(&firsts[s]) || !rd_raw(&accr[s]) || !rd_raw(&outr[s])) return fail("PROTOCOL", "truncated stage");
        ys[s] = malloc(sizeof(i128) * (size_t)nx);
        if (!ys[s]) return fail("OOM", "alloc");
    }
    for (int s = 0; s < ns; s++) {
        if (fast) {
            StageF *d = &s64[s];
            d->npar = malloc(sizeof(int64_t) * (size_t)nt);
            d->gain = malloc(sizeof(FC) * (size_t)nt);
            d->tapin = malloc(sizeof(FC) * (size_t)nt);
            d->sign = malloc(sizeof(int) * (size_t)nt);
            if (!d->npar || !d->gain || !d->tapin || !d->sign) return fail("OOM", "alloc");
            for (int k = 0; k < nt; k++) {
                size_t j = (size_t)s * (size_t)nt + (size_t)k;
                d->npar[k] = (int64_t)npar[j]; d->sign[k] = sg[j];
                fc_make(&d->gain[k], &gr[j]); fc_make(&d->tapin[k], &tr[j]);
            }
            d->first = firsts[s]; fc_make(&d->acc, &accr[s]); fc_make(&d->out, &outr[s]);
        } else {
            Stage128 *d = &s128[s];
            if (!stage_alloc128(d, nt)) return fail("OOM", "alloc");
            for (int k = 0; k < nt; k++) {
                size_t j = (size_t)s * (size_t)nt + (size_t)k;
                d->npar[k] = npar[j]; d->sign[k] = sg[j];
                make_cast128(&d->gain[k], &gr[j]); make_cast128(&d->tapin[k], &tr[j]);
            }
            d->first = firsts[s]; make_cast128(&d->acc, &accr[s]); make_cast128(&d->out, &outr[s]);
        }
    }
    if (g_stream) {
        int rc = stream_eval(ns, nt, ofl, fast, s64, s128, ys);
        for (int s = 0; s < ns; s++) free(ys[s]);
        free(npar); free(gr); free(tr); free(sg); free(firsts); free(accr); free(outr); free(ys); free(s128); free(s64);
        return rc;
    }
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    if (fast) {
        size_t pad = (size_t)nt, len64 = pad + (size_t)nx;
        int64_t *prev = calloc(len64, sizeof(int64_t)), *cur = NULL;
        if (!prev) return fail("OOM", "alloc");
        for (long i = 0; i < nx; i++) prev[pad + (size_t)i] = (int64_t)x_in[i];
        for (int s = 0; s < ns; s++) {
            cur = calloc(len64, sizeof(int64_t));
            if (!cur || !run_stage_fast(&s64[s], nt, prev + pad, cur + pad, nx)) return fail("OOM", "alloc");
            for (long i = 0; i < nx; i++) ys[s][i] = cur[pad + (size_t)i];
            free(prev);
            prev = cur;
        }
        free(prev);
    } else {
        for (int s = 0; s < ns; s++) run_stage128(&s128[s], nt, s == 0 ? x_in : ys[s - 1], ys[s], nx);
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    long ns_t = (long)(t1.tv_sec - t0.tv_sec) * 1000000000L + (t1.tv_nsec - t0.tv_nsec);
    printf("R %ld %ld %d\n", ns_t, nx, ns);
    for (int s = 0; s < ns; s++) {
        /* per-stage summary: fnv1a64 over (lo64,hi64) words, min, max */
        uint64_t h = 1469598103934665603ULL;
        i128 mn = ys[s][0], mx = ys[s][0];
        for (long i = 0; i < nx; i++) {
            i128 v = ys[s][i];
            uint64_t w[2] = {(uint64_t)(u128)v, (uint64_t)((u128)v >> 64)};
            for (int j = 0; j < 2; j++)
                for (int b = 0; b < 8; b++) { h ^= (w[j] >> (8 * b)) & 0xff; h *= 1099511628211ULL; }
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        printf("S %d %016llx ", s, (unsigned long long)h);
        print_i128(stdout, mn); putchar(' '); print_i128(stdout, mx); putchar('\n');
    }
    if (nref > 0) {
        int F = ofl > rfl ? ofl : rfl;
        i128 md = 0;
        for (long i = 0; i < nx; i++) {
            i128 d = ys[ns - 1][i] * ((i128)1 << (F - ofl)) - ref[i] * ((i128)1 << (F - rfl));
            if (d < 0) d = -d;
            if (d > md) md = d;
        }
        printf("M "); print_i128(stdout, md); printf(" %d\n", F);
    }
    /* outputs: mode 0 = none, 1 = final, 2 = all stages */
    if (mode >= 1) {
        for (int s = (mode == 2 ? 0 : ns - 1); s < ns; s++) {
            printf("O %d", s);
            for (long i = 0; i < nx; i++) { putchar(' '); print_i128(stdout, ys[s][i]); }
            putchar('\n');
        }
    }
    printf("D\n");
    for (int s = 0; s < ns; s++) {
        if (fast) { free(s64[s].npar); free(s64[s].gain); free(s64[s].tapin); free(s64[s].sign); } else stage_free128(&s128[s]);
        free(ys[s]);
    }
    free(npar); free(gr); free(tr); free(sg); free(firsts); free(accr); free(outr); free(ys); free(s128); free(s64);
    return 0;
}

static void base_drop(void) {
    if (!base.valid) return;
    for (int s = 0; s < base.ns; s++) { stagef_free(&base.st[s]); free(base.out[s]); }
    free(base.st); free(base.out);
    base.valid = 0;
}

/* Handle one top-level token. Returns 0 to continue, 1 on END, >1 on a fatal error (already reported). */
static int dispatch(const char *tok) {
    if (!strcmp(tok, "X")) {
        i128 v; int ok = 1;
        long nn; ok &= rd_i128(&v); nn = (long)v;
        ok &= rd_int(&xs) && rd_int(&xwl) && rd_int(&xfl);
        if (!ok || nn < 1) return 1 + fail("PROTOCOL", "bad X header");
        free(x_in);
        base_drop();
        x_in = malloc(sizeof(i128) * (size_t)nn);
        if (!x_in) return 1 + fail("OOM", "alloc x");
        for (long i = 0; i < nn; i++) if (!rd_i128(&x_in[i])) return 1 + fail("PROTOCOL", "truncated X");
        nx = nn;
    } else if (!strcmp(tok, "NOREF")) {
        nref = 0;
    } else if (!strcmp(tok, "REF")) {
        i128 v; int ok = rd_i128(&v);
        long nn = (long)v;
        ok &= rd_int(&rs) && rd_int(&rwl) && rd_int(&rfl);
        if (!ok || nn < 1) return 1 + fail("PROTOCOL", "bad REF header");
        free(ref);
        ref = malloc(sizeof(i128) * (size_t)nn);
        if (!ref) return 1 + fail("OOM", "alloc ref");
        for (long i = 0; i < nn; i++) if (!rd_i128(&ref[i])) return 1 + fail("PROTOCOL", "truncated REF");
        nref = nn;
    } else if (!strcmp(tok, "DREF")) {
        /* double-precision cascade reference on the current x: DREF ns nt c(1,1)..c(1,nt) c(2,1).. ;
           stored as REF with 60 fraction bits (exact for |y| >= 2^-7, else within 2^-61) */
        int dns, dnt;
        if (!x_in || !rd_int(&dns) || !rd_int(&dnt) || dns < 1 || dns > 64 || dnt < 1 || dnt > 4096)
            return 1 + fail("PROTOCOL", "bad DREF header");
        double *c = malloc(sizeof(double) * (size_t)dns * (size_t)dnt);
        double *u = malloc(sizeof(double) * (size_t)nx), *y = malloc(sizeof(double) * (size_t)nx);
        if (!c || !u || !y) return 1 + fail("OOM", "alloc dref");
        for (int j = 0; j < dns * dnt; j++) {
            char *t, *e;
            if (!next_tok(&t)) return 1 + fail("PROTOCOL", "truncated DREF");
            c[j] = strtod(t, &e);
            if (e == t) return 1 + fail("PROTOCOL", "bad DREF coefficient");
        }
        for (long i = 0; i < nx; i++) u[i] = ldexp((double)x_in[i], -xfl);
        for (int s = 0; s < dns; s++) {
            const double *cs = c + (size_t)s * (size_t)dnt;
            for (long i = 0; i < nx; i++) {
                double acc = 0.0;
                for (int k = 0; k < dnt && k <= i; k++) acc += cs[k] * u[i - k];
                y[i] = acc;
            }
            double *t = u; u = y; y = t;
        }
        free(ref);
        ref = malloc(sizeof(i128) * (size_t)nx);
        if (!ref) return 1 + fail("OOM", "alloc ref");
        for (long i = 0; i < nx; i++) ref[i] = (i128)nearbyint(ldexp(u[i], 60));
        rs = 1; rwl = 126; rfl = 60; nref = nx;
        free(c); free(u); free(y);
    } else if (!strcmp(tok, "BASE") && g_stream) {
        g_want_base = 1;
    } else if (!strcmp(tok, "C")) {
        if (!x_in) return 1 + fail("PROTOCOL", "C before X");
        if (nref > 0 && nref != nx) return 1 + fail("PROTOCOL", "REF length differs from X");
        if (read_candidate()) return 2;
    } else if (!strcmp(tok, "END")) return 1;
    else return 1 + fail("PROTOCOL", "unknown token");
    return 0;
}

int main(int argc, char **argv) {
    char *tok;
    int r = 0;
    if (argc > 1 && !strcmp(argv[1], "-s")) {
        /* streaming: one command group per line, answer flushed after each candidate */
        g_stream = 1;
        size_t cap = 0;
        ssize_t got;
        buf = NULL;
        while ((got = getline(&buf, &cap, stdin)) > 0) {
            pos = 0; len = (size_t)got;
            while (next_tok(&tok)) {
                r = dispatch(tok);
                if (r) break;
            }
            if (r == 1) return 0;
            if (r > 1) return 1;
        }
        return 0;
    }
    size_t cap = 1 << 20;
    buf = malloc(cap + 1);
    if (!buf) return fail("OOM", "stdin buffer");
    size_t n;
    while ((n = fread(buf + len, 1, cap - len, stdin)) > 0) {
        len += n;
        if (len == cap) { cap *= 2; buf = realloc(buf, cap + 1); if (!buf) return fail("OOM", "stdin buffer"); }
    }
    while (next_tok(&tok)) {
        r = dispatch(tok);
        if (r == 1) break;
        if (r > 1) return 1;
    }
    return 0;
}
