#define _POSIX_C_SOURCE 200809L
/*
 * fxgraph_core: step interpreter for compiled fixed-point block diagrams (fast path of engine/fxgraph).
 * The Python front-end resolves all types and casts; this program only executes integer instructions.
 * Values are __int128 (front-end guarantees every intermediate fits 126 bits).
 *
 * Input (whitespace-separated decimal tokens):
 *   P nslots ninstr nsteps
 *   <ninstr instructions>            see the op list below; C = sh rm s wl sat (a cast)
 *   IN slot v0 .. v(nsteps-1)        one line per Inport
 *   INIT slot value                  one per Unit Delay (initial state)
 *   DLY nd (dslot srcslot)*nd        delay updates at the end of each step
 *   REC nr slot*nr                   slots to record
 *   END
 * Ops: 1 GAIN dst src npar C | 2 PROD dst a b C | 3 CAST dst src C | 4 SAT dst src lo hi
 *      5 UN dst src neg C (abs if neg=0, minus if neg=1; C has sh 0) | 6 REL dst a b sha shb op
 *      7 SUM dst n first accC(sh0) outC (src C sign)*n | 8 MINMAX dst n ismax outC (src C)*n | 9 COPY dst src
 * Output: "O slot v0 .. v(nsteps-1)" per recorded slot, then "T core_ns", then "D".
 * Errors: "E CODE message".
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

/* diagnostics (engine/fxgraph "diagnostics": true on the C path): per slot wraps, saturations, their first steps,
   divisions by zero and the first one -- the counts the Python reference (Design.run_diagnostics) keeps, counted at
   every 128-bit quantisation point while the instruction writing that slot runs. Off (NULL) unless a DIAG section */
static long *g_dc = NULL;
static long g_dcur = -1, g_dstep = 0;
static inline void diag_hit(int sat) {
    if (!g_dc || g_dcur < 0) return;
    long *e = g_dc + 6 * g_dcur;
    if (sat) { if (e[1]++ == 0) e[3] = g_dstep; }
    else { if (e[0]++ == 0) e[2] = g_dstep; }
}
static inline void diag_div0(void) {
    if (!g_dc || g_dcur < 0) return;
    long *e = g_dc + 6 * g_dcur;
    if (e[4]++ == 0) e[5] = g_dstep;
}
#define DIAG_HIT(sat) diag_hit(sat)
#define TY i128
#define UTY u128
#define SUF 128
#include "stage_tmpl.h"
#undef TY
#undef UTY
#undef SUF
#undef DIAG_HIT
#define DIAG_HIT(sat) ((void)0)

/* a strict Multiport Switch (DiagnosticForDefault Error, or one data port) whose control leaves the
   data ports stops Simulink; the instruction carries ncontig negated, the first such event is recorded here and
   reported by the run modes (one-shot: E SIM_DIAGNOSTIC slot step; candidates: an infinite error) */
static int g_diag = 0;
static int g_oneshot = 0;   /* the one-shot run mode: instructions with state (DEN) run only there */
static long g_diag_slot = -1;

static int fail(const char *code, const char *msg) { printf("E %s %s\n", code, msg); fflush(stdout); return 1; }

static char *buf;
static size_t pos, len;

static int tok(char **s) {
    while (pos < len && (buf[pos] == ' ' || buf[pos] == '\n' || buf[pos] == '\r' || buf[pos] == '\t')) pos++;
    if (pos >= len) return 0;
    *s = buf + pos;
    while (pos < len && !(buf[pos] == ' ' || buf[pos] == '\n' || buf[pos] == '\r' || buf[pos] == '\t')) pos++;
    buf[pos] = 0;
    if (pos < len) pos++;
    return 1;
}

static int rd(i128 *o) {
    char *s;
    if (!tok(&s)) return 0;
    int neg = 0;
    if (*s == '-') { neg = 1; s++; }
    if (*s < '0' || *s > '9') return 0;
    u128 v = 0;
    while (*s >= '0' && *s <= '9') { v = v * 10 + (unsigned)(*s - '0'); s++; }
    *o = neg ? -(i128)v : (i128)v;
    return 1;
}

static int rdi(long *o) { i128 v; if (!rd(&v)) return 0; *o = (long)v; return 1; }

static void pr(i128 v) {
    char t[48]; int n = 0, neg = v < 0;
    u128 u = neg ? -(u128)v : (u128)v;
    if (!u) t[n++] = '0';
    while (u) { t[n++] = (char)('0' + (int)(u % 10)); u /= 10; }
    if (neg) putchar('-');
    while (n) putchar(t[--n]);
}

#include "fast_kernel.h"

/* every cast is kept in both forms: 128-bit (general) and 64-bit (FC, valid when the front-end marks the
 * candidate fast: all intermediates fit 62 bits) */
typedef struct { int op, dst, a, b, n, x, y, fw, wide; i128 k1, k2; Cast128 c, c2; FC f, f2; int *src, *sg, *aux; Cast128 *cs; FC *fs; i128 *kv; } Ins;

static int rdcast(Cast128 *c, FC *f, long nslots) {
    long v[5];
    (void)nslots;
    for (int i = 0; i < 5; i++) if (!rdi(&v[i])) return 0;
    if (v[3] < 1 || v[3] > 126 || v[0] > 126 || v[0] < -126) return 0;
    Raw r = {(int)v[0], (int)v[1], (int)v[2], (int)v[3], (int)v[4]};
    make_cast128(c, &r);
    if (r.wl <= 62 && r.sh <= 62 && r.sh >= -62) fc_make(f, &r); else memset(f, 0, sizeof(*f));
    return 1;
}

static int slot(long v, long nslots) { return v >= 0 && v < nslots; }

static int parse_ins(Ins *ins, long ni, long ns) {
    for (long i = 0; i < ni; i++) {
        Ins *p = &ins[i];
        long op, d, a, b, m, x, y;
        if (!rdi(&op) || !rdi(&d) || !slot(d, ns)) return fail("PROTOCOL", "bad instruction");
        p->op = (int)op; p->dst = (int)d;
        switch (op) {
        case 1: if (!rdi(&a) || !slot(a, ns) || !rd(&p->k1) || !rdcast(&p->c, &p->f, ns)) return fail("PROTOCOL", "GAIN"); p->a = (int)a; break;
        case 2: if (!rdi(&a) || !rdi(&b) || !slot(a, ns) || !slot(b, ns) || !rdcast(&p->c, &p->f, ns)) return fail("PROTOCOL", "PROD");
            p->a = (int)a; p->b = (int)b; break;
        case 3: if (!rdi(&a) || !slot(a, ns) || !rdcast(&p->c, &p->f, ns)) return fail("PROTOCOL", "CAST"); p->a = (int)a; break;
        case 4: if (!rdi(&a) || !slot(a, ns) || !rd(&p->k1) || !rd(&p->k2)) return fail("PROTOCOL", "SAT"); p->a = (int)a; break;
        case 5: if (!rdi(&a) || !slot(a, ns) || !rdi(&x) || !rdcast(&p->c, &p->f, ns)) return fail("PROTOCOL", "UN"); p->a = (int)a; p->x = (int)x; break;
        case 6: if (!rdi(&a) || !rdi(&b) || !rdi(&x) || !rdi(&y) || !rdi(&m) || !slot(a, ns) || !slot(b, ns) || x < 0 || y < 0 || x > 126 || y > 126)
                return fail("PROTOCOL", "REL");
            p->a = (int)a; p->x = (int)x; p->b = (int)b; p->y = (int)y; p->n = (int)m; break;
        case 7: case 8:
            if (!rdi(&m) || m < 1 || m > 1024) return fail("PROTOCOL", "SUM/MINMAX arity");
            p->n = (int)m;
            if (!rdi(&x)) return fail("PROTOCOL", "SUM/MINMAX");
            p->x = (int)x;
            if (op == 7 && !rdcast(&p->c2, &p->f2, ns)) return fail("PROTOCOL", "SUM acc");
            if (!rdcast(&p->c, &p->f, ns)) return fail("PROTOCOL", "SUM/MINMAX out");
            p->src = malloc(sizeof(int) * (size_t)m); p->sg = malloc(sizeof(int) * (size_t)m);
            p->cs = malloc(sizeof(Cast128) * (size_t)m); p->fs = malloc(sizeof(FC) * (size_t)m);
            if (!p->src || !p->sg || !p->cs || !p->fs) return fail("OOM", "alloc");
            for (long j = 0; j < m; j++) {
                long sv, sg = 1;
                if (!rdi(&sv) || !slot(sv, ns) || !rdcast(&p->cs[j], &p->fs[j], ns)) return fail("PROTOCOL", "SUM/MINMAX input");
                if (op == 7 && !rdi(&sg)) return fail("PROTOCOL", "SUM sign");
                p->src[j] = (int)sv; p->sg[j] = (int)sg;
            }
            break;
        case 9: if (!rdi(&a) || !slot(a, ns)) return fail("PROTOCOL", "COPY"); p->a = (int)a; break;
        case 10: if (!rd(&p->k1)) return fail("PROTOCOL", "CONST"); break;
        case 11: {   /* SHIFT dst src left k s wl */
            long lf, k, ss, wl;
            if (!rdi(&a) || !slot(a, ns) || !rdi(&lf) || !rdi(&k) || !rdi(&ss) || !rdi(&wl) || k < 0 || wl < 1 || wl > 126)
                return fail("PROTOCOL", "SHIFT");
            Raw r = {0, 0, (int)ss, (int)wl, 0};
            make_cast128(&p->c, &r);
            p->a = (int)a; p->x = (int)lf; p->y = (int)(k > 127 ? 127 : k);
            break;
        }
        case 12: case 13: case 14: case 15: {
            /* 12 LOGIC dst opcode n src*n | 13 SWITCH dst u1 u2 u3 mode q castA castB
               14 MPSW dst ctrl cfl base ncontig ndata (src cast)*ndata
               15 PRODN dst n special f_out rm_mul rm_div s wl sat (src fl isdiv)*n */
            long v1 = 0, v2 = 0, v3 = 0, v4 = 0;
            if (op == 12) { if (!rdi(&x) || !rdi(&m) || m < 1 || m > 1024) return fail("PROTOCOL", "LOGIC"); p->x = (int)x; }
            else if (op == 13) m = 3;
            else if (op == 14) {
                if (!rdi(&a) || !slot(a, ns) || !rdi(&v1) || !rdi(&v2) || !rdi(&v3) || !rdi(&m) || m < 1 || m > 1023)
                    return fail("PROTOCOL", "MPSW");
                p->x = (int)v1; p->y = (int)v2; p->k1 = v3; p->a = (int)m; m += 1;
            } else {
                long sp, fo, rmm, rmd, ss, wl, sat;
                if (!rdi(&m) || !rdi(&sp) || !rdi(&fo) || !rdi(&rmm) || !rdi(&rmd) || !rdi(&ss) || !rdi(&wl) || !rdi(&sat)
                    || m < 1 || m > 1024 || wl < 1 || wl > 126)
                    return fail("PROTOCOL", "PRODN");
                Raw r = {0, 0, (int)ss, (int)wl, (int)sat};
                make_cast128(&p->c, &r);
                p->x = (int)sp; p->k1 = fo; p->a = (int)rmm; p->b = (int)rmd;
            }
            p->n = (int)m;
            p->src = calloc((size_t)m, sizeof(int)); p->sg = calloc((size_t)m, sizeof(int));
            p->aux = calloc((size_t)m, sizeof(int)); p->cs = calloc((size_t)m, sizeof(Cast128)); p->fs = calloc((size_t)m, sizeof(FC));
            if (!p->src || !p->sg || !p->aux || !p->cs || !p->fs) return fail("OOM", "alloc");
            if (op == 14) p->src[0] = (int)a;
            for (long j = (op == 14 ? 1 : 0); j < m; j++) {
                long sv;
                if (!rdi(&sv) || !slot(sv, ns)) return fail("PROTOCOL", "operand slot");
                p->src[j] = (int)sv;
                if (op == 14 && !rdcast(&p->cs[j - 1], &p->fs[j - 1], ns)) return fail("PROTOCOL", "MPSW cast");
                if (op == 15) {
                    long fl, dv;
                    if (!rdi(&fl) || !rdi(&dv) || fl < -126 || fl > 126) return fail("PROTOCOL", "PRODN operand");
                    p->aux[j] = (int)fl; p->sg[j] = (int)dv;
                }
            }
            if (op == 13) {
                if (!rdi(&v4) || !rd(&p->k1) || !rdcast(&p->cs[0], &p->fs[0], ns) || !rdcast(&p->cs[1], &p->fs[1], ns))
                    return fail("PROTOCOL", "SWITCH");
                p->x = (int)v4;
            }
            break;
        }
        case 16: {   /* BITW dst n code s wl usemask mask src*n */
            long code, ss, wl, um;
            if (!rdi(&m) || !rdi(&code) || !rdi(&ss) || !rdi(&wl) || !rdi(&um) || !rd(&p->k1) || m < 1 || m > 1024
                || wl < 1 || wl > 126) return fail("PROTOCOL", "BITW");
            p->n = (int)m; p->x = (int)code; p->y = (int)ss; p->b = (int)wl; p->a = (int)um;
            p->src = calloc((size_t)m, sizeof(int));
            if (!p->src) return fail("OOM", "alloc");
            for (long j = 0; j < m; j++) { long sv; if (!rdi(&sv) || !slot(sv, ns)) return fail("PROTOCOL", "BITW src"); p->src[j] = (int)sv; }
            break;
        }
        case 17: {   /* EXTR dst src lo nb s */
            long lo, nb, ss;
            if (!rdi(&a) || !slot(a, ns) || !rdi(&lo) || !rdi(&nb) || !rdi(&ss) || lo < 0 || lo > 126 || nb < 1 || nb > 126)
                return fail("PROTOCOL", "EXTR");
            p->a = (int)a; p->x = (int)lo; p->y = (int)nb; p->b = (int)ss;
            break;
        }
        case 19: {   /* LUTL dst src n sa sb ff fs fw rm sat its itw itf ulast dsh C (bp tab)*n slope*n: 1-D linear
                        point-slope; slope = table step at the table's precision, product divided by 2^(ff + dsh) */
            long v[12];
            if (!rdi(&a) || !slot(a, ns) || !rdi(&m) || m < 2 || m > 1000000) return fail("PROTOCOL", "LUTL");
            for (int q = 0; q < 12; q++) if (!rdi(&v[q])) return fail("PROTOCOL", "LUTL header");
            if (v[0] < 0 || v[0] > 126 || v[1] < 0 || v[1] > 126 || v[2] < 0 || v[2] > 62 || v[4] < 1 || v[4] > 64
                || v[8] < 1 || v[8] > 126 || v[11] < 0 || v[2] + v[11] > 120) return fail("PROTOCOL", "LUTL fields");
            p->a = (int)a; p->n = (int)m;
            p->aux = malloc(sizeof(int) * 12);
            p->cs = calloc(3, sizeof(Cast128)); p->fs = calloc(2, sizeof(FC));
            if (!p->aux || !p->cs || !p->fs) return fail("OOM", "alloc");
            for (int q = 0; q < 12; q++) p->aux[q] = (int)v[q];
            Raw rf = {0, 0, (int)v[3], (int)v[4], 1}, ri = {0, 0, (int)v[7], (int)v[8], (int)v[6]};
            /* the slope term frac * (y1 - y0) keeps its sign: signed, one bit wider than an unsigned intermediate
               type (a falling segment with an unsigned intermediate type) */
            Raw rt = {0, 0, 1, (int)v[8] + (v[7] ? 0 : 1), (int)v[6]};
            make_cast128(&p->cs[0], &rf);
            make_cast128(&p->cs[1], &ri);
            make_cast128(&p->cs[2], &rt);
            if (!rdcast(&p->c, &p->f, ns)) return fail("PROTOCOL", "LUTL out cast");
            p->kv = malloc(sizeof(i128) * 3 * (size_t)m);
            if (!p->kv) return fail("OOM", "alloc");
            for (long j = 0; j < 3 * m; j++) if (!rd(&p->kv[j])) return fail("PROTOCOL", "LUTL table");
            break;
        }
        case 20: {   /* LUT2 dst src1 src2 mode n1 n2 sa1 sb1 sa2 sb2 ff fs fw rm sat its itw ulast 0 C bp1*n1 bp2*n2 tab*(n1*n2)
                        2-D table, Clip; mode 0 Flat, 1 Nearest (tab = output values), 2 linear point-slope (tab in the
                        intermediate type, C = output conversion) */
            long v[16], b2;
            if (!rdi(&a) || !slot(a, ns) || !rdi(&b2) || !slot(b2, ns)) return fail("PROTOCOL", "LUT2");
            for (int q = 0; q < 16; q++) if (!rdi(&v[q])) return fail("PROTOCOL", "LUT2 header");
            if (v[0] < 0 || v[0] > 2 || v[1] < 2 || v[2] < 2 || v[1] > 100000 || v[2] > 100000 || v[1] * v[2] > 4000000
                || v[3] < 0 || v[3] > 126 || v[4] < 0 || v[4] > 126 || v[5] < 0 || v[5] > 126 || v[6] < 0 || v[6] > 126
                || v[7] < 0 || v[7] > 62 || v[9] < 1 || v[9] > 64 || v[13] < 1 || v[13] > 126)
                return fail("PROTOCOL", "LUT2 fields");
            p->a = (int)a; p->b = (int)b2; p->n = 2;
            p->aux = malloc(sizeof(int) * 16);
            p->cs = calloc(2, sizeof(Cast128)); p->fs = calloc(2, sizeof(FC));
            if (!p->aux || !p->cs || !p->fs) return fail("OOM", "alloc");
            for (int q = 0; q < 16; q++) p->aux[q] = (int)v[q];
            Raw rf = {0, 0, (int)v[8], (int)v[9], 1}, ri = {0, 0, (int)v[12], (int)v[13], (int)v[11]};
            make_cast128(&p->cs[0], &rf);
            make_cast128(&p->cs[1], &ri);
            if (!rdcast(&p->c, &p->f, ns)) return fail("PROTOCOL", "LUT2 out cast");
            if (v[15] < 0 || v[7] + v[15] > 120) return fail("PROTOCOL", "LUT2 dsh");
            size_t nk = (size_t)(v[1] + v[2] + 2 * v[1] * v[2]);   /* bp1, bp2, tab (intermediate), tab (table type) */
            p->kv = malloc(sizeof(i128) * nk);
            if (!p->kv) return fail("OOM", "alloc");
            for (size_t j = 0; j < nk; j++) if (!rd(&p->kv[j])) return fail("PROTOCOL", "LUT2 table");
            break;
        }
        case 18: {   /* LUT dst src mode n sa sb (bp out)*n */
            long mode, sa, sb;
            if (!rdi(&a) || !slot(a, ns) || !rdi(&mode) || !rdi(&m) || !rdi(&sa) || !rdi(&sb) || m < 2 || m > 1000000
                || sa < 0 || sa > 126 || sb < 0 || sb > 126) return fail("PROTOCOL", "LUT");
            p->a = (int)a; p->x = (int)mode; p->n = (int)m; p->y = (int)sa; p->b = (int)sb;
            p->kv = malloc(sizeof(i128) * 2 * (size_t)m);
            if (!p->kv) return fail("OOM", "alloc");
            for (long j = 0; j < 2 * m; j++) if (!rd(&p->kv[j])) return fail("PROTOCOL", "LUT table");
            break;
        }
        case 21: {   /* PSHIFT dst src ksrc dir mode bp s wl lo hi oorlo oorchk: Shift Arithmetic with the shift from a port
                        (ref/fxcore.py _blk_shift / _port_shift); a step the Python reference stops or refuses on is
                        handed back (E FALLBACK slot step) so the Python path reports it */
            long b2, dir, mode, bp, ss, wl, oorlo, oorchk;
            i128 lo, hi;
            if (!rdi(&a) || !slot(a, ns) || !rdi(&b2) || !slot(b2, ns) || !rdi(&dir) || !rdi(&mode) || !rdi(&bp) ||
                !rdi(&ss) || !rdi(&wl) || !rd(&lo) || !rd(&hi) || !rdi(&oorlo) || !rdi(&oorchk) || wl < 1 || wl > 64 ||
                dir < 0 || dir > 2 || mode < 0 || mode > 2) return fail("PROTOCOL", "PSHIFT");
            Raw r = {0, 0, (int)ss, (int)wl, 0};
            make_cast128(&p->c, &r);
            p->a = (int)a; p->b = (int)b2; p->x = (int)dir; p->y = (int)mode; p->n = (int)bp;
            p->k1 = lo; p->k2 = hi;
            p->sg = malloc(sizeof(int) * 3);
            if (!p->sg) return fail("OOM", "alloc");
            p->sg[0] = (int)oorlo; p->sg[1] = (int)oorchk; p->sg[2] = (int)wl;
            break;
        }
        case 22: {   /* DEN dst u en r N ic enab mode: the Delay with an enable port and / or an external reset, dialog
                        length and initial condition (ref/fxcore.py _blk_delayport); state kept in the instruction,
                        so it runs in the one-shot mode only (elsewhere: E FALLBACK). mode 0 None 1 Level 2 Level hold
                        3 Rising 4 Falling 5 Either */
            long b2, rr, nn, enab, mode;
            i128 ic;
            if (!rdi(&a) || !slot(a, ns) || !rdi(&b2) || !slot(b2, ns) || !rdi(&rr) || !slot(rr, ns) || !rdi(&nn) ||
                nn < 1 || nn > 100000 || !rd(&ic) || !rdi(&enab) || !rdi(&mode) || mode < 0 || mode > 5)
                return fail("PROTOCOL", "DEN");
            p->a = (int)a; p->b = (int)b2; p->n = (int)nn; p->x = (int)enab; p->y = (int)mode; p->k1 = ic;
            p->sg = malloc(sizeof(int) * 2);
            p->kv = malloc(sizeof(i128) * (size_t)(nn + 3));
            if (!p->sg || !p->kv) return fail("OOM", "alloc");
            p->sg[0] = (int)rr; p->sg[1] = 0;   /* reset slot; ring head */
            for (long j = 0; j < nn; j++) p->kv[j] = ic;
            p->kv[nn] = enab ? 0 : ic;   /* the held output: 0 before the first enabled step */
            p->kv[nn + 1] = p->kv[nn + 2] = 0;   /* the reset one and two steps earlier */
            break;
        }
        case 23: {   /* DPORT dst u k x0 r kind lim cap dg pdf rising icport ic L: the Delay with the length (kind 0) or
                        the initial condition (kind 1) from a port (ref/fxcore.py _blk_delayport / delayport_source);
                        a ring buffer of the input kept in the instruction (one-shot mode only, elsewhere E FALLBACK);
                        a step the Python reference stops or refuses on is handed back (E FALLBACK) */
            long ks, xs, rs, kind, lim, cap, dg, pdf, rising, icport, L;
            i128 ic;
            if (!rdi(&a) || !slot(a, ns) || !rdi(&ks) || !slot(ks, ns) || !rdi(&xs) || !slot(xs, ns) || !rdi(&rs) ||
                !slot(rs, ns) || !rdi(&kind) || !rdi(&lim) || !rdi(&cap) || !rdi(&dg) || !rdi(&pdf) || !rdi(&rising) ||
                !rdi(&icport) || !rd(&ic) || !rdi(&L) || kind < 0 || kind > 1 || lim < 0 || cap < 0 || cap > 100000 ||
                L < 0 || L > 100000) return fail("PROTOCOL", "DPORT");
            p->a = (int)a; p->b = (int)ks; p->k1 = ic;
            p->aux = malloc(sizeof(int) * 12);
            long B = (kind == 0 ? cap : L) + 1;
            p->kv = malloc(sizeof(i128) * (size_t)(B + 4));
            if (!p->aux || !p->kv) return fail("OOM", "alloc");
            int *x = p->aux;
            x[0] = (int)xs; x[1] = (int)rs; x[2] = (int)kind; x[3] = (int)lim; x[4] = (int)cap; x[5] = (int)dg;
            x[6] = (int)pdf; x[7] = (int)rising; x[8] = (int)icport; x[9] = (int)L; x[10] = (int)B;
            x[11] = 0;   /* the step */
            for (long j = 0; j < B + 4; j++) p->kv[j] = 0;   /* buffer; [B] x0 at step 0; [B+1] last; [B+2] prev reset */
            break;
        }
        case 31: {   /* PLK dst src mode sa ffl n bp*n: Prelookup (ref/fxcore.py _prelookup_kf); bp at the common scale,
                        the input shifted up by sa; mode 0 the interval k, 1 the fraction floored into ffl bits */
            long md, sa, ffl;
            if (!rdi(&a) || !slot(a, ns) || !rdi(&md) || !rdi(&sa) || !rdi(&ffl) || !rdi(&m) || md < 0 || md > 1 || sa < 0
                || sa > 120 || ffl < 0 || ffl > 62 || m < 2 || m > 1000000) return fail("PROTOCOL", "PLK");
            p->a = (int)a; p->x = (int)md; p->y = (int)sa; p->b = (int)ffl; p->n = (int)m;
            p->kv = malloc(sizeof(i128) * (size_t)m);
            if (!p->kv) return fail("OOM", "alloc");
            for (long j = 0; j < m; j++) if (!rd(&p->kv[j])) return fail("PROTOCOL", "PLK breakpoints");
            break;
        }
        case 30: {   /* IPL dst dims mode n1 n2 ff1 ff2 its itw itf os ow of mag src*(2 dims) tab*(n1 n2):
                        Interpolation Using Prelookup (ref/fxcore.py _blk_interp_pl); tab column-major, in the output
                        type (Flat / Nearest) or the intermediate type (linear) */
            long v[13];
            for (int q = 0; q < 13; q++) if (!rdi(&v[q])) return fail("PROTOCOL", "IPL header");
            if (v[0] < 1 || v[0] > 2 || v[1] < 0 || v[1] > 2 || v[2] < 2 || v[3] < 1 || v[2] > 1000000 || v[3] > 1000000
                || v[2] * v[3] > 4000000 || v[4] < 0 || v[4] > 62 || v[5] < 0 || v[5] > 62 || v[7] < 1 || v[7] > 64
                || v[10] < 1 || v[10] > 64 || v[8] - v[11] > 120 || v[11] - v[8] > 120 || (v[0] == 2 && v[3] < 2))
                return fail("PROTOCOL", "IPL fields");
            p->aux = malloc(sizeof(int) * 13);
            p->n = (int)(2 * v[0]);
            p->src = calloc((size_t)p->n, sizeof(int));
            if (!p->aux || !p->src) return fail("OOM", "alloc");
            for (int q = 0; q < 13; q++) p->aux[q] = (int)v[q];
            for (int j = 0; j < p->n; j++) { long sv; if (!rdi(&sv) || !slot(sv, ns)) return fail("PROTOCOL", "IPL src"); p->src[j] = (int)sv; }
            size_t nt = (size_t)(v[2] * v[3]);
            p->kv = malloc(sizeof(i128) * nt);
            if (!p->kv) return fail("OOM", "alloc");
            for (size_t j = 0; j < nt; j++) if (!rd(&p->kv[j])) return fail("PROTOCOL", "IPL table");
            break;
        }
        default: return fail("PROTOCOL", "unknown op");
        }
        /* Floor/wrap/signed loops apply when every 64-bit cast of a GAIN or SUM is of that kind
           (fc_is_fw is only meaningful for valid 64-bit casts; the flag is only used on the fast path) */
        p->fw = 0;
        if (!getenv("FXGRAPH_NO_FW")) {
            if (op == 1) p->fw = fc_is_fw(&p->f);
            else if (op == 7) {
                int ok = fc_is_fw(&p->f2);
                for (int j = 0; j < p->n && ok; j++) ok = fc_is_fw(&p->fs[j]);
                p->fw = ok;
            }
        }
    }
    return 0;
}

/* ---- instructions (CONST, SHIFT, LOGIC, SWITCH, MPSW, PRODN); same rules as ref/fxcore.py */
static i128 rdiv128(i128 num, i128 den, int rm) {
    if (den < 0) { num = -num; den = -den; }
    i128 q = num / den, r = num % den;
    if (r < 0) { q -= 1; r += den; }          /* floor division, 0 <= r < den */
    if (r == 0) return q;
    switch (rm) {
    case RM_FLOOR: return q;
    case RM_CEIL: return q + 1;
    case RM_ZERO: return num < 0 ? q + 1 : q;
    default: {
        i128 c = 2 * r - den;
        if (c > 0) return q + 1;
        if (c < 0) return q;
        if (rm == RM_NEAREST) return q + 1;
        if (rm == RM_ROUND) return num > 0 ? q + 1 : q;
        return q + (q & 1);
    }
    }
}

static i128 scaled_div(i128 num, int e, i128 den, int rm) {   /* round(num * 2^e / den) */
    if (e >= 0) return rdiv128(num * ((i128)1 << e), den, rm);
    return rdiv128(num, den * ((i128)1 << (-e)), rm);
}

static int ins_srcs(const Ins *p, int *out);

static i128 ipl_wrap(i128 v, int s, int w) {   /* the stored integer v wrapped into an s / w-bit word */
    i128 q = v & ((((i128)1) << w) - 1);
    return (s && ((q >> (w - 1)) & 1)) ? q - (((i128)1) << w) : q;
}

static i128 op_new(const Ins *p, const i128 *in) {
    switch (p->op) {
    case 10: return p->k1;
    case 11: {
        i128 v = in[0];
        if (p->x) return ovf128(p->y >= 127 ? 0 : (i128)((u128)v << p->y), &p->c);
        return p->y >= 127 ? (v < 0 ? -1 : 0) : v >> p->y;
    }
    case 21: {
        i128 n = in[0], k = in[1];
        int w = p->sg[2], dir = p->x;
        if (k > (i128)2147483647 || (p->sg[1] && !(k >= p->sg[0] && k <= 31)) ||
            (p->y == 1 && !(k >= p->k1 && k <= p->k2)) ||
            (p->y == 0 && ((k < 0 ? -k : k) > 7 || (k < 0 ? -k : k) >= w))) {
            if (!g_diag) { g_diag = 3; g_diag_slot = p->dst; }   /* the Python reference stops / refuses here */
            return 0;
        }
        if (p->y == 2) {   /* the general rule (_port_shift) */
            i128 kk = k < -32768 ? -32768 : (k > 32767 ? 32767 : k);
            i128 a = dir == 1 ? (kk > 0 ? kk : 0) : (dir == 0 ? (kk > 0 ? -kk : 0) : kk);
            if (a == 0) return n;
            i128 m = a - p->n;
            if (!(m > -32768 && m <= 32767)) {
                i128 wm = ((m + 32768) & 0xFFFF) - 32768;
                i128 aa = a < 0 ? -a : a;
                a = (wm >= 0 || wm == -32768) ? aa : -aa;
            }
            if (a > 0) return n >> (a < w + 1 ? a : w + 1);
            i128 sh = -a < w + 1 ? -a : w + 1;
            return ovf128((i128)((u128)n << (int)sh), &p->c);
        }
        {   /* the recorded-range rule: Left k > 0 / Bidirectional k < 0 shift left; Right / Bidirectional k > 0 right */
            i128 ak = k < 0 ? -k : k;
            int sh = (int)(ak < w + 1 ? ak : w + 1);
            if ((dir == 0 && k > 0) || (dir == 2 && k < 0)) return ovf128((i128)((u128)n << sh), &p->c);
            if ((dir == 1 && k > 0) || (dir == 2 && k > 0)) return n >> sh;
            return n;
        }
    }
    case 22: {
        if (!g_oneshot) { if (!g_diag) { g_diag = 3; g_diag_slot = p->dst; } return 0; }
        Ins *q = (Ins *)p;
        i128 *st = (i128 *)q->kv;
        int N = q->n, en = q->x ? in[1] > 0 : 1, mode = q->y;
        i128 rt = mode ? in[2] : 0, r1 = st[N + 1], r2 = st[N + 2];
        int sg = (rt > 0) - (rt < 0), s1 = (r1 > 0) - (r1 < 0), s2 = (r2 > 0) - (r2 < 0);
        int rise = (s1 < 0 && sg >= 0) || (s1 == 0 && sg > 0 && s2 >= 0);
        int fall = (s1 > 0 && sg <= 0) || (s1 == 0 && sg < 0 && s2 <= 0);
        int rst = mode == 1 ? (rt != 0 || r1 != 0) : mode == 2 ? rt != 0 : mode == 3 ? rise : mode == 4 ? fall :
                  mode == 5 ? (rise || fall) : 0;
        if (en) {
            if (rst) { for (int j = 0; j < N; j++) st[j] = q->k1; q->sg[1] = 0; }
            int h = q->sg[1];
            st[N] = st[h];          /* the oldest state */
            st[h] = in[0];          /* u shifted in */
            q->sg[1] = (h + 1) % N;
        }
        st[N + 2] = r1; st[N + 1] = rt;
        return st[N];
    }
    case 23: {
        if (!g_oneshot) { if (!g_diag) { g_diag = 3; g_diag_slot = p->dst; } return 0; }
        Ins *q = (Ins *)p;
        int *x = q->aux;
        i128 *buf = q->kv;
        long B = x[10], step = x[11];
        i128 u = in[0];
        buf[step % B] = u;
        if (step == 0) buf[B] = in[2];
        x[11] = (int)(step + 1);
        if (x[2] == 1) {   /* the initial condition from a port, dialog length L */
            long L = x[9];
            return step >= L ? buf[(step - L) % B] : buf[B];
        }
        i128 nk = in[1];
        long lim = x[3], cap = x[4];
        int fb = 0;
        if (!(nk >= 0 && nk <= lim) && x[5]) fb = 1;   /* DiagnosticForDelayLength other than None */
        i128 out = 0;
        if (!fb && (x[6] || x[7])) {   /* PreventDirectFeedthrough / Rising reset */
            i128 lo = x[6] ? 1 : 0;
            if (nk < lo) nk = lo;
            if (nk > lim) nk = lim;
            long last = (long)buf[B + 1];
            if (x[7]) {
                i128 r = in[3], pr = buf[B + 2];
                int rose = r != 0 && (step == 0 || pr == 0);
                if (rose) last = step;
                buf[B + 1] = last; buf[B + 2] = r;
                if (rose && last == step && nk == 0) fb = 1;   /* not recorded: refused by the reference */
            }
            if (!fb) out = (step - nk >= last) ? buf[(step - (long)nk) % B] : q->k1;
        } else if (!fb) {
            if (nk < 0) nk = 0;
            if (nk > lim) nk = lim;
            if (nk > cap) fb = 1;
            else if (step - nk >= 0) out = buf[(step - (long)nk) % B];
            else out = x[8] ? buf[B] : q->k1;
        }
        if (fb) { if (!g_diag) { g_diag = 3; g_diag_slot = p->dst; } return 0; }
        return out;
    }
    case 12: {
        int cnt = 0, all = 1, any = 0;
        for (int j = 0; j < p->n; j++) { int t = in[j] != 0; cnt += t; all &= t; any |= t; }
        switch (p->x) {
        case 0: return all; case 1: return any; case 2: return !all; case 3: return !any;
        case 4: return cnt & 1; case 5: return !(cnt & 1); default: return in[0] == 0;
        }
    }
    case 13: {
        int take = p->x == 0 ? in[1] != 0 : (p->x == 1 ? in[1] >= p->k1 : in[1] > p->k1);
        return take ? do_cast128(in[0], &p->cs[0]) : do_cast128(in[2], &p->cs[1]);
    }
    case 14: {
        i128 c = in[0];
        int cf = p->x, nd = p->a, idx = nd - 1;
        i128 k;
        int integral = 1;
        if (cf <= 0) k = c * ((i128)1 << (-cf));
        else { integral = (c & (((i128)1 << cf) - 1)) == 0; k = c >> cf; }
        {
            int strict = p->k1 < 0, hit = 0;
            i128 nc = strict ? -p->k1 : p->k1;
            if (integral) { i128 i = k - p->y; if (i >= 0 && i < nc) { idx = (int)i; hit = 1; } }
            if (strict && !hit && !g_diag) { g_diag = 1; g_diag_slot = p->dst; }
        }
        return do_cast128(in[1 + idx], &p->cs[idx]);
    }
    case 16: {   /* BITW: two's-complement bit operations at the input word length */
        i128 mm = ((i128)1 << p->b) - 1, acc = in[0] & mm;
        int code = p->x;
        for (int j = 1; j < p->n; j++) {
            i128 v = in[j] & mm;
            if (code == 0 || code == 2) acc &= v; else if (code == 1 || code == 3) acc |= v; else if (code == 4) acc ^= v;
        }
        if (p->a) {
            i128 v = p->k1 & mm;
            if (code == 0 || code == 2) acc &= v; else if (code == 1 || code == 3) acc |= v; else if (code == 4) acc ^= v;
        }
        if (code == 2 || code == 3 || code == 5) acc = ~acc & mm;
        if (p->y && ((acc >> (p->b - 1)) & 1)) acc -= (i128)1 << p->b;
        return acc;
    }
    case 17: {   /* EXTR: bit field [lo, lo+nb) of the two's-complement value (sign extension above the word) */
        i128 v = (in[0] >> p->x) & (((i128)1 << p->y) - 1);
        if (p->b && ((v >> (p->y - 1)) & 1)) v -= (i128)1 << p->y;
        return v;
    }
    case 19: {   /* 1-D linear point-slope (ref/fxcore.py _blk_lut1d): y = qi(y0 + qt(frac * (y1 - y0))), qt signed */
        const int *x = p->aux;   /* sa sb ff fs fw rm sat its itw itf ulast */
        int last = p->n - 1;
        i128 u = in[0] * ((i128)1 << x[0]);
        i128 b0 = p->kv[0] * ((i128)1 << x[1]), bl = p->kv[2 * last] * ((i128)1 << x[1]);
        if (u < b0) u = b0;
        if (u > bl) u = bl;
        i128 y;
        if ((x[10] & 1) && u >= bl) y = p->kv[2 * last + 1];
        else {
            int k = 0;
            while (k < last - 1 && u >= p->kv[2 * (k + 1)] * ((i128)1 << x[1])) k++;
            i128 bk = p->kv[2 * k] * ((i128)1 << x[1]), bk1 = p->kv[2 * (k + 1)] * ((i128)1 << x[1]);
            i128 fr = rdiv128((u - bk) * ((i128)1 << x[2]), bk1 - bk, x[5]);
            /* ulast bit 3 = a fraction rounding to 1 is not recorded for this block
               (UseLastTableValue on): the run stops there, naming the block and step (E UNRECORDED) */
            if ((x[10] & 8) && fr >= ((i128)1 << x[2]) && !g_diag) { g_diag = 2; g_diag_slot = p->dst; }
            /* ulast bit 2 = a fraction that rounds to 1 is 1 (inherited fraction type), not saturated */
            i128 fq = ((x[10] & 4) && fr >= ((i128)1 << x[2])) ? ((i128)1 << x[2]) : ovf128(fr, &p->cs[0]);
            i128 t0 = p->kv[2 * k + 1], d = p->kv[2 * p->n + k];   /* slope at the table's precision */
            i128 pr;
            /* ulast bit 1 = the term of a falling segment is formed from the magnitude,
               y0 - round(f * (y0 - y1)) (host, 32-bit table type) */
            if ((x[10] & 2) && d < 0) pr = -ovf128(rdiv128(fq * (-d), (i128)1 << (x[2] + x[11]), x[5]), &p->cs[2]);
            else pr = ovf128(rdiv128(fq * d, (i128)1 << (x[2] + x[11]), x[5]), &p->cs[2]);
            y = ovf128(t0 + pr, &p->cs[1]);
        }
        return do_cast128(y, &p->c);
    }
    case 20: {   /* 2-D table (ref/fxcore.py _blk_lut2d): index per dimension; linear interpolates along dimension 1
                    at the two neighbouring columns, then along dimension 2, each step y0 + qi(f * (y1 - y0)) in the
                    intermediate type */
        const int *x = p->aux;   /* mode n1 n2 sa1 sb1 sa2 sb2 ff fs fw rm sat its itw ulast */
        int nn[2] = {x[1], x[2]}, k[2], nx[2];
        const i128 *bp[2] = {p->kv, p->kv + x[1]}, *tab = p->kv + x[1] + x[2];
        i128 f[2];
        for (int d = 0; d < 2; d++) {
            int last = nn[d] - 1, sa = x[3 + 2 * d], sb = x[4 + 2 * d];
            i128 u = in[d] * ((i128)1 << sa);
            i128 b0 = bp[d][0] * ((i128)1 << sb), bl = bp[d][last] * ((i128)1 << sb);
            if (u < b0) u = b0;
            if (u > bl) u = bl;
            int kk = 0;
            while (kk < last - 1 && u >= bp[d][kk + 1] * ((i128)1 << sb)) kk++;
            i128 bk = bp[d][kk] * ((i128)1 << sb), bk1 = bp[d][kk + 1] * ((i128)1 << sb);
            if (x[0] < 2) {
                int up = x[0] == 0 ? u >= bk1 : (u - bk) >= (bk1 - u);
                k[d] = kk + up;
                continue;
            }
            if (x[14] && u >= bl) { kk = last; f[d] = 0; }
            else f[d] = ovf128(rdiv128((u - bk) * ((i128)1 << x[7]), bk1 - bk, x[10]), &p->cs[0]);
            k[d] = kk;
            nx[d] = kk + 1 < last ? kk + 1 : last;
        }
        if (x[0] < 2) return tab[(size_t)k[0] * nn[1] + k[1]];
        i128 one = (i128)1 << x[7], y[2];
        const i128 *tabT = tab + (size_t)nn[0] * nn[1];   /* table values at the table's precision */
        for (int c = 0; c < 2; c++) {
            int col = c ? nx[1] : k[1];
            i128 y0 = tab[(size_t)k[0] * nn[1] + col];
            i128 d = tabT[(size_t)nx[0] * nn[1] + col] - tabT[(size_t)k[0] * nn[1] + col];
            i128 pr = ovf128(rdiv128(f[0] * d, (i128)1 << (x[7] + x[15]), x[10]), &p->cs[1]);
            y[c] = ovf128(y0 + pr, &p->cs[1]);
        }
        i128 pr = ovf128(rdiv128(f[1] * (y[1] - y[0]), one, x[10]), &p->cs[1]);
        return do_cast128(ovf128(y[0] + pr, &p->cs[1]), &p->c);
    }
    case 31: {   /* PLK: see parse_ins */
        int last = p->n - 1;
        i128 u = in[0] * ((i128)1 << p->y);
        if (u < p->kv[0]) u = p->kv[0];
        if (u > p->kv[last]) u = p->kv[last];
        int k = 0;
        while (k < last - 1 && u >= p->kv[k + 1]) k++;
        if (p->x == 0) return k;
        return ((u - p->kv[k]) * ((i128)1 << p->b)) / (p->kv[k + 1] - p->kv[k]);
    }
    case 30: {   /* IPL: see parse_ins */
        const int *x = p->aux;   /* dims mode n1 n2 ff1 ff2 its itw itf os ow of mag */
        int dims = x[0], mode = x[1], nn[2] = {x[2], x[3]}, k[2] = {0, 0}, k1[2] = {0, 0};
        i128 fn[2] = {0, 0};
        for (int d = 0; d < dims; d++) {
            i128 kk = in[2 * d];
            if (kk < 0) kk = 0;
            if (kk > nn[d] - 2) kk = nn[d] - 2;
            k[d] = (int)kk; fn[d] = in[2 * d + 1];
            k1[d] = k[d] + 1 < nn[d] - 1 ? k[d] + 1 : nn[d] - 1;
        }
        if (mode < 2) {
            int id[2] = {0, 0};
            for (int d = 0; d < dims; d++) {
                i128 one = (i128)1 << x[4 + d];
                int up = fn[d] >= one || (mode == 1 && 2 * fn[d] >= one);
                id[d] = k[d] + up < nn[d] - 1 ? k[d] + up : nn[d] - 1;
            }
            return p->kv[(size_t)id[1] * nn[0] + id[0]];
        }
        int its = x[6], itw = x[7];
        i128 m = itw >= 127 ? -1 : (((i128)1 << itw) - 1);
        (void)m;
        i128 col[2];
        int ncol = dims == 2 ? 2 : 1;
        for (int c = 0; c < ncol; c++) {
            int cc = dims == 2 ? (c ? k1[1] : k[1]) : 0;
            i128 y0 = p->kv[(size_t)cc * nn[0] + k[0]], y1 = p->kv[(size_t)cc * nn[0] + k1[0]], t;
            if (x[12] && y1 < y0) { t = ipl_wrap((fn[0] * (y0 - y1)) >> x[4], its, itw); col[c] = ipl_wrap(y0 - t, its, itw); }
            else { t = ipl_wrap((fn[0] * (y1 - y0)) >> x[4], its, itw); col[c] = ipl_wrap(y0 + t, its, itw); }
        }
        i128 y = col[0];
        if (dims == 2) {
            i128 y0 = col[0], y1 = col[1], t;
            if (x[12] && y1 < y0) { t = ipl_wrap((fn[1] * (y0 - y1)) >> x[5], its, itw); y = ipl_wrap(y0 - t, its, itw); }
            else { t = ipl_wrap((fn[1] * (y1 - y0)) >> x[5], its, itw); y = ipl_wrap(y0 + t, its, itw); }
        }
        int sh = x[11] - x[8];
        y = sh >= 0 ? y * ((i128)1 << sh) : y >> (-sh);
        return ipl_wrap(y, x[9], x[10]);
    }
    case 18: {   /* LUT, Flat (x=0) or Nearest (x=1), Clip: kv = (breakpoint, output value) pairs */
        int last = p->n - 1;
        i128 u = in[0] * ((i128)1 << p->y), b0 = p->kv[0] * ((i128)1 << p->b), bl = p->kv[2 * last] * ((i128)1 << p->b);
        if (u < b0) u = b0;
        if (u > bl) u = bl;
        int k = 0;
        while (k < last - 1 && u >= p->kv[2 * (k + 1)] * ((i128)1 << p->b)) k++;
        i128 bk = p->kv[2 * k] * ((i128)1 << p->b), bk1 = p->kv[2 * (k + 1)] * ((i128)1 << p->b);
        int up = p->x == 0 ? u >= bk1 : (u - bk) >= (bk1 - u);
        return p->kv[2 * (k + up) + 1];
    }
    default: {   /* 15 PRODN */
        const Cast128 *oc = &p->c;
        int fo = (int)p->k1, rmm = p->a, rmd = p->b;
        i128 lo = oc->lo, hi = oc->hi;
        if (p->x) {   /* inputs '/' then '*': u2 / u1 directly */
            if (in[0] == 0) { diag_div0(); return in[1] < 0 ? lo : hi; }
            return ovf128(scaled_div(in[1], p->aux[0] + fo - p->aux[1], in[0], rmd), oc);
        }
        i128 acc;
        int afl;
        if (!p->sg[0]) {
            acc = in[0]; afl = p->aux[0];
            if (p->n == 1) return ovf128(rdiv128(acc, afl - fo > 0 ? (i128)1 << (afl - fo) : 1, rmm) *
                                         (afl - fo < 0 ? ((i128)1 << (fo - afl)) : 1), oc);
        } else {
            /* recordings: a zero divisor gives the output type's extreme (max, or min for a negative dividend)
               as that step's value, and the chain goes on with it (the following steps wrap or saturate as usual) */
            if (in[0] == 0) diag_div0();
            acc = in[0] == 0 ? hi : ovf128(scaled_div(1, p->aux[0] + fo, in[0], rmd), oc);
            afl = fo;
        }
        for (int j = 1; j < p->n; j++) {
            if (!p->sg[j]) {
                i128 v = acc * in[j];
                int sh = afl + p->aux[j] - fo;
                acc = ovf128(sh > 0 ? rdiv128(v, (i128)1 << sh, rmm) : v * ((i128)1 << (-sh)), oc);
            } else {
                if (in[j] == 0) { diag_div0(); acc = acc < 0 ? lo : hi; }   /* see above: the chain continues */
                else acc = ovf128(scaled_div(acc, p->aux[j] + fo - afl, in[j], rmd), oc);
            }
            afl = fo;
        }
        return acc;
    }
    }
}

static i128 op_new_slots(const Ins *p, const i128 *val) {
    i128 in[1025];
    int srcs[1025];
    int m = ins_srcs(p, srcs);
    for (int j = 0; j < m; j++) in[j] = val[srcs[j]];
    return op_new(p, in);
}

static void exec_int(const Ins *ins, long ni, i128 *val) {
    for (long i = 0; i < ni; i++) {
        const Ins *p = &ins[i];
        i128 v;
        g_dcur = p->dst;
        switch (p->op) {
        case 1: v = do_cast128(val[p->a] * p->k1, &p->c); break;
        case 2: v = do_cast128(val[p->a] * val[p->b], &p->c); break;
        case 3: v = do_cast128(val[p->a], &p->c); break;
        case 4: v = val[p->a]; if (v < p->k1) v = p->k1; if (v > p->k2) v = p->k2; break;
        case 5: v = val[p->a]; v = p->x ? -v : (v < 0 ? -v : v); v = do_cast128(v, &p->c); break;
        case 6: {
            i128 a = val[p->a] * ((i128)1 << p->x), b = val[p->b] * ((i128)1 << p->y);
            switch (p->n) {
            case 0: v = a == b; break; case 1: v = a != b; break; case 2: v = a < b; break;
            case 3: v = a <= b; break; case 4: v = a > b; break; default: v = a >= b; break;
            }
            break;
        }
        case 7: {
            i128 acc = 0;
            if (p->x >= 0) acc = do_cast128(val[p->src[p->x]], &p->cs[p->x]);
            for (int j = 0; j < p->n; j++) {
                if (j == p->x) continue;
                i128 xv = do_cast128(val[p->src[j]], &p->cs[j]);
                acc = ovf128(p->sg[j] > 0 ? acc + xv : acc - xv, &p->c2);
            }
            v = do_cast128(acc, &p->c);
            break;
        }
        case 8: {
            i128 best = do_cast128(val[p->src[0]], &p->cs[0]);
            for (int j = 1; j < p->n; j++) {
                i128 xv = do_cast128(val[p->src[j]], &p->cs[j]);
                if (p->x ? xv > best : xv < best) best = xv;
            }
            v = do_cast128(best, &p->c);
            break;
        }
        case 9: v = val[p->a]; break;
        default: v = op_new_slots(p, val); break;
        }
        val[p->dst] = v;
    }
}

/* ---------------------------------------------------------------- streaming mode (-s), for search drivers
 * One command per line; inputs and the double reference persist across candidates.
 *   IN slot fl n v0..v(n-1)                     add an input (all inputs must have the same length)
 *   DREF ns ni <double instrs> nd (dslot src ic)*nd nr slot*nr
 *        double-precision simulation of the same graph (casts are identities); keeps the waveforms of
 *        the nr slots as reference; answers "Q slot min max" for every slot, then "D"
 *        double instrs: 1 GAIN d s g | 2 PROD d a b | 3 CAST d s | 4 SAT d s lo hi | 5 UN d s neg |
 *                       6 REL d a b op | 7 SUM d n (src sign)*n | 8 MINMAX d n ismax src*n | 9 COPY d s |
 *                       10 CONST d v | 12 LOGIC d op n src*n | 13 SWITCH d u1 u2 u3 mode thr |
 *                       14 MPSW d ctrl base ncontig ndata src*ndata | 15 PRODN d n special (src isdiv)*n
 *   C ns ni fast nw widx*nw <int instrs> nd (dslot src ic)*nd ne (slot fl)*ne
 *        fast=1: every stored value fits int64; the nw listed instructions need 128-bit intermediates
 *        bit-exact run; answers "R core_ns", then per evaluated slot
 *        "M slot max|y*2^-fl - ref| mean(y-ref) var(y-ref) argmax", then "D"
 *   REFV slot n v0..v(n-1)                      replace a recorded reference waveform (e.g. a given design's output)
 *   DOUT                                        print the recorded double waveforms: "W slot n v0..v(n-1)" (%.17g, exact
 *                                               round trip), then "D" (the error search's ideal reference)
 *   END
 */
typedef struct { int op, dst, a, b, n, x; double k1, k2; int *src, *sg; double *kv; } DIns;

static long s_nt = -1;
static int s_nin;
static int *s_inslot;
static i128 **s_inval;
static double **s_indbl;
static int s_nref, *s_refslot;
static double **s_ref;

/* v * 2^-f - r, exact enough when v has more than 53 significant bits (the error search compared a
 * 2^60 + 1 output as the double 2^60): the integer part of r * 2^f is subtracted in integers first */
static double edif(i128 v, int f, double r) {
    if (v < ((i128)1 << 53) && v > -((i128)1 << 53)) return ldexp((double)v, -f) - r;
    double rs = ldexp(r, f);
    if (!isfinite(rs) || fabs(rs) >= 1.0e37) return ldexp((double)v, -f) - r;
    double ri = floor(rs);
    return ldexp((double)(v - (i128)ri) - (rs - ri), -f);
}

static int rdd(double *o) { char *t, *e; if (!tok(&t)) return 0; *o = strtod(t, &e); return e != t; }

static int s_in(void) {
    long sl, fl, nn;
    if (!rdi(&sl) || !rdi(&fl) || !rdi(&nn) || sl < 0 || nn < 1) return fail("PROTOCOL", "IN header");
    if (s_nt >= 0 && nn != s_nt) return fail("PROTOCOL", "inputs differ in length");
    s_nt = nn;
    s_inslot = realloc(s_inslot, sizeof(int) * (size_t)(s_nin + 1));
    s_inval = realloc(s_inval, sizeof(i128 *) * (size_t)(s_nin + 1));
    s_indbl = realloc(s_indbl, sizeof(double *) * (size_t)(s_nin + 1));
    if (!s_inslot || !s_inval || !s_indbl) return fail("OOM", "alloc");
    s_inslot[s_nin] = (int)sl;
    s_inval[s_nin] = malloc(sizeof(i128) * (size_t)nn);
    s_indbl[s_nin] = malloc(sizeof(double) * (size_t)nn);
    if (!s_inval[s_nin] || !s_indbl[s_nin]) return fail("OOM", "alloc");
    for (long k = 0; k < nn; k++) {
        if (!rd(&s_inval[s_nin][k])) return fail("PROTOCOL", "truncated IN");
        s_indbl[s_nin][k] = ldexp((double)s_inval[s_nin][k], -(int)fl);
    }
    s_nin++;
    return 0;
}

static int s_dref(void) {
    long ns, ni, nd, nr;
    if (s_nt < 1) return fail("PROTOCOL", "DREF before IN");
    if (!rdi(&ns) || !rdi(&ni) || ns < 1 || ni < 0) return fail("PROTOCOL", "DREF header");
    for (int k = 0; k < s_nin; k++) if (s_inslot[k] >= ns) return fail("PROTOCOL", "IN slot out of range");
    DIns *ins = calloc((size_t)(ni ? ni : 1), sizeof(DIns));
    double *val = calloc((size_t)ns, sizeof(double)), *mn = malloc(sizeof(double) * (size_t)ns),
           *mx = malloc(sizeof(double) * (size_t)ns);
    if (!ins || !val || !mn || !mx) return fail("OOM", "alloc");
    for (long i = 0; i < ni; i++) {
        DIns *p = &ins[i];
        long op, d, a, b, m, x = 0;
        if (!rdi(&op) || !rdi(&d) || !slot(d, ns)) return fail("PROTOCOL", "DREF instruction");
        p->op = (int)op; p->dst = (int)d;
        switch (op) {
        case 1: if (!rdi(&a) || !slot(a, ns) || !rdd(&p->k1)) return fail("PROTOCOL", "DGAIN"); p->a = (int)a; break;
        case 2: if (!rdi(&a) || !rdi(&b) || !slot(a, ns) || !slot(b, ns)) return fail("PROTOCOL", "DPROD"); p->a = (int)a; p->b = (int)b; break;
        case 3: case 9: if (!rdi(&a) || !slot(a, ns)) return fail("PROTOCOL", "DCAST"); p->a = (int)a; break;
        case 4: if (!rdi(&a) || !slot(a, ns) || !rdd(&p->k1) || !rdd(&p->k2)) return fail("PROTOCOL", "DSAT"); p->a = (int)a; break;
        case 5: if (!rdi(&a) || !slot(a, ns) || !rdi(&x)) return fail("PROTOCOL", "DUN"); p->a = (int)a; p->x = (int)x; break;
        case 6: if (!rdi(&a) || !rdi(&b) || !rdi(&m) || !slot(a, ns) || !slot(b, ns)) return fail("PROTOCOL", "DREL");
            p->a = (int)a; p->b = (int)b; p->n = (int)m; break;
        case 7: case 8:
            if (!rdi(&m) || m < 1 || m > 1024) return fail("PROTOCOL", "DSUM arity");
            p->n = (int)m;
            if (op == 8 && !rdi(&x)) return fail("PROTOCOL", "DMINMAX");
            p->x = (int)x;
            p->src = malloc(sizeof(int) * (size_t)m); p->sg = malloc(sizeof(int) * (size_t)m);
            if (!p->src || !p->sg) return fail("OOM", "alloc");
            for (long j = 0; j < m; j++) {
                long sv, sg = 1;
                if (!rdi(&sv) || !slot(sv, ns)) return fail("PROTOCOL", "DSUM input");
                if (op == 7 && !rdi(&sg)) return fail("PROTOCOL", "DSUM sign");
                p->src[j] = (int)sv; p->sg[j] = (int)sg;
            }
            break;
        case 10: if (!rdd(&p->k1)) return fail("PROTOCOL", "DCONST"); break;
        case 12: case 13: case 14: case 15: {
            /* 12 LOGIC d opcode n src*n | 13 SWITCH d u1 u2 u3 mode thr | 14 MPSW d ctrl base ncontig ndata src*ndata
               15 PRODN d n special (src isdiv)*n */
            long v1 = 0, v2 = 0, sp = 0;
            if (op == 12) { if (!rdi(&x) || !rdi(&m) || m < 1 || m > 1024) return fail("PROTOCOL", "DLOGIC"); }
            else if (op == 13) m = 3;
            else if (op == 14) {
                if (!rdi(&a) || !slot(a, ns) || !rdi(&v1) || !rdi(&v2) || !rdi(&m) || m < 1 || m > 1023)
                    return fail("PROTOCOL", "DMPSW");
                p->a = (int)v1; p->b = (int)v2; m += 1;
            } else { if (!rdi(&m) || !rdi(&sp) || m < 1 || m > 1024) return fail("PROTOCOL", "DPRODN"); x = sp; }
            p->x = (int)x; p->n = (int)m;
            p->src = calloc((size_t)m, sizeof(int)); p->sg = calloc((size_t)m, sizeof(int));
            if (!p->src || !p->sg) return fail("OOM", "alloc");
            if (op == 14) p->src[0] = (int)a;
            for (long jj = (op == 14 ? 1 : 0); jj < m; jj++) {
                long sv, dv = 0;
                if (!rdi(&sv) || !slot(sv, ns)) return fail("PROTOCOL", "double operand");
                if (op == 15 && !rdi(&dv)) return fail("PROTOCOL", "DPRODN operand");
                p->src[jj] = (int)sv; p->sg[jj] = (int)dv;
            }
            if (op == 13) { if (!rdi(&x) || !rdd(&p->k1)) return fail("PROTOCOL", "DSWITCH"); p->x = (int)x; }
            break;
        }
        case 18: {   /* LUT d src mode n (bp tab)*n */
            if (!rdi(&a) || !slot(a, ns) || !rdi(&x) || !rdi(&m) || m < 2 || m > 1000000) return fail("PROTOCOL", "DLUT");
            p->a = (int)a; p->x = (int)x; p->n = (int)m;
            p->kv = malloc(sizeof(double) * 2 * (size_t)m);
            if (!p->kv) return fail("OOM", "alloc");
            for (long j = 0; j < 2 * m; j++) if (!rdd(&p->kv[j])) return fail("PROTOCOL", "DLUT table");
            break;
        }
        case 20: {   /* LUT2 d src1 src2 mode n1 n2 bp1*n1 bp2*n2 tab*(n1*n2) */
            long b2, n1, n2;
            if (!rdi(&a) || !slot(a, ns) || !rdi(&b2) || !slot(b2, ns) || !rdi(&x) || !rdi(&n1) || !rdi(&n2)
                || n1 < 2 || n2 < 2 || n1 > 100000 || n2 > 100000 || n1 * n2 > 4000000) return fail("PROTOCOL", "DLUT2");
            p->a = (int)a; p->b = (int)b2; p->x = (int)x; p->n = (int)n1;
            p->sg = malloc(sizeof(int));
            if (!p->sg) return fail("OOM", "alloc");
            p->sg[0] = (int)n2;
            size_t nk = (size_t)(n1 + n2 + n1 * n2);
            p->kv = malloc(sizeof(double) * nk);
            if (!p->kv) return fail("OOM", "alloc");
            for (size_t j = 0; j < nk; j++) if (!rdd(&p->kv[j])) return fail("PROTOCOL", "DLUT2 table");
            break;
        }
        default: return fail("PROTOCOL", "unknown double op");
        }
    }
    if (!rdi(&nd) || nd < 0 || nd > ns) return fail("PROTOCOL", "DREF delays");
    int *ds = malloc(sizeof(int) * (size_t)(nd + 1)), *dsrc = malloc(sizeof(int) * (size_t)(nd + 1));
    double *tmp = malloc(sizeof(double) * (size_t)(nd + 1));
    if (!ds || !dsrc || !tmp) return fail("OOM", "alloc");
    for (long k = 0; k < nd; k++) {
        long a, b;
        double ic;
        if (!rdi(&a) || !rdi(&b) || !rdd(&ic) || !slot(a, ns) || !slot(b, ns)) return fail("PROTOCOL", "DREF delay");
        ds[k] = (int)a; dsrc[k] = (int)b; val[a] = ic;
    }
    if (!rdi(&nr) || nr < 0 || nr > ns) return fail("PROTOCOL", "DREF records");
    for (int k = 0; k < s_nref; k++) free(s_ref[k]);
    free(s_ref); free(s_refslot);
    s_nref = (int)nr;
    s_refslot = malloc(sizeof(int) * (size_t)(nr + 1));
    s_ref = malloc(sizeof(double *) * (size_t)(nr + 1));
    if (!s_refslot || !s_ref) return fail("OOM", "alloc");
    for (long k = 0; k < nr; k++) {
        long a;
        if (!rdi(&a) || !slot(a, ns)) return fail("PROTOCOL", "DREF record slot");
        s_refslot[k] = (int)a;
        s_ref[k] = malloc(sizeof(double) * (size_t)s_nt);
        if (!s_ref[k]) return fail("OOM", "alloc");
    }
    for (long i = 0; i < ns; i++) { mn[i] = 1e300; mx[i] = -1e300; }
    for (long step = 0; step < s_nt; step++) {
        for (int k = 0; k < s_nin; k++) val[s_inslot[k]] = s_indbl[k][step];
        for (long i = 0; i < ni; i++) {
            const DIns *p = &ins[i];
            double v;
            switch (p->op) {
            case 1: v = val[p->a] * p->k1; break;
            case 2: v = val[p->a] * val[p->b]; break;
            case 4: v = val[p->a]; if (v < p->k1) v = p->k1; if (v > p->k2) v = p->k2; break;
            case 5: v = p->x ? -val[p->a] : fabs(val[p->a]); break;
            case 6: {
                double a = val[p->a], b = val[p->b];
                switch (p->n) {
                case 0: v = a == b; break; case 1: v = a != b; break; case 2: v = a < b; break;
                case 3: v = a <= b; break; case 4: v = a > b; break; default: v = a >= b; break;
                }
                break;
            }
            case 7: v = 0.0; for (int j = 0; j < p->n; j++) v = p->sg[j] > 0 ? v + val[p->src[j]] : v - val[p->src[j]]; break;
            case 8: v = val[p->src[0]];
                for (int j = 1; j < p->n; j++) { double w = val[p->src[j]]; if (p->x ? w > v : w < v) v = w; }
                break;
            case 10: v = p->k1; break;
            case 12: {
                int cnt = 0, all = 1, any = 0;
                for (int q = 0; q < p->n; q++) { int t = val[p->src[q]] != 0; cnt += t; all &= t; any |= t; }
                switch (p->x) {
                case 0: v = all; break; case 1: v = any; break; case 2: v = !all; break; case 3: v = !any; break;
                case 4: v = cnt & 1; break; case 5: v = !(cnt & 1); break; default: v = val[p->src[0]] == 0; break;
                }
                break;
            }
            case 13: {
                double u2 = val[p->src[1]];
                int take = p->x == 0 ? u2 != 0 : (p->x == 1 ? u2 >= p->k1 : u2 > p->k1);
                v = take ? val[p->src[0]] : val[p->src[2]];
                break;
            }
            case 14: {
                double k = val[p->src[0]] - p->a;
                int nd_ = p->n - 1, idx = nd_ - 1;
                if (isfinite(k) && k == floor(k) && k >= 0 && k < p->b) idx = (int)k;
                v = val[p->src[1 + idx]];
                break;
            }
            case 15: {
                if (p->x) { v = val[p->src[1]] / val[p->src[0]]; break; }
                double acc = p->sg[0] ? 1.0 / val[p->src[0]] : val[p->src[0]];
                for (int q = 1; q < p->n; q++) acc = p->sg[q] ? acc / val[p->src[q]] : acc * val[p->src[q]];
                v = acc;
                break;
            }
            case 18: {
                int last = p->n - 1, k = 0;
                double u = val[p->a];
                if (u < p->kv[0]) u = p->kv[0];
                if (u > p->kv[2 * last]) u = p->kv[2 * last];
                while (k < last - 1 && u >= p->kv[2 * (k + 1)]) k++;
                double bk = p->kv[2 * k], bk1 = p->kv[2 * (k + 1)];
                if (p->x == 2) {   /* linear point-slope (ideal reference) */
                    double tk = p->kv[2 * k + 1], tk1 = p->kv[2 * (k + 1) + 1];
                    v = tk + ((u - bk) / (bk1 - bk)) * (tk1 - tk);
                    break;
                }
                int up = p->x == 0 ? u >= bk1 : (u - bk) >= (bk1 - u);
                v = p->kv[2 * (k + up) + 1];
                break;
            }
            case 20: {   /* ideal 2-D table: Flat / Nearest pick, linear = bilinear (dimension 1 first) */
                int nn[2] = {p->n, p->sg[0]}, k[2];
                const double *bp[2] = {p->kv, p->kv + p->n}, *tab = p->kv + p->n + p->sg[0];
                double f[2], uu[2] = {val[p->a], val[p->b]};
                for (int d = 0; d < 2; d++) {
                    int last = nn[d] - 1, kk = 0;
                    double u = uu[d];
                    if (u < bp[d][0]) u = bp[d][0];
                    if (u > bp[d][last]) u = bp[d][last];
                    while (kk < last - 1 && u >= bp[d][kk + 1]) kk++;
                    double bk = bp[d][kk], bk1 = bp[d][kk + 1];
                    if (p->x < 2) { k[d] = kk + (p->x == 0 ? u >= bk1 : (u - bk) >= (bk1 - u)); f[d] = 0; }
                    else { k[d] = kk; f[d] = (u - bk) / (bk1 - bk); }
                }
                if (p->x < 2) { v = tab[(size_t)k[0] * nn[1] + k[1]]; break; }
                double t00 = tab[(size_t)k[0] * nn[1] + k[1]], t10 = tab[(size_t)(k[0] + 1) * nn[1] + k[1]];
                double t01 = tab[(size_t)k[0] * nn[1] + k[1] + 1], t11 = tab[(size_t)(k[0] + 1) * nn[1] + k[1] + 1];
                double a0 = t00 + f[0] * (t10 - t00), a1 = t01 + f[0] * (t11 - t01);
                v = a0 + f[1] * (a1 - a0);
                break;
            }
            default: v = val[p->a]; break;
            }
            val[p->dst] = v;
        }
        for (long i = 0; i < ns; i++) {
            if (!isfinite(val[i])) { mn[i] = NAN; mx[i] = NAN; continue; }   /* NaN marks a non-finite slot */
            if (val[i] < mn[i]) mn[i] = val[i];
            if (val[i] > mx[i]) mx[i] = val[i];
        }
        for (int k = 0; k < s_nref; k++) s_ref[k][step] = val[s_refslot[k]];
        for (long k = 0; k < nd; k++) tmp[k] = val[dsrc[k]];
        for (long k = 0; k < nd; k++) val[ds[k]] = tmp[k];
    }
    for (long i = 0; i < ns; i++) printf("Q %ld %.17g %.17g\n", i, mn[i], mx[i]);
    printf("D\n");
    fflush(stdout);
    for (long i = 0; i < ni; i++) { free(ins[i].src); free(ins[i].sg); free(ins[i].kv); }
    free(ins); free(val); free(mn); free(mx); free(ds); free(dsrc); free(tmp);
    return 0;
}

/* Array mode for graphs without any cycle (delays included): every instruction runs over a chunk of
 * samples in a topological order, Unit Delays become one-sample shifts with a carried value. Same
 * per-element arithmetic and the same error-statistics order as the step loop, so results are identical.
 * Returns 1 if it ran, 0 if the graph has a cycle (caller uses the step loop), -1 on allocation failure. */
#define ACHUNK 256
static int ins_srcs(const Ins *p, int *out) {
    switch (p->op) {
    case 2: case 6: case 20: case 21: out[0] = p->a; out[1] = p->b; return 2;
    case 22: out[0] = p->a; out[1] = p->b; out[2] = p->sg[0]; return 3;
    case 23: out[0] = p->a; out[1] = p->b; out[2] = p->aux[0]; out[3] = p->aux[1]; return 4;
    case 7: case 8: case 12: case 13: case 14: case 15: case 16: case 30: for (int j = 0; j < p->n; j++) out[j] = p->src[j]; return p->n;
    case 10: return 0;
    default: out[0] = p->a; return 1;
    }
}

/* 64-bit storage, 128-bit arithmetic: instructions whose intermediates exceed 62 bits on the fast path */
static void exec_wide64(const Ins *p, int64_t *mem, i128 *acc, int m) {
    int64_t *y = mem + (size_t)p->dst * ACHUNK;
    const int64_t *a = mem + (size_t)p->a * ACHUNK, *b = mem + (size_t)p->b * ACHUNK;
    switch (p->op) {
    case 1: for (int j = 0; j < m; j++) y[j] = (int64_t)do_cast128((i128)a[j] * p->k1, &p->c); break;
    case 2: for (int j = 0; j < m; j++) y[j] = (int64_t)do_cast128((i128)a[j] * (i128)b[j], &p->c); break;
    case 3: for (int j = 0; j < m; j++) y[j] = (int64_t)do_cast128((i128)a[j], &p->c); break;
    case 4: for (int j = 0; j < m; j++) { i128 v = a[j]; if (v < p->k1) v = p->k1; if (v > p->k2) v = p->k2; y[j] = (int64_t)v; } break;
    case 5: for (int j = 0; j < m; j++) { i128 v = a[j]; v = p->x ? -v : (v < 0 ? -v : v); y[j] = (int64_t)do_cast128(v, &p->c); } break;
    case 6: {
        i128 sa = (i128)1 << p->x, sb = (i128)1 << p->y;
        for (int j = 0; j < m; j++) {
            i128 u = (i128)a[j] * sa, w = (i128)b[j] * sb, v;
            switch (p->n) {
            case 0: v = u == w; break; case 1: v = u != w; break; case 2: v = u < w; break;
            case 3: v = u <= w; break; case 4: v = u > w; break; default: v = u >= w; break;
            }
            y[j] = (int64_t)v;
        }
        break;
    }
    case 7: {
        if (p->x >= 0) { const int64_t *xs = mem + (size_t)p->src[p->x] * ACHUNK;
            for (int j = 0; j < m; j++) acc[j] = do_cast128((i128)xs[j], &p->cs[p->x]); }
        else for (int j = 0; j < m; j++) acc[j] = 0;
        for (int q = 0; q < p->n; q++) {
            if (q == p->x) continue;
            const int64_t *xs = mem + (size_t)p->src[q] * ACHUNK;
            for (int j = 0; j < m; j++) {
                i128 xv = do_cast128((i128)xs[j], &p->cs[q]);
                acc[j] = ovf128(p->sg[q] > 0 ? acc[j] + xv : acc[j] - xv, &p->c2);
            }
        }
        for (int j = 0; j < m; j++) y[j] = (int64_t)do_cast128(acc[j], &p->c);
        break;
    }
    case 8: {
        const int64_t *x0 = mem + (size_t)p->src[0] * ACHUNK;
        for (int j = 0; j < m; j++) acc[j] = do_cast128((i128)x0[j], &p->cs[0]);
        for (int q = 1; q < p->n; q++) {
            const int64_t *xs = mem + (size_t)p->src[q] * ACHUNK;
            for (int j = 0; j < m; j++) { i128 v = do_cast128((i128)xs[j], &p->cs[q]); if (p->x ? v > acc[j] : v < acc[j]) acc[j] = v; }
        }
        for (int j = 0; j < m; j++) y[j] = (int64_t)do_cast128(acc[j], &p->c);
        break;
    }
    default: for (int j = 0; j < m; j++) y[j] = a[j]; break;
    }
}

#define TY i128
#define SUF 128
#define CAST_C(v, p) do_cast128((v), &(p)->c)
#define CAST_I(v, p, q) do_cast128((v), &(p)->cs[q])
#define OVF_A(v, p) ovf128((v), &(p)->c2)
#define FW_OK 0
#include "graph_arr_tmpl.h"
#undef TY
#undef SUF
#undef CAST_C
#undef CAST_I
#undef OVF_A
#undef FW_OK
#define TY int64_t
#define SUF 64
#define CAST_C(v, p) fc_cast((v), &(p)->f)
#define CAST_I(v, p, q) fc_cast((v), &(p)->fs[q])
#define OVF_A(v, p) fc_cast((v), &(p)->f2)
#define FW_OK 1
#include "graph_arr_tmpl.h"
#undef TY
#undef SUF
#undef CAST_C
#undef CAST_I
#undef OVF_A
#undef FW_OK

static int run_array(const Ins *ins, long ni, long ns, int fast, const int *ds, const int *dsrc, const i128 *ic, long nd,
                     const int *es, const int *ef, const int *er, long ne,
                     double *emax, double *esum, double *esq, long *earg, const double *tol, double *per) {
    int *prod = malloc(sizeof(int) * (size_t)ns), *indeg = calloc((size_t)ns, sizeof(int));
    int *order = malloc(sizeof(int) * (size_t)(ni + nd + 1)), *queue = malloc(sizeof(int) * (size_t)ns);
    int *srcs = malloc(sizeof(int) * 1025);
    if (!prod || !indeg || !order || !queue || !srcs) return -1;
    for (long i = 0; i < ns; i++) prod[i] = -1;          /* -1: input or unproduced; i: instr; ni+k: delay k */
    for (long i = 0; i < ni; i++) prod[ins[i].dst] = (int)i;
    for (long k = 0; k < nd; k++) prod[ds[k]] = (int)(ni + k);
    /* users lists via counting */
    int *ucount = calloc((size_t)ns, sizeof(int));
    if (!ucount) return -1;
    for (long i = 0; i < ni; i++) { int m = ins_srcs(&ins[i], srcs); for (int j = 0; j < m; j++) ucount[srcs[j]]++; indeg[ins[i].dst] += m; }
    for (long k = 0; k < nd; k++) { ucount[dsrc[k]]++; indeg[ds[k]] += 1; }
    int *ustart = malloc(sizeof(int) * (size_t)(ns + 1)), *ulist, *ufill = calloc((size_t)ns, sizeof(int));
    if (!ustart || !ufill) return -1;
    ustart[0] = 0;
    for (long i = 0; i < ns; i++) ustart[i + 1] = ustart[i] + ucount[i];
    ulist = malloc(sizeof(int) * (size_t)(ustart[ns] + 1));
    if (!ulist) return -1;
    for (long i = 0; i < ni; i++) { int m = ins_srcs(&ins[i], srcs); for (int j = 0; j < m; j++) ulist[ustart[srcs[j]] + ufill[srcs[j]]++] = ins[i].dst; }
    for (long k = 0; k < nd; k++) ulist[ustart[dsrc[k]] + ufill[dsrc[k]]++] = ds[k];
    long qh = 0, qt = 0, no = 0;
    for (long i = 0; i < ns; i++) if (indeg[i] == 0) queue[qt++] = (int)i;
    while (qh < qt) {
        int sl = queue[qh++];
        if (prod[sl] >= 0) order[no++] = prod[sl];
        for (int u = ustart[sl]; u < ustart[sl + 1]; u++) if (--indeg[ulist[u]] == 0) queue[qt++] = ulist[u];
    }
    int acyclic = qt == ns;
    free(indeg); free(queue); free(ucount); free(ustart); free(ulist); free(ufill);
    if (!acyclic) { free(prod); free(order); free(srcs); return 0; }
    if (fast) {
        int64_t *mem = malloc(sizeof(int64_t) * (size_t)ns * ACHUNK), *carry = malloc(sizeof(int64_t) * (size_t)(nd + 1));
        int64_t *acc = malloc(sizeof(int64_t) * ACHUNK);
        i128 *wacc = malloc(sizeof(i128) * ACHUNK);
        if (!mem || !carry || !acc || !wacc) return -1;
        for (long k = 0; k < nd; k++) carry[k] = (int64_t)ic[ds[k]];
        for (long c0 = 0; c0 < s_nt; c0 += ACHUNK) {
            int m = (int)(s_nt - c0 < ACHUNK ? s_nt - c0 : ACHUNK);
            for (int k = 0; k < s_nin; k++) {
                int64_t *y = mem + (size_t)s_inslot[k] * ACHUNK;
                for (int j = 0; j < m; j++) y[j] = (int64_t)s_inval[k][c0 + j];
            }
            arr_exec64(ins, ni, order, no, ds, dsrc, mem, carry, acc, m, wacc);
            for (long k = 0; k < ne; k++) {
                const int64_t *y = mem + (size_t)es[k] * ACHUNK;
                const double *r = s_ref[er[k]] + c0;
                for (int j = 0; j < m; j++) {
                    double d = edif((i128)y[j], ef[k], r[j]);
                    esum[k] += d; esq[k] += d * d;
                    if (fabs(d) > emax[k]) { emax[k] = fabs(d); earg[k] = c0 + j; }
                    if (per) {   /* a non-finite reference is an infinite error (NaN was skipped) */
                        double e = isfinite(r[j]) ? fabs(d) / tol[k] : INFINITY;
                        if (e > per[c0 + j]) per[c0 + j] = e;
                    }
                }
            }
        }
        free(mem); free(carry); free(acc); free(wacc);
    } else {
        i128 *mem = malloc(sizeof(i128) * (size_t)ns * ACHUNK), *carry = malloc(sizeof(i128) * (size_t)(nd + 1));
        i128 *acc = malloc(sizeof(i128) * ACHUNK);
        if (!mem || !carry || !acc) return -1;
        for (long k = 0; k < nd; k++) carry[k] = ic[ds[k]];
        for (long c0 = 0; c0 < s_nt; c0 += ACHUNK) {
            int m = (int)(s_nt - c0 < ACHUNK ? s_nt - c0 : ACHUNK);
            for (int k = 0; k < s_nin; k++) {
                i128 *y = mem + (size_t)s_inslot[k] * ACHUNK;
                for (int j = 0; j < m; j++) y[j] = s_inval[k][c0 + j];
            }
            arr_exec128(ins, ni, order, no, ds, dsrc, mem, carry, acc, m, NULL);
            for (long k = 0; k < ne; k++) {
                const i128 *y = mem + (size_t)es[k] * ACHUNK;
                const double *r = s_ref[er[k]] + c0;
                for (int j = 0; j < m; j++) {
                    double d = edif((i128)y[j], ef[k], r[j]);
                    esum[k] += d; esq[k] += d * d;
                    if (fabs(d) > emax[k]) { emax[k] = fabs(d); earg[k] = c0 + j; }
                    if (per) {   /* a non-finite reference is an infinite error (NaN was skipped) */
                        double e = isfinite(r[j]) ? fabs(d) / tol[k] : INFINITY;
                        if (e > per[c0 + j]) per[c0 + j] = e;
                    }
                }
            }
        }
        free(mem); free(carry); free(acc);
    }
    free(prod); free(order); free(srcs);
    return 1;
}

static int s_cand(void) {
    long ns, ni, nd, ne;
    if (s_nt < 1 || !s_nref) return fail("PROTOCOL", "C before IN/DREF");
    long fastl;
    if (!rdi(&ns) || !rdi(&ni) || !rdi(&fastl) || ns < 1 || ni < 0) return fail("PROTOCOL", "C header");
    int fast = fastl != 0;
    long nw;
    if (!rdi(&nw) || nw < 0 || nw > ni) return fail("PROTOCOL", "C wide count");
    long *widx = malloc(sizeof(long) * (size_t)(nw + 1));
    if (!widx) return fail("OOM", "alloc");
    for (long k = 0; k < nw; k++) if (!rdi(&widx[k]) || widx[k] < 0 || widx[k] >= ni) return fail("PROTOCOL", "C wide index");
    Ins *ins = calloc((size_t)(ni ? ni : 1), sizeof(Ins));
    i128 *val = calloc((size_t)ns, sizeof(i128));
    if (!ins || !val) return fail("OOM", "alloc");
    if (parse_ins(ins, ni, ns)) return 1;
    for (long k = 0; k < nw; k++) ins[widx[k]].wide = 1;
    free(widx);
    if (!rdi(&nd) || nd < 0 || nd > ns) return fail("PROTOCOL", "C delays");
    int *ds = malloc(sizeof(int) * (size_t)(nd + 1)), *dsrc = malloc(sizeof(int) * (size_t)(nd + 1));
    i128 *tmp = malloc(sizeof(i128) * (size_t)(nd + 1));
    if (!ds || !dsrc || !tmp) return fail("OOM", "alloc");
    for (long k = 0; k < nd; k++) {
        long a, b;
        if (!rdi(&a) || !rdi(&b) || !slot(a, ns) || !slot(b, ns) || !rd(&val[a])) return fail("PROTOCOL", "C delay");
        ds[k] = (int)a; dsrc[k] = (int)b;
    }
    if (!rdi(&ne) || ne < 1 || ne > s_nref) return fail("PROTOCOL", "C eval count");
    int *es = malloc(sizeof(int) * (size_t)ne), *ef = malloc(sizeof(int) * (size_t)ne), *er = malloc(sizeof(int) * (size_t)ne);
    double *emax = calloc((size_t)ne, sizeof(double)), *esum = calloc((size_t)ne, sizeof(double)),
           *esq = calloc((size_t)ne, sizeof(double));
    long *earg = calloc((size_t)ne, sizeof(long));
    if (!es || !ef || !er || !emax || !esum || !esq || !earg) return fail("OOM", "alloc");
    for (long k = 0; k < ne; k++) {
        long a, f;
        if (!rdi(&a) || !rdi(&f) || !slot(a, ns)) return fail("PROTOCOL", "C eval slot");
        es[k] = (int)a; ef[k] = (int)f; er[k] = -1;
        for (int j = 0; j < s_nref; j++) if (s_refslot[j] == a) er[k] = j;
        if (er[k] < 0) return fail("PROTOCOL", "evaluated slot has no reference");
    }
    for (int k = 0; k < s_nin; k++) if (s_inslot[k] >= ns) return fail("PROTOCOL", "IN slot out of range");
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    int mode_array = 0;
    g_diag = 0;
    if (!getenv("FXGRAPH_NO_ARRAY")) {
        mode_array = run_array(ins, ni, ns, fast, ds, dsrc, val, nd, es, ef, er, ne, emax, esum, esq, earg, NULL, NULL);
        if (mode_array < 0) return fail("OOM", "alloc");
    }
    if (!mode_array)
    for (long step = 0; step < s_nt; step++) {
        for (int k = 0; k < s_nin; k++) val[s_inslot[k]] = s_inval[k][step];
        exec_int(ins, ni, val);
        for (long k = 0; k < ne; k++) {
            double d = edif(val[es[k]], ef[k], s_ref[er[k]][step]);
            esum[k] += d; esq[k] += d * d;
            if (fabs(d) > emax[k]) { emax[k] = fabs(d); earg[k] = step; }
        }
        for (long k = 0; k < nd; k++) tmp[k] = val[dsrc[k]];
        for (long k = 0; k < nd; k++) val[ds[k]] = tmp[k];
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    printf("R %ld\n", (long)(t1.tv_sec - t0.tv_sec) * 1000000000L + (t1.tv_nsec - t0.tv_nsec));
    if (g_diag) for (long k = 0; k < ne; k++) emax[k] = INFINITY;   /* Simulink stops on this input: not a candidate */
    for (long k = 0; k < ne; k++) {
        double mean = esum[k] / (double)s_nt;
        printf("M %d %.17g %.17g %.17g %ld\n", es[k], emax[k], mean, esq[k] / (double)s_nt - mean * mean, earg[k]);
    }
    printf("D\n");
    fflush(stdout);
    for (long i = 0; i < ni; i++) { free(ins[i].src); free(ins[i].sg); free(ins[i].aux); free(ins[i].cs); free(ins[i].fs); }
    free(ins); free(val); free(ds); free(dsrc); free(tmp); free(es); free(ef); free(er);
    free(emax); free(esum); free(esq); free(earg);
    return 0;
}

/* CP: as C (array mode when it applies), followed by one tolerance per evaluated slot; prints "P n r0 .. r(n-1)" with r = the largest
 * |fixed - ref| / tolerance over the evaluated slots at each step (the error search's per-step measure,
 * engine/fxerrsearch.py; computed as its Python fallback does: ldexp of the stored integer minus the reference) */
static int s_candp(void) {
    long ns, ni, nd, ne, fastl, nw;
    if (s_nt < 1 || !s_nref) return fail("PROTOCOL", "CP before IN/DREF");
    if (!rdi(&ns) || !rdi(&ni) || !rdi(&fastl) || ns < 1 || ni < 0) return fail("PROTOCOL", "CP header");
    if (!rdi(&nw) || nw < 0 || nw > ni) return fail("PROTOCOL", "CP wide count");
    long *widx = malloc(sizeof(long) * (size_t)(nw + 1));
    if (!widx) return fail("OOM", "alloc");
    for (long k = 0; k < nw; k++) if (!rdi(&widx[k]) || widx[k] < 0 || widx[k] >= ni) return fail("PROTOCOL", "CP wide index");
    Ins *ins = calloc((size_t)(ni ? ni : 1), sizeof(Ins));
    i128 *val = calloc((size_t)ns, sizeof(i128));
    if (!ins || !val) return fail("OOM", "alloc");
    if (parse_ins(ins, ni, ns)) return 1;
    for (long k = 0; k < nw; k++) ins[widx[k]].wide = 1;
    free(widx);
    if (!rdi(&nd) || nd < 0 || nd > ns) return fail("PROTOCOL", "CP delays");
    int *ds = malloc(sizeof(int) * (size_t)(nd + 1)), *dsrc = malloc(sizeof(int) * (size_t)(nd + 1));
    i128 *tmp = malloc(sizeof(i128) * (size_t)(nd + 1));
    if (!ds || !dsrc || !tmp) return fail("OOM", "alloc");
    for (long k = 0; k < nd; k++) {
        long a, b;
        if (!rdi(&a) || !rdi(&b) || !slot(a, ns) || !slot(b, ns) || !rd(&val[a])) return fail("PROTOCOL", "CP delay");
        ds[k] = (int)a; dsrc[k] = (int)b;
    }
    if (!rdi(&ne) || ne < 1 || ne > s_nref) return fail("PROTOCOL", "CP eval count");
    int *es = malloc(sizeof(int) * (size_t)ne), *ef = malloc(sizeof(int) * (size_t)ne), *er = malloc(sizeof(int) * (size_t)ne);
    double *tol = malloc(sizeof(double) * (size_t)ne), *per = malloc(sizeof(double) * (size_t)s_nt);
    if (!es || !ef || !er || !tol || !per) return fail("OOM", "alloc");
    for (long k = 0; k < ne; k++) {
        long a, f;
        if (!rdi(&a) || !rdi(&f) || !slot(a, ns)) return fail("PROTOCOL", "CP eval slot");
        es[k] = (int)a; ef[k] = (int)f; er[k] = -1;
        for (int j = 0; j < s_nref; j++) if (s_refslot[j] == a) er[k] = j;
        if (er[k] < 0) return fail("PROTOCOL", "evaluated slot has no reference");
    }
    for (long k = 0; k < ne; k++) if (!rdd(&tol[k]) || !(tol[k] > 0)) return fail("PROTOCOL", "CP tolerance");
    for (int k = 0; k < s_nin; k++) if (s_inslot[k] >= ns) return fail("PROTOCOL", "IN slot out of range");
    int mode_array = 0;
    g_diag = 0;
    if (!getenv("FXGRAPH_NO_ARRAY")) {
        double *emax = calloc((size_t)ne, sizeof(double)), *esum = calloc((size_t)ne, sizeof(double)),
               *esq = calloc((size_t)ne, sizeof(double));
        long *earg = calloc((size_t)ne, sizeof(long));
        if (!emax || !esum || !esq || !earg) return fail("OOM", "alloc");
        for (long i = 0; i < s_nt; i++) per[i] = 0.0;
        mode_array = run_array(ins, ni, ns, fastl != 0, ds, dsrc, val, nd, es, ef, er, ne, emax, esum, esq, earg, tol, per);
        if (mode_array < 0) return fail("OOM", "alloc");
        free(emax); free(esum); free(esq); free(earg);
        if (g_diag) mode_array = 0;   /* rerun step by step: the diagnostic then names its step */
        g_diag = 0;
    }
    if (!mode_array)
    for (long step = 0; step < s_nt; step++) {
        for (int k = 0; k < s_nin; k++) val[s_inslot[k]] = s_inval[k][step];
        exec_int(ins, ni, val);
        if (g_diag) { printf("E %s %ld %ld\n", g_diag == 3 ? "FALLBACK" : g_diag == 2 ? "UNRECORDED" : "SIM_DIAGNOSTIC", g_diag_slot, step);
                      fflush(stdout); return 1; }
        double m = 0.0;
        for (long k = 0; k < ne; k++) {
            double rv = s_ref[er[k]][step];
            double e = isfinite(rv) ? fabs(edif(val[es[k]], ef[k], rv)) / tol[k] : INFINITY;
            if (e > m) m = e;   /* a non-finite reference is an infinite error, as in Python */
        }
        per[step] = m;
        for (long k = 0; k < nd; k++) tmp[k] = val[dsrc[k]];
        for (long k = 0; k < nd; k++) val[ds[k]] = tmp[k];
    }
    printf("P %ld", s_nt);
    for (long i = 0; i < s_nt; i++) printf(" %.17g", per[i]);
    printf("\nD\n");
    fflush(stdout);
    for (long i = 0; i < ni; i++) { free(ins[i].src); free(ins[i].sg); free(ins[i].aux); free(ins[i].cs); free(ins[i].fs); }
    free(ins); free(val); free(ds); free(dsrc); free(tmp); free(es); free(ef); free(er); free(tol); free(per);
    return 0;
}

/* DOUT: the recorded double-reference waveforms, exactly (%.17g round-trips every finite double; inf / nan as text) */
static int s_dout(void) {
    if (s_nt < 1 || !s_nref) return fail("PROTOCOL", "DOUT before IN/DREF");
    for (int k = 0; k < s_nref; k++) {
        printf("W %d %ld", s_refslot[k], s_nt);
        for (long i = 0; i < s_nt; i++) printf(" %.17g", s_ref[k][i]);
        printf("\n");
    }
    printf("D\n");
    fflush(stdout);
    return 0;
}

/* REFV slot n v0..v(n-1): replace the reference waveform of a slot already recorded by DREF (doubles) */
static int s_refv(void) {
    long sl, nn;
    if (!rdi(&sl) || !rdi(&nn) || nn != s_nt) return fail("PROTOCOL", "REFV header");
    int k = -1;
    for (int j = 0; j < s_nref; j++) if (s_refslot[j] == sl) k = j;
    if (k < 0) return fail("PROTOCOL", "REFV slot has no reference");
    for (long i = 0; i < nn; i++) if (!rdd(&s_ref[k][i])) return fail("PROTOCOL", "truncated REFV");
    return 0;
}

static int stream_main(void) {
    size_t cap = 0;
    ssize_t got;
    char *t;
    buf = NULL;
    while ((got = getline(&buf, &cap, stdin)) > 0) {
        pos = 0; len = (size_t)got;
        if (!tok(&t)) continue;
        int r;
        if (!strcmp(t, "IN")) r = s_in();
        else if (!strcmp(t, "DREF")) r = s_dref();
        else if (!strcmp(t, "C")) r = s_cand();
        else if (!strcmp(t, "CP")) r = s_candp();
        else if (!strcmp(t, "REFV")) r = s_refv();
        else if (!strcmp(t, "DOUT")) r = s_dout();
        else if (!strcmp(t, "END")) return 0;
        else r = fail("PROTOCOL", "unknown command");
        if (r) return 1;
    }
    return 0;
}

int main(int argc, char **argv) {
    if (argc > 1 && !strcmp(argv[1], "-s")) return stream_main();
    size_t cap = 1 << 20;
    buf = malloc(cap + 1);
    if (!buf) return fail("OOM", "stdin buffer");
    size_t n;
    while ((n = fread(buf + len, 1, cap - len, stdin)) > 0) {
        len += n;
        if (len == cap) { cap *= 2; buf = realloc(buf, cap + 1); if (!buf) return fail("OOM", "stdin buffer"); }
    }
    char *t;
    long ns, ni, nt;
    if (!tok(&t) || strcmp(t, "P") || !rdi(&ns) || !rdi(&ni) || !rdi(&nt) || ns < 1 || ni < 0 || nt < 1)
        return fail("PROTOCOL", "bad program header");
    Ins *ins = calloc((size_t)(ni ? ni : 1), sizeof(Ins));
    i128 *val = calloc((size_t)ns, sizeof(i128));
    if (!ins || !val) return fail("OOM", "alloc");
    if (parse_ins(ins, ni, ns)) return 1;
    /* inputs, delays, records */
    int nin = 0, *inslot = NULL;
    i128 **inval = NULL;
    long nd = 0, nr = 0;
    int *dslot = NULL, *dsrc = NULL, *rslot = NULL;
    while (tok(&t)) {
        if (!strcmp(t, "IN")) {
            long s;
            if (!rdi(&s) || !slot(s, ns)) return fail("PROTOCOL", "IN slot");
            inslot = realloc(inslot, sizeof(int) * (size_t)(nin + 1));
            inval = realloc(inval, sizeof(i128 *) * (size_t)(nin + 1));
            if (!inslot || !inval) return fail("OOM", "alloc");
            inslot[nin] = (int)s;
            inval[nin] = malloc(sizeof(i128) * (size_t)nt);
            if (!inval[nin]) return fail("OOM", "alloc");
            for (long k = 0; k < nt; k++) if (!rd(&inval[nin][k])) return fail("PROTOCOL", "truncated IN");
            nin++;
        } else if (!strcmp(t, "INIT")) {
            long s;
            if (!rdi(&s) || !slot(s, ns) || !rd(&val[s])) return fail("PROTOCOL", "INIT");
        } else if (!strcmp(t, "DLY")) {
            if (!rdi(&nd) || nd < 0 || nd > ns) return fail("PROTOCOL", "DLY");
            dslot = malloc(sizeof(int) * (size_t)(nd + 1)); dsrc = malloc(sizeof(int) * (size_t)(nd + 1));
            if (!dslot || !dsrc) return fail("OOM", "alloc");
            for (long k = 0; k < nd; k++) {
                long a, b;
                if (!rdi(&a) || !rdi(&b) || !slot(a, ns) || !slot(b, ns)) return fail("PROTOCOL", "DLY pair");
                dslot[k] = (int)a; dsrc[k] = (int)b;
            }
        } else if (!strcmp(t, "REC")) {
            if (!rdi(&nr) || nr < 0 || nr > ns) return fail("PROTOCOL", "REC");
            rslot = malloc(sizeof(int) * (size_t)(nr + 1));
            if (!rslot) return fail("OOM", "alloc");
            for (long k = 0; k < nr; k++) { long a; if (!rdi(&a) || !slot(a, ns)) return fail("PROTOCOL", "REC slot"); rslot[k] = (int)a; }
        } else if (!strcmp(t, "DIAG")) {
            g_dc = calloc((size_t)ns * 6, sizeof(long));
            if (!g_dc) return fail("OOM", "diagnostics");
        } else if (!strcmp(t, "END")) break;
        else return fail("PROTOCOL", "unknown section");
    }
    i128 *rec = malloc(sizeof(i128) * (size_t)(nr ? nr : 1) * (size_t)nt);
    i128 *tmp = malloc(sizeof(i128) * (size_t)(nd ? nd : 1));
    if (!rec || !tmp) return fail("OOM", "alloc records");
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    g_diag = 0;
    g_oneshot = 1;
    for (long step = 0; step < nt; step++) {
        for (int k = 0; k < nin; k++) val[inslot[k]] = inval[k][step];
        g_dstep = step;
        exec_int(ins, ni, val);
        g_dcur = -1;
        if (g_diag) { printf("E %s %ld %ld\n", g_diag == 3 ? "FALLBACK" : g_diag == 2 ? "UNRECORDED" : "SIM_DIAGNOSTIC", g_diag_slot, step);
                      fflush(stdout); return 1; }
        for (long k = 0; k < nr; k++) rec[(size_t)k * (size_t)nt + (size_t)step] = val[rslot[k]];
        for (long k = 0; k < nd; k++) tmp[k] = val[dsrc[k]];
        for (long k = 0; k < nd; k++) val[dslot[k]] = tmp[k];
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    for (long k = 0; k < nr; k++) {
        printf("O %d", rslot[k]);
        for (long s = 0; s < nt; s++) { putchar(' '); pr(rec[(size_t)k * (size_t)nt + (size_t)s]); }
        putchar('\n');
    }
    if (g_dc)
        for (long k = 0; k < ns; k++) {
            const long *e = g_dc + 6 * k;
            if (e[0] || e[1] || e[4])
                printf("X %ld %ld %ld %ld %ld %ld %ld\n", k, e[0], e[1], e[0] ? e[2] : -1, e[1] ? e[3] : -1, e[4],
                       e[4] ? e[5] : -1);
        }
    printf("T %ld\nD\n", (long)(t1.tv_sec - t0.tv_sec) * 1000000000L + (t1.tv_nsec - t0.tv_nsec));
    return 0;
}
