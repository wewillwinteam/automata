/* Minimal JSON: a DOM parser for the config and bot actions, and a growable
 * string buffer for writing observations. Single header, no dependencies. */
#ifndef MR_JSON_H
#define MR_JSON_H

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { J_NULL, J_BOOL, J_NUM, J_STR, J_ARR, J_OBJ } JType;

typedef struct JNode {
    JType type;
    double num;           /* J_NUM, J_BOOL (0/1) */
    char *str;            /* J_STR */
    int n;                /* J_ARR / J_OBJ child count */
    struct JNode **kids;  /* children */
    char **keys;          /* J_OBJ keys */
} JNode;

typedef struct { const char *s; const char *end; int err; } JP;

static void jp_ws(JP *p) { while (p->s < p->end && isspace((unsigned char)*p->s)) p->s++; }

static JNode *jnew(JType t) { JNode *n = calloc(1, sizeof(JNode)); n->type = t; return n; }

static JNode *jp_value(JP *p, int depth);

static char *jp_string(JP *p) {
    /* p->s points at the opening quote */
    p->s++;
    size_t cap = 32, len = 0;
    char *out = malloc(cap);
    while (p->s < p->end && *p->s != '"') {
        char c = *p->s++;
        if (c == '\\' && p->s < p->end) {
            char e = *p->s++;
            switch (e) {
            case 'n': c = '\n'; break;
            case 't': c = '\t'; break;
            case 'r': c = '\r'; break;
            case 'b': c = '\b'; break;
            case 'f': c = '\f'; break;
            case 'u': {
                unsigned v = 0;
                for (int i = 0; i < 4 && p->s < p->end; i++) {
                    char h = *p->s++;
                    v = v * 16 + (unsigned)(isdigit((unsigned char)h) ? h - '0' : (tolower((unsigned char)h) - 'a' + 10));
                }
                c = v < 128 ? (char)v : '?';
                break;
            }
            default: c = e;
            }
        }
        if (len + 2 > cap) { cap *= 2; out = realloc(out, cap); }
        out[len++] = c;
    }
    if (p->s >= p->end) p->err = 1; else p->s++;
    out[len] = 0;
    return out;
}

static void jp_push(JNode *n, JNode *kid, char *key) {
    n->kids = realloc(n->kids, sizeof(JNode *) * (n->n + 1));
    if (n->type == J_OBJ) n->keys = realloc(n->keys, sizeof(char *) * (n->n + 1));
    n->kids[n->n] = kid;
    if (n->type == J_OBJ) n->keys[n->n] = key;
    n->n++;
}

static JNode *jp_value(JP *p, int depth) {
    jp_ws(p);
    if (p->s >= p->end || depth > 64) { p->err = 1; return NULL; }
    char c = *p->s;
    if (c == '{') {
        JNode *n = jnew(J_OBJ);
        p->s++;
        jp_ws(p);
        if (p->s < p->end && *p->s == '}') { p->s++; return n; }
        while (!p->err) {
            jp_ws(p);
            if (p->s >= p->end || *p->s != '"') { p->err = 1; break; }
            char *k = jp_string(p);
            jp_ws(p);
            if (p->s >= p->end || *p->s != ':') { free(k); p->err = 1; break; }
            p->s++;
            JNode *v = jp_value(p, depth + 1);
            if (!v) { free(k); break; }
            jp_push(n, v, k);
            jp_ws(p);
            if (p->s < p->end && *p->s == ',') { p->s++; continue; }
            if (p->s < p->end && *p->s == '}') { p->s++; break; }
            p->err = 1;
        }
        return n;
    }
    if (c == '[') {
        JNode *n = jnew(J_ARR);
        p->s++;
        jp_ws(p);
        if (p->s < p->end && *p->s == ']') { p->s++; return n; }
        while (!p->err) {
            JNode *v = jp_value(p, depth + 1);
            if (!v) break;
            jp_push(n, v, NULL);
            jp_ws(p);
            if (p->s < p->end && *p->s == ',') { p->s++; continue; }
            if (p->s < p->end && *p->s == ']') { p->s++; break; }
            p->err = 1;
        }
        return n;
    }
    if (c == '"') { JNode *n = jnew(J_STR); n->str = jp_string(p); return n; }
    if (!strncmp(p->s, "true", 4)) { p->s += 4; JNode *n = jnew(J_BOOL); n->num = 1; return n; }
    if (!strncmp(p->s, "false", 5)) { p->s += 5; JNode *n = jnew(J_BOOL); n->num = 0; return n; }
    if (!strncmp(p->s, "null", 4)) { p->s += 4; return jnew(J_NULL); }
    if (c == '-' || isdigit((unsigned char)c)) {
        char *e;
        double v = strtod(p->s, &e);
        if (e == p->s) { p->err = 1; return NULL; }
        p->s = e;
        JNode *n = jnew(J_NUM);
        n->num = v;
        return n;
    }
    p->err = 1;
    return NULL;
}

static void jfree(JNode *n) {
    if (!n) return;
    for (int i = 0; i < n->n; i++) {
        jfree(n->kids[i]);
        if (n->keys) free(n->keys[i]);
    }
    free(n->kids);
    free(n->keys);
    free(n->str);
    free(n);
}

/* Parse; returns NULL on any syntax error. */
static JNode *jparse(const char *s, size_t len) {
    JP p = {s, s + len, 0};
    JNode *n = jp_value(&p, 0);
    if (p.err) { jfree(n); return NULL; }
    return n;
}

static JNode *jkey(const JNode *o, const char *k) {
    if (!o || o->type != J_OBJ) return NULL;
    for (int i = 0; i < o->n; i++)
        if (!strcmp(o->keys[i], k)) return o->kids[i];
    return NULL;
}

/* jpath(o, "weapons.gun.damage") */
static JNode *jpath(const JNode *o, const char *path) {
    char buf[128];
    const char *s = path;
    const JNode *cur = o;
    while (cur && *s) {
        const char *dot = strchr(s, '.');
        size_t l = dot ? (size_t)(dot - s) : strlen(s);
        if (l >= sizeof buf) return NULL;
        memcpy(buf, s, l);
        buf[l] = 0;
        cur = jkey(cur, buf);
        s = dot ? dot + 1 : s + l;
    }
    return (JNode *)cur;
}

static double jnum(const JNode *n, double dflt) {
    if (n && (n->type == J_NUM || n->type == J_BOOL)) return n->num;
    return dflt;
}

static double jpnum(const JNode *o, const char *path, double dflt) { return jnum(jpath(o, path), dflt); }

static JNode *jidx(const JNode *a, int i) {
    if (!a || a->type != J_ARR || i < 0 || i >= a->n) return NULL;
    return a->kids[i];
}

static int jtruthy(const JNode *n) {
    if (!n) return 0;
    switch (n->type) {
    case J_NULL: return 0;
    case J_BOOL: case J_NUM: return n->num != 0;
    case J_STR: return n->str[0] != 0;
    default: return n->n > 0;
    }
}

/* ------------------------------------------------------------- writer */

typedef struct { char *b; size_t len, cap; } SB;

static void sb_grow(SB *s, size_t need) {
    if (s->len + need + 1 <= s->cap) return;
    size_t c = s->cap ? s->cap : 4096;
    while (c < s->len + need + 1) c *= 2;
    s->b = realloc(s->b, c);
    s->cap = c;
}

static inline void sb_put(SB *s, const char *t, size_t l) {
    if (s->len + l + 1 > s->cap) sb_grow(s, l);
    memcpy(s->b + s->len, t, l);
    s->len += l;
    s->b[s->len] = 0;
}

static void sb_s(SB *s, const char *t) { sb_put(s, t, strlen(t)); }
/* string literal without the strlen */
#define SB_LIT(s, lit) sb_put((s), (lit), sizeof(lit) - 1)

static inline void sb_c(SB *s, char c) { if (s->len + 2 > s->cap) sb_grow(s, 1); s->b[s->len++] = c; s->b[s->len] = 0; }

static void sb_f(SB *s, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    char tmp[512];
    int l = vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    if (l < 0) return;
    if ((size_t)l < sizeof tmp) { sb_put(s, tmp, (size_t)l); return; }
    sb_grow(s, (size_t)l);
    va_start(ap, fmt);
    vsnprintf(s->b + s->len, (size_t)l + 1, fmt, ap);
    va_end(ap);
    s->len += (size_t)l;
}

/* number with d decimals, trailing zeros trimmed; non-finite -> null.
 * Hand-rolled with a two-digit table: snprintf("%.*f") dominated the
 * observation writer. */
static const char MR_DIG2[] =
    "00010203040506070809101112131415161718192021222324252627282930313233343536373839"
    "40414243444546474849505152535455565758596061626364656667686970717273747576777879"
    "8081828384858687888990919293949596979899";

static inline int mr_utoa(char *out, unsigned long long v) {
    char tmp[24];
    int n = 24;
    while (v >= 100) { unsigned r = (unsigned)(v % 100); v /= 100; n -= 2; memcpy(tmp + n, MR_DIG2 + 2 * r, 2); }
    if (v >= 10) { n -= 2; memcpy(tmp + n, MR_DIG2 + 2 * v, 2); }
    else tmp[--n] = (char)('0' + v);
    memcpy(out, tmp + n, (size_t)(24 - n));
    return 24 - n;
}

/* write q (= |v| * 10^D rounded) with D decimals, zeros trimmed.
 * D is a compile-time constant in each specialisation, so / and % by
 * 10^D become multiplications. */
#define MR_NUM_BODY(D, U)                                                       \
    {                                                                           \
        unsigned long long ip = q / (U), fp = q - ip * (U);                     \
        sb_grow(s, 48);                                                         \
        char *o = s->b + s->len;                                                \
        int l = 0;                                                              \
        if (neg && q) o[l++] = '-';                                             \
        l += mr_utoa(o + l, ip);                                                \
        if (fp) {                                                               \
            int dd = (D);                                                       \
            while (fp % 10 == 0) { fp /= 10; dd--; }                            \
            o[l++] = '.';                                                       \
            for (int i = dd - 1; i >= 0; i--) { o[l + i] = (char)('0' + fp % 10); fp /= 10; } \
            l += dd;                                                            \
        }                                                                       \
        s->len += (size_t)l;                                                    \
        s->b[s->len] = 0;                                                       \
    }

static void sb_num(SB *s, double v, int d) {
    if (!isfinite(v)) { sb_s(s, "null"); return; }
    static const double P10[] = {1, 10, 100, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9};
    if (d < 0) d = 0;
    if (d > 9) d = 9;
    double sc = fabs(v) * P10[d];
    if (sc >= 9.0e17) {
        char tmp[64];
        int l = snprintf(tmp, sizeof tmp, "%.*f", d, v);
        sb_put(s, tmp, (size_t)l);
        return;
    }
    unsigned long long q = (unsigned long long)(sc + 0.5);
    int neg = v < 0;
    switch (d) {
    case 0: MR_NUM_BODY(0, 1ULL) break;
    case 1: MR_NUM_BODY(1, 10ULL) break;
    case 2: MR_NUM_BODY(2, 100ULL) break;
    case 3: MR_NUM_BODY(3, 1000ULL) break;
    case 4: MR_NUM_BODY(4, 10000ULL) break;
    case 6: MR_NUM_BODY(6, 1000000ULL) break;
    default: {
        static const unsigned long long U10[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
        MR_NUM_BODY(d, U10[d])
    }
    }
}

static void sb_int(SB *s, long long v) {
    sb_grow(s, 24);
    char *o = s->b + s->len;
    int l = 0;
    if (v < 0) { o[l++] = '-'; v = -v; }
    l += mr_utoa(o + l, (unsigned long long)v);
    s->len += (size_t)l;
    s->b[s->len] = 0;
}

static void sb_str(SB *s, const char *t) {
    sb_c(s, '"');
    for (; *t; t++) {
        unsigned char c = (unsigned char)*t;
        if (c == '"' || c == '\\') { sb_c(s, '\\'); sb_c(s, (char)c); }
        else if (c < 0x20) sb_f(s, "\\u%04x", c);
        else sb_c(s, (char)c);
    }
    sb_c(s, '"');
}

/* re-serialise a parsed node compactly (used for config sub-tables) */
static inline void sb_json(SB *s, const JNode *n) {
    if (!n) { sb_s(s, "null"); return; }
    switch (n->type) {
    case J_NULL: sb_s(s, "null"); break;
    case J_BOOL: sb_s(s, n->num ? "true" : "false"); break;
    case J_NUM: {
        if (n->num == floor(n->num) && fabs(n->num) < 1e15) sb_f(s, "%.0f", n->num);
        else sb_f(s, "%.17g", n->num);
        break;
    }
    case J_STR: sb_str(s, n->str); break;
    case J_ARR:
        sb_c(s, '[');
        for (int i = 0; i < n->n; i++) { if (i) sb_c(s, ','); sb_json(s, n->kids[i]); }
        sb_c(s, ']');
        break;
    case J_OBJ:
        sb_c(s, '{');
        for (int i = 0; i < n->n; i++) {
            if (i) sb_c(s, ',');
            sb_str(s, n->keys[i]);
            sb_c(s, ':');
            sb_json(s, n->kids[i]);
        }
        sb_c(s, '}');
        break;
    }
}

#endif
