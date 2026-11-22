/* Port of the parts of mech/world.go that the brain uses. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bot.h"
#include "json.h"

#define GEO_CELL 16.0
#define STEP_UP 1.2
static const double NAV_RADIUS = 2.6;

static const int NBRS[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

/* Go's amd64 math.Hypot: max * sqrt(1 + (min/max)^2) */
double go_hypot(double p, double q) {
    p = fabs(p);
    q = fabs(q);
    if (isinf(p) || isinf(q)) return INFINITY;
    if (isnan(p) || isnan(q)) return NAN;
    double mx = p > q ? p : q, mn = p > q ? q : p;
    if (mx == 0) return 0;
    mn = mn / mx;
    return mx * sqrt(1 + mn * mn);
}

static int imax(int a, int b) { return a > b ? a : b; }
static int imin(int a, int b) { return a < b ? a : b; }

/* ------------------------------------------------------------- loading */

static char *slurp(const char *path, size_t *len) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *s = malloc((size_t)n + 1);
    if (fread(s, 1, (size_t)n, f) != (size_t)n) { fclose(f); free(s); return NULL; }
    s[n] = 0;
    fclose(f);
    *len = (size_t)n;
    return s;
}

static int cmp_f64(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b;
    return x < y ? -1 : x > y;
}

typedef struct { int32_t idx; float base, top; uint8_t bridge; } CellIv;

static int cmp_iv(const void *a, const void *b) {
    const CellIv *x = a, *y = b;
    if (x->idx != y->idx) return x->idx < y->idx ? -1 : 1;
    if (x->base != y->base) return x->base < y->base ? -1 : 1;
    return 0;
}

static int cell_of(const World *w, double x, double y) {
    int i = (int)floor((x - w->gx0) / GEO_CELL), j = (int)floor((y - w->gy0) / GEO_CELL);
    if (i < 0 || j < 0 || i >= w->gnx || j >= w->gny) return -1;
    return j * w->gnx + i;
}

static void for_cells_range(const World *w, double x0, double y0, double x1, double y1, int *i0, int *i1, int *j0, int *j1) {
    *i0 = imax(0, (int)floor((x0 - w->gx0) / GEO_CELL));
    *i1 = imin(w->gnx - 1, (int)floor((x1 - w->gx0) / GEO_CELL));
    *j0 = imax(0, (int)floor((y0 - w->gy0) / GEO_CELL));
    *j1 = imin(w->gny - 1, (int)floor((y1 - w->gy0) / GEO_CELL));
}

static int point_in_poly(const double *e, int ne, double x, double y) {
    int in = 0;
    for (int i = 0; i < ne * 4; i += 4) {
        double x0 = e[i], y0 = e[i + 1], x1 = e[i + 2], y1 = e[i + 3];
        if (((y0 > y) != (y1 > y)) && x < (x1 - x0) * (y - y0) / (y1 - y0) + x0) in = !in;
    }
    return in;
}

static void init_geo(World *w) {
    w->gx0 = w->bx0 - GEO_CELL;
    w->gy0 = w->by0 - GEO_CELL;
    w->gnx = (int)ceil((w->bx1 - w->bx0) / GEO_CELL) + 3;
    w->gny = (int)ceil((w->by1 - w->by0) / GEO_CELL) + 3;
    int nc = w->gnx * w->gny;
    w->gridp_off = calloc((size_t)nc + 1, sizeof(int32_t));
    for (int pass = 0; pass < 2; pass++) {
        int32_t *fill = NULL;
        if (pass == 1) {
            for (int c = 0; c < nc; c++) w->gridp_off[c + 1] += w->gridp_off[c];
            w->gridp = malloc(sizeof(int32_t) * (size_t)(w->gridp_off[nc] + 1));
            fill = calloc((size_t)nc, sizeof(int32_t));
        }
        for (int pi = 0; pi < w->nprisms; pi++) {
            Prism *p = &w->prisms[pi];
            int i0, i1, j0, j1;
            for_cells_range(w, p->minx, p->miny, p->maxx, p->maxy, &i0, &i1, &j0, &j1);
            for (int j = j0; j <= j1; j++)
                for (int i = i0; i <= i1; i++) {
                    int c = j * w->gnx + i;
                    if (pass == 0) w->gridp_off[c + 1]++;
                    else w->gridp[w->gridp_off[c] + fill[c]++] = pi;
                }
        }
        free(fill);
    }
    w->bnx = (w->nx + 7) / 8;
    w->bny = (w->ny + 7) / 8;
    float *bm = malloc(sizeof(float) * (size_t)w->bnx * w->bny);
    for (int i = 0; i < w->bnx * w->bny; i++) bm[i] = -INFINITY;
    for (int j = 0; j < w->ny; j++)
        for (int i = 0; i < w->nx; i++) {
            int b = (j / 8) * w->bnx + i / 8;
            float h = w->ground[j * w->nx + i];
            if (h > bm[b]) bm[b] = h;
        }
    w->block_max = malloc(sizeof(float) * (size_t)w->bnx * w->bny);
    for (int j = 0; j < w->bny; j++)
        for (int i = 0; i < w->bnx; i++) {
            float v = bm[j * w->bnx + i];
            if (i + 1 < w->bnx && bm[j * w->bnx + i + 1] > v) v = bm[j * w->bnx + i + 1];
            if (j + 1 < w->bny && bm[(j + 1) * w->bnx + i] > v) v = bm[(j + 1) * w->bnx + i];
            if (i + 1 < w->bnx && j + 1 < w->bny && bm[(j + 1) * w->bnx + i + 1] > v) v = bm[(j + 1) * w->bnx + i + 1];
            w->block_max[j * w->bnx + i] = v;
        }
    free(bm);
}

static void build_nav(World *w);

World *world_load(const char *dir) {
    char path[2048];
    size_t len;
    snprintf(path, sizeof path, "%s/v2/world.json", dir);
    char *raw = slurp(path, &len);
    if (!raw) return NULL;
    JNode *wj = jparse(raw, len);
    free(raw);
    if (!wj) return NULL;
    World *w = calloc(1, sizeof(World));
    w->minx = jpnum(wj, "terrain.minX", 0);
    w->miny = jpnum(wj, "terrain.minY", 0);
    w->nx = (int)jpnum(wj, "terrain.nx", 0);
    w->ny = (int)jpnum(wj, "terrain.ny", 0);
    w->bx0 = jpnum(wj, "bounds.minX", 0);
    w->bx1 = jpnum(wj, "bounds.maxX", 0);
    w->by0 = jpnum(wj, "bounds.minY", 0);
    w->by1 = jpnum(wj, "bounds.maxY", 0);
    size_t n = (size_t)w->nx * w->ny;
    snprintf(path, sizeof path, "%s/v2/terrain.i16", dir);
    char *tb = slurp(path, &len);
    if (!tb || len < n * 2 || n == 0) { free(tb); jfree(wj); free(w); return NULL; }
    w->ground = malloc(sizeof(float) * n);
    for (size_t i = 0; i < n; i++) {
        int16_t v = (int16_t)((uint8_t)tb[i * 2] | ((uint16_t)(uint8_t)tb[i * 2 + 1] << 8));
        w->ground[i] = (float)v * 0.01f;
    }
    free(tb);

    JNode *ps = jkey(wj, "prisms");
    w->nprisms = ps ? ps->n : 0;
    w->prisms = calloc((size_t)w->nprisms + 1, sizeof(Prism));
    for (int pi = 0; pi < w->nprisms; pi++) {
        JNode *pj = ps->kids[pi];
        Prism *q = &w->prisms[pi];
        JNode *k = jkey(pj, "kind");
        const char *ks = k && k->type == J_STR ? k->str : "";
        q->kind = !strcmp(ks, "bridge") ? 'r' : !strcmp(ks, "building") ? 'b' : !strcmp(ks, "stadium") ? 's' : 'o';
        q->base = jnum(jkey(pj, "base"), 0);
        q->top = jnum(jkey(pj, "top"), 0);
        q->minx = q->miny = INFINITY;
        q->maxx = q->maxy = -INFINITY;
        JNode *ring = jkey(pj, "ring"), *holes = jkey(pj, "holes");
        int nr = 1 + (holes && holes->type == J_ARR ? holes->n : 0);
        int tot = 0;
        for (int r = 0; r < nr; r++) {
            JNode *rr = r == 0 ? ring : holes->kids[r - 1];
            if (rr && rr->type == J_ARR) tot += rr->n / 2;
        }
        q->edges = malloc(sizeof(double) * 4 * (size_t)(tot + 1));
        q->nedges = 0;
        for (int r = 0; r < nr; r++) {
            JNode *rr = r == 0 ? ring : holes->kids[r - 1];
            if (!rr || rr->type != J_ARR) continue;
            int m = rr->n / 2;
            for (int a = 0; a < m; a++) {
                int bb = (a + 1) % m;
                double x0 = jnum(rr->kids[a * 2], 0), y0 = jnum(rr->kids[a * 2 + 1], 0);
                double x1 = jnum(rr->kids[bb * 2], 0), y1 = jnum(rr->kids[bb * 2 + 1], 0);
                double *e = &q->edges[q->nedges * 4];
                e[0] = x0; e[1] = y0; e[2] = x1; e[3] = y1;
                q->nedges++;
                q->minx = fmin(q->minx, x0); q->maxx = fmax(q->maxx, x0);
                q->miny = fmin(q->miny, y0); q->maxy = fmax(q->maxy, y0);
            }
        }
    }
    init_geo(w);

    /* rasterise prism height intervals onto the 1 m terrain vertices */
    size_t niv = 0, civ = 1 << 16;
    CellIv *ivs = malloc(sizeof(CellIv) * civ);
    double *xs = malloc(sizeof(double) * 4096);
    size_t cxs = 4096;
    for (int pi = 0; pi < w->nprisms; pi++) {
        JNode *pj = ps->kids[pi];
        JNode *ring = jkey(pj, "ring"), *holes = jkey(pj, "holes");
        if (!ring || ring->type != J_ARR) continue;
        double miny = INFINITY, maxy = -INFINITY;
        for (int k = 1; k < ring->n; k += 2) { double v = jnum(ring->kids[k], 0); miny = fmin(miny, v); maxy = fmax(maxy, v); }
        int j0 = (int)ceil(miny - w->miny), j1 = (int)floor(maxy - w->miny);
        float base = (float)w->prisms[pi].base, top = (float)w->prisms[pi].top;
        uint8_t bridge = w->prisms[pi].kind == 'r';
        int nr = 1 + (holes && holes->type == J_ARR ? holes->n : 0);
        for (int j = imax(j0, 0); j <= j1 && j < w->ny; j++) {
            double y = w->miny + (double)j + 1e-4;
            size_t nxs = 0;
            for (int r = 0; r < nr; r++) {
                JNode *rr = r == 0 ? ring : holes->kids[r - 1];
                if (!rr || rr->type != J_ARR) continue;
                int m = rr->n / 2;
                for (int a = 0; a < m; a++) {
                    int bb = (a + 1) % m;
                    double x0 = jnum(rr->kids[a * 2], 0), y0 = jnum(rr->kids[a * 2 + 1], 0);
                    double x1 = jnum(rr->kids[bb * 2], 0), y1 = jnum(rr->kids[bb * 2 + 1], 0);
                    if ((y0 <= y) != (y1 <= y)) {
                        if (nxs == cxs) { cxs *= 2; xs = realloc(xs, sizeof(double) * cxs); }
                        xs[nxs++] = x0 + (y - y0) / (y1 - y0) * (x1 - x0);
                    }
                }
            }
            qsort(xs, nxs, sizeof(double), cmp_f64);
            for (size_t k = 0; k + 1 < nxs; k += 2) {
                int i0 = (int)ceil(xs[k] - w->minx), i1 = (int)floor(xs[k + 1] - w->minx);
                for (int i = imax(i0, 0); i <= i1 && i < w->nx; i++) {
                    if (niv == civ) { civ *= 2; ivs = realloc(ivs, sizeof(CellIv) * civ); }
                    ivs[niv].idx = j * w->nx + i;
                    ivs[niv].base = base;
                    ivs[niv].top = top;
                    ivs[niv].bridge = bridge;
                    niv++;
                }
            }
        }
    }
    free(xs);
    jfree(wj);
    qsort(ivs, niv, sizeof(CellIv), cmp_iv);
    w->top = malloc(sizeof(float) * n);
    w->obst = calloc(n, 1);
    w->has_slab = calloc(n, 1);
    w->slab_off = calloc(n + 1, sizeof(int32_t));
    memcpy(w->top, w->ground, sizeof(float) * n);
    Slab *slabs = malloc(sizeof(Slab) * (niv + 1));
    int32_t nsl = 0;
    size_t k = 0;
    int32_t last_idx = 0;
    while (k < niv) {
        int32_t idx = ivs[k].idx;
        for (int32_t c = last_idx; c <= idx; c++) w->slab_off[c] = nsl;
        size_t e = k;
        while (e < niv && ivs[e].idx == idx) e++;
        float g = w->ground[idx], top = g, deck = -1e9f;
        int low = 0;
        for (size_t q = k; q < e; q++) {
            CellIv *iv = &ivs[q];
            if (iv->top <= top) continue;
            if (iv->base <= top + 1.5f) { top = iv->top; continue; }
            slabs[nsl].base = iv->base;
            slabs[nsl].top = iv->top;
            nsl++;
            w->has_slab[idx] = 1;
            if (iv->base - top < 10.5f) {
                if (iv->bridge && iv->top - g < 6.5f) deck = iv->top;
                else low = 1;
            }
        }
        (void)deck;
        w->top[idx] = top;
        w->obst[idx] = (top - g > 1.2f) || low;
        last_idx = idx + 1;
        k = e;
    }
    for (int32_t c = last_idx; c <= (int32_t)n; c++) w->slab_off[c] = nsl;
    w->slabs = slabs;
    free(ivs);
    build_nav(w);
    return w;
}

World *world_flat(void) {
    World *w = calloc(1, sizeof(World));
    w->minx = -513; w->miny = -334; w->nx = 1070; w->ny = 724;
    w->bx0 = -512; w->bx1 = 555; w->by0 = -333; w->by1 = 388;
    size_t n = (size_t)w->nx * w->ny;
    w->ground = calloc(n, sizeof(float));
    w->top = calloc(n, sizeof(float));
    w->obst = calloc(n, 1);
    w->has_slab = calloc(n, 1);
    w->slab_off = calloc(n + 1, sizeof(int32_t));
    w->slabs = calloc(1, sizeof(Slab));
    w->prisms = calloc(1, sizeof(Prism));
    init_geo(w);
    build_nav(w);
    return w;
}

/* ------------------------------------------------------------- queries */

int world_vidx(const World *w, double x, double y) {
    int i = (int)round(x - w->minx), j = (int)round(y - w->miny);
    if (i < 0) i = 0; else if (i >= w->nx) i = w->nx - 1;
    if (j < 0) j = 0; else if (j >= w->ny) j = w->ny - 1;
    return j * w->nx + i;
}

int world_in_bounds(const World *w, double x, double y) { return x > w->bx0 && x < w->bx1 && y > w->by0 && y < w->by1; }

double world_ground_at(const World *w, double x, double y) {
    double fx = x - w->minx, fy = y - w->miny;
    fx = fmax(0, fmin(fx, (double)(w->nx - 1) - 1e-6));
    fy = fmax(0, fmin(fy, (double)(w->ny - 1) - 1e-6));
    int i = (int)fx, j = (int)fy;
    double tx = fx - i, ty = fy - j;
    double a = w->ground[j * w->nx + i], b = w->ground[j * w->nx + i + 1];
    double c = w->ground[(j + 1) * w->nx + i], d = w->ground[(j + 1) * w->nx + i + 1];
    return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
}

double world_stack_kind(const World *w, double x, double y, int deck, int *id) {
    double g = world_ground_at(w, x, y), z = g;
    int pid = -1;
    int c = cell_of(w, x, y);
    if (c >= 0) {
        for (int pass = 0; pass < 8; pass++) {
            int moved = 0;
            for (int k = w->gridp_off[c]; k < w->gridp_off[c + 1]; k++) {
                int pi = w->gridp[k];
                const Prism *p = &w->prisms[pi];
                if (p->top <= z || x < p->minx || x > p->maxx || y < p->miny || y > p->maxy) continue;
                if (!(p->base <= z + 1.5 || (deck && p->kind == 'r' && p->top - g < 6.5))) continue;
                if (point_in_poly(p->edges, p->nedges, x, y)) { z = p->top; pid = pi; moved = 1; }
            }
            if (!moved) break;
        }
    }
    if (id) *id = pid;
    return z;
}

double world_walk_at(const World *w, double x, double y) { return world_stack_kind(w, x, y, 1, NULL); }
double world_top_at(const World *w, double x, double y) { return world_stack_kind(w, x, y, 0, NULL); }

int world_solid(const World *w, double x, double y, double z) {
    if (z < world_ground_at(w, x, y)) return 1;
    int c = cell_of(w, x, y);
    if (c < 0) return 0;
    for (int k = w->gridp_off[c]; k < w->gridp_off[c + 1]; k++) {
        const Prism *p = &w->prisms[w->gridp[k]];
        if (z > p->base && z < p->top && x >= p->minx && x <= p->maxx && y >= p->miny && y <= p->maxy &&
            point_in_poly(p->edges, p->nedges, x, y))
            return 1;
    }
    return 0;
}

static int solid_raster(const World *w, double x, double y, double z) {
    int idx = world_vidx(w, x, y);
    if (z < (double)w->top[idx] - 0.05) return 1;
    if (w->has_slab[idx])
        for (int k = w->slab_off[idx]; k < w->slab_off[idx + 1]; k++)
            if (z > (double)w->slabs[k].base && z < (double)w->slabs[k].top) return 1;
    return 0;
}

static int wall_within(const World *w, double x, double y, double z, double r, double h) {
    double lo = z + STEP_UP, hi = z + h, r2 = r * r;
    int i0, i1, j0, j1;
    for_cells_range(w, x - r, y - r, x + r, y + r, &i0, &i1, &j0, &j1);
    for (int j = j0; j <= j1; j++)
        for (int i = i0; i <= i1; i++) {
            int c = j * w->gnx + i;
            for (int k = w->gridp_off[c]; k < w->gridp_off[c + 1]; k++) {
                const Prism *p = &w->prisms[w->gridp[k]];
                if (p->top <= lo || p->base >= hi || x + r < p->minx || x - r > p->maxx || y + r < p->miny || y - r > p->maxy)
                    continue;
                const double *e = p->edges;
                for (int q = 0; q < p->nedges * 4; q += 4) {
                    double x0 = e[q], y0 = e[q + 1], ux = e[q + 2] - x0, uy = e[q + 3] - y0;
                    double l2 = ux * ux + uy * uy, t = 0;
                    if (l2 > 1e-9) t = fmax(0, fmin(1, ((x - x0) * ux + (y - y0) * uy) / l2));
                    double mx = x0 + ux * t - x, my = y0 + uy * t - y;
                    if (mx * mx + my * my < r2) return 1;
                }
                if (point_in_poly(e, p->nedges, x, y)) return 1;
            }
        }
    return 0;
}

static int standable(const World *w, double x, double y, double z, double r) {
    if (x < w->bx0 + r || x > w->bx1 - r || y < w->by0 + r || y > w->by1 - r) return 0;
    return !wall_within(w, x, y, z, r, 9);
}

int world_cell(const World *w, double x, double y) {
    int ci = (int)round((x - w->minx) / w->ns), cj = (int)round((y - w->miny) / w->ns);
    if (ci < 0) ci = 0; else if (ci >= w->nnx) ci = w->nnx - 1;
    if (cj < 0) cj = 0; else if (cj >= w->nny) cj = w->nny - 1;
    return cj * w->nnx + ci;
}

void world_cell_pos(const World *w, int c, double *x, double *y) {
    *x = w->minx + (double)(c % w->nnx) * w->ns;
    *y = w->miny + (double)(c / w->nnx) * w->ns;
}

int world_free(const World *w, double x, double y) { return !w->blocked[world_cell(w, x, y)]; }

int world_can_step(const World *w, int a, int b, int diag) {
    if (w->blocked[b]) return 0;
    float rise = w->surf[b] - w->surf[a];
    return rise <= (diag ? 3.1f : 2.2f);
}

void world_queue_reserve(World *w, int n) {
    if (n > w->cqueue) {
        w->cqueue = n;
        w->queue = realloc(w->queue, sizeof(int32_t) * (size_t)n);
    }
}

static void build_nav(World *w) {
    w->ns = 2;
    w->nnx = (w->nx + 1) / 2;
    w->nny = (w->ny + 1) / 2;
    int n = w->nnx * w->nny;
    w->blocked = calloc((size_t)n, 1);
    w->surf = malloc(sizeof(float) * (size_t)n);
    for (int c = 0; c < n; c++) {
        double x, y;
        world_cell_pos(w, c, &x, &y);
        int id;
        double z = world_stack_kind(w, x, y, 1, &id);
        w->surf[c] = (float)z;
        int roof = id >= 0 && w->prisms[id].kind != 'r' && z - world_ground_at(w, x, y) > 1.5;
        w->blocked[c] = roof || !standable(w, x, y, z, NAV_RADIUS);
    }
    w->clearance = malloc(sizeof(float) * (size_t)n);
    size_t cq = (size_t)n * 2 + 16;
    int32_t *q = malloc(sizeof(int32_t) * cq);
    int nq = 0;
    for (int c = 0; c < n; c++) {
        if (w->blocked[c]) { w->clearance[c] = 0; q[nq++] = c; }
        else w->clearance[c] = 1e9f;
    }
    static const int O4[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (int h = 0; h < nq; h++) {
        int c = q[h];
        float d = w->clearance[c] + (float)w->ns;
        if (d > 20) continue;
        int ci = c % w->nnx, cj = c / w->nnx;
        for (int o = 0; o < 4; o++) {
            int i = ci + O4[o][0], j = cj + O4[o][1];
            if (i < 0 || j < 0 || i >= w->nnx || j >= w->nny) continue;
            int nb = j * w->nnx + i;
            if (w->clearance[nb] > d) {
                w->clearance[nb] = d;
                if ((size_t)nq == cq) { cq *= 2; q = realloc(q, sizeof(int32_t) * cq); }
                q[nq++] = nb;
            }
        }
    }
    for (int c = 0; c < n; c++) if (w->clearance[c] > 20) w->clearance[c] = 20;

    w->comp = calloc((size_t)n, sizeof(int32_t));
    int32_t label = 0, best_n = 0;
    int best_size = 0;
    for (int c0 = 0; c0 < n; c0++) {
        if (w->blocked[c0] || w->comp[c0] != 0) continue;
        label++;
        w->comp[c0] = label;
        nq = 0;
        q[nq++] = c0;
        for (int h = 0; h < nq; h++) {
            int c = q[h];
            int ci = c % w->nnx, cj = c / w->nnx;
            for (int o = 0; o < 4; o++) {
                int i = ci + O4[o][0], j = cj + O4[o][1];
                if (i < 0 || j < 0 || i >= w->nnx || j >= w->nny) continue;
                int nb = j * w->nnx + i;
                if (w->comp[nb] == 0 && world_can_step(w, c, nb, 0) && world_can_step(w, nb, c, 0)) {
                    w->comp[nb] = label;
                    q[nq++] = nb;
                }
            }
        }
        if (nq > best_size) { best_size = nq; best_n = label; }
    }
    free(q);
    w->main_comp = best_n;
    w->g = calloc((size_t)n, sizeof(float));
    w->came = calloc((size_t)n, sizeof(int32_t));
    w->stamp = calloc((size_t)n, sizeof(uint32_t));
    w->closed = calloc((size_t)n, sizeof(uint32_t));
    world_queue_reserve(w, 65536);
}

int world_nearest_free(const World *w, int c) {
    if (!w->blocked[c]) return c;
    int ci = c % w->nnx, cj = c / w->nnx;
    int best = -1, best_d = 1 << 30;
    for (int r = 1; r <= 20; r++) {
        for (int dj = -r; dj <= r; dj++)
            for (int di = -r; di <= r; di++) {
                if (imax(abs(di), abs(dj)) != r) continue;
                int i = ci + di, j = cj + dj;
                if (i < 0 || j < 0 || i >= w->nnx || j >= w->nny) continue;
                if (!w->blocked[j * w->nnx + i]) {
                    int d = di * di + dj * dj;
                    if (d < best_d) { best = j * w->nnx + i; best_d = d; }
                }
            }
        if (best >= 0) return best;
    }
    return -1;
}

static void heap_push(World *w, float f, int32_t idx) {
    if (w->nheap == w->cheap) { w->cheap = w->cheap ? w->cheap * 2 : 4096; w->heap = realloc(w->heap, sizeof(*w->heap) * (size_t)w->cheap); }
    w->heap[w->nheap].f = f;
    w->heap[w->nheap].idx = idx;
    int i = w->nheap++;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (w->heap[p].f <= w->heap[i].f) break;
        __typeof__(w->heap[0]) t = w->heap[p]; w->heap[p] = w->heap[i]; w->heap[i] = t;
        i = p;
    }
}

static int32_t heap_pop(World *w) {
    int32_t top = w->heap[0].idx;
    int last = w->nheap - 1;
    w->heap[0] = w->heap[last];
    w->nheap = last;
    int i = 0;
    for (;;) {
        int l = 2 * i + 1, r = 2 * i + 2, m = i;
        if (l < last && w->heap[l].f < w->heap[m].f) m = l;
        if (r < last && w->heap[r].f < w->heap[m].f) m = r;
        if (m == i) break;
        __typeof__(w->heap[0]) t = w->heap[m]; w->heap[m] = w->heap[i]; w->heap[i] = t;
        i = m;
    }
    return top;
}

int world_path(World *w, double sx, double sy, double gx, double gy, int max_expand, int32_t **path, int *npath, int *cpath,
               const double *avoid, int navoid) {
    *npath = 0;
    int s = world_nearest_free(w, world_cell(w, sx, sy));
    int g = world_nearest_free(w, world_cell(w, gx, gy));
    if (s < 0 || g < 0) return 0;
    if (s == g) {
        if (*cpath < 1) { *cpath = 64; *path = realloc(*path, sizeof(int32_t) * 64); }
        (*path)[0] = s;
        *npath = 1;
        return 1;
    }
    w->gen++;
    uint32_t gen = w->gen;
    w->nheap = 0;
    int gi = g % w->nnx, gj = g / w->nnx;
#define HEUR(c) ((float)((fmax(fabs((double)((c) % w->nnx - gi)), fabs((double)((c) / w->nnx - gj))) + \
                          0.4142 * fmin(fabs((double)((c) % w->nnx - gi)), fabs((double)((c) / w->nnx - gj)))) * w->ns * 1.15))
    w->g[s] = 0;
    w->came[s] = -1;
    w->stamp[s] = gen;
    heap_push(w, HEUR(s), s);
    int best = s;
    float best_h = HEUR(s);
    int found = 0;
    for (int n = 0; w->nheap > 0 && n < max_expand; n++) {
        int c = heap_pop(w);
        if (w->closed[c] == gen) continue;
        w->closed[c] = gen;
        if (c == g) { found = 1; best = c; break; }
        float hc = HEUR(c);
        if (hc < best_h) { best = c; best_h = hc; }
        int ci = c % w->nnx, cj = c / w->nnx;
        for (int k = 0; k < 8; k++) {
            int i = ci + NBRS[k][0], j = cj + NBRS[k][1];
            if (i < 0 || j < 0 || i >= w->nnx || j >= w->nny) continue;
            int nb = j * w->nnx + i;
            int diag = k >= 4;
            if (w->closed[nb] == gen || !world_can_step(w, c, nb, diag)) continue;
            float step = (float)w->ns;
            if (diag) {
                if (w->blocked[cj * w->nnx + i] || w->blocked[j * w->nnx + ci]) continue;
                step *= 1.4142f;
            }
            float cl = w->clearance[nb];
            if (cl < 4) step *= 1.0f + (4.0f - cl) * 0.15f;
            if (navoid) {
                double nx, ny;
                world_cell_pos(w, nb, &nx, &ny);
                for (int a = 0; a < navoid; a++) {
                    double ddx = nx - avoid[3 * a], ddy = ny - avoid[3 * a + 1];
                    if (ddx * ddx + ddy * ddy < avoid[3 * a + 2] * avoid[3 * a + 2]) step += 20.0f;   /* ~10 cells of detour */
                }
            }
            float ng = w->g[c] + step;
            if (w->stamp[nb] != gen || ng < w->g[nb]) {
                w->g[nb] = ng;
                w->came[nb] = c;
                w->stamp[nb] = gen;
                heap_push(w, ng + HEUR(nb), nb);
            }
        }
    }
#undef HEUR
    int len = 0;
    for (int c = best; c >= 0; c = w->came[c]) {
        if (len == *cpath) { *cpath = *cpath ? *cpath * 2 : 256; *path = realloc(*path, sizeof(int32_t) * (size_t)*cpath); }
        (*path)[len++] = c;
        if (c == s) break;
    }
    for (int i = 0, j = len - 1; i < j; i++, j--) { int32_t t = (*path)[i]; (*path)[i] = (*path)[j]; (*path)[j] = t; }
    *npath = len;
    return found;
}

int world_line_walkable(const World *w, double x0, double y0, double x1, double y1) {
    double dx = x1 - x0, dy = y1 - y0;
    double d = go_hypot(dx, dy);
    int n = (int)(d / 1.0) + 1;
    int prev = world_cell(w, x0, y0);
    for (int k = 1; k <= n; k++) {
        double t = (double)k / (double)n;
        int c = world_cell(w, x0 + dx * t, y0 + dy * t);
        if (c == prev) continue;
        if (w->blocked[c] || w->surf[c] - w->surf[prev] > 3.1f) return 0;
        prev = c;
    }
    return 1;
}

int world_main_near(World *w, int c) {
    if (!w->main_near) {
        int n = w->nnx * w->nny;
        int32_t *near = malloc(sizeof(int32_t) * (size_t)n);
        uint8_t *depth = calloc((size_t)n, 1);
        int32_t *q = malloc(sizeof(int32_t) * (size_t)n);
        int nq = 0;
        for (int i = 0; i < n; i++) {
            near[i] = -1;
            if (!w->blocked[i] && w->comp[i] == w->main_comp) { near[i] = i; q[nq++] = i; }
        }
        for (int h = 0; h < nq; h++) {
            int cc = q[h];
            if (depth[cc] >= 20) continue;
            int ci = cc % w->nnx, cj = cc / w->nnx;
            for (int o = 0; o < 8; o++) {
                int i = ci + NBRS[o][0], j = cj + NBRS[o][1];
                if (i < 0 || j < 0 || i >= w->nnx || j >= w->nny) continue;
                int nb = j * w->nnx + i;
                if (near[nb] < 0) { near[nb] = near[cc]; depth[nb] = depth[cc] + 1; q[nq++] = nb; }
            }
        }
        free(depth);
        free(q);
        w->main_near = near;
    }
    return w->main_near[c];
}

int world_off_main(const World *w, double x, double y) {
    int c = world_nearest_free(w, world_cell(w, x, y));
    return c < 0 || w->comp[c] != w->main_comp;
}

int world_reachable(const World *w, double x, double y) {
    int c = world_cell(w, x, y);
    return !w->blocked[c] && w->comp[c] == w->main_comp;
}

int world_escape_plan(World *w, double x, double y, double *launch, double *land) {
    int s = world_nearest_free(w, world_cell(w, x, y));
    if (s < 0) return 0;
    w->gen++;
    uint32_t gen = w->gen;
    world_queue_reserve(w, 60000 * 9 + 16);
    int32_t *queue = w->queue;
    int nq = 0;
    queue[nq++] = s;
    w->stamp[s] = gen;
    w->g[s] = 0;
    int best = -1;
    double best_s = INFINITY;
    for (int h = 0; h < nq && h < 60000; h++) {
        int c = queue[h];
        int ci = c % w->nnx, cj = c / w->nnx;
        int m = world_main_near(w, c);
        if (m >= 0) {
            double cx, cy, mx, my;
            world_cell_pos(w, c, &cx, &cy);
            world_cell_pos(w, m, &mx, &my);
            double sc = (double)w->g[c] * w->ns + 4 * go_hypot(mx - cx, my - cy);
            if (sc < best_s) { best = c; best_s = sc; }
        }
        for (int k = 0; k < 8; k++) {
            int i = ci + NBRS[k][0], j = cj + NBRS[k][1];
            if (i < 0 || j < 0 || i >= w->nnx || j >= w->nny) continue;
            int nb = j * w->nnx + i;
            if (w->stamp[nb] == gen || !world_can_step(w, c, nb, k >= 4)) continue;
            if (k >= 4 && (w->blocked[cj * w->nnx + i] || w->blocked[j * w->nnx + ci])) continue;
            w->stamp[nb] = gen;
            w->g[nb] = w->g[c] + 1;
            queue[nq++] = nb;
        }
    }
    if (best < 0) return 0;
    double lx, ly, mx, my;
    world_cell_pos(w, best, &lx, &ly);
    world_cell_pos(w, world_main_near(w, best), &mx, &my);
    double d = go_hypot(mx - lx, my - ly);
    if (d > 0.1) {
        double ex = mx + (mx - lx) / d * 5, ey = my + (my - ly) / d * 5;
        if (world_in_bounds(w, ex, ey) && world_reachable(w, ex, ey)) { mx = ex; my = ey; }
    }
    launch[0] = lx; launch[1] = ly;
    land[0] = mx; land[1] = my;
    return 1;
}

double world_exposure(const World *w, double x, double y) {
    double z = world_walk_at(w, x, y) + 7;
    double total = 0;
    for (int k = 0; k < 24; k++) {
        double a = (double)k * M_PI / 12;
        double c = go_cos(a), s = go_sin(a);
        double d = 2.5;
        for (; d < 200; d += 2.5) {
            double px = x + c * d, py = y + s * d;
            if (!world_in_bounds(w, px, py) || solid_raster(w, px, py, z)) break;
        }
        total += d;
    }
    return total / (24 * 200);
}

/* ---------------------------------------------------------- shot rays */

/* 0 = no hit within [0, tmax] */
static int ray_prism(const World *w, int pi, double x, double y, double z, double dx, double dy, double dz, double tmax) {
    const Prism *p = &w->prisms[pi];
    double t0 = 0, t1 = tmax;
#define SLAB(o, d, lo, hi)                                          \
    do {                                                            \
        if (fabs(d) < 1e-9) {                                       \
            if (!((o) >= (lo) && (o) <= (hi))) return 0;            \
        } else {                                                    \
            double a_ = ((lo) - (o)) / (d), b_ = ((hi) - (o)) / (d); \
            if (a_ > b_) { double t_ = a_; a_ = b_; b_ = t_; }      \
            t0 = fmax(t0, a_); t1 = fmin(t1, b_);                    \
            if (!(t0 <= t1)) return 0;                              \
        }                                                           \
    } while (0)
    SLAB(x, dx, p->minx, p->maxx);
    SLAB(y, dy, p->miny, p->maxy);
    SLAB(z, dz, p->base, p->top);
#undef SLAB
    if (z >= p->base && z <= p->top && point_in_poly(p->edges, p->nedges, x, y)) return 1;
    double best = tmax;
    int ok = 0;
    const double *e = p->edges;
    for (int i = 0; i < p->nedges * 4; i += 4) {
        double ex = e[i + 2] - e[i], ey = e[i + 3] - e[i + 1];
        double den = dx * ey - dy * ex;
        if (fabs(den) < 1e-9) continue;
        double px = e[i] - x, py = e[i + 1] - y;
        double t = (px * ey - py * ex) / den, s = (px * dy - py * dx) / den;
        if (t < 0 || t >= best || s < 0 || s > 1) continue;
        double hz = z + dz * t;
        if (hz < p->base || hz > p->top) continue;
        best = t;
        ok = 1;
    }
    if (fabs(dz) > 1e-9) {
        const double capz[2] = {p->top, p->base}, capn[2] = {1, -1};
        for (int k = 0; k < 2; k++) {
            double t = (capz[k] - z) / dz;
            if (t < 0 || t >= best || (capn[k] > 0 && dz > 0) || (capn[k] < 0 && dz < 0)) continue;
            double hx = x + dx * t, hy = y + dy * t;
            if (point_in_poly(e, p->nedges, hx, hy)) { best = t; ok = 1; }
        }
    }
    return ok;
}

static double above_t(const World *w, double x, double y, double z, double dx, double dy, double dz, double t) {
    return z + dz * t - world_ground_at(w, x + dx * t, y + dy * t);
}

static int ray_terrain(const World *w, double x, double y, double z, double x1, double y1, double z1, double tmax) {
    double dx = x1 - x, dy = y1 - y, dz = z1 - z;
    double hl = go_hypot(dx, dy);
    const double blk = 8.0;
    if (above_t(w, x, y, z, dx, dy, dz, 0) < 0) return 1;
    double step = 1.0;
    if (hl > 1e-9) step = 0.5 / hl;
    double t = 0;
    for (int it = 0; t < tmax; it++) {
        if (it > 100000) return 0;
        double px = x + dx * t, py = y + dy * t;
        int bi = (int)floor((px - w->minx) / blk), bj = (int)floor((py - w->miny) / blk);
        int inb = bi >= 0 && bj >= 0 && bi < w->bnx && bj < w->bny;
        double tb = tmax;
        if (hl > 1e-9) {
            double ex = w->minx + (double)bi * blk, ey = w->miny + (double)bj * blk;
            double nx = INFINITY, ny = INFINITY;
            if (dx > 1e-9) nx = (ex + blk - px) / dx;
            else if (dx < -1e-9) nx = (ex - px) / dx;
            if (dy > 1e-9) ny = (ey + blk - py) / dy;
            else if (dy < -1e-9) ny = (ey - py) / dy;
            tb = fmin(t + fmax(0, fmin(nx, ny)) + 1e-6, tmax);
        }
        if (inb && fmin(z + dz * t, z + dz * tb) > (double)w->block_max[bj * w->bnx + bi]) { t = tb; continue; }
        double s = fmin(t + step, tmax);
        if (above_t(w, x, y, z, dx, dy, dz, s) < 0) return 1;   /* Go bisects for the hit point; only the hit matters here */
        if (s >= tmax) break;
        t = s;
    }
    return 0;
}

/* Clear == ShotClear == !anyHit(a, b, trunks=false) */
int world_clear(const World *w, const double *a, const double *b) {
    double x = a[0], y = a[1], z = a[2];
    double dx = b[0] - x, dy = b[1] - y, dz = b[2] - z;
    int32_t seen[64];
    int nseen = 0;
    /* walkCells over the 16 m geo grid */
    double x1 = b[0], y1 = b[1];
    int ci = (int)floor((x - w->gx0) / GEO_CELL), cj = (int)floor((y - w->gy0) / GEO_CELL);
    int ei = (int)floor((x1 - w->gx0) / GEO_CELL), ej = (int)floor((y1 - w->gy0) / GEO_CELL);
    int si = dx > 0 ? 1 : -1, sj = dy > 0 ? 1 : -1;
    double tdx = INFINITY, tdy = INFINITY, tmx = INFINITY, tmy = INFINITY;
    if (fabs(dx) > 1e-9) {
        tdx = GEO_CELL / fabs(dx);
        tmx = dx > 0 ? (((double)(ci + 1)) * GEO_CELL + w->gx0 - x) / fabs(dx) : (x - ((double)ci * GEO_CELL + w->gx0)) / fabs(dx);
    }
    if (fabs(dy) > 1e-9) {
        tdy = GEO_CELL / fabs(dy);
        tmy = dy > 0 ? (((double)(cj + 1)) * GEO_CELL + w->gy0 - y) / fabs(dy) : (y - ((double)cj * GEO_CELL + w->gy0)) / fabs(dy);
    }
    double best = INFINITY;
    for (int k = 0; k < 4096; k++) {
        if (ci >= 0 && cj >= 0 && ci < w->gnx && cj < w->gny) {
            int c = cj * w->gnx + ci;
            for (int q = w->gridp_off[c]; q < w->gridp_off[c + 1]; q++) {
                int32_t pi = w->gridp[q];
                int dup = 0;
                for (int u = 0; u < nseen; u++) if (seen[u] == pi) { dup = 1; break; }
                if (dup) continue;
                if (nseen < 64) seen[nseen++] = pi;
                if (ray_prism(w, pi, x, y, z, dx, dy, dz, 1)) return 0;
            }
            best = 1;
        }
        if (ci == ei && cj == ej) break;
        double t;
        if (tmx < tmy) { t = tmx; tmx += tdx; ci += si; }
        else { t = tmy; tmy += tdy; cj += sj; }
        if (t > 1 || best < t) break;
    }
    return !ray_terrain(w, x, y, z, b[0], b[1], b[2], 1);
}
