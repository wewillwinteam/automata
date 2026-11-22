/* Port of mech/brain.go. Function and field names follow the Go code. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bot.h"
#include "gorand_tab.h"

#define LAT 0.075
#define PI 3.14159265358979323846

static double hyp(double x, double y) { return go_hypot(x, y); }
static double clampd(double v, double lo, double hi) { return fmax(lo, fmin(hi, v)); }
static double angdiff(double a, double b) {
    double d = fmod(a - b, 2 * PI);
    if (d > PI) d -= 2 * PI;
    else if (d < -PI) d += 2 * PI;
    return d;
}

/* ------------------------------------------------- Go's math/rand (v1) */

typedef struct { int tap, feed; int64_t vec[607]; } GoRand;

static int32_t seedrand(int32_t x) {
    const int32_t A = 48271, Q = 44488, R = 3399;
    int32_t hi = x / Q, lo = x % Q;
    x = A * lo - R * hi;
    if (x < 0) x += 2147483647;
    return x;
}

static void gorand_seed(GoRand *r, int64_t seed) {
    r->tap = 0;
    r->feed = 607 - 273;
    seed = seed % 2147483647;
    if (seed < 0) seed += 2147483647;
    if (seed == 0) seed = 89482311;
    int32_t x = (int32_t)seed;
    for (int i = -20; i < 607; i++) {
        x = seedrand(x);
        if (i >= 0) {
            int64_t u = (int64_t)((uint64_t)(int64_t)x << 40);
            x = seedrand(x);
            u ^= (int64_t)((uint64_t)(int64_t)x << 20);
            x = seedrand(x);
            u ^= (int64_t)x;
            u ^= GO_RNG_COOKED[i];
            r->vec[i] = u;
        }
    }
}

static uint64_t gorand_u64(GoRand *r) {
    if (--r->tap < 0) r->tap += 607;
    if (--r->feed < 0) r->feed += 607;
    uint64_t x = (uint64_t)r->vec[r->feed] + (uint64_t)r->vec[r->tap];
    r->vec[r->feed] = (int64_t)x;
    return x;
}

static double gorand_f64(GoRand *r) {
    for (;;) {
        int64_t v = (int64_t)(gorand_u64(r) & 0x7FFFFFFFFFFFFFFFULL);
        double f = (double)v / 9223372036854775808.0;
        if (f != 1) return f;
    }
}

/* -------------------------------------------------------------- state */

typedef struct {
    MechSeen m;
    double seen, hitme, dist;
    int los;
    double closing;
    int engage;
} Track;

typedef struct { int id; int kind; double pos[3]; double hp; double seen; } Known;
typedef struct { int n, cap; Track *a; } TrackTab;
typedef struct { int n, cap; Known *a; } KnownTab;
typedef struct { double x, y, r; } Circle;

typedef struct {
    double RetreatHP, RecoverHP, GunRange, LightRange, LaserRange, EngageDist;
    int FarmNPC, Explore, Ghost;
    double HomeExpo, LootRadius, CrateUntil;
    int UseDrops, Dodge, NoMortar, Peek, LaserFirst, CrowdHide;
    double Vulture, Ambush, LaserSeek;
    int Lethal;
    double Alert, Shy, EarlyHide;
    int Evade;
    double Odds, OddsFlee;
    int Practice;
    double Stick, HPw, LowBonus, MaxTarget;
    int LoseRule;
    double FightLoot, FarmHP;
    int AllFoes;
    double CubeGoal;
    int Econ, LaserUp;
    double EconSafe;
    int Opening, NanoUp;
    double Finish, FinishHP;
    int FinishRush;
    double Pursue;
} Params;

struct Brain {
    World *W;
    Config cfg;
    int slot;
    GoRand rng;
    Params P;
    TrackTab tracks, npcs;
    KnownTab crates, cubes, pickups;
    struct { int id; double until; } *bad;
    int nbad, cbad;
    double now;
    int32_t *path; int npath, cpath;
    double path_goal[2], path_at;
    int path_i;
    double strafe, strafe_until;
    int retreat;
    double cover_at, cover_goal[2];
    int cover_ok;
    double last_hurt, hurt_bearing;
    int loot_id;
    double loot_since;
    Circle hold;
    double hold_start;
    double home_xy[2], home_at;
    int home_set;
    double explore[2], explore_until;
    double stuck_since, unstuck_until, unstuck_dir[2], last_threat_at, last_aim[2];
    const char *mode;
    int panics;
    double esc_launch[2], esc_land[2], esc_at;
    int esc_ok, esc_flying;
    double odds_flee_until;
    Params base;
    int base_set, prac_on, open_on;
    double npc_hurt, npc_bearing;
    int not_practice, start_alive, kills, other_dead;
    int stick_id;
    double stick_at, lose_at;
    /* scratch */
    int32_t *bfs; int cbfs;
    /* experiment switches (brain_set_opts); all off = the Go bot */
    struct {
        int finish_old;       /* finish=old: pre-20bd565 rule (target < 200 HP or < half ours) */
        double oddsflee;      /* oddsflee=P: in the ghost profile, back off when odds() < P */
        int unstuck_mem;      /* unstuck=mem: after getting stuck, route around that spot for 10 s */
    } opt;
    double avoid[3 * 8], avoid_until[8];
    int navoid;
};

static Params default_params(void) {
    Params p;
    memset(&p, 0, sizeof p);
    p.RetreatHP = 0.38; p.RecoverHP = 0.8; p.GunRange = 85; p.LightRange = 28; p.LaserRange = 85; p.EngageDist = 60;
    p.FarmNPC = 1; p.Explore = 0; p.Ghost = 1; p.HomeExpo = 120; p.LootRadius = 90; p.CrateUntil = 150; p.UseDrops = 0;
    p.Dodge = 1; p.NoMortar = 1; p.Peek = 1; p.LaserFirst = 0; p.CrowdHide = 1; p.Practice = 1;
    p.LaserFirst = 1;
    return p;
}

static Params squad_params(Params p) {
    p.Ghost = 0; p.Peek = 0; p.LaserFirst = 1; p.LaserRange = 120; p.LootRadius = 150; p.CrateUntil = 1e9;
    p.Stick = 120; p.HPw = 0.2; p.LowBonus = 100; p.MaxTarget = 240; p.LoseRule = 1; p.FightLoot = 70; p.FarmHP = 0.5;
    p.LaserSeek = 240; p.LaserUp = 1; p.Lethal = 1; p.Finish = 20; p.FightLoot = 40;
    return p;
}

Brain *brain_new(const Hello *h, World *w) {
    Brain *b = calloc(1, sizeof(Brain));
    int64_t seed = (int64_t)h->seed;
    if (seed == 0) seed = (int64_t)h->slot + 1;
    b->W = w;
    b->cfg = h->cfg;
    b->slot = h->slot;
    gorand_seed(&b->rng, seed);
    b->P = default_params();
    b->strafe = 1;
    b->loot_id = -1;
    b->last_hurt = -100;
    b->last_threat_at = -100;
    b->hold_start = -1;
    b->mode = "idle";
    return b;
}

int brain_set_opts(Brain *b, const char *opts) {
    if (!opts) return 0;
    char buf[256];
    snprintf(buf, sizeof buf, "%s", opts);
    for (char *tok = strtok(buf, ","); tok; tok = strtok(NULL, ",")) {
        char *eq = strchr(tok, '=');
        const char *v = eq ? eq + 1 : "1";
        if (eq) *eq = 0;
        if (!strcmp(tok, "finish")) b->opt.finish_old = !strcmp(v, "old");
        else if (!strcmp(tok, "oddsflee")) b->opt.oddsflee = atof(v);
        else if (!strcmp(tok, "unstuck")) b->opt.unstuck_mem = !strcmp(v, "mem");
        else if (!*tok) continue;
        else return -1;
    }
    return 0;
}

void brain_free(Brain *b) {
    if (!b) return;
    free(b->tracks.a); free(b->npcs.a); free(b->crates.a); free(b->cubes.a); free(b->pickups.a);
    free(b->bad); free(b->path); free(b->bfs);
    free(b);
}

const char *brain_mode(const Brain *b) { return b->mode; }

static Track *ttab_get(TrackTab *t, int id) {
    for (int i = 0; i < t->n; i++) if (t->a[i].m.id == id) return &t->a[i];
    return NULL;
}
static Track *ttab_add(TrackTab *t, int id) {
    if (t->n == t->cap) { t->cap = t->cap ? t->cap * 2 : 32; t->a = realloc(t->a, sizeof(Track) * (size_t)t->cap); }
    Track *r = &t->a[t->n++];
    memset(r, 0, sizeof *r);
    r->m.id = id;
    r->hitme = -100;
    return r;
}
static void ttab_del(TrackTab *t, int i) { t->a[i] = t->a[--t->n]; }

static Known *ktab_get(KnownTab *t, int id) {
    for (int i = 0; i < t->n; i++) if (t->a[i].id == id) return &t->a[i];
    return NULL;
}
static Known *ktab_add(KnownTab *t, int id) {
    if (t->n == t->cap) { t->cap = t->cap ? t->cap * 2 : 64; t->a = realloc(t->a, sizeof(Known) * (size_t)t->cap); }
    Known *r = &t->a[t->n++];
    memset(r, 0, sizeof *r);
    r->id = id;
    return r;
}

static int bad_get(const Brain *b, int id, double *until) {
    for (int i = 0; i < b->nbad; i++) if (b->bad[i].id == id) { *until = b->bad[i].until; return 1; }
    return 0;
}
static void bad_set(Brain *b, int id, double until) {
    for (int i = 0; i < b->nbad; i++) if (b->bad[i].id == id) { b->bad[i].until = until; return; }
    if (b->nbad == b->cbad) { b->cbad = b->cbad ? b->cbad * 2 : 16; b->bad = realloc(b->bad, sizeof(*b->bad) * (size_t)b->cbad); }
    b->bad[b->nbad].id = id;
    b->bad[b->nbad].until = until;
    b->nbad++;
}

static int cube_cap(const Brain *b);
static double scale_of(const Brain *b, int cubes) { return 1 + b->cfg.size_per_cube * (double)cubes; }

static void center_of(const Brain *b, const Track *t, double *c) {
    double h = b->cfg.mech.height * scale_of(b, t->m.cubes);
    if (t->m.npc) h = b->cfg.npc_height;
    c[0] = t->m.pos[0]; c[1] = t->m.pos[1]; c[2] = t->m.pos[2] + h * 0.5;
}

static void eye_of(const Brain *b, const Track *t, double *e) {
    e[0] = t->m.pos[0]; e[1] = t->m.pos[1]; e[2] = t->m.pos[2] + b->cfg.mech.eye_height * scale_of(b, t->m.cubes);
}

static WeaponState *has_weapon(Self *me, int kind, int *idx) {
    for (int i = 0; i < me->nweap; i++)
        if (me->weap[i].present && me->weap[i].kind == kind) { if (idx) *idx = i; return &me->weap[i]; }
    if (idx) *idx = -1;
    return NULL;
}

static int holds(const MechSeen *m, int kind) {
    for (int i = 0; i < m->nweap; i++) if (m->weap[i] == kind) return 1;
    return 0;
}

/* ------------------------------------------------------------ observe */

static void upd_known(Brain *b, KnownTab *tab, const Item *items, int n, const Self *me, const double *eye, double now) {
    for (int i = 0; i < n; i++) {
        const Item *it = &items[i];
        Known *k = ktab_get(tab, it->id);
        if (!k) k = ktab_add(tab, it->id);
        k->kind = it->kind;
        memcpy(k->pos, it->pos, sizeof k->pos);
        k->hp = it->hp;
        k->seen = now;
    }
    for (int i = 0; i < tab->n;) {
        Known *k = &tab->a[i];
        if (k->seen != now) {
            double d = hyp(k->pos[0] - me->pos[0], k->pos[1] - me->pos[1]);
            double tgt[3] = {k->pos[0], k->pos[1], k->pos[2] + 1.5};
            if (d < 40 && now - k->seen > 0.35 && world_clear(b->W, eye, tgt)) { tab->a[i] = tab->a[--tab->n]; continue; }
        }
        i++;
    }
}

static void observe(Brain *b, Obs *o) {
    Self *me = &o->self;
    double now = o->t;
    if (me->stats.radius == 0) me->stats = b->cfg.mech;
    double eye[3] = {me->pos[0], me->pos[1], me->pos[2] + me->stats.eye_height};
    double muzzle[3] = {me->pos[0], me->pos[1], me->pos[2] + me->stats.muzzle_height};
    for (int i = 0; i < o->nmechs; i++) {
        MechSeen *m = &o->mechs[i];
        TrackTab *tab = m->npc ? &b->npcs : &b->tracks;
        Track *t = ttab_get(tab, m->id);
        if (!t) t = ttab_add(tab, m->id);
        t->m = *m;
        t->seen = now;
        double dx = m->pos[0] - me->pos[0], dy = m->pos[1] - me->pos[1];
        t->dist = hyp(dx, dy);
        double c[3];
        center_of(b, t, c);
        t->los = world_clear(b->W, muzzle, c);
        if (!t->los && m->is_mech) {
            double h = b->cfg.mech.height * scale_of(b, m->cubes);
            double c2[3] = {m->pos[0], m->pos[1], m->pos[2] + h * 0.85};
            t->los = world_clear(b->W, muzzle, c2);
        }
        if (t->dist > 1) t->closing = -((m->vel[0] - me->vel[0]) * dx + (m->vel[1] - me->vel[1]) * dy) / t->dist;
    }
    for (int i = 0; i < b->tracks.n;) { if (now - b->tracks.a[i].seen > 20) ttab_del(&b->tracks, i); else i++; }
    for (int i = 0; i < b->npcs.n;) { if (now - b->npcs.a[i].seen > 120) ttab_del(&b->npcs, i); else i++; }
    upd_known(b, &b->crates, o->crates, o->ncrates, me, eye, now);
    upd_known(b, &b->cubes, o->cubes, o->ncubes, me, eye, now);
    upd_known(b, &b->pickups, o->pickups, o->npickups, me, eye, now);
    for (int i = 0; i < o->nevents; i++) {
        Event *e = &o->events[i];
        if (e->type == EV_DAMAGED) {
            if (e->weapon_npc && e->has_bearing) { b->npc_hurt = now; b->npc_bearing = e->bearing; }
            if (e->has_by) {
                b->last_hurt = now;
                if (e->has_bearing) b->hurt_bearing = e->bearing;
                Track *t = ttab_get(&b->tracks, e->by);
                if (t) t->hitme = now;
                Track *n = ttab_get(&b->npcs, e->by);
                if (n) n->hitme = now;
            }
        } else if (e->type == EV_KILL) {
            b->kills++;
            for (int k = 0; k < b->tracks.n; k++) if (b->tracks.a[k].m.id == e->target) { ttab_del(&b->tracks, k); break; }
        }
    }
    if (!b->base_set) { b->base = b->P; b->base_set = 1; b->start_alive = o->alive; }
    if (b->start_alive - o->alive - b->kills > 0) b->other_dead++;
    else b->other_dead = 0;
    if (b->other_dead >= 5) b->not_practice = 1;
    int on = b->base.Practice && !b->not_practice && now > 20;
    if (on != b->prac_on) {
        b->prac_on = on;
        b->P = b->base;
        if (on) b->P = squad_params(b->base);
    }
    int open = b->base.Opening && !b->not_practice && !b->prac_on && b->start_alive - o->alive - b->kills <= 0;
    if (open != b->open_on) {
        b->open_on = open;
        b->P = b->base;
        if (open) { b->P.LaserFirst = 1; b->P.LaserSeek = 240; b->P.LootRadius = 150; }
        else if (b->prac_on) b->P = squad_params(b->base);
    }
    Zone *z = &o->zone;
    if (b->hold_start != z->shrink_start) {
        b->hold_start = z->shrink_start;
        b->hold = (Circle){z->center[0], z->center[1], z->radius};
    } else if (now <= z->shrink_start) {
        b->hold = (Circle){z->center[0], z->center[1], z->radius};
    }
}

/* --------------------------------------------------------------- zone */

static Circle zone_at(const Brain *b, const Zone *z, double t) {
    if (t <= z->shrink_start || z->shrink_end <= z->shrink_start) return b->hold;
    double f = clampd((t - z->shrink_start) / (z->shrink_end - z->shrink_start), 0, 1);
    return (Circle){b->hold.x + (z->next_center[0] - b->hold.x) * f, b->hold.y + (z->next_center[1] - b->hold.y) * f,
                    b->hold.r + (z->next_radius - b->hold.r) * f};
}

static Circle cur_zone(const Brain *b, const Zone *z) {
    Circle live = {z->center[0], z->center[1], z->radius};
    if (b->now <= z->shrink_start) return live;
    if (fabs(live.r - b->hold.r) > 0.01 || fabs(live.x - b->hold.x) > 0.01) return live;
    return zone_at(b, z, b->now);
}

static double zone_deadline(const Brain *b, const Zone *z, double x, double y, double pad) {
    if (hyp(x - z->next_center[0], y - z->next_center[1]) <= z->next_radius - pad) return INFINITY;
    for (int k = 0; k <= 24; k++) {
        double t = z->shrink_start + (z->shrink_end - z->shrink_start) * (double)k / 24;
        Circle c = zone_at(b, z, t);
        if (hyp(x - c.x, y - c.y) > c.r - pad) return t;
    }
    return z->shrink_end;
}

static double zone_penalty(const Brain *b, const Zone *z, double x, double y) {
    Circle c = cur_zone(b, z);
    double p = 0;
    double d = hyp(x - c.x, y - c.y) - c.r;
    if (d > -5) p += 150 + 5 * (d + 5);
    double dn = hyp(x - z->next_center[0], y - z->next_center[1]) - z->next_radius;
    if (dn > -5) {
        double soon = clampd(1 - (z->shrink_start - b->now) / 25, 0.1, 1);
        p += soon * (25 + 0.8 * (dn + 5));
    }
    return p;
}

static void zone_goal(const Zone *z, double x, double y, double *gx, double *gy) {
    double margin = clampd(0.3 * z->next_radius, 3, 30);
    if (z->next_radius <= 30) { *gx = z->next_center[0]; *gy = z->next_center[1]; return; }
    double dx = x - z->next_center[0], dy = y - z->next_center[1];
    double d = hyp(dx, dy);
    if (d < 1) { *gx = x; *gy = y; return; }
    double r = fmax(0, z->next_radius - margin - 5);
    if (d <= r) { *gx = x; *gy = y; return; }
    *gx = z->next_center[0] + dx / d * r;
    *gy = z->next_center[1] + dy / d * r;
}

/* ----------------------------------------------------------- upgrades */

static const char *UPG_ORDER[] = {"plating", "armor", "gun_damage", "nanites", "gun_rate", "lightning_efficiency",
                                  "lightning_range", "laser_damage", "servos", "laser_charge", "gun_mag", "gun_velocity",
                                  "gyros", "lightning_chain", "reactor", "capacitor", "mortar_reload", "mortar_splash",
                                  "mortar_velocity", "sensors", "jets", "laser_pierce", NULL};
static const char *UPG_ORDER_LASER[] = {"plating", "armor", "laser_damage", "laser_charge", "nanites", "gun_damage",
                                        "gun_rate", "lightning_efficiency", "lightning_range", "servos", "gun_mag",
                                        "gun_velocity", "gyros", "lightning_chain", "reactor", "capacitor",
                                        "mortar_reload", "mortar_splash", "mortar_velocity", "sensors", "jets",
                                        "laser_pierce", NULL};
static const char *UPG_ORDER_NANO[] = {"plating", "armor", "nanites", "laser_damage", "laser_charge", "gun_damage",
                                       "gun_rate", "lightning_efficiency", "lightning_range", "servos", "gun_mag",
                                       "gun_velocity", "gyros", "lightning_chain", "reactor", "capacitor",
                                       "mortar_reload", "mortar_splash", "mortar_velocity", "sensors", "jets",
                                       "laser_pierce", NULL};

static int pick_upgrade(const Offer *of, int laser_up, int nano_up) {
    const char **order = UPG_ORDER;
    if (laser_up) order = UPG_ORDER_LASER;
    if (nano_up) order = UPG_ORDER_NANO;
    int len = 0;
    while (order[len]) len++;
    int best = 0, best_rank = 1 << 30;
    for (int i = 0; i < of->n; i++) {
        int rank = len;
        for (int r = 0; r < len; r++) if (!strcmp(order[r], of->ids[i])) { rank = r; break; }
        if (rank < best_rank) { best = i; best_rank = rank; }
    }
    return best;
}

static int weapon_rank(int k) { return k == WK_LIGHTNING ? 3 : k == WK_LASER ? 2 : k == WK_MORTAR ? 1 : 0; }

/* ------------------------------------------------------------- travel */

static void unit_to(double x, double y, double tx, double ty, double *mx, double *my) {
    double l = hyp(tx - x, ty - y);
    if (l < 1e-6) { *mx = 0; *my = 0; return; }
    *mx = (tx - x) / l;
    *my = (ty - y) / l;
}

static void travel(Brain *b, const Self *me, double gx, double gy, double *mx, double *my) {
    double x = me->pos[0], y = me->pos[1];
    double d = hyp(gx - x, gy - y);
    World *w = b->W;
    if (d < 0.8) { *mx = 0; *my = 0; return; }
    if (me->pos[2] - world_ground_at(w, x, y) > 3 && w->obst[world_vidx(w, x, y)]) { unit_to(x, y, gx, gy, mx, my); return; }
    if (d < 45 && world_line_walkable(w, x, y, gx, gy)) { unit_to(x, y, gx, gy, mx, my); return; }
    if (b->npath == 0 || hyp(gx - b->path_goal[0], gy - b->path_goal[1]) > 5 || b->now - b->path_at > 2.5) {
        int na = 0;
        double av[3 * 8];
        for (int i = 0; i < b->navoid; i++)
            if (b->avoid_until[i] > b->now) { av[3 * na] = b->avoid[3 * i]; av[3 * na + 1] = b->avoid[3 * i + 1]; av[3 * na + 2] = b->avoid[3 * i + 2]; na++; }
        world_path(w, x, y, gx, gy, 22000, &b->path, &b->npath, &b->cpath, na ? av : NULL, na);
        b->path_goal[0] = gx;
        b->path_goal[1] = gy;
        b->path_at = b->now;
        b->path_i = 0;
    }
    if (b->npath == 0) { unit_to(x, y, gx, gy, mx, my); return; }
    int bi = b->path_i;
    double bd = INFINITY;
    for (int i = b->path_i; i < b->npath && i < b->path_i + 10; i++) {
        double px, py;
        world_cell_pos(w, b->path[i], &px, &py);
        double dd = hyp(px - x, py - y);
        if (dd < bd) { bi = i; bd = dd; }
    }
    b->path_i = bi;
    if (bd > 12) { b->npath = 0; unit_to(x, y, gx, gy, mx, my); return; }
    int top = bi + 14 < b->npath - 1 ? bi + 14 : b->npath - 1;
    for (int i = top; i > bi; i--) {
        double px, py;
        world_cell_pos(w, b->path[i], &px, &py);
        if (world_line_walkable(w, x, y, px, py)) {
            if (i == b->npath - 1 && hyp(px - gx, py - gy) < 4 && world_line_walkable(w, x, y, gx, gy)) { unit_to(x, y, gx, gy, mx, my); return; }
            unit_to(x, y, px, py, mx, my);
            return;
        }
    }
    if (bi + 1 < b->npath) {
        double px, py;
        world_cell_pos(w, b->path[bi + 1], &px, &py);
        unit_to(x, y, px, py, mx, my);
        return;
    }
    unit_to(x, y, gx, gy, mx, my);
}

/* ------------------------------------------------------------- escape */

static int escape(Brain *b, Self *me, double *mx, double *my, int *boost, int *jump) {
    double x = me->pos[0], y = me->pos[1], zf = me->pos[2];
    World *w = b->W;
    if (me->grounded) {
        if (zf - world_ground_at(w, x, y) > 3 && w->obst[world_vidx(w, x, y)]) { b->esc_ok = b->esc_flying = 0; return 0; }
        if (!world_off_main(w, x, y)) { b->esc_ok = b->esc_flying = 0; return 0; }
        b->esc_flying = 0;
        if (!b->esc_ok || b->now - b->esc_at > 6) {
            b->esc_ok = world_escape_plan(w, x, y, b->esc_launch, b->esc_land);
            if (!b->esc_ok) { b->esc_launch[0] = b->esc_launch[1] = b->esc_land[0] = b->esc_land[1] = 0; }
            b->esc_at = b->now;
            if (!b->esc_ok) return 0;
        }
        if (hyp(b->esc_launch[0] - x, b->esc_launch[1] - y) > 2.5) {
            travel(b, me, b->esc_launch[0], b->esc_launch[1], mx, my);
            return 1;
        }
        if (me->energy < fmin(70, me->max_energy * 0.7)) { *mx = 0; *my = 0; return 1; }
        b->esc_flying = 1;
        *jump = 1;
    } else if (!b->esc_flying) {
        return 0;
    }
    double dx = b->esc_land[0] - x, dy = b->esc_land[1] - y;
    double d = hyp(dx, dy);
    if (d < 1.5) return 1;
    *mx = dx / d;
    *my = dy / d;
    double need = -1e9;
    for (double t = 0; t <= fmin(d, 30); t += 1) need = fmax(need, world_top_at(w, x + *mx * t, y + *my * t));
    *boost = zf < need + 2.5 && me->energy > 1;
    return 1;
}

static int want_pickup(const Brain *b, Self *me, int kind) {
    if (kind == WK_GUN) return me->nweap > 0 && !me->weap[0].present;
    if (has_weapon(me, kind, NULL)) return 0;
    if (kind == WK_MORTAR && b->P.NoMortar) return 0;
    int cur = 0;
#define RANK(k) ((b->P.LaserFirst && (k) == WK_LASER) ? 4 : weapon_rank(k))
    if (me->nweap > 1 && me->weap[1].present) cur = RANK(me->weap[1].kind);
    return RANK(kind) > cur;
#undef RANK
}

static void lead(const double *muzzle, const double *tp, const double *tv, double speed, int grounded, double *yaw, double *pitch) {
    if (speed <= 0) speed = 252;
    double t = 0, px = 0, py = 0, pz = 0;
    for (int k = 0; k < 4; k++) {
        double tt = LAT + t;
        px = tp[0] + tv[0] * tt; py = tp[1] + tv[1] * tt; pz = tp[2];
        if (!grounded) pz += tv[2] * tt * 0.5;
        double d = sqrt((px - muzzle[0]) * (px - muzzle[0]) + (py - muzzle[1]) * (py - muzzle[1]) + (pz - muzzle[2]) * (pz - muzzle[2]));
        t = fmax(0, d - 3) / speed;
    }
    *yaw = go_atan2(py - muzzle[1], px - muzzle[0]);
    *pitch = go_atan2(pz - muzzle[2], hyp(px - muzzle[0], py - muzzle[1]));
}

static int mortar_aim(Brain *b, const double *tube, const double *foot, const double *tv, double v, double g, const MechStats *st,
                      double *yaw, double *pitch) {
    if (v <= 0 || g <= 0) return 0;
    double t = 0;
    *yaw = *pitch = 0;
    for (int k = 0; k < 3; k++) {
        double px = foot[0] + tv[0] * (LAT + t), py = foot[1] + tv[1] * (LAT + t);
        double dx = hyp(px - tube[0], py - tube[1]);
        double dz = foot[2] - tube[2];
        double disc = v * v * v * v - g * (g * dx * dx + 2 * dz * v * v);
        if (disc < 0 || dx < 1) return 0;
        *pitch = go_atan((v * v - sqrt(disc)) / (g * dx));
        *yaw = go_atan2(py - tube[1], px - tube[0]);
        t = dx / (v * go_cos(*pitch));
    }
    if (*pitch < st->min_pitch || *pitch > st->max_pitch) return 0;
    double c = go_cos(*pitch), s = go_sin(*pitch);
    for (double tt = 0.1; tt < t * 0.9; tt += 0.1) {
        double d = v * c * tt;
        double px = tube[0] + go_cos(*yaw) * d, py = tube[1] + go_sin(*yaw) * d;
        double pz = tube[2] + v * s * tt - 0.5 * g * tt * tt;
        if (world_solid(b->W, px, py, pz)) return 0;
    }
    return 1;
}

static int dodge_shell(Brain *b, Obs *o, double *mx, double *my) {
    Self *me = &o->self;
    double g = b->cfg.mortar.gravity;
    if (g <= 0) g = 22;
    double rad = b->cfg.mortar.splash_radius + me->stats.radius + 2;
    for (int i = 0; i < o->nproj; i++) {
        Proj *p = &o->proj[i];
        if (!p->mortar) continue;
        double px = p->pos[0], py = p->pos[1], pz = p->pos[2], vz = p->vel[2];
        for (double t = 0; t < 4; t += 0.05) {
            px += p->vel[0] * 0.05;
            py += p->vel[1] * 0.05;
            pz += vz * 0.05;
            vz -= g * 0.05;
            if (world_solid(b->W, px, py, pz)) {
                double d = hyp(px - me->pos[0], py - me->pos[1]);
                if (d < rad && t < 2.5) {
                    double dx = me->pos[0] - px, dy = me->pos[1] - py;
                    if (d < 0.5) { dx = -p->vel[1]; dy = p->vel[0]; d = hyp(dx, dy); }
                    dx /= d;
                    dy /= d;
                    static const double ROT[5] = {0, 0.7, -0.7, 1.4, -1.4};
                    for (int r = 0; r < 5; r++) {
                        double c = go_cos(ROT[r]), s = go_sin(ROT[r]);
                        double ex = dx * c - dy * s, ey = dx * s + dy * c;
                        if (world_line_walkable(b->W, me->pos[0], me->pos[1], me->pos[0] + ex * 6, me->pos[1] + ey * 6)) {
                            *mx = ex;
                            *my = ey;
                            return 1;
                        }
                    }
                }
                break;
            }
        }
    }
    return 0;
}

static double pref_range(const Brain *b, Self *me, const Track *t) {
    WeaponState *light = has_weapon(me, WK_LIGHTNING, NULL), *laser = has_weapon(me, WK_LASER, NULL);
    double r = b->P.GunRange;
    if (light && me->energy > 12) r = b->P.LightRange;
    else if (laser) r = b->P.LaserRange;
    if (t->m.npc) return 58;
    if (!light) {
        if (holds(&t->m, WK_LIGHTNING)) r = fmax(r, 64);
        if (holds(&t->m, WK_MORTAR)) r = fmax(r, 80);
    }
    return r;
}

static void fight_move(Brain *b, Obs *o, Track *tg, Track **seen, int nseen, double want_los, double *omx, double *omy) {
    Self *me = &o->self;
    World *w = b->W;
    double x = me->pos[0], y = me->pos[1];
    double rp = pref_range(b, me, tg);
    double bx = tg->m.pos[0] - x, by = tg->m.pos[1] - y;
    double bd = hyp(bx, by) + 1e-9;
    double ux = bx / bd, uy = by / bd;
    if (b->now >= b->strafe_until) {
        if (gorand_f64(&b->rng) < 0.6) b->strafe = -b->strafe;
        b->strafe_until = b->now + 0.3 + gorand_f64(&b->rng) * 0.7;
    }
    double sx = -uy * b->strafe, sy = ux * b->strafe;
    double tc[3];
    center_of(b, tg, tc);
    double mh = me->stats.muzzle_height;
    Known *cube = NULL;
    for (int i = 0; i < b->cubes.n; i++) {
        Known *c = &b->cubes.a[i];
        double d = hyp(c->pos[0] - x, c->pos[1] - y);
        if (me->cubes < cube_cap(b) && d < 40 && (cube == NULL || d < hyp(cube->pos[0] - x, cube->pos[1] - y)) && fabs(c->pos[2] - me->pos[2]) < 4) cube = c;
    }
    double best = -INFINITY, mx = 0, my = 0;
    int blocked_side = 0;
    int rings = b->P.Peek ? 2 : 1;
    for (int k = -1; k < 16 * rings; k++) {
        double cx = x, cy = y, dx = 0, dy = 0, r = 0;
        if (k >= 0) {
            double a = (double)(k % 16) * PI / 8;
            r = 6 * (double)(1 + k / 16);
            dx = go_cos(a);
            dy = go_sin(a);
            cx = x + dx * r;
            cy = y + dy * r;
            if (!world_line_walkable(w, x, y, cx, cy) || !world_free(w, cx, cy)) {
                if (k < 16 && dx * sx + dy * sy > 0.9) blocked_side = 1;
                continue;
            }
        }
        double cz = world_walk_at(w, cx, cy);
        double d = hyp(tg->m.pos[0] - cx, tg->m.pos[1] - cy);
        double s = -1.1 * fabs(d - rp) - r * 0.4;
        double from[3] = {cx, cy, cz + mh};
        int los = world_clear(w, from, tc);
        if (los) s += 45 * want_los;
        if (b->P.Peek && k >= 0 && k < 16 && (los == (want_los > 0))) {
            for (int q = 0; q < 16; q += 2) {
                double qx = cx + go_cos((double)q * PI / 8) * 7, qy = cy + go_sin((double)q * PI / 8) * 7;
                if (world_free(w, qx, qy)) {
                    double qf[3] = {qx, qy, world_walk_at(w, qx, qy) + mh};
                    if (world_clear(w, qf, tc) != los) { s += 8; break; }
                }
            }
        }
        double body[3] = {cx, cy, cz + me->stats.height * 0.6};
        for (int i = 0; i < nseen; i++) {
            Track *e = seen[i];
            if (e == tg || hyp(e->m.pos[0] - cx, e->m.pos[1] - cy) > 170) continue;
            double ey[3];
            eye_of(b, e, ey);
            if (world_clear(w, ey, body)) s -= 30;
        }
        s -= zone_penalty(b, &o->zone, cx, cy);
        if (cube) s += 1.5 * (hyp(cube->pos[0] - x, cube->pos[1] - y) - hyp(cube->pos[0] - cx, cube->pos[1] - cy));
        double cl = (double)w->clearance[world_cell(w, cx, cy)];
        if (cl < 4) s -= (4 - cl) * 2.5;
        if (want_los > 0) s += 9 * (dx * sx + dy * sy);
        if (k < 0) {
            s -= 8;
            if (want_los < 0 && !los) s += 14;
        }
        if (s > best) { best = s; mx = dx; my = dy; }
    }
    if (blocked_side) { b->strafe = -b->strafe; b->strafe_until = b->now + 0.5; }
    *omx = mx;
    *omy = my;
}

static void travel_cover(Brain *b, const Self *me, double *mx, double *my) {
    if (hyp(b->cover_goal[0] - me->pos[0], b->cover_goal[1] - me->pos[1]) < 1.5) { *mx = 0; *my = 0; return; }
    travel(b, me, b->cover_goal[0], b->cover_goal[1], mx, my);
}

typedef struct { Track *t; double d; } ThreatD;
static int cmp_threat(const void *a, const void *b) {
    double x = ((const ThreatD *)a)->d, y = ((const ThreatD *)b)->d;
    return x < y ? -1 : x > y;
}

static int cover_hidden(Brain *b, const Self *me, Track **threats, int nt, double cx, double cy) {
    double cz = world_walk_at(b->W, cx, cy);
    for (int i = 0; i < nt; i++) {
        double e[3];
        eye_of(b, threats[i], e);
        double p1[3] = {cx, cy, cz + me->stats.height}, p2[3] = {cx, cy, cz + me->stats.height * 0.5};
        if (world_clear(b->W, e, p1) || world_clear(b->W, e, p2)) return 0;
    }
    return 1;
}

static int cover_move(Brain *b, Obs *o, Track **seen, int nseen, double *mx, double *my) {
    Self *me = &o->self;
    double x = me->pos[0], y = me->pos[1];
    ThreatD td[MAXSEEN + 8];
    int nt = 0;
    for (int i = 0; i < nseen && nt < MAXSEEN + 8; i++) {
        double d = hyp(seen[i]->m.pos[0] - x, seen[i]->m.pos[1] - y);
        if (d < 230) { td[nt].t = seen[i]; td[nt].d = d; nt++; }
    }
    if (nt == 0) return 0;
    if (nt > 4) { qsort(td, (size_t)nt, sizeof td[0], cmp_threat); nt = 4; }
    Track *threats[MAXSEEN + 8];
    for (int i = 0; i < nt; i++) threats[i] = td[i].t;
    if (b->cover_ok && b->now - b->cover_at < 0.4) { travel_cover(b, me, mx, my); return 1; }
    b->cover_at = b->now;
    if (cover_hidden(b, me, threats, nt, x, y) && zone_penalty(b, &o->zone, x, y) == 0) {
        b->cover_ok = 1;
        b->cover_goal[0] = x;
        b->cover_goal[1] = y;
        *mx = 0;
        *my = 0;
        return 1;
    }
    World *w = b->W;
    int s = world_nearest_free(w, world_cell(w, x, y));
    if (s < 0) return 0;
    w->gen++;
    uint32_t gen = w->gen;
    int nq = 0;
#define QPUSH(v) do { if (nq == b->cbfs) { b->cbfs = b->cbfs ? b->cbfs * 2 : 8192; b->bfs = realloc(b->bfs, sizeof(int32_t) * (size_t)b->cbfs); } b->bfs[nq++] = (v); } while (0)
    QPUSH(s);
    w->stamp[s] = gen;
    w->g[s] = 0;
    int best_c = -1;
    double best_s = INFINITY;
    int checked = 0;
    static const int NB[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    for (int h = 0; h < nq && checked < 350; h++) {
        int c = b->bfs[h];
        int ci = c % w->nnx, cj = c / w->nnx;
        double steps = (double)w->g[c];
        if (steps > 50 || (best_c >= 0 && steps * 2 > best_s + 30)) break;
        if ((ci + cj) % 2 == 0 && steps > 0) {
            double cx, cy;
            world_cell_pos(w, c, &cx, &cy);
            checked++;
            if (cover_hidden(b, me, threats, nt, cx, cy)) {
                double sc = steps * 2 + zone_penalty(b, &o->zone, cx, cy);
                double near = INFINITY;
                for (int i = 0; i < nt; i++) near = fmin(near, hyp(threats[i]->m.pos[0] - cx, threats[i]->m.pos[1] - cy));
                sc -= fmin(near, 80) * 0.25;
                if (sc < best_s) { best_c = c; best_s = sc; }
            }
        }
        for (int k = 0; k < 8; k++) {
            int i = ci + NB[k][0], j = cj + NB[k][1];
            if (i < 0 || j < 0 || i >= w->nnx || j >= w->nny) continue;
            int nb = j * w->nnx + i;
            if (w->stamp[nb] == gen || !world_can_step(w, c, nb, k >= 4)) continue;
            if (k >= 4 && (w->blocked[cj * w->nnx + i] || w->blocked[j * w->nnx + ci])) continue;
            w->stamp[nb] = gen;
            w->g[nb] = w->g[c] + 1;
            QPUSH(nb);
        }
    }
#undef QPUSH
    if (best_c < 0) { b->cover_ok = 0; return 0; }
    double gx, gy;
    world_cell_pos(w, best_c, &gx, &gy);
    b->cover_ok = 1;
    b->cover_goal[0] = gx;
    b->cover_goal[1] = gy;
    travel_cover(b, me, mx, my);
    return 1;
}

static int cube_cap(const Brain *b) {
    int c = b->cfg.stat_cap;
    int g = (int)b->P.CubeGoal;
    return g > c ? g : c;
}

/* ---------------------------------------------------------------- loot */

enum { LK_PICKUP, LK_CUBE, LK_CRATE, LK_NPC, LK_DROP };
static const char *LK_MODE[] = {"loot:pickup", "loot:cube", "loot:crate", "loot:npc", "loot:drop"};
typedef struct { int kind, id; double pos[3], value; } LootChoice;

typedef struct {
    Brain *b; Self *me; Zone *z; Circle cur; double x, y, now, best_u; int econ, near_only, have; LootChoice best;
} LootCtx;

static void consider(LootCtx *L, int kind, int id, const double *pos, double value) {
    Brain *b = L->b;
    if (L->near_only && hyp(pos[0] - L->x, pos[1] - L->y) > 60) return;
    double until;
    if (bad_get(b, id, &until) && until > L->now) return;
    double d = hyp(pos[0] - L->x, pos[1] - L->y);
    if (d > 260) return;
    if (hyp(pos[0] - L->cur.x, pos[1] - L->cur.y) > L->cur.r - 6) return;
    double dl = zone_deadline(b, L->z, pos[0], pos[1], 6);
    if (L->now + d * 1.4 / L->me->stats.walk_speed + 5 > dl) return;
    if (pos[2] - world_ground_at(b->W, pos[0], pos[1]) > 3.2) return;
    int nf = world_nearest_free(b->W, world_cell(b->W, pos[0], pos[1]));
    if (nf < 0 || b->W->comp[nf] != b->W->main_comp) return;
    double u = value / (d + 18);
    if (id == b->loot_id) u *= 1.35;
    if (L->econ) {
        u = value - 0.6 * d;
        if (id == b->loot_id) u += 30;
    }
    if (u > L->best_u) {
        L->best_u = u;
        L->have = 1;
        L->best.kind = kind;
        L->best.id = id;
        memcpy(L->best.pos, pos, sizeof L->best.pos);
        L->best.value = value;
    }
}

static int choose_loot(Brain *b, Obs *o, double hp_frac, int danger, LootChoice *out) {
    Self *me = &o->self;
    LootCtx L;
    memset(&L, 0, sizeof L);
    L.b = b; L.me = me; L.z = &o->zone; L.x = me->pos[0]; L.y = me->pos[1]; L.now = b->now;
    L.cur = cur_zone(b, &o->zone);
    L.best_u = 0;
    L.econ = b->P.Econ;
    double x = L.x, y = L.y, now = L.now;
    if (L.econ) {
        int mech_near = 0;
        for (int i = 0; i < b->tracks.n; i++) if (b->tracks.a[i].seen == now) mech_near = 1;
        int hurt = hp_frac <= 0.45;
        for (int i = 0; i < b->tracks.n; i++) if (b->P.EconSafe > 0 && now - b->tracks.a[i].seen < b->P.EconSafe) L.near_only = 1;
        for (int i = 0; i < b->pickups.n; i++) {
            Known *p = &b->pickups.a[i];
            if (!want_pickup(b, me, p->kind) || hurt) continue;
            double v = 150;
            if (me->nweap > 1 && !me->weap[1].present) v = 260;
            if (p->kind == WK_LASER) v = fmax(v, 270) + 20;
            consider(&L, LK_PICKUP, p->id, p->pos, v);
        }
        for (int i = 0; i < b->cubes.n; i++) {
            Known *c = &b->cubes.a[i];
            if (!hurt || hyp(c->pos[0] - x, c->pos[1] - y) < 35) consider(&L, LK_CUBE, c->id, c->pos, 110);
        }
        for (int i = 0; i < b->crates.n; i++) {
            Known *c = &b->crates.a[i];
            if (!hurt || hyp(c->pos[0] - x, c->pos[1] - y) < 35) consider(&L, LK_CRATE, c->id, c->pos, 105);
        }
        if (b->P.FarmNPC && hp_frac > 0.6 && !mech_near) {
            for (int i = 0; i < b->npcs.n; i++) {
                Track *n = &b->npcs.a[i];
                double v = now - n->seen > 12 ? 155 : 170;
                consider(&L, LK_NPC, n->m.id, n->m.pos, v);
            }
        }
        for (int i = 0; i < o->ndrops; i++) {
            Drop *d = &o->drops[i];
            if (b->P.UseDrops && !hurt && hyp(d->pos[0] - x, d->pos[1] - y) / me->stats.walk_speed < d->lands_in + 25)
                consider(&L, LK_DROP, d->id, d->pos, 170);
        }
        if (!L.have) { b->loot_id = -1; return 0; }
        if (L.best.id != b->loot_id) { b->loot_id = L.best.id; b->loot_since = now; }
        else {
            double lim = 30.0;
            if (now - b->loot_since > lim || (L.best.kind == LK_CRATE && now - b->loot_since > lim - 6 && hyp(L.best.pos[0] - x, L.best.pos[1] - y) < 50)) {
                bad_set(b, L.best.id, now + 60);
                b->loot_id = -1;
                return 0;
            }
        }
        *out = L.best;
        return 1;
    }
    for (int i = 0; i < b->pickups.n; i++) {
        Known *p = &b->pickups.a[i];
        double rad = b->P.LootRadius;
        int seek = p->kind == WK_LASER && b->P.LaserSeek > rad;
        if (seek) rad = b->P.LaserSeek;
        if (want_pickup(b, me, p->kind) && hyp(p->pos[0] - x, p->pos[1] - y) < rad) {
            double v = 60;
            if (me->nweap > 1 && !me->weap[1].present) v = 130;
            if (seek) v = fmax(v, 150);
            consider(&L, LK_PICKUP, p->id, p->pos, v);
        }
    }
    double cube_v = 75.0;
    int capped = me->cubes >= cube_cap(b);   /* a663e54 */
    for (int i = 0; i < b->cubes.n; i++) {
        Known *c = &b->cubes.a[i];
        if (!capped && hyp(c->pos[0] - x, c->pos[1] - y) < fmax(60, b->P.LootRadius * 0.7)) consider(&L, LK_CUBE, c->id, c->pos, cube_v);
    }
    double farm_hp = b->P.FarmHP > 0 ? b->P.FarmHP : 0.7;
    if (hp_frac > farm_hp && !danger && now < b->P.CrateUntil) {
        int no_second = me->nweap > 1 && !me->weap[1].present;
        for (int i = 0; i < b->crates.n; i++) {
            Known *c = &b->crates.a[i];
            if ((!capped || no_second) && hyp(c->pos[0] - x, c->pos[1] - y) < b->P.LootRadius) consider(&L, LK_CRATE, c->id, c->pos, cube_v * 0.6);
        }
        if (b->P.FarmNPC && me->cubes < cube_cap(b)) {
            for (int i = 0; i < b->npcs.n; i++) {
                Track *n = &b->npcs.a[i];
                if (now - n->seen < 12 && hyp(n->m.pos[0] - x, n->m.pos[1] - y) < b->P.LootRadius) consider(&L, LK_NPC, n->m.id, n->m.pos, 85);
            }
        }
    }
    for (int i = 0; i < o->ndrops; i++) {
        Drop *d = &o->drops[i];
        double dist = hyp(d->pos[0] - x, d->pos[1] - y);
        if (b->P.UseDrops && dist / me->stats.walk_speed < d->lands_in + 25 && !danger) consider(&L, LK_DROP, d->id, d->pos, 260);
    }
    if (!L.have) { b->loot_id = -1; return 0; }
    if (L.best.id != b->loot_id) { b->loot_id = L.best.id; b->loot_since = now; }
    else if (now - b->loot_since > 30) { bad_set(b, L.best.id, now + 60); b->loot_id = -1; return 0; }
    *out = L.best;
    return 1;
}

static void explore_move(Brain *b, Obs *o, double *mx, double *my) {
    Self *me = &o->self;
    double x = me->pos[0], y = me->pos[1];
    Zone *z = &o->zone;
    if (b->now > b->explore_until || hyp(b->explore[0] - x, b->explore[1] - y) < 6) {
        for (int t = 0; t < 20; t++) {
            double a = gorand_f64(&b->rng) * 2 * PI;
            double r = sqrt(gorand_f64(&b->rng)) * z->next_radius * 0.8;
            double px = z->next_center[0] + go_cos(a) * r, py = z->next_center[1] + go_sin(a) * r;
            if (!world_in_bounds(b->W, px, py) || !world_free(b->W, px, py)) continue;
            double d = hyp(px - x, py - y);
            if (d < 50 || d > 220) continue;
            b->explore[0] = px;
            b->explore[1] = py;
            b->explore_until = b->now + 30;
            break;
        }
    }
    if (b->now > b->explore_until) { *mx = 0; *my = 0; return; }
    travel(b, me, b->explore[0], b->explore[1], mx, my);
}

static double home_cost(Brain *b, Obs *o, double px, double py) {
    Zone *z = &o->zone;
    double crowd = 0;
    for (int i = 0; i < b->tracks.n; i++) {
        Track *t = &b->tracks.a[i];
        if (b->now - t->seen < 20) crowd += fmax(0, 80 - hyp(t->m.pos[0] - px, t->m.pos[1] - py)) / 80;
    }
    for (int i = 0; i < o->ndrops; i++) crowd += fmax(0, 60 - hyp(o->drops[i].pos[0] - px, o->drops[i].pos[1] - py)) / 60;
    double edge = hyp(px - z->next_center[0], py - z->next_center[1]) / fmax(30, z->next_radius);
    return b->P.HomeExpo * world_exposure(b->W, px, py) + 0.25 * hyp(px - o->self.pos[0], py - o->self.pos[1]) + 40 * crowd + 10 * edge;
}

static int home(Brain *b, Obs *o, double *hx, double *hy) {
    Zone *z = &o->zone;
    int out = !b->home_set || hyp(b->home_xy[0] - z->next_center[0], b->home_xy[1] - z->next_center[1]) > z->next_radius * 0.8;
    if (out || b->now - b->home_at > 12) {
        b->home_at = b->now;
        double best[2] = {0, 0}, best_c = INFINITY;
        for (int k = 0; k < 40; k++) {
            double a = gorand_f64(&b->rng) * 2 * PI;
            double r = sqrt(gorand_f64(&b->rng)) * z->next_radius * 0.7;
            double px = z->next_center[0] + go_cos(a) * r, py = z->next_center[1] + go_sin(a) * r;
            if (!world_in_bounds(b->W, px, py) || !world_reachable(b->W, px, py)) continue;
            double c = home_cost(b, o, px, py);
            if (c < best_c) { best[0] = px; best[1] = py; best_c = c; }
        }
        if (!isinf(best_c) && (out || best_c < home_cost(b, o, b->home_xy[0], b->home_xy[1]) - 15)) {
            b->home_xy[0] = best[0];
            b->home_xy[1] = best[1];
            b->home_set = 1;
        }
    }
    *hx = b->home_xy[0];
    *hy = b->home_xy[1];
    return b->home_set;
}

static int hide_from_bearing(Brain *b, Obs *o, Track **seen, int nseen, double *mx, double *my) {
    Self *me = &o->self;
    Track ghost;
    memset(&ghost, 0, sizeof ghost);
    ghost.m.pos[0] = me->pos[0] + go_cos(b->hurt_bearing) * 80;
    ghost.m.pos[1] = me->pos[1] + go_sin(b->hurt_bearing) * 80;
    ghost.m.pos[2] = me->pos[2];
    Track *list[MAXSEEN + 8];
    int n = 0;
    list[n++] = &ghost;
    for (int i = 0; i < nseen && n < MAXSEEN + 8; i++) list[n++] = seen[i];
    return cover_move(b, o, list, n, mx, my);
}

static int dodge(Brain *b, Obs *o, double *odx, double *ody) {
    Self *me = &o->self;
    double cx = me->pos[0], cy = me->pos[1], cz = me->pos[2] + me->stats.height * 0.56;
    double reach = me->stats.radius + 1.1;
    double dx = 0, dy = 0;
    int laser = 0;
    for (int i = 0; i < o->nproj; i++) {
        Proj *p = &o->proj[i];
        if (p->mortar) continue;
        double v2 = p->vel[0] * p->vel[0] + p->vel[1] * p->vel[1] + p->vel[2] * p->vel[2];
        if (v2 < 1) continue;
        double tc = ((cx - p->pos[0]) * p->vel[0] + (cy - p->pos[1]) * p->vel[1] + (cz - p->pos[2]) * p->vel[2]) / v2;
        if (tc <= 0 || tc > 0.7) continue;
        double mxx = p->pos[0] + p->vel[0] * tc - cx, myy = p->pos[1] + p->vel[1] * tc - cy;
        if (hyp(mxx, myy) < reach) {
            double vh = hyp(p->vel[0], p->vel[1]);
            if (vh < 1) continue;
            double px = -p->vel[1] / vh, py = p->vel[0] / vh;
            if (px * mxx + py * myy > 0) { px = -px; py = -py; }
            dx += px;
            dy += py;
        }
    }
    for (int i = 0; i < o->nmechs; i++) {
        MechSeen *m = &o->mechs[i];
        if (!m->is_mech || !m->charging) continue;
        double to_me = go_atan2(cy - m->pos[1], cx - m->pos[0]);
        if (fabs(angdiff(m->aim[0], to_me)) > 0.15) continue;
        dx += go_cos(to_me + PI / 2 * b->strafe) * 1.5;
        dy += go_sin(to_me + PI / 2 * b->strafe) * 1.5;
        laser = 1;
    }
    double l = hyp(dx, dy);
    if (l > 0.01) { *odx = dx / l; *ody = dy / l; return laser ? 2 : 1; }
    *odx = 0;
    *ody = 0;
    return 0;
}

static double reach_of(Self *me) {
    double r = 0;
    for (int i = 0; i < me->nweap; i++) {
        WeaponState *w = &me->weap[i];
        if (!w->present) continue;
        if (w->kind == WK_GUN) r = fmax(r, fmin(w->stats.range - 10, 130));
        else if (w->kind == WK_LASER) r = fmax(r, w->stats.range - 5);
        else if (w->kind == WK_LIGHTNING) r = fmax(r, w->stats.range);
    }
    return r;
}

static double odds(Brain *b, Self *me, Track *t, double now, int opening) {
    int my_w = 0, th_w = t->m.nweap;
    for (int i = 0; i < me->nweap; i++) if (me->weap[i].present) my_w++;
    if (th_w == 0) th_w = 1;
#define CAPC(c) fmin((double)(c), 10)
    double busy_me = 0, busy_them = 0;
    for (int i = 0; i < b->tracks.n; i++) {
        Track *u = &b->tracks.a[i];
        if (u->m.id == t->m.id || now - u->seen > 2.5) continue;
        if (now - u->hitme < 6) busy_me = 1;
        double br = go_atan2(u->m.pos[1] - t->m.pos[1], u->m.pos[0] - t->m.pos[0]);
        if (fabs(angdiff(t->m.aim[0], br)) < 0.2 && hyp(u->m.pos[0] - t->m.pos[0], u->m.pos[1] - t->m.pos[1]) < 160) busy_them = 1;
    }
    double z = 1.5093 * log(fmax(me->hp, 1) / fmax(t->m.hp, 1)) + 0.7049 * (CAPC(me->cubes) - CAPC(t->m.cubes)) / 10 +
               0.9583 * (double)(my_w - th_w) - 0.5592 * (busy_me - busy_them);
#undef CAPC
    if (opening) {
        z += 0.0562 + 0.7475 * fmin(t->dist, 200) / 100;
        double to_me = go_atan2(me->pos[1] - t->m.pos[1], me->pos[0] - t->m.pos[0]);
        if (fabs(angdiff(t->m.aim[0], to_me)) > 1.75) z += 0.8235;
    }
    return 1 / (1 + exp(-z));
}

/* --------------------------------------------------------------- step */

static void step(Brain *b, Obs *o, Action *act) {
    Self *me = &o->self;
    b->now = o->t;
    double now = o->t;
    observe(b, o);
    MechStats *st = &me->stats;
    World *W = b->W;
    double x = me->pos[0], y = me->pos[1], zf = me->pos[2];
    Zone *z = &o->zone;
    double hp_frac = me->hp / fmax(1, me->max_hp);

    if (o->has_offer && o->offer.n > 0) {
        act->has_pick = 1;
        act->pick = pick_upgrade(&o->offer, b->P.LaserUp, b->P.NanoUp);
    }
    WeaponState *gun = has_weapon(me, WK_GUN, NULL), *light = has_weapon(me, WK_LIGHTNING, NULL),
                *laser = has_weapon(me, WK_LASER, NULL);
    /* 2fe5f04: all-in phase from the 25 m circle, not 70 m */
    int late = z->next_radius <= 25 || o->alive <= 3;

    Track *seen[MAXSEEN * 2];
    int nseen = 0;
    Track *target = NULL, *hidden = NULL, *shy = NULL;
    double best_score = INFINITY;
    double max_t = 215.0, hpw = 60.0 / 1400;
    if (b->P.MaxTarget > 0) max_t = b->P.MaxTarget;
    if (b->P.HPw > 0) hpw = b->P.HPw;
    for (int ti = 0; ti < b->tracks.n; ti++) {
        Track *t = &b->tracks.a[ti];
        if (now - t->seen > 2.5) continue;
        t->engage = 0;
        if (nseen < MAXSEEN * 2) seen[nseen++] = t;
        if (t->seen != now) { t->los = 0; continue; }
        double to_me = go_atan2(y - t->m.pos[1], x - t->m.pos[0]);
        int aimed = fabs(angdiff(t->m.aim[0], to_me)) < 0.22;
        if (b->P.Ghost) {
            t->engage = t->dist < b->P.EngageDist || now - t->hitme < 4 || (aimed && t->m.charging && t->dist < 120) ||
                        (t->m.hp < 350 && t->dist < 120) || o->alive <= 2;
            double off = fabs(angdiff(t->m.aim[0], to_me));
            if (b->P.Vulture > 0 && hp_frac > 0.75 && t->m.hp < b->P.Vulture * t->m.max_hp && off > 0.6 && t->dist < 165) t->engage = 1;
            if (b->P.Alert > 0 && t->los && t->dist < b->P.Alert && (t->closing > 3 || aimed)) t->engage = 1;
            if (b->P.Shy > 0 && t->los && t->dist < b->P.Shy && t->dist >= b->P.EngageDist && (t->closing > 3 || aimed) && !t->engage &&
                (shy == NULL || t->dist < shy->dist))
                shy = t;
            if (b->P.Ambush > 0 && hp_frac > 0.85 && off > 1.75 && t->dist < b->P.Ambush) t->engage = 1;
            if (b->P.Odds > 0 && !t->engage && t->los && hp_frac > 0.5 && t->dist < reach_of(me) && odds(b, me, t, now, 1) >= b->P.Odds) t->engage = 1;
        } else if (b->P.AllFoes && t->dist < max_t) {
            t->engage = 1;
        } else if (t->dist < b->P.EngageDist || now - t->hitme < 6 || late) {
            t->engage = 1;
        } else if (t->closing > 3 && t->dist < 180) {
            t->engage = 1;
        } else if (t->m.hp < 0.4 * t->m.max_hp && t->dist < 160) {
            t->engage = 1;
        } else if (aimed && t->dist < 190) {
            t->engage = 1;
        }
        if (t->engage && !t->los && t->dist < max_t && (hidden == NULL || t->dist < hidden->dist)) hidden = t;
        if (!t->engage || !t->los || t->dist > max_t) continue;
        double s = t->dist + hpw * t->m.hp;
        if (b->P.LowBonus > 0 && t->m.hp <= 0.3 * t->m.max_hp) s -= b->P.LowBonus;
        if (b->P.Stick > 0 && t->m.id == b->stick_id && now - b->stick_at < 2) s -= b->P.Stick;
        if (now - t->hitme < 4) s -= 40;
        if (t->closing > 3) s -= 15;
        if (s < best_score) { best_score = s; target = t; }
    }
    if (target) { b->stick_id = target->m.id; b->stick_at = now; }
    Track *foe = target;
    if (!foe && b->P.Peek) foe = hidden;
    if (!foe && b->P.Pursue > 0 && hidden && hidden->m.hp < b->P.Pursue * hidden->m.max_hp) foe = hidden;
    int danger = 0, crowd = 0;
    for (int i = 0; i < nseen; i++) {
        Track *t = seen[i];
        if (t->engage || now - t->hitme < 4) danger = 1;
        if (t->seen == now && (t->engage || (t->los && t->dist < 150))) crowd++;
    }
    if (danger || now - b->last_hurt < 3) b->last_threat_at = now;

    /* 20bd565: finishing needs the target within our burst, and <= 2 mechs on us */
    int finishing = 0;
    if (b->opt.finish_old) {
        finishing = target && target->dist < 120 && (target->m.hp < 200 || target->m.hp < me->hp * 0.5);
    } else if (target && target->dist < 120 && crowd <= 2) {
        int cc = me->cubes < b->cfg.stat_cap ? me->cubes : b->cfg.stat_cap;
        double mult = 1 + b->cfg.damage_per_cube * (double)cc;
        double burst = 0.0;
        if (gun && gun->stats.cooldown > 0) burst += 1.5 * 0.5 * gun->stats.damage / gun->stats.cooldown;
        if (laser && laser->cooldown < 1 && target->dist < laser->stats.range) burst += 0.8 * laser->stats.damage;
        finishing = target->m.hp < fmax(200, burst * mult);
    }
    WeaponState *my_laser = laser;
    int outgunned = target && holds(&target->m, WK_LASER) && !my_laser && target->dist > 55 && hp_frac < 0.8;
    int swamped = b->P.CrowdHide && (crowd >= 3 || (crowd == 2 && hp_frac < 0.65) || outgunned);
    double odds_flee = b->P.OddsFlee;
    if (b->opt.oddsflee > 0 && !b->prac_on) odds_flee = b->opt.oddsflee;   /* ghost profile only: the final's FFA */
    if (odds_flee > 0 && target && !finishing && odds(b, me, target, now, 0) < odds_flee) b->odds_flee_until = now + 5;
    if (now < b->odds_flee_until && !finishing) swamped = 1;
    if (!b->retreat && ((hp_frac < b->P.RetreatHP && now - b->last_threat_at < 2) || swamped)) { b->retreat = 1; b->cover_at = -1; }
    if (b->retreat && !swamped && (hp_frac > b->P.RecoverHP || (now - b->last_threat_at > 7 && hp_frac > 0.55))) b->retreat = 0;
    if (b->P.LoseRule) {
        int near = 0;
        for (int i = 0; i < nseen; i++) if (seen[i]->seen == now && seen[i]->dist < 160) near++;
        int losing = (near >= 2 && hp_frac < 0.55) || (near >= 3 && hp_frac < 0.8);
        if (target) {
            double tf = target->m.hp / fmax(1, target->m.max_hp);
            if (hp_frac < 0.35 && tf > hp_frac + 0.1) losing = 1;
            if (tf < 0.25 && hp_frac > 0.2 && near <= 2) losing = 0;
        }
        if (losing) b->lose_at = now;
        b->retreat = now - b->lose_at < 1.5 && b->lose_at > 0;
    }
    int exposed = 0;
    for (int i = 0; i < nseen; i++) if (seen[i]->seen == now && seen[i]->los && seen[i]->dist < 230) exposed = 1;
    for (int i = 0; i < nseen; i++) if (seen[i]->seen == now && seen[i]->dist < 45) exposed = 0;

    double mx = 0, my = 0;
    int boost = 0;
    const char *mode = "idle";
    Track *farm = NULL;
    int have_farm_pos = 0;
    double farm_pos[3] = {0, 0, 0};

    double zgx, zgy;
    zone_goal(z, x, y, &zgx, &zgy);
    double zdist = hyp(zgx - x, zgy - y);
    double t_need = zdist * 1.45 / st->walk_speed + 4;
    double deadline = zone_deadline(b, z, x, y, 8);
    int zone_must = !me->in_zone || now + t_need + 3 >= deadline;
    int zone_should = now + t_need + 14 >= deadline;

    int esc_jump = 0;
    if (escape(b, me, &mx, &my, &boost, &esc_jump)) {
        mode = "escape";
    } else if (now < b->unstuck_until) {
        mx = b->unstuck_dir[0]; my = b->unstuck_dir[1];
        mode = "unstuck";
    } else if (dodge_shell(b, o, &mx, &my)) {
        boost = me->energy > 15;
        mode = "dodge";
    } else if (zone_must && zdist > 2) {
        travel(b, me, zgx, zgy, &mx, &my);
        boost = (!me->in_zone || now + t_need >= deadline) && me->energy > 5;
        mode = "zone";
    } else if (b->retreat && !finishing && !late) {
        if (cover_move(b, o, seen, nseen, &mx, &my)) {
            boost = me->energy > 10 && (mx != 0 || my != 0);
            mode = "cover";
        } else if (target) {
            fight_move(b, o, target, seen, nseen, 1, &mx, &my);
            mode = "cornered";
        } else {
            travel(b, me, zgx, zgy, &mx, &my);
            if (b->npc_hurt > 0 && now - b->npc_hurt < 3 && mx == 0 && my == 0) {
                double side = fmod(now, 3) > 1.5 ? -1.0 : 1.0;
                mx = -go_sin(b->npc_bearing) * side;
                my = go_cos(b->npc_bearing) * side;
            }
            mode = "flee";
        }
    } else if (!target && now - b->last_hurt < 2.5 && !late && b->P.Ghost && hide_from_bearing(b, o, seen, nseen, &mx, &my)) {
        boost = me->energy > 30 && (mx != 0 || my != 0);
        mode = "cover";
    } else if (now < b->P.EarlyHide && exposed && !finishing && cover_move(b, o, seen, nseen, &mx, &my)) {
        boost = me->energy > 20 && (mx != 0 || my != 0);
        mode = "earlyhide";
    } else if (!foe && shy && !late && cover_move(b, o, seen, nseen, &mx, &my)) {
        boost = me->energy > 40 && (mx != 0 || my != 0);
        mode = "shy";
    } else if (b->P.Evade && foe && !late && !finishing && foe->dist > b->P.EngageDist * 1.2 && foe->m.hp > 0.45 * foe->m.max_hp &&
               cover_move(b, o, seen, nseen, &mx, &my)) {
        boost = me->energy > 30 && (mx != 0 || my != 0);
        mode = "evade";
    } else if (foe) {
        int ready = gun && gun->ammo > 0 && gun->reload <= 0.25 && foe->dist <= gun->stats.range - 10;
        int hot = laser && laser->charge >= laser->stats.charge - 0.3 && foe->dist <= laser->stats.range - 5;
        if (hot || (light && foe->dist <= light->stats.range + 6 && me->energy >= light->stats.energy * 3)) ready = 1;
        if (!b->P.Peek && (light || laser)) ready = 1;
        double want_los = ready ? 1 : -1.0;
        if (b->P.Peek && !hot) {
            for (int i = 0; i < nseen; i++) {
                Track *t = seen[i];
                if (t->seen == now && t->m.charging && fabs(angdiff(t->m.aim[0], go_atan2(y - t->m.pos[1], x - t->m.pos[0]))) < 0.3) want_los = -1.6;
            }
        }
        double fin_hp = b->P.FinishHP > 0 ? b->P.FinishHP : 0.38;
        if (b->P.Finish > 0 && foe->m.hp < fin_hp * foe->m.max_hp && hp_frac > 0.42 && crowd <= 2) {
            Params keep = b->P;
            b->P.GunRange = b->P.LaserRange = b->P.LightRange = b->P.Finish;
            fight_move(b, o, foe, seen, nseen, want_los, &mx, &my);
            b->P = keep;
            if (b->P.FinishRush && foe->dist > 2 * b->P.Finish && me->energy > 30) boost = 1;
        } else {
            fight_move(b, o, foe, seen, nseen, want_los, &mx, &my);
        }
        if (light && foe->dist > 50 && me->energy > 55 && want_los > 0) boost = 1;
        mode = "fight";
        if (b->P.FightLoot > 0 && me->cubes < cube_cap(b)) {   /* a663e54 */
            double r = b->P.FightLoot;
            if (hp_frac <= 0.5) r *= 0.57;
            int have = 0;
            double bc[3];
            for (int i = 0; i < b->cubes.n; i++) {
                Known *c = &b->cubes.a[i];
                double d = hyp(c->pos[0] - x, c->pos[1] - y);
                if (d < r) { r = d; memcpy(bc, c->pos, sizeof bc); have = 1; }
            }
            if (have) { travel(b, me, bc[0], bc[1], &mx, &my); mode = "fight:cube"; }
        }
    } else if (zone_should && zdist > 2) {
        travel(b, me, zgx, zgy, &mx, &my);
        mode = "zone";
    } else {
        LootChoice lc;
        if (choose_loot(b, o, hp_frac, danger, &lc)) {
            mode = LK_MODE[lc.kind];
            if (lc.kind == LK_NPC) {
                Track *t = ttab_get(&b->npcs, lc.id);
                if (t && now - t->seen < 1 && t->los && t->dist < 75) {
                    farm = t;
                    fight_move(b, o, t, seen, nseen, 1, &mx, &my);
                } else {
                    travel(b, me, lc.pos[0], lc.pos[1], &mx, &my);
                }
            } else if (lc.kind == LK_CRATE) {
                double d = hyp(lc.pos[0] - x, lc.pos[1] - y);
                double cp[3] = {lc.pos[0], lc.pos[1], lc.pos[2] + b->cfg.crate_height * 0.5};
                double from[3] = {x, y, zf + st->muzzle_height};
                if (d < 50 && world_clear(W, from, cp)) {
                    memcpy(farm_pos, cp, sizeof cp);
                    have_farm_pos = 1;
                    if (d < 12) { mx = (x - lc.pos[0]) / fmax(d, 0.1); my = (y - lc.pos[1]) / fmax(d, 0.1); }
                } else {
                    travel(b, me, lc.pos[0], lc.pos[1], &mx, &my);
                }
            } else {
                travel(b, me, lc.pos[0], lc.pos[1], &mx, &my);
                if (lc.kind == LK_DROP && hyp(lc.pos[0] - x, lc.pos[1] - y) < 2.5) { mx = 0; my = 0; }
            }
        } else if (zdist > 3 && (!b->P.Ghost || z->next_radius <= 30)) {
            travel(b, me, zgx, zgy, &mx, &my);
            mode = "zone-walk";
        } else if (b->P.Ghost) {
            double hx, hy;
            if (home(b, o, &hx, &hy) && hyp(hx - x, hy - y) > 3) {
                travel(b, me, hx, hy, &mx, &my);
                if (!danger && now - b->last_hurt > 6 && !b->open_on) {
                    double q = fmin(1, 4.6 / st->walk_speed);
                    mx *= q;
                    my *= q;
                }
                mode = "home";
            }
        } else if (b->P.Explore && !late && me->cubes < cube_cap(b) && hp_frac > 0.7 && !danger) {
            explore_move(b, o, &mx, &my);
            mode = "explore";
        }
    }

    if (esc_jump) act->jump = 1;
    if (b->P.Dodge && strcmp(mode, "unstuck") && strcmp(mode, "escape")) {
        double dx, dy;
        int r = dodge(b, o, &dx, &dy);
        if (dx != 0 || dy != 0) {
            mx = mx * 0.4 + dx * 0.9;
            my = my * 0.4 + dy * 0.9;
            double l = hyp(mx, my);
            if (l > 1) { mx /= l; my /= l; }
            if (r == 2 && me->energy > 25) boost = 1;
        }
    }
    if (!strcmp(mode, "escape")) {
        b->stuck_since = -1;
    } else if ((mx != 0 || my != 0) && hyp(me->vel[0], me->vel[1]) < 1.2 && me->grounded) {
        if (b->stuck_since < 0) b->stuck_since = now;
        else if (now - b->stuck_since > 0.7) {
            double a = gorand_f64(&b->rng) * 2 * PI;
            b->unstuck_dir[0] = go_cos(a);
            b->unstuck_dir[1] = go_sin(a);
            b->unstuck_until = now + 0.5;
            b->stuck_since = -1;
            b->npath = 0;
            act->jump = 1;
            if (b->opt.unstuck_mem) {
                /* remember the spot ahead that stopped us; replans route around it for 10 s */
                double l = hyp(mx, my);
                int slot = b->navoid < 8 ? b->navoid++ : 0;
                for (int i = 0; i < b->navoid; i++) if (b->avoid_until[i] <= now) { slot = i; break; }
                b->avoid[3 * slot] = x + (l > 1e-9 ? mx / l : 0) * 3;
                b->avoid[3 * slot + 1] = y + (l > 1e-9 ? my / l : 0) * 3;
                b->avoid[3 * slot + 2] = 4.0;
                b->avoid_until[slot] = now + 10;
            }
        }
    } else {
        b->stuck_since = -1;
    }

    double muzzle[3] = {x + me->vel[0] * LAT, y + me->vel[1] * LAT, zf + st->muzzle_height};
    double aim[2] = {me->aim[0], 0};
    int have_aim = 0;
    double tp[3] = {0, 0, 0}, tv[3] = {0, 0, 0}, tr = 0, th = 0;
    int shootable = 0, is_mech = 0, defend = 0;
    if (!target && b->npc_hurt > 0 && now - b->npc_hurt < 3) {
        Track *worst = NULL;
        double best_off = 0.7;
        for (int i = 0; i < b->npcs.n; i++) {
            Track *n = &b->npcs.a[i];
            if (now - n->seen > 0.5 || !n->los || n->dist > 150) continue;
            double off = fabs(angdiff(go_atan2(n->m.pos[1] - y, n->m.pos[0] - x), b->npc_bearing));
            if (now - n->hitme < 3) off = 0;
            if (off < best_off) { best_off = off; worst = n; }
        }
        if (worst) { farm = worst; have_farm_pos = 0; defend = 1; }
    }
    if (b->P.Econ && !target && !farm && !have_farm_pos && gun && !b->retreat) {
        double reach = 110.0;
        for (int i = 0; i < b->tracks.n; i++) if (now - b->tracks.a[i].seen < 4) reach = 50;
        for (int i = 0; i < b->crates.n; i++) {
            Known *c = &b->crates.a[i];
            double until;
            if (bad_get(b, c->id, &until) && until > now) continue;
            double d = hyp(c->pos[0] - x, c->pos[1] - y);
            double cp[3] = {c->pos[0], c->pos[1], c->pos[2] + b->cfg.crate_height * 0.5};
            if (d < reach && d > 6 && world_clear(W, muzzle, cp)) { reach = d; memcpy(farm_pos, cp, sizeof cp); have_farm_pos = 1; }
        }
    }
    if (target) {
        double sc = scale_of(b, target->m.cubes);
        tr = b->cfg.mech.radius * sc;
        th = b->cfg.mech.height * sc;
        memcpy(tp, target->m.pos, sizeof tp);
        memcpy(tv, target->m.vel, sizeof tv);
        tp[2] += th * 0.5;
        if (!world_clear(W, muzzle, tp)) tp[2] = target->m.pos[2] + th * 0.85;
        shootable = 1;
        is_mech = 1;
    } else if (farm) {
        tr = b->cfg.npc_radius;
        th = b->cfg.npc_height;
        memcpy(tp, farm->m.pos, sizeof tp);
        memcpy(tv, farm->m.vel, sizeof tv);
        tp[2] += th * 0.5;
        shootable = 1;
    } else if (have_farm_pos) {
        tr = b->cfg.crate_radius;
        th = b->cfg.crate_height;
        memcpy(tp, farm_pos, sizeof tp);
        shootable = 1;
    }
    if (!is_mech && !target && (b->retreat || hp_frac < 0.5) && !b->P.Econ && !defend) shootable = 0;

    if (shootable) {
        double dist = hyp(tp[0] - x, tp[1] - y);
        int grounded = fabs(tv[2]) < 0.5;
        double gy = 0, gp = 0;
        int gun_ok = 0;
        if (gun) {
            lead(muzzle, tp, tv, gun->stats.speed, grounded, &gy, &gp);
            gun_ok = dist <= gun->stats.range * 0.96;
        }
        double dp[3] = {tp[0] + tv[0] * (LAT + 0.05), tp[1] + tv[1] * (LAT + 0.05), tp[2]};
        double hy = go_atan2(dp[1] - muzzle[1], dp[0] - muzzle[0]);
        double hp = go_atan2(dp[2] - muzzle[2], hyp(dp[0] - muzzle[0], dp[1] - muzzle[1]));
        aim[0] = gy; aim[1] = gp;
        if (!gun) { aim[0] = hy; aim[1] = hp; }
        have_aim = 1;
        double tol_y = go_atan2(tr * 0.75, fmax(dist, 1)), tol_p = go_atan2(th * 0.4, fmax(dist, 1));
#define SLEW(wy, wp) (fmax(0, fabs(angdiff((wy), me->aim[0])) - st->turn_rate * 0.05) < tol_y && \
                      fmax(0, fabs((wp) - me->aim[1]) - st->pitch_rate * 0.05) < tol_p)
        for (int i = 0; i < me->nweap && i < 2; i++) {
            WeaponState *w = &me->weap[i];
            if (!w->present) continue;
            if (w->kind == WK_GUN) {
                if (gun_ok && w->ammo > 0 && w->reload <= 0 && SLEW(gy, gp)) act->fire[i] = 1;
            } else if (w->kind == WK_LIGHTNING) {
                double cone = w->stats.cone_deg * PI / 180;
                if (dist < w->stats.range - 1.5 && fabs(angdiff(hy, me->aim[0])) < cone * 0.5 - 0.05 && me->energy >= w->stats.energy) act->fire[i] = 1;
            } else if (w->kind == WK_LASER) {
                if (!is_mech || dist > w->stats.range * 0.97) continue;
                double full = w->stats.charge;
                if (b->P.Lethal && w->charge >= w->stats.min_charge && w->charge < full - 0.051) {
                    int cc = me->cubes < b->cfg.stat_cap ? me->cubes : b->cfg.stat_cap;
                    double dmg = w->stats.damage * (w->stats.min_frac + (1 - w->stats.min_frac) * w->charge / full) *
                                 (1 + b->cfg.damage_per_cube * (double)cc);
                    if (holds(&target->m, WK_MORTAR)) dmg *= 1 + w->stats.cookoff;
                    if (dmg * 0.8 >= target->m.hp && fabs(angdiff(hy, me->aim[0])) < tol_y * 0.9 && fabs(hp - me->aim[1]) < tol_p) {
                        aim[0] = hy; aim[1] = hp;
                        act->fire[i] = 0;
                        continue;
                    }
                }
                if (w->charge < full - 0.051) {
                    act->fire[i] = 1;
                    if (w->charge > full * 0.45) { aim[0] = hy; aim[1] = hp; }
                } else {
                    aim[0] = hy; aim[1] = hp;
                    double ey = fabs(angdiff(hy, me->aim[0])), ep = fabs(hp - me->aim[1]);
                    act->fire[i] = !(ey < tol_y * 0.9 && ep < tol_p);
                }
            } else if (w->kind == WK_MORTAR) {
                if (!is_mech || w->cooldown > 0.25 || dist < 16 || dist > 200) continue;
                double tube[3] = {muzzle[0], muzzle[1], zf + st->height + 0.5};
                double foot[3] = {tp[0], tp[1], tp[2] - th * 0.5};
                double my2, mp;
                if (mortar_aim(b, tube, foot, tv, w->stats.speed, w->stats.gravity, st, &my2, &mp)) {
                    if (!gun || gun->reload > 0 || gun->ammo == 0 || !gun_ok) { aim[0] = my2; aim[1] = mp; }
                    double ey = fmax(0, fabs(angdiff(my2, me->aim[0])) - st->turn_rate * 0.05);
                    double ep = fmax(0, fabs(mp - me->aim[1]) - st->pitch_rate * 0.05);
                    if (ey < 0.04 && ep < 0.03) act->fire[i] = 1;
                }
            }
        }
#undef SLEW
    }
    if (!target && foe) {
        for (int i = 0; i < me->nweap && i < 2; i++) {
            WeaponState *w = &me->weap[i];
            if (w->present && w->kind == WK_LASER && w->cooldown <= 0.05 && foe->dist < w->stats.range &&
                w->charge < w->stats.charge + w->stats.hold_s - 0.35)
                act->fire[i] = 1;
        }
    }
    /* 738afcd: hold a charged laser through brief target loss */
    if (!target) {
        Track *stt = ttab_get(&b->tracks, b->stick_id);
        if (stt && now - b->stick_at < 1 && now - stt->seen < 1) {
            for (int i = 0; i < me->nweap; i++) {
                WeaponState *w = &me->weap[i];
                if (!w->present || i > 1 || w->kind != WK_LASER || w->charge <= 0 || act->fire[i] || stt->dist >= w->stats.range) continue;
                act->fire[i] = 1;
                double c[3];
                center_of(b, stt, c);
                aim[0] = go_atan2(c[1] - muzzle[1], c[0] - muzzle[0]);
                aim[1] = go_atan2(c[2] - muzzle[2], hyp(c[0] - muzzle[0], c[1] - muzzle[1]));
                have_aim = 1;
            }
        }
    }
    /* charging halves our speed: no new charge while running for cover or the zone */
    if (!strcmp(mode, "cover") || !strcmp(mode, "flee") || !strcmp(mode, "escape") || (!strcmp(mode, "zone") && !me->in_zone)) {
        for (int i = 0; i < me->nweap && i < 2; i++) {
            WeaponState *w = &me->weap[i];
            if (w->present && w->kind == WK_LASER && w->charge <= 0) act->fire[i] = 0;
        }
    }

    if (!have_aim) {
        Track *watch = NULL;
        for (int i = 0; i < nseen; i++) {
            Track *t = seen[i];
            if (t->engage || now - t->hitme < 6)
                if (!watch || t->dist < watch->dist) watch = t;
        }
        if (watch) aim[0] = go_atan2(watch->m.pos[1] - y, watch->m.pos[0] - x);
        else if (b->npc_hurt > 0 && now - b->npc_hurt < 2.5) aim[0] = b->npc_bearing;
        else if (now - b->last_hurt < 2.5) aim[0] = b->hurt_bearing;
        else if (mx != 0 || my != 0) aim[0] = go_atan2(my, mx) + 1.15 * go_sin(now * 1.3);
        else aim[0] = me->aim[0] + 0.35;
        aim[1] = 0;
    }
    aim[1] = clampd(aim[1], st->min_pitch, st->max_pitch);
    act->has_aim = 1;
    act->aim[0] = aim[0];
    act->aim[1] = aim[1];

    if (gun && gun->reload <= 0 && gun->ammo < gun->stats.mag) {
        if (!target && !farm && !have_farm_pos && (gun->ammo <= gun->stats.mag * 0.6 || now - b->last_threat_at > 3)) act->reload = 1;
    }
    int slot1 = (me->nweap > 1 && me->weap[1].present) ? me->weap[1].kind : WK_NONE;
    for (int i = 0; i < o->npickups; i++) {
        Item *p = &o->pickups[i];
        if (hyp(p->pos[0] - x, p->pos[1] - y) > b->cfg.loot_pickup_radius - 0.3 || fabs(p->pos[2] - zf) > 3.6) continue;
        if (!want_pickup(b, me, p->kind)) continue;
        act->has_take = 1;
        act->take = p->id;
        if (slot1 != WK_NONE && me->nweap > 0 && me->weap[0].present) {
            if (slot1 == WK_LASER && me->weap[1].charge > 0) { act->has_take = 0; break; }
            act->has_drop = 1;
            act->drop = 1;
        }
        break;
    }
    act->move[0] = mx;
    act->move[1] = my;
    act->boost = boost && (mx != 0 || my != 0);
    b->mode = mode;
}

void brain_act(Brain *b, Obs *o, Action *act) {
    memset(act, 0, sizeof *act);
    act->tick = o->tick;
    step(b, o, act);
    if (act->has_aim) {
        if (isnan(act->aim[0]) || isinf(act->aim[0]) || isnan(act->aim[1]) || isinf(act->aim[1])) act->has_aim = 0;
        else { b->last_aim[0] = act->aim[0]; b->last_aim[1] = act->aim[1]; }
    }
    if (isnan(act->move[0]) || isnan(act->move[1]) || isinf(act->move[0]) || isinf(act->move[1])) { act->move[0] = act->move[1] = 0; }
}
