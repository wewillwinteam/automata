/* JSON <-> structs for the protocol. Observations are read with a streaming
 * parser straight into Obs (no allocation); hello uses the DOM in json.h. */
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bot.h"
#include "json.h"

const char *WK_NAME[4] = {"gun", "mortar", "laser", "lightning"};

int wkind_of(const char *s) {
    if (!s) return WK_NONE;
    if (!strcmp(s, "gun")) return WK_GUN;
    if (!strcmp(s, "mortar")) return WK_MORTAR;
    if (!strcmp(s, "laser")) return WK_LASER;
    if (!strcmp(s, "lightning")) return WK_LIGHTNING;
    return WK_OTHER;
}

/* ------------------------------------------------------------- hello */

static void read_mech_stats_dom(const JNode *o, MechStats *m) {
#define F(name) m->name = jnum(jkey(o, #name), 0)
    F(radius); F(height); F(eye_height); F(muzzle_height); F(hp); F(walk_speed); F(boost_speed); F(ground_accel);
    F(air_accel); F(gravity); F(jump_speed); F(jet_accel); F(max_rise_speed); F(step_up); F(max_slope_deg); F(energy);
    F(energy_regen); F(energy_regen_delay); F(boost_drain); F(jet_drain); F(turn_rate); F(pitch_rate); F(min_pitch);
    F(max_pitch); F(regen_delay); F(regen_per_s); F(charge_move_mult); F(self_damage_mult); F(armor); F(rear_mult);
    F(front_arc_deg); F(near_range_bonus); F(vision_bonus); F(hearing);
#undef F
}

int parse_hello(const char *s, size_t n, Hello *h) {
    memset(h, 0, sizeof *h);
    JNode *o = jparse(s, n);
    if (!o) return -1;
    h->slot = (int)jnum(jkey(o, "slot"), 0);
    JNode *seed = jkey(o, "seed");
    if (seed && seed->type == J_NUM) {
        /* re-read the digits: a double cannot hold every 64-bit seed */
        const char *p = strstr(s, "\"seed\"");
        if (p) {
            p = strchr(p, ':');
            if (p) h->seed = strtoull(p + 1, NULL, 10);
        }
        if (!h->seed) h->seed = (uint64_t)seed->num;
    }
    JNode *dir = jpath(o, "map.dir");
    snprintf(h->map_dir, sizeof h->map_dir, "%s", dir && dir->type == J_STR ? dir->str : "");
    JNode *c = jkey(o, "config");
    read_mech_stats_dom(jkey(c, "mech"), &h->cfg.mech);
    h->cfg.mortar.gravity = jpnum(c, "weapons.mortar.gravity", 0);
    h->cfg.mortar.splash_radius = jpnum(c, "weapons.mortar.splash_radius", 0);
    h->cfg.size_per_cube = jpnum(c, "cubes.size_per_cube", 0);
    h->cfg.damage_per_cube = jpnum(c, "cubes.damage_per_cube", 0);
    h->cfg.stat_cap = (int)jpnum(c, "cubes.stat_cap", 0);
    h->cfg.crate_height = jpnum(c, "crates.height", 0);
    h->cfg.crate_radius = jpnum(c, "crates.radius", 0);
    h->cfg.loot_pickup_radius = jpnum(c, "loot.pickup_radius", 0);
    h->cfg.npc_height = jpnum(c, "npc.height", 0);
    h->cfg.npc_radius = jpnum(c, "npc.radius", 0);
    jfree(o);
    return 0;
}

/* ------------------------------------------------- streaming obs reader */

typedef struct { const char *p, *end; int err; } R;

static void ws(R *r) { while (r->p < r->end && (unsigned char)*r->p <= ' ') r->p++; }
static int peek(R *r) { ws(r); return r->p < r->end ? *r->p : 0; }
static int eat(R *r, char c) { if (peek(r) == c) { r->p++; return 1; } return 0; }

static double num(R *r) {
    ws(r);
    if (r->p >= r->end) { r->err = 1; return 0; }
    if (*r->p == 'n') { r->p += 4; return 0; }          /* null */
    if (*r->p == 't') { r->p += 4; return 1; }          /* true */
    if (*r->p == 'f') { r->p += 5; return 0; }          /* false */
    /* fast path for plain decimals; strtod for exponents */
    const char *s = r->p;
    int neg = 0;
    if (*s == '-') { neg = 1; s++; }
    unsigned long long ip = 0;
    int nd = 0;
    while (s < r->end && *s >= '0' && *s <= '9' && nd < 18) { ip = ip * 10 + (unsigned)(*s - '0'); s++; nd++; }
    double v = (double)ip;
    if (s < r->end && (*s == '.' || *s == 'e' || *s == 'E' || (*s >= '0' && *s <= '9'))) {
        char *e;
        v = strtod(r->p, &e);
        if (e == r->p) { r->err = 1; return 0; }
        r->p = e;
        return v;
    }
    if (s == r->p + neg) { r->err = 1; return 0; }
    r->p = s;
    return neg ? -v : v;
}

static int is_null(R *r) {
    if (peek(r) == 'n') { r->p += 4; return 1; }
    return 0;
}

/* string into buf (truncated); returns length */
static int str(R *r, char *buf, int cap) {
    ws(r);
    if (r->p >= r->end || *r->p != '"') { r->err = 1; if (cap) buf[0] = 0; return 0; }
    r->p++;
    int n = 0;
    while (r->p < r->end && *r->p != '"') {
        char c = *r->p++;
        if (c == '\\' && r->p < r->end) {
            char e = *r->p++;
            if (e == 'u') { r->p += 4; c = '?'; }
            else c = e == 'n' ? '\n' : e == 't' ? '\t' : e;
        }
        if (n < cap - 1) buf[n++] = c;
    }
    if (r->p < r->end) r->p++;
    if (cap) buf[n] = 0;
    return n;
}

static void skip(R *r) {
    int c = peek(r);
    if (c == '"') { char t[1]; str(r, t, 0); return; }
    if (c == '{' || c == '[') {
        int depth = 0;
        int in_str = 0;
        while (r->p < r->end) {
            char ch = *r->p++;
            if (in_str) {
                if (ch == '\\') r->p++;
                else if (ch == '"') in_str = 0;
                continue;
            }
            if (ch == '"') in_str = 1;
            else if (ch == '{' || ch == '[') depth++;
            else if (ch == '}' || ch == ']') { if (--depth == 0) return; }
        }
        r->err = 1;
        return;
    }
    num(r);
}

static void vec(R *r, double *v, int n) {
    for (int i = 0; i < n; i++) v[i] = 0;
    if (is_null(r)) return;
    if (!eat(r, '[')) { r->err = 1; return; }
    int i = 0;
    if (eat(r, ']')) return;
    for (;;) {
        double x = num(r);
        if (i < n) v[i] = x;
        i++;
        if (eat(r, ',')) continue;
        if (eat(r, ']')) return;
        r->err = 1;
        return;
    }
}

/* iterate a JSON array: ARR(r) { ...element... } */
static int arr_begin(R *r) {
    if (is_null(r)) return 0;
    if (!eat(r, '[')) { r->err = 1; return 0; }
    if (eat(r, ']')) return 0;
    return 1;
}
static int arr_next(R *r) {
    if (eat(r, ',')) return 1;
    if (!eat(r, ']')) r->err = 1;
    return 0;
}

static void mech_stats(R *r, MechStats *m) {
    memset(m, 0, sizeof *m);
    if (is_null(r)) return;
    if (!eat(r, '{')) { r->err = 1; return; }
    if (eat(r, '}')) return;
    for (;;) {
        char key[32];
        str(r, key, sizeof key);
        if (!eat(r, ':')) { r->err = 1; return; }
#define F(name) if (!strcmp(key, #name)) { m->name = num(r); } else
        F(radius) F(height) F(eye_height) F(muzzle_height) F(hp) F(walk_speed) F(boost_speed) F(ground_accel)
        F(air_accel) F(gravity) F(jump_speed) F(jet_accel) F(max_rise_speed) F(step_up) F(max_slope_deg) F(energy)
        F(energy_regen) F(energy_regen_delay) F(boost_drain) F(jet_drain) F(turn_rate) F(pitch_rate) F(min_pitch)
        F(max_pitch) F(regen_delay) F(regen_per_s) F(charge_move_mult) F(self_damage_mult) F(armor) F(rear_mult)
        F(front_arc_deg) F(near_range_bonus) F(vision_bonus) F(hearing) skip(r);
#undef F
        if (r->err) return;
        if (eat(r, ',')) continue;
        if (eat(r, '}')) return;
        r->err = 1;
        return;
    }
}

static void weapon_stats(R *r, WeaponStats *m) {
    memset(m, 0, sizeof *m);
    if (is_null(r)) return;
    if (!eat(r, '{')) { r->err = 1; return; }
    if (eat(r, '}')) return;
    for (;;) {
        char key[32];
        str(r, key, sizeof key);
        if (!eat(r, ':')) { r->err = 1; return; }
#define F(name) if (!strcmp(key, #name)) { m->name = num(r); } else
        F(damage) F(cooldown) F(speed) F(range) F(spread_deg) F(mag) F(reload) F(splash) F(splash_radius) F(splash_edge)
        F(gravity) F(ttl) F(knock) F(knock_up) F(drain) F(charge) F(min_charge) F(min_frac) F(hold_s) F(pierce)
        F(cookoff) F(cone_deg) F(chain) F(chain_range) F(chain_falloff) F(energy) F(short_s) skip(r);
#undef F
        if (r->err) return;
        if (eat(r, ',')) continue;
        if (eat(r, '}')) return;
        r->err = 1;
        return;
    }
}

static void weapon_state(R *r, WeaponState *w) {
    memset(w, 0, sizeof *w);
    w->kind = WK_NONE;
    if (is_null(r)) return;
    w->present = 1;
    if (!eat(r, '{')) { r->err = 1; return; }
    if (eat(r, '}')) return;
    for (;;) {
        char key[32];
        str(r, key, sizeof key);
        if (!eat(r, ':')) { r->err = 1; return; }
        if (!strcmp(key, "kind")) { char k[24]; str(r, k, sizeof k); w->kind = wkind_of(k); }
        else if (!strcmp(key, "cooldown")) w->cooldown = num(r);
        else if (!strcmp(key, "ammo")) w->ammo = num(r);
        else if (!strcmp(key, "reload")) w->reload = num(r);
        else if (!strcmp(key, "charge")) w->charge = num(r);
        else if (!strcmp(key, "stats")) weapon_stats(r, &w->stats);
        else skip(r);
        if (r->err) return;
        if (eat(r, ',')) continue;
        if (eat(r, '}')) return;
        r->err = 1;
        return;
    }
}

static void self_obj(R *r, Self *s) {
    if (!eat(r, '{')) { r->err = 1; return; }
    if (eat(r, '}')) return;
    for (;;) {
        char key[32];
        str(r, key, sizeof key);
        if (!eat(r, ':')) { r->err = 1; return; }
        if (!strcmp(key, "id")) s->id = (int)num(r);
        else if (!strcmp(key, "pos")) vec(r, s->pos, 3);
        else if (!strcmp(key, "vel")) vec(r, s->vel, 3);
        else if (!strcmp(key, "aim")) vec(r, s->aim, 2);
        else if (!strcmp(key, "hp")) s->hp = num(r);
        else if (!strcmp(key, "max_hp")) s->max_hp = num(r);
        else if (!strcmp(key, "energy")) s->energy = num(r);
        else if (!strcmp(key, "max_energy")) s->max_energy = num(r);
        else if (!strcmp(key, "grounded")) s->grounded = num(r) != 0;
        else if (!strcmp(key, "cubes")) s->cubes = (int)num(r);
        else if (!strcmp(key, "in_zone")) s->in_zone = num(r) != 0;
        else if (!strcmp(key, "stats")) mech_stats(r, &s->stats);
        else if (!strcmp(key, "weapons")) {
            s->nweap = 0;
            if (arr_begin(r)) {
                do {
                    WeaponState tmp;
                    weapon_state(r, s->nweap < MAXW ? &s->weap[s->nweap] : &tmp);
                    s->nweap++;
                } while (!r->err && arr_next(r));
            }
            if (s->nweap > MAXW) s->nweap = MAXW;
        } else skip(r);
        if (r->err) return;
        if (eat(r, ',')) continue;
        if (eat(r, '}')) return;
        r->err = 1;
        return;
    }
}

static void mech_seen(R *r, MechSeen *m) {
    memset(m, 0, sizeof *m);
    if (!eat(r, '{')) { r->err = 1; return; }
    if (eat(r, '}')) return;
    for (;;) {
        char key[32];
        str(r, key, sizeof key);
        if (!eat(r, ':')) { r->err = 1; return; }
        if (!strcmp(key, "id")) m->id = (int)num(r);
        else if (!strcmp(key, "kind")) { char k[16]; str(r, k, sizeof k); m->npc = !strcmp(k, "npc"); m->is_mech = !strcmp(k, "mech"); }
        else if (!strcmp(key, "pos")) vec(r, m->pos, 3);
        else if (!strcmp(key, "vel")) vec(r, m->vel, 3);
        else if (!strcmp(key, "aim")) vec(r, m->aim, 2);
        else if (!strcmp(key, "hp")) m->hp = num(r);
        else if (!strcmp(key, "max_hp")) m->max_hp = num(r);
        else if (!strcmp(key, "cubes")) m->cubes = (int)num(r);
        else if (!strcmp(key, "charging")) m->charging = num(r) != 0;
        else if (!strcmp(key, "weapons")) {
            m->nweap = 0;
            if (arr_begin(r)) {
                do {
                    char k[24];
                    str(r, k, sizeof k);
                    if (m->nweap < 4) m->weap[m->nweap++] = wkind_of(k);
                } while (!r->err && arr_next(r));
            }
        } else skip(r);
        if (r->err) return;
        if (eat(r, ',')) continue;
        if (eat(r, '}')) return;
        r->err = 1;
        return;
    }
}

static void item(R *r, Item *it, int has_kind) {
    memset(it, 0, sizeof *it);
    it->kind = WK_NONE;
    if (!eat(r, '{')) { r->err = 1; return; }
    if (eat(r, '}')) return;
    for (;;) {
        char key[32];
        str(r, key, sizeof key);
        if (!eat(r, ':')) { r->err = 1; return; }
        if (!strcmp(key, "id")) it->id = (int)num(r);
        else if (!strcmp(key, "pos")) vec(r, it->pos, 3);
        else if (!strcmp(key, "hp")) it->hp = num(r);
        else if (has_kind && !strcmp(key, "kind")) { char k[24]; str(r, k, sizeof k); it->kind = wkind_of(k); }
        else skip(r);
        if (r->err) return;
        if (eat(r, ',')) continue;
        if (eat(r, '}')) return;
        r->err = 1;
        return;
    }
}

static void items(R *r, Item *arr, int *n, int has_kind) {
    *n = 0;
    if (!arr_begin(r)) return;
    do {
        Item tmp;
        item(r, *n < MAXITEMS ? &arr[*n] : &tmp, has_kind);
        if (*n < MAXITEMS) (*n)++;
    } while (!r->err && arr_next(r));
}

static void proj(R *r, Proj *p) {
    memset(p, 0, sizeof *p);
    if (!eat(r, '{')) { r->err = 1; return; }
    if (eat(r, '}')) return;
    for (;;) {
        char key[32];
        str(r, key, sizeof key);
        if (!eat(r, ':')) { r->err = 1; return; }
        if (!strcmp(key, "id")) p->id = (int)num(r);
        else if (!strcmp(key, "kind")) { char k[16]; str(r, k, sizeof k); p->mortar = !strcmp(k, "mortar"); }
        else if (!strcmp(key, "pos")) vec(r, p->pos, 3);
        else if (!strcmp(key, "vel")) vec(r, p->vel, 3);
        else skip(r);
        if (r->err) return;
        if (eat(r, ',')) continue;
        if (eat(r, '}')) return;
        r->err = 1;
        return;
    }
}

static void event(R *r, Event *e) {
    memset(e, 0, sizeof *e);
    if (!eat(r, '{')) { r->err = 1; return; }
    if (eat(r, '}')) return;
    for (;;) {
        char key[32];
        str(r, key, sizeof key);
        if (!eat(r, ':')) { r->err = 1; return; }
        if (!strcmp(key, "type")) {
            char k[24];
            str(r, k, sizeof k);
            e->type = !strcmp(k, "damaged") ? EV_DAMAGED : !strcmp(k, "kill") ? EV_KILL : EV_OTHER;
        } else if (!strcmp(key, "weapon")) {
            if (peek(r) == '"') { char k[24]; str(r, k, sizeof k); e->weapon_npc = !strcmp(k, "npc"); } else skip(r);
        } else if (!strcmp(key, "bearing")) {
            if (!is_null(r)) { e->has_bearing = 1; e->bearing = num(r); }
        } else if (!strcmp(key, "by")) {
            if (!is_null(r)) { e->has_by = 1; e->by = (int)num(r); }
        } else if (!strcmp(key, "target")) e->target = (int)num(r);
        else skip(r);
        if (r->err) return;
        if (eat(r, ',')) continue;
        if (eat(r, '}')) return;
        r->err = 1;
        return;
    }
}

static void drop(R *r, Drop *d) {
    memset(d, 0, sizeof *d);
    if (!eat(r, '{')) { r->err = 1; return; }
    if (eat(r, '}')) return;
    for (;;) {
        char key[32];
        str(r, key, sizeof key);
        if (!eat(r, ':')) { r->err = 1; return; }
        if (!strcmp(key, "id")) d->id = (int)num(r);
        else if (!strcmp(key, "pos")) vec(r, d->pos, 3);
        else if (!strcmp(key, "radius")) d->radius = num(r);
        else if (!strcmp(key, "lands_in")) d->lands_in = num(r);
        else if (!strcmp(key, "capture_s")) d->capture_s = num(r);
        else if (!strcmp(key, "progress")) d->progress = num(r);
        else skip(r);
        if (r->err) return;
        if (eat(r, ',')) continue;
        if (eat(r, '}')) return;
        r->err = 1;
        return;
    }
}

static void zone(R *r, Zone *z) {
    memset(z, 0, sizeof *z);
    if (is_null(r)) return;
    if (!eat(r, '{')) { r->err = 1; return; }
    if (eat(r, '}')) return;
    for (;;) {
        char key[32];
        str(r, key, sizeof key);
        if (!eat(r, ':')) { r->err = 1; return; }
        if (!strcmp(key, "center")) vec(r, z->center, 2);
        else if (!strcmp(key, "radius")) z->radius = num(r);
        else if (!strcmp(key, "next_center")) vec(r, z->next_center, 2);
        else if (!strcmp(key, "next_radius")) z->next_radius = num(r);
        else if (!strcmp(key, "shrink_start")) z->shrink_start = num(r);
        else if (!strcmp(key, "shrink_end")) z->shrink_end = num(r);
        else if (!strcmp(key, "dps")) z->dps = num(r);
        else skip(r);
        if (r->err) return;
        if (eat(r, ',')) continue;
        if (eat(r, '}')) return;
        r->err = 1;
        return;
    }
}

static void offer(R *r, Obs *o) {
    o->has_offer = 0;
    o->offer.n = 0;
    if (is_null(r)) return;
    o->has_offer = 1;
    if (!eat(r, '{')) { r->err = 1; return; }
    if (eat(r, '}')) return;
    for (;;) {
        char key[32];
        str(r, key, sizeof key);
        if (!eat(r, ':')) { r->err = 1; return; }
        if (!strcmp(key, "choices")) {
            if (arr_begin(r)) {
                do {
                    if (!eat(r, '{')) { r->err = 1; return; }
                    if (!eat(r, '}')) {
                        for (;;) {
                            char k2[32];
                            str(r, k2, sizeof k2);
                            if (!eat(r, ':')) { r->err = 1; return; }
                            if (!strcmp(k2, "id") && o->offer.n < MAXCHOICE) str(r, o->offer.ids[o->offer.n], 40);
                            else skip(r);
                            if (r->err) return;
                            if (eat(r, ',')) continue;
                            if (eat(r, '}')) break;
                            r->err = 1;
                            return;
                        }
                    }
                    if (o->offer.n < MAXCHOICE) o->offer.n++;
                } while (!r->err && arr_next(r));
            }
        } else skip(r);
        if (r->err) return;
        if (eat(r, ',')) continue;
        if (eat(r, '}')) return;
        r->err = 1;
        return;
    }
}

int parse_obs(const char *s, size_t n, Obs *o) {
    R rr = {s, s + n, 0}, *r = &rr;
    o->tick = 0; o->t = 0; o->alive = 0;
    memset(&o->self, 0, sizeof o->self);
    o->nmechs = o->nproj = o->ncrates = o->ncubes = o->npickups = o->nevents = o->ndrops = 0;
    memset(&o->zone, 0, sizeof o->zone);
    o->has_offer = 0;
    if (!eat(r, '{')) return -1;
    if (eat(r, '}')) return 0;
    for (;;) {
        char key[32];
        str(r, key, sizeof key);
        if (!eat(r, ':')) return -1;
        if (!strcmp(key, "tick")) o->tick = (int)num(r);
        else if (!strcmp(key, "t")) o->t = num(r);
        else if (!strcmp(key, "alive")) o->alive = (int)num(r);
        else if (!strcmp(key, "self")) { if (!is_null(r)) self_obj(r, &o->self); }
        else if (!strcmp(key, "mechs")) {
            if (arr_begin(r)) {
                do {
                    MechSeen tmp;
                    mech_seen(r, o->nmechs < MAXSEEN ? &o->mechs[o->nmechs] : &tmp);
                    if (o->nmechs < MAXSEEN) o->nmechs++;
                } while (!r->err && arr_next(r));
            }
        } else if (!strcmp(key, "projectiles")) {
            if (arr_begin(r)) {
                do {
                    Proj tmp;
                    proj(r, o->nproj < MAXPROJ ? &o->proj[o->nproj] : &tmp);
                    if (o->nproj < MAXPROJ) o->nproj++;
                } while (!r->err && arr_next(r));
            }
        } else if (!strcmp(key, "crates")) items(r, o->crates, &o->ncrates, 0);
        else if (!strcmp(key, "cubes")) items(r, o->cubes, &o->ncubes, 0);
        else if (!strcmp(key, "pickups")) items(r, o->pickups, &o->npickups, 1);
        else if (!strcmp(key, "events")) {
            if (arr_begin(r)) {
                do {
                    Event tmp;
                    event(r, o->nevents < MAXEVENTS ? &o->events[o->nevents] : &tmp);
                    if (o->nevents < MAXEVENTS) o->nevents++;
                } while (!r->err && arr_next(r));
            }
        } else if (!strcmp(key, "drops")) {
            if (arr_begin(r)) {
                do {
                    Drop tmp;
                    drop(r, o->ndrops < MAXDROPS ? &o->drops[o->ndrops] : &tmp);
                    if (o->ndrops < MAXDROPS) o->ndrops++;
                } while (!r->err && arr_next(r));
            }
        } else if (!strcmp(key, "zone")) zone(r, &o->zone);
        else if (!strcmp(key, "offer")) offer(r, o);
        else skip(r);
        if (r->err) return -1;
        if (eat(r, ',')) continue;
        if (eat(r, '}')) return 0;
        return -1;
    }
}

int parse_tick(const char *s, size_t n) {
    const char *p = s, *end = s + n;
    while (p + 6 < end) {
        if (!memcmp(p, "\"tick\"", 6)) {
            p += 6;
            while (p < end && (*p == ' ' || *p == ':')) p++;
            return atoi(p);
        }
        p++;
    }
    return 0;
}

/* Go json.Marshal(Action) field order and omitempty rules */
static int fmt_f(char *b, int cap, double v) {
    if (v == 0) return snprintf(b, (size_t)cap, "0");
    return snprintf(b, (size_t)cap, "%.17g", v);
}

int write_action(char *buf, int cap, const Action *a) {
    int n = 0;
#define W(...) do { int k_ = snprintf(buf + n, (size_t)(cap - n), __VA_ARGS__); if (k_ < 0 || k_ >= cap - n) return -1; n += k_; } while (0)
    char x[40], y[40];
    fmt_f(x, sizeof x, a->move[0]);
    fmt_f(y, sizeof y, a->move[1]);
    W("{\"tick\":%d,\"move\":[%s,%s],\"boost\":%s,\"jump\":%s", a->tick, x, y, a->boost ? "true" : "false", a->jump ? "true" : "false");
    if (a->has_aim) {
        fmt_f(x, sizeof x, a->aim[0]);
        fmt_f(y, sizeof y, a->aim[1]);
        W(",\"aim\":[%s,%s]", x, y);
    }
    W(",\"fire\":[%s,%s],\"reload\":%s", a->fire[0] ? "true" : "false", a->fire[1] ? "true" : "false", a->reload ? "true" : "false");
    if (a->has_pick) W(",\"pick\":%d", a->pick);
    if (a->has_take) W(",\"take\":%d", a->take);
    if (a->has_drop) W(",\"drop\":%d", a->drop);
    W("}\n");
#undef W
    return n;
}
