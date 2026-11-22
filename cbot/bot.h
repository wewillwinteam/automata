/* C port of the try_again (giddygiddyboy) Mech Royale bot, commit 20bd565
 * (= 94b1c8b, the revert of f5bd2dc).
 *
 * Layout mirrors the Go package: world.c = mech/world.go (only what the brain
 * uses), brain.c = mech/brain.go, parse.c = the JSON side of cmd/bot/main.go,
 * main.c = the stdin/stdout loop. Names follow the Go code so the two can be
 * read side by side.
 */
#ifndef CBOT_H
#define CBOT_H

#include <stddef.h>
#include <stdint.h>

#define MAXW 2          /* weapon slots */
#define MAXSEEN 64      /* mechs + npcs in one observation */
#define MAXPROJ 512
#define MAXITEMS 256
#define MAXEVENTS 128
#define MAXDROPS 16
#define MAXCHOICE 8

enum { WK_NONE = -1, WK_GUN, WK_MORTAR, WK_LASER, WK_LIGHTNING, WK_OTHER };
int wkind_of(const char *s);
extern const char *WK_NAME[4];

typedef struct {
    double radius, height, eye_height, muzzle_height, hp, walk_speed, boost_speed, ground_accel, air_accel,
        gravity, jump_speed, jet_accel, max_rise_speed, step_up, max_slope_deg, energy, energy_regen,
        energy_regen_delay, boost_drain, jet_drain, turn_rate, pitch_rate, min_pitch, max_pitch, regen_delay,
        regen_per_s, charge_move_mult, self_damage_mult, armor, rear_mult, front_arc_deg, near_range_bonus,
        vision_bonus, hearing;
} MechStats;

typedef struct {
    double damage, cooldown, speed, range, spread_deg, mag, reload, splash, splash_radius, splash_edge, gravity,
        ttl, knock, knock_up, drain, charge, min_charge, min_frac, hold_s, pierce, cookoff, cone_deg, chain,
        chain_range, chain_falloff, energy, short_s;
} WeaponStats;

/* only the config fields the brain reads */
typedef struct {
    MechStats mech;
    WeaponStats mortar;
    double size_per_cube, damage_per_cube;
    int stat_cap;
    double crate_height, crate_radius;
    double loot_pickup_radius;
    double npc_height, npc_radius;
} Config;

typedef struct {
    int slot;
    uint64_t seed;
    char map_dir[1024];
    Config cfg;
} Hello;

typedef struct {
    int present;          /* JSON null -> 0 */
    int kind;             /* WK_* */
    double cooldown, ammo, reload, charge;
    WeaponStats stats;
} WeaponState;

typedef struct {
    int id;
    double pos[3], vel[3], aim[2];
    double hp, max_hp, energy, max_energy;
    int grounded, cubes, in_zone;
    int nweap;            /* len(self.weapons) */
    WeaponState weap[MAXW];
    MechStats stats;
} Self;

typedef struct {
    int id;
    int npc;              /* kind == "npc" */
    int is_mech;          /* kind == "mech" */
    double pos[3], vel[3], aim[2];
    double hp, max_hp;
    int cubes;
    int nweap;
    int weap[4];          /* WK_* */
    int charging;
} MechSeen;

typedef struct { int id; int mortar; double pos[3], vel[3]; } Proj;
typedef struct { int id; int kind; double pos[3]; double hp; } Item;   /* kind: WK_* for pickups */

enum { EV_OTHER, EV_DAMAGED, EV_KILL };
typedef struct {
    int type;
    int weapon_npc;       /* weapon == "npc" */
    int has_bearing; double bearing;
    int has_by; int by;
    int target;
} Event;

typedef struct { int id; double pos[3], radius, lands_in, capture_s, progress; } Drop;

typedef struct {
    double center[2], radius, next_center[2], next_radius, shrink_start, shrink_end, dps;
} Zone;

typedef struct { int n; char ids[MAXCHOICE][40]; } Offer;

typedef struct {
    int tick;
    double t;
    int alive;
    Self self;
    int nmechs; MechSeen mechs[MAXSEEN];
    int nproj; Proj proj[MAXPROJ];
    int ncrates; Item crates[MAXITEMS];
    int ncubes; Item cubes[MAXITEMS];
    int npickups; Item pickups[MAXITEMS];
    int nevents; Event events[MAXEVENTS];
    int ndrops; Drop drops[MAXDROPS];
    Zone zone;
    int has_offer; Offer offer;
} Obs;

typedef struct {
    int tick;
    double move[2];
    int boost, jump;
    int has_aim; double aim[2];
    int fire[2];
    int reload;
    int has_pick; int pick;
    int has_take; int take;
    int has_drop; int drop;
} Action;

/* ------------------------------------------------------------- world */

typedef struct { float base, top; } Slab;

typedef struct {
    char kind;            /* 'b' building, 'r' bridge, 's' stadium, other */
    double base, top;
    double *edges; int nedges;   /* x0 y0 x1 y1 per edge, nedges*4 doubles */
    double minx, miny, maxx, maxy;
} Prism;

typedef struct World {
    double minx, miny;
    int nx, ny;
    float *ground, *top;
    uint8_t *obst, *has_slab;
    int32_t *slab_off; Slab *slabs;    /* CSR by vertex index */
    double bx0, bx1, by0, by1;
    double ns;
    int nnx, nny;
    uint8_t *blocked;
    float *surf, *clearance;
    int32_t *comp, main_comp;
    int32_t *main_near;
    /* geo */
    int nprisms; Prism *prisms;
    double gx0, gy0;
    int gnx, gny;
    int32_t *gridp_off, *gridp;        /* CSR prism lists per 16 m cell */
    int bnx, bny;
    float *block_max;
    /* search scratch (shared by every brain using this world; brains run one at a time) */
    float *g;
    int32_t *came;
    uint32_t *stamp, *closed;
    uint32_t gen;
    struct { float f; int32_t idx; } *heap;
    int nheap, cheap;
    int32_t *queue; int cqueue;
} World;

World *world_load(const char *dir);
World *world_flat(void);
double world_ground_at(const World *w, double x, double y);
double world_walk_at(const World *w, double x, double y);
double world_top_at(const World *w, double x, double y);
double world_stack_kind(const World *w, double x, double y, int deck, int *id);
int world_solid(const World *w, double x, double y, double z);
int world_clear(const World *w, const double *a, const double *b);
int world_vidx(const World *w, double x, double y);
int world_cell(const World *w, double x, double y);
void world_cell_pos(const World *w, int c, double *x, double *y);
int world_free(const World *w, double x, double y);
int world_nearest_free(const World *w, int c);
int world_can_step(const World *w, int a, int b, int diag);
/* avoid: nav triples (x, y, r); cells inside cost extra (unstuck memory). NULL = Go behaviour */
int world_path(World *w, double sx, double sy, double gx, double gy, int max_expand, int32_t **path, int *npath, int *cpath,
               const double *avoid, int navoid);
int world_line_walkable(const World *w, double x0, double y0, double x1, double y1);
int world_main_near(World *w, int c);
int world_off_main(const World *w, double x, double y);
int world_escape_plan(World *w, double x, double y, double *launch, double *land);
int world_reachable(const World *w, double x, double y);
int world_in_bounds(const World *w, double x, double y);
double world_exposure(const World *w, double x, double y);
void world_queue_reserve(World *w, int n);

/* ------------------------------------------------------------- brain */

typedef struct Brain Brain;
Brain *brain_new(const Hello *h, World *w);
void brain_free(Brain *b);
/* experiment switches, "key=value,key=value"; unknown keys are an error (-1).
 * Defaults reproduce the Go bot bit for bit. */
int brain_set_opts(Brain *b, const char *opts);
void brain_act(Brain *b, Obs *o, Action *act);
const char *brain_mode(const Brain *b);

/* ------------------------------------------------------------- JSON */

int parse_hello(const char *s, size_t n, Hello *h);
int parse_obs(const char *s, size_t n, Obs *o);   /* 0 = ok */
int parse_tick(const char *s, size_t n);
int write_action(char *buf, int cap, const Action *a);

/* Go's math helpers where C differs */
double go_hypot(double p, double q);
double go_atan(double x);
double go_atan2(double y, double x);
double go_sin(double x);
double go_cos(double x);

#endif
