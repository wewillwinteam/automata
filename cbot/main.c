/* stdin/stdout loop, same contract as the Go bot's cmd/bot/main.go:
 * answer hello with ready, act on the NEWEST queued observation only,
 * exit on end or EOF. Logs go to stderr. */
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "bot.h"

#define NAME "giddygiddyboy-c"

static char *buf;
static size_t blen, bcap;

/* read whatever stdin has; block if wait and nothing is buffered. -1 on EOF */
static int fill(int wait) {
    for (;;) {
        if (blen + 65536 + 1 > bcap) {
            bcap = bcap ? bcap * 2 : 1 << 20;
            while (blen + 65536 + 1 > bcap) bcap *= 2;
            buf = realloc(buf, bcap);
        }
        struct pollfd pf = {0, POLLIN, 0};
        int r = poll(&pf, 1, wait ? -1 : 0);
        if (r < 0 && errno == EINTR) continue;
        if (r <= 0) return 0;
        ssize_t n = read(0, buf + blen, bcap - blen - 1);
        if (n < 0) { if (errno == EINTR) continue; return -1; }
        if (n == 0) return -1;
        blen += (size_t)n;
        return 1;
    }
}

static void send_str(const char *s, size_t n) {
    size_t off = 0;
    while (off < n) {
        ssize_t w = write(1, s + off, n - off);
        if (w < 0) { if (errno == EINTR) continue; exit(0); }
        off += (size_t)w;
    }
}

static const char *find_type(const char *s, size_t n) {
    /* the "type" value of a top-level message */
    const char *p = s, *end = s + n;
    while (p + 7 < end) {
        if (!memcmp(p, "\"type\"", 6)) {
            p += 6;
            while (p < end && (*p == ' ' || *p == ':')) p++;
            return p < end && *p == '"' ? p + 1 : NULL;
        }
        p++;
    }
    return NULL;
}

int main(void) {
    World *world = NULL;
    Brain *brain = NULL;
    Hello hello;
    static Obs obs;
    char *latest = NULL;
    size_t latest_len = 0, latest_cap = 0;
    int eof = 0;
    /* CBOT_EVERY=1: answer every observation in order (replay/equivalence tests) */
    int every = getenv("CBOT_EVERY") != NULL;
    for (;;) {
        /* wait for at least one full line, then drain what else is queued */
        while (!memchr(buf ? buf : "", '\n', blen)) {
            if (eof || fill(1) < 0) return 0;
        }
        while (fill(0) > 0) {}
        int have = 0;
        size_t start = 0;
        for (;;) {
            char *nl = memchr(buf + start, '\n', blen - start);
            if (!nl) break;
            size_t ll = (size_t)(nl - (buf + start));
            const char *line = buf + start;
            const char *t = find_type(line, ll);
            if (t && !strncmp(t, "hello\"", 6)) {
                if (parse_hello(line, ll, &hello) == 0) {
                    const char *dir = hello.map_dir[0] ? hello.map_dir : "/arena";
                    world = world_load(dir);
                    if (!world) {
                        fprintf(stderr, "map: cannot load %s - playing on a flat map\n", dir);
                        world = world_flat();
                    }
                    brain_free(brain);
                    brain = brain_new(&hello, world);
                    if (brain_set_opts(brain, getenv("CBOT_OPTS")) != 0) fprintf(stderr, "bad CBOT_OPTS\n");
                }
                const char *ready = "{\"type\":\"ready\",\"name\":\"" NAME "\"}\n";
                send_str(ready, strlen(ready));
            } else if (t && !strncmp(t, "obs\"", 4)) {
                if (ll + 1 > latest_cap) { latest_cap = ll + 1024; latest = realloc(latest, latest_cap); }
                memcpy(latest, line, ll);
                latest_len = ll;
                have = 1;
                if (every) {
                    Action a1;
                    char o1[1024];
                    if (!brain || parse_obs(latest, latest_len, &obs) != 0) { memset(&a1, 0, sizeof a1); a1.tick = parse_tick(latest, latest_len); }
                    else brain_act(brain, &obs, &a1);
                    int k = write_action(o1, sizeof o1, &a1);
                    if (k > 0) {
                        o1[k - 1] = '\t';   /* replace the newline: append the mode */
                        send_str(o1, (size_t)k);
                        const char *md = brain ? brain_mode(brain) : "none";
                        send_str(md, strlen(md));
                        send_str("\n", 1);
                    }
                    have = 0;
                }
            } else if (t && !strncmp(t, "end\"", 4)) {
                return 0;
            }
            start += ll + 1;
        }
        memmove(buf, buf + start, blen - start);
        blen -= start;
        if (!have) continue;
        Action act;
        char out[1024];
        if (!brain || parse_obs(latest, latest_len, &obs) != 0) {
            memset(&act, 0, sizeof act);
            act.tick = parse_tick(latest, latest_len);
        } else {
            brain_act(brain, &obs, &act);
        }
        int n = write_action(out, sizeof out, &act);
        if (n > 0) send_str(out, (size_t)n);
    }
}
