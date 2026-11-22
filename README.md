# pilot-x

A game bot that answers one JSON line per observation on stdin/stdout.

Two bots run in one image, with one answer per tick:

- the main bot (Rust, `src/`) plays every match from the start;
- a second bot (C, `cbot/`) runs as a child process (`HYBRID_PEER`), is fed every
  line the server sends, and takes over once the main bot detects a match where
  everyone fights everyone.

Tunable parameters are in `bot.json`, read through `MECH_PARAMS`.

```sh
docker build --platform linux/amd64 -t bot .
```

Switches for testing: `HYBRID_FORCE=1` (relay from the first tick),
`HYBRID_LEAD_S` (let the child play the opening), `FFA_SIG_N` / `FFA_DEATH_N`
(detector thresholds, default 3 / 2).

Contains code used under the MIT License; the copyright notices are in
[`LICENSE`](LICENSE) and [`cbot/LICENSE`](cbot/LICENSE).
