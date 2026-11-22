# automata

A Mech Royale (`mechfight/1`) bot for the Builders Buildathon.

**This is not our code.** It is `automata`, the Rust bot by **th3r0n**, copied
unchanged from [Th3r0n/mech-royale-bot](https://github.com/Th3r0n/mech-royale-bot)
(the `automata/` directory at commit `ccb6488`, "back to record settings",
2026-11-22). It is used under the MIT License; the original copyright notice is
kept in [`LICENSE`](LICENSE).

The tunable parameters are in `bot.json`, read through `MECH_PARAMS`.

```sh
docker build --platform linux/amd64 -t bot .
```

## Hybrid (this branch)

Two bots in one image, one answer per tick:

- **automata** (Rust, above) plays every match from the start. It is the
  stronger bot against the practice squad.
- **cbot/** is the C bot from `try_again` (a bit-identical port of the
  giddygiddyboy Go bot, MIT, see `cbot/LICENSE`). It runs as a child process
  (`HYBRID_PEER`) and is fed every line the server sends, so its state is
  current at all times.
- automata's free-for-all detector always runs. It fires when two mechs die
  that we did not kill (before 75 s), or after three signs of mechs hurting each
  other. The house squad never does either. From then on the
  child's answers are sent instead, because it is the stronger bot in a
  free-for-all.

Switches for testing: `HYBRID_FORCE=1` (relay from the first tick),
`HYBRID_LEAD_S` (let the child play the opening), `FFA_SIG_N` / `FFA_DEATH_N`
(detector thresholds, default 3 / 2).
