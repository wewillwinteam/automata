FROM rust:1.95@sha256:f49565f188ee00bc2a18dd418183f2c5f23ef7d6e691890517ed341a598f67c3 AS build
WORKDIR /src
COPY . .
RUN cargo build --release --locked \
 && install -m 0755 target/release/automata /automata
# The free-for-all bot (try_again's C port). -ffp-contract=off keeps it
# bit-identical to the Go bot it was ported from.
RUN cd cbot && gcc -O2 -ffp-contract=off -static -o /cbot main.c brain.c world.c parse.c gomath.c -lm

FROM debian:trixie-slim@sha256:a72b4433de1cf7d6bae0c4cee74d0547f58601a643b3375f070c51600d63b6b2
COPY --from=build --chmod=0755 /automata /usr/local/bin/automata
COPY --from=build --chmod=0755 /cbot /usr/local/bin/cbot
COPY --chmod=0644 bot.json /bot.json
ENV MECH_PARAMS=/bot.json
# automata plays until it sees mechs hurting each other (a free-for-all),
# then relays the answers of this bot, which it has fed every observation.
ENV HYBRID_PEER=/usr/local/bin/cbot
USER 65534:65534
WORKDIR /tmp
ENTRYPOINT ["/usr/local/bin/automata"]
