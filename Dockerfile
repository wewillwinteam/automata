FROM rust:1.95@sha256:f49565f188ee00bc2a18dd418183f2c5f23ef7d6e691890517ed341a598f67c3 AS build
WORKDIR /src
COPY . .
RUN cargo build --release --locked \
 && install -m 0755 target/release/automata /automata

FROM debian:trixie-slim@sha256:a72b4433de1cf7d6bae0c4cee74d0547f58601a643b3375f070c51600d63b6b2
COPY --from=build --chmod=0755 /automata /usr/local/bin/automata
COPY --chmod=0644 bot.json /bot.json
ENV MECH_PARAMS=/bot.json
USER 65534:65534
WORKDIR /tmp
ENTRYPOINT ["/usr/local/bin/automata"]
