# Prebuilt static linux/amd64 binary from the MIT Apex-Go v2 source in this repo.
FROM scratch
COPY apex-go /mechbot
# Randomized zig-zag only while under ranged gun fire (existing dodge/jink path).
ENV BOT_PARAMS='{"Jink":0.6,"OpenSecs":45}'
USER 65534:65534
ENTRYPOINT ["/mechbot"]
