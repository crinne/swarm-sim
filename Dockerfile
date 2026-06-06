FROM alpine:3.22 AS builder

RUN apk add --no-cache build-base cmake

WORKDIR /src
COPY . .

RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --parallel \
    && ctest --test-dir build --output-on-failure

FROM alpine:3.22 AS runtime

RUN apk add --no-cache libstdc++ \
    && addgroup -S -g 10001 swarm \
    && adduser -S -D -H -u 10001 -G swarm swarm

COPY --from=builder /src/build/proxy /usr/local/bin/proxy
COPY --from=builder /src/build/gcs /usr/local/bin/gcs
COPY --from=builder /src/build/drone /usr/local/bin/drone

USER 10001:10001

EXPOSE 8080
EXPOSE 14550/udp
EXPOSE 14560/udp

ENTRYPOINT ["/usr/local/bin/gcs"]
