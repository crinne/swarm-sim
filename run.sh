#!/bin/bash
set -e

BINARY_DIR="$(dirname "$0")/build"
LOSS=${1:-0.0}

echo "Starting swarm simulator (loss=${LOSS})"

$BINARY_DIR/proxy --loss $LOSS &
PROXY_PID=$!

sleep 0.5

$BINARY_DIR/gcs &
GCS_PID=$!

sleep 0.5

$BINARY_DIR/drone --id 1 --x 0   --y 0   &
$BINARY_DIR/drone --id 2 --x 50  --y 50  &
$BINARY_DIR/drone --id 3 --x -50 --y 50  &

echo "Swarm running — open http://localhost:8080"
echo "Press Ctrl+C to stop"

trap "kill $PROXY_PID $GCS_PID; pkill drone" EXIT
wait