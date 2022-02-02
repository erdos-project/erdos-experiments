#!/usr/bin/env bash
# Measures latency across threads for different message sizes.
set -x

FREQUENCY=30
WARMUP_SAMPLES=100
SAMPLES=10000

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

CSV_FILENAME="$OUT_DIR/msg-size-latency-intra-process.csv"

tmp_files=()
for msg_size in 10000 100000 1000000 10000000
do
    filename="$TMP_DIR/msg-size-latency-$msg_size-intra-process.csv"
    tmp_files+=("$filename")

    cargo run --bin=broadcast_latency --release -- \
        --frequency=$FREQUENCY \
        --msg-size=$msg_size \
        --num-receivers=1 \
        --output=$filename \
        --samples=$SAMPLES \
        --warmup-samples=$WARMUP_SAMPLES \
        --threads=16
done

# Merge files
cat ${tmp_files[0]} > $CSV_FILENAME
for filename in ${tmp_files[@]:1}
do
    tail -n +2 $filename >> $CSV_FILENAME
done