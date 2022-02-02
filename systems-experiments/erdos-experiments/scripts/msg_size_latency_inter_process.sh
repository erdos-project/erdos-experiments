#!/usr/bin/env bash
# Measures latency across processes for different messages sizes.
set -x

FREQUENCY=30
WARMUP_SAMPLES=100
SAMPLES=10000

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

CSV_FILENAME="$OUT_DIR/msg-size-latency-inter-process.csv"

CONTROL_ADDRESSES="127.0.0.1:9000,127.0.0.1:9002"
DATA_ADDRESSES="127.0.0.1:9001,127.0.0.1:9003"

tmp_files=()

for msg_size in 10000 100000 1000000 10000000
do
    filename="$TMP_DIR/msg-size-latency-$msg_size-inter-process.csv"
    tmp_files+=("$filename")

    cargo run --bin=broadcast_latency --release -- \
        --index=1 \
        --control-addresses=$CONTROL_ADDRESSES \
        --data-addresses=$DATA_ADDRESSES \
        --frequency=$FREQUENCY \
        --msg-size=$msg_size \
        --num-receivers=1 \
        --output=$filename \
        --samples=$SAMPLES \
        --warmup-samples=$WARMUP_SAMPLES \
        --threads=4 &
    pid=$!

    cargo run --bin=broadcast_latency --release -- \
        --index=0 \
        --control-addresses=$CONTROL_ADDRESSES \
        --data-addresses=$DATA_ADDRESSES \
        --frequency=$FREQUENCY \
        --msg-size=$msg_size \
        --num-receivers=1 \
        --output=$filename \
        --samples=$SAMPLES \
        --warmup-samples=$WARMUP_SAMPLES \
        --threads=4
    
    # Cleanup
    kill $pid
    sleep 1
done

# Merge files
cat ${tmp_files[0]} > $CSV_FILENAME
for filename in ${tmp_files[@]:1}
do
    tail -n +2 $filename >> $CSV_FILENAME
done