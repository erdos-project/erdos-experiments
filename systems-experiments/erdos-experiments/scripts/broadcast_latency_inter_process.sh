#!/usr/bin/env bash
# Measures latency across processes for varying numbers of receivers.
set -x

FREQUENCY=30
WARMUP_SAMPLES=100
SAMPLES=10000
MSG_SIZE=6220800    # Approximately 6 MB, the size of a 1080p camera frame.

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

CSV_FILENAME="$OUT_DIR/broadcast-latency-inter-process.csv"

control_addresses="127.0.0.1:9000"
data_addresses="127.0.0.1:9001"

tmp_files=()

for num_receivers in {1..5}
do
    filename="$TMP_DIR/broadcast-latency-$num_receivers-inter-process.csv"
    tmp_files+=("$filename")

    control_addresses="$control_addresses,127.0.0.1:$((9000 + 2 * $num_receivers))"
    data_addresses="$data_addresses,127.0.0.1:$((9001 + 2 * $num_receivers))"

    # Launch receiver processes.
    pids=()
    for i in $(seq 1 $num_receivers)
    do
        cargo run --bin=broadcast_latency --release -- \
            --index=$i \
            --control-addresses=$control_addresses \
            --data-addresses=$data_addresses \
            --frequency=$FREQUENCY \
            --msg-size=$MSG_SIZE \
            --num-receivers=$num_receivers \
            --output=$filename \
            --samples=$SAMPLES \
            --warmup-samples=$WARMUP_SAMPLES \
            --threads=4 &
        pids+=("$!")
    done

    cargo run --bin=broadcast_latency --release -- \
        --index=0 \
        --control-addresses=$control_addresses \
        --data-addresses=$data_addresses \
        --frequency=$FREQUENCY \
        --msg-size=$MSG_SIZE \
        --num-receivers=$num_receivers \
        --output=$filename \
        --samples=$SAMPLES \
        --warmup-samples=$WARMUP_SAMPLES \
        --threads=4

    python3 ../scripts/add_column_to_csv.py \
            -i $filename -o $filename -c "num_receivers" -v $num_receivers
    
    # Wait for addresses to free up
    for pid in ${pids[@]}
    do
        kill $pid
    done
    sleep 1
done

# Merge files
cat ${tmp_files[0]} > $CSV_FILENAME
for filename in ${tmp_files[@]:1}
do
    tail -n +2 $filename >> $CSV_FILENAME
done