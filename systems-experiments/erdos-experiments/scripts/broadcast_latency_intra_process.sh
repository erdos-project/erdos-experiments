#!/usr/bin/env bash
# Measures latency across threads for varying numbers of receivers.
set -x

FREQUENCY=30
WARMUP_SAMPLES=100
SAMPLES=10000
MSG_SIZE=6220800    # Approximately 6 MB, the size of a 1080p camera frame.

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

CSV_FILENAME="$OUT_DIR/broadcast-latency-intra-process.csv"

tmp_files=()
for num_receivers in {1..5}
do
    filename="$TMP_DIR/broadcast-latency-$num_receivers-intra-process.csv"
    tmp_files+=("$filename")

    cargo run --bin=broadcast_latency --release -- \
        --frequency=$FREQUENCY \
        --msg-size=$MSG_SIZE \
        --num-receivers=$num_receivers \
        --output=$filename \
        --samples=$SAMPLES \
        --warmup-samples=$WARMUP_SAMPLES \
        --threads=16
    
    python3 ../scripts/add_column_to_csv.py \
            -i $filename -o $filename -c "num_receivers" -v $num_receivers
done

# Merge files
cat ${tmp_files[0]} > $CSV_FILENAME
for filename in ${tmp_files[@]:1}
do
    tail -n +2 $filename >> $CSV_FILENAME
done