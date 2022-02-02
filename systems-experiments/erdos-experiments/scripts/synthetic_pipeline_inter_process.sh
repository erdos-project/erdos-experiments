#!/usr/bin/env bash
# Measures latency of a synthetic AV pipeline at different scales across processes.
set -x

FREQUENCY=30
WARMUP_SAMPLES=100
SAMPLES=10000

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

CSV_FILENAME="$OUT_DIR/synthetic-pipeline-inter-process.csv"

tmp_files=()
for num_copies in {1..5}
do
    filename="$TMP_DIR/synthetic-pipeline-$num_copies-inter-process.csv"
    tmp_files+=("$filename")

    control_addresses="127.0.0.1:9000"
    data_addresses="127.0.0.1:9001"
    for i in $(seq 1 $((15 * $num_copies)))
    do
        control_addresses="$control_addresses,127.0.0.1:$((9000 + 2 * $i))"
        data_addresses="$data_addresses,127.0.0.1:$((9001 + 2 * $i))"
    done

    pids=()
    for i in $(seq 1 $((15 * $num_copies)))
    do
        cargo run --bin=synthetic_pipeline --release -- \
            --index=$i \
            --control-addresses=$control_addresses \
            --data-addresses=$data_addresses \
            --frequency=$FREQUENCY \
            --output=$filename \
            --samples=$SAMPLES \
            --warmup-samples=$WARMUP_SAMPLES \
            --threads=4 \
            --pipeline-copies=$num_copies &
        pids+=("$!")
    done

    cargo run --bin=synthetic_pipeline --release -- \
        --frequency=$FREQUENCY \
        --output=$filename \
        --samples=$SAMPLES \
        --warmup-samples=$WARMUP_SAMPLES \
        --threads=16 \
        --pipeline-copies=$num_copies

    python3 ../scripts/add_column_to_csv.py \
        -i $filename -o $filename -c "pipeline_copies" -v $num_copies

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