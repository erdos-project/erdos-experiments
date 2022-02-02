#!/usr/bin/env bash
# Measures latency of a synthetic AV pipeline at different scales, within a process.
set -x

FREQUENCY=30
WARMUP_SAMPLES=10
SAMPLES=10

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

CSV_FILENAME="$OUT_DIR/synthetic-pipeline-intra-process.csv"

tmp_files=()
for num_copies in {1..5}
do
    filename="$TMP_DIR/synthetic-pipeline-$num_copies-intra-process.csv"
    tmp_files+=("$filename")

    cargo run --bin=synthetic_pipeline --release -- \
        --frequency=$FREQUENCY \
        --output=$filename \
        --samples=$SAMPLES \
        --warmup-samples=$WARMUP_SAMPLES \
        --threads=45 \
        --pipeline-copies=$num_copies

    python3 ../scripts/add_column_to_csv.py \
        -i $filename -o $filename -c "pipeline_copies" -v $num_copies
done

# Merge files
cat ${tmp_files[0]} > $CSV_FILENAME
for filename in ${tmp_files[@]:1}
do
    tail -n +2 $filename >> $CSV_FILENAME
done