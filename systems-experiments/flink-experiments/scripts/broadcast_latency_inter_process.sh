#!/bin/bash
set -x

FREQUENCY=30
MSG_SIZE=6220800    # Approx 6 MB, size of a 1080p camera frame.

NUM_WARMUP_MSGS=100
NUM_MSGS=10000

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/flink/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

csv_filename="$OUT_DIR/broadcast-latency-inter-process.csv"
tmp_files=()
for num_receivers in {1..5}
do
    filename="$TMP_DIR/broadcast-$num_receivers-inter-process.csv"
    tmp_files+=("$filename")
    bash scripts/run_broadcast_experiment.sh \
        $MSG_SIZE $num_receivers $NUM_MSGS $NUM_WARMUP_MSGS $FREQUENCY $filename
    python3 ../scripts/add_column_to_csv.py \
        -i $filename -o $filename -c "num_receivers" -v $num_receivers
done

# Merge files
cat ${tmp_files[0]} > $csv_filename
for filename in ${tmp_files[@]:1}
do
    tail -n +2 $filename >> $csv_filename
done

rm -rf $TMP_DIR
