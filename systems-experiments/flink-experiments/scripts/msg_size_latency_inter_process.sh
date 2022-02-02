#!/bin/bash
set -x

FREQUENCY=30

NUM_WARMUP_MSGS=100
NUM_MSGS=10000

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/flink/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

csv_filename="$OUT_DIR/msg-size-latency-inter-process.csv"
tmp_files=()
for msg_size in 10000 100000 1000000 10000000
do
    filename="$TMP_DIR/msg-size-$msg_size-inter-process.csv"
    tmp_files+=("$filename")
    bash scripts/run_broadcast_experiment.sh \
        $msg_size 1 $NUM_MSGS $NUM_WARMUP_MSGS $FREQUENCY $filename
done

# Merge files
cat ${tmp_files[0]} > $csv_filename
for filename in ${tmp_files[@]:1}
do
    tail -n +2 $filename >> $csv_filename
done

rm -rf $TMP_DIR
