#!/bin/bash
set -x

FREQUENCY=30

NUM_WARMUP_MSGS=100
NUM_MSGS=10000

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/flink/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

csv_filename="$OUT_DIR/synthetic-pipeline-latency-intra-process.csv"
tmp_files=()
for pipeline_copies in {1..5}
do
    filename="$TMP_DIR/synthetic-pipeline-$pipeline_copies-intra-process.csv"
    tmp_files+=("$filename")
    mvn exec:java -Dexec.mainClass=flink_experiments.SyntheticPipeline \
        -Dexec.args="--frequency=$FREQUENCY --pipeline-copies=$pipeline_copies --samples=$NUM_MSGS --warmup-samples=$NUM_WARMUP_MSGS --output=$filename"
    python3 ../scripts/add_column_to_csv.py \
        -i $filename -o $filename -c "pipeline_copies" -v $pipeline_copies
done

# Merge files
cat ${tmp_files[0]} > $csv_filename
for filename in ${tmp_files[@]:1}
do
    tail -n +2 $filename >> $csv_filename
done

rm -rf $TMP_DIR
