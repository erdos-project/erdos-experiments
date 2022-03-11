!#!/bin/bash

FREQUENCY="30.0"

NUM_WARMUP_MSGS=100
NUM_MSGS=10000

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

TOTAL_MSGS=$(($NUM_WARMUP_MSGS + $NUM_MSGS))

colcon build
. install/setup.bash

csv_filename="$OUT_DIR/msg-size-latency-intra-process.csv"
tmp_files=()
for msg_size in 10000 100000 1000000 10000000
do
    filename="$TMP_DIR/msg-size-$msg_size-intra-process.csv"    
    tmp_files+=("$filename")

    ros2 run latency broadcast 1 \
        --ros-args \
        -p msg_size:=$msg_size \
        -p num_msgs:=$TOTAL_MSGS \
        -p frequency:=$FREQUENCY \
        -p warmup_samples:=$NUM_WARMUP_MSGS \
        -p samples:=$NUM_MSGS \
        -p is_inter_process:=false \
        -p filename:=$filename
done

# Merge files    
cat ${tmp_files[0]} > $csv_filename    
for filename in ${tmp_files[@]:1}    
do    
    tail -n +2 $filename >> $csv_filename    
done