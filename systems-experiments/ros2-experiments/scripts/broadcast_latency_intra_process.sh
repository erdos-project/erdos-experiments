!#!/bin/bash

FREQUENCY="30.0"
MSG_SIZE=6220800    # Approx 6 MB, size of a 1080p camera frame.

NUM_WARMUP_MSGS=100
NUM_MSGS=10000

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

TOTAL_MSGS=$(($NUM_WARMUP_MSGS + $NUM_MSGS))

colcon build
. install/setup.bash

csv_filename="$OUT_DIR/broadcast-latency-intra-process.csv"    
tmp_files=()
for num_receivers in {1..5}
do
    filename="$TMP_DIR/broadcast-$num_receivers-intra-process.csv"    
    tmp_files+=("$filename")

    ros2 run latency broadcast $num_receivers \
        --ros-args \
        -p msg_size:=$MSG_SIZE \
        -p num_msgs:=$TOTAL_MSGS \
        -p frequency:=$FREQUENCY \
        -p warmup_samples:=$NUM_WARMUP_MSGS \
        -p samples:=$NUM_MSGS \
        -p is_inter_process:=false \
        -p filename:=$filename

    python3 ../scripts/add_column_to_csv.py \
            -i $filename -o $filename -c "num_receivers" -v $num_receivers
done

# Merge files    
cat ${tmp_files[0]} > $csv_filename    
for filename in ${tmp_files[@]:1}    
do    
    tail -n +2 $filename >> $csv_filename    
done
