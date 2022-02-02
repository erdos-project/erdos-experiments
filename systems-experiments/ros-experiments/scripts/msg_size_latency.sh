#!/bin/bash
set -x

FREQUENCY=30

NUM_WARMUP_MSGS=100
NUM_MSGS=10000

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

for type in "inter-process"
do
    csv_filename="$OUT_DIR/msg-size-latency-$type.csv"
    tmp_files=()
    for msg_size in 10000 100000 1000000 10000000
    do
        roscore &
        sleep 3

        filename="$TMP_DIR/msg-size-$msg_size-$type.csv"
        tmp_files+=("$filename")
        if [ "$type" == "inter-process" ]; then
            rosrun latency csv_sink csv_sink samples $filename $NUM_WARMUP_MSGS $NUM_MSGS &
            rosrun latency receiver receiver data samples &
            rosrun latency binary_source source data $msg_size $NUM_WARMUP_MSGS $NUM_MSGS $FREQUENCY
        fi
        # Wait everything to shut down.
        sleep 10

        pkill -9 "receiver"
        pkill -9 "csv_sink"
        pkill -9 "roscore"
        pkill -9 "rosmaster"
        pkill -9 "rosout"
    done

    # Merge files
    cat ${tmp_files[0]} > $csv_filename
    for filename in ${tmp_files[@]:1}
    do
        tail -n +2 $filename >> $csv_filename
    done
    rm -rf $TMP_DIR
done
