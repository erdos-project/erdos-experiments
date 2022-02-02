#!/bin/bash
set -x

FREQUENCY=30
MSG_SIZE=6220800    # Approx 6 MB, size of a 1080p camera frame.

NUM_WARMUP_MSGS=100
NUM_MSGS=10000

OUT_DIR="results/$(hostname)/"
TMP_DIR="/tmp/erdos-experiments/"

mkdir -p "$OUT_DIR"
mkdir -p "$TMP_DIR"

for type in "inter-process"
do
    csv_filename="$OUT_DIR/broadcast-latency-$type.csv"
    tmp_files=()
    for num_receivers in {1..5}
    do
        roscore &
        sleep 3

        filename="$TMP_DIR/broadcast-$num_receivers-$type.csv"
        tmp_files+=("$filename")
        if [ "$type" == "inter-process" ]; then
            rosrun latency csv_sink csv_sink samples $filename $NUM_WARMUP_MSGS $NUM_MSGS &
            for i in $(seq 1 $num_receivers)
            do
                rosrun latency receiver "receiver_$i" data samples &
            done
            rosrun latency binary_source source data $MSG_SIZE $NUM_WARMUP_MSGS $NUM_MSGS $FREQUENCY
        fi
        # Wait everything to shut down.
        sleep 10

        pkill -9 "receiver"
        pkill -9 "csv_sink"
        pkill -9 "roscore"
        pkill -9 "rosmaster"
        pkill -9 "rosout"

        python3 ../scripts/add_column_to_csv.py \
            -i $filename -o $filename -c "num_receivers" -v $num_receivers
    done

    # Merge files
    cat ${tmp_files[0]} > $csv_filename
    for filename in ${tmp_files[@]:1}
    do
        tail -n +2 $filename >> $csv_filename
    done
done
