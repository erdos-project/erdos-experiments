#!/usr/bin/env bash

DIR=$(dirname $0)

bash "$DIR/broadcast_latency_inter_process.sh"
bash "$DIR/broadcast_latency_intra_process.sh"
bash "$DIR/msg_size_latency_inter_process.sh"
bash "$DIR/msg_size_latency_intra_process.sh"
bash "$DIR/synthetic_pipeline_inter_process.sh"
bash "$DIR/synthetic_pipeline_intra_process.sh"
