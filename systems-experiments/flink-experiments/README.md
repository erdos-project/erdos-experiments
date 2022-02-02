# ROS Systems Experiments

This folder contains the code to reproduce the Flink results in figure 8,
which measures callback invocation delay for several communication patterns:

- Sending messages of varying sizes.
- Broadcasting messages to varying numbers of receivers.
- Performance for a synthetic AV pipeline with no-ops for computation.

## Executing the Experiments

1. Install maven and docker-compose.
2. From this directory, execute `bash scripts/run_all.sh`.

To execute a specific experiment, run the following script with `bash`:

- `scripts/broadcast_latency_inter_process.sh`: measures latency for
broadcasting messages across proceses to varying numbers of receivers.
- `scripts/broadcast_latency_intra_process.sh`: measures latency for
broadcasting messages to varying numbers of receivers collocated within the same process.
- `scripts/msg_size_latency_inter_process.sh`: measures latency for
sending messages of varying sizes across proceses.
- `scripts/msg_size_latency_intra_process.sh`: measures latency for
sending messages of varying sizes within a proceses.

## Retreiving results

The experiments will save results as CSVs in `results/$hostname/`, for example:
```
results/
└── erdos3
    ├── broadcast-latency-inter-process.csv
    ├── broadcast-latency-intra-process.csv
    ├── msg-size-latency-inter-process.csv
    ├── msg-size-latency-intra-process.csv
```

How each file relates to the figures:
- `broadcast-latency-inter-process.csv`: figure 8b, right half.
- `broadcast-latency-intra-process.csv`: figure 8b: left half.
- `msg-size-latency-inter-process.csv`: figure 8a: right half.
- `msg-size-latency-intra-process.csv`: figure 8a: left half.

## Plotting (TODO)
