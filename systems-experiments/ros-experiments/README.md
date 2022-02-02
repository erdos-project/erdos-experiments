# ROS Systems Experiments

This folder contains the code to reproduce the ROS results in figure 8,
which measures callback invocation delay for several communication patterns:

- Sending messages of varying sizes.
- Broadcasting messages to varying numbers of receivers.
- Performance for a synthetic AV pipeline with no-ops for computation.

## Executing the Experiments

1. Install docker.
2. From this directory, execute `bash scripts/run_all.sh`.

To execute a specific experiment, run the following script with `bash`:

- `scripts/broadcast_latency_docker.sh`: measures latency for
broadcasting messages across proceses to varying numbers of receivers.
- `scripts/msg_size_latency_docker.sh`: measures latency for
sending messages of varying sizes across proceses.

## Retreiving results

The experiments will save results as CSVs in `results/$hostname/`, for example:
```
results/
└── erdos3
    ├── broadcast-latency-inter-process.csv
    └── msg-size-latency-inter-process.csv
```

How each file relates to the figures:
- `broadcast-latency-inter-process.csv`: figure 8b, right half.
- `msg-size-latency-inter-process.csv`: figure 8a: right half.

## Plotting (TODO)

