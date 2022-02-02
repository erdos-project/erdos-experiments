# ERDOS Systems Experiments

This folder contains the code to reproduce the ERDOS results in figure 8,
which measures callback invocation delay for several communication patterns:

- Sending messages of varying sizes.
- Broadcasting messages to varying numbers of receivers.
- Performance for a synthetic AV pipeline with no-ops for computation.

## Executing the Experiments

1. [Install rust](https://www.rust-lang.org/tools/install).
2. Install the pandas python library `pip3 install pandas`.
3. From this directory, execute `bash scripts/run_all.sh`.

Note that you might see the following error message.
```
thread 'node-0' panicked at 'called `Result::unwrap()` on an `Err` value: SendError(DestroyedOperator(3008c544-99b4-6f2c-73c8-3a0a9bdb1622))thread '', node-0/home/peter/.cargo/git/checkouts/erdos-a16558f7c11460ad/3b1235a/erdos/src/node/operator_executors/mod.rs' panicked at ':called `Result::unwrap()` on an `Err` value: SendError(DestroyedOperator(3031b52e-4721-4f0b-7909-e8a4603dc447))278:14thread '
```
This is expected behavior due to issues with shutting down ERDOS,
and may be fixed in a future update.
It should have no impacts on the experiments.

To execute a specific experiment, run the following script with `bash`:

- `scripts/broadcast_latency_inter_process.sh`: measures latency for
broadcasting messages across proceses to varying numbers of receivers.
- `scripts/broadcast_latency_intra_process.sh`: measures latency for
broadcasting messages to varying numbers of receivers collocated within the same process.
- `scripts/msg_size_latency_inter_process.sh`: measures latency for
sending messages of varying sizes across proceses.
- `scripts/msg_size_latency_intra_process.sh`: measures latency for
sending messages of varying sizes within a proceses.
- `scripts/synthetic_pipeline_inter_process.sh`: measures end-to-end latency
for a synthetic AV pipeline in which operators are placed on different processes.
- `scripts/synthetic_pipeline_intra_process.sh`: measures end-to-end latency
for a synthetic AV pipeline in which operators are colocated within a process.

## Retreiving results

The experiments will save results as CSVs in `results/$hostname/`, for example:
```
results/
└── erdos3
    ├── broadcast-latency-inter-process.csv
    ├── broadcast-latency-intra-process.csv
    ├── msg-size-latency-inter-process.csv
    ├── msg-size-latency-intra-process.csv
    ├── synthetic-pipeline-inter-process.csv
    └── synthetic-pipeline-intra-process.csv
```

How each file relates to the figures:
- `broadcast-latency-inter-process.csv`: figure 8b, right half.
- `broadcast-latency-intra-process.csv`: figure 8b: left half.
- `msg-size-latency-inter-process.csv`: figure 8a: right half.
- `msg-size-latency-intra-process.csv`: figure 8a: left half.
- `synthetic-pipeline-inter-process.csv`: figure 8c: right half.
- `synthetic-pipeline-intra-process.csv`: figure 8c: left half.

Note that the synthetic pipeline results should have much lower latency than
figure 8c in the paper draft due to improvements in ERDOS's implementation.

## Plotting (TODO)
