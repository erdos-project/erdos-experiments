use clap::Arg;
use erdos::{
    dataflow::{
        operators::{Concat, FlatMapOperator},
        stream::ExtractStream,
    },
    node::{Node, NodeId},
    Configuration, OperatorConfig,
};

use erdos_experiments::{
    messages::{BinaryData, LatencySample},
    operators::{binary_source::BinarySource, csv_sink::CSVSink},
};

/// Sets up a dataflow to measure latency when broadcasting messages.
/// num_receivers: number of receiver operators to which to broadcast.
/// msg_size: byte size of the message.
/// send_frequency: frequency at which to send messages, in Hz.
/// samples: number of samples to take which are written to CSV.
/// warmup_samples: number of samples to take before writing to CSV.
/// csv_filename: file to which to write results.
/// is_inter_process: whether operators are split across processes or colocated
///     within the same process (stresses TCP vs shared memory message passing).
/// node_id: the current node's ID.
fn setup_dataflow(
    num_receivers: usize,
    msg_size: usize,
    send_frequency: f32,
    samples: usize,
    warmup_samples: usize,
    csv_filename: impl ToString,
    node_id: NodeId,
    is_inter_process: bool,
) -> Option<ExtractStream<()>> {
    let mut assigned_node = 0;
    let node_increment = if is_inter_process { 1 } else { 0 };

    let source_stream = erdos::connect_source(
        move || BinarySource::new(msg_size, warmup_samples, samples, send_frequency),
        || {},
        OperatorConfig::new().node(assigned_node),
    );

    // Allows the runtime to co-located operators on the same OS thread, instead of
    // pushing messages across threads.
    let data_stream = erdos::connect_one_in_one_out(
        move || {
            FlatMapOperator::new(|x: &BinaryData| {
                let mut data = x.clone();
                data.send_time = erdos_experiments::ns_since_unix_epoch();
                std::iter::once(data)
            })
        },
        || {},
        OperatorConfig::new().node(assigned_node),
        &source_stream,
    );

    assigned_node += node_increment;

    let mut sample_streams = vec![];
    for _ in 0..num_receivers {
        let sample_stream = erdos::connect_one_in_one_out(
            move || {
                FlatMapOperator::new(move |x: &BinaryData| {
                    let recv_time = erdos_experiments::ns_since_unix_epoch();
                    let latency_sample = LatencySample {
                        latency_secs: (recv_time - x.send_time) as f64 / 1e9,
                        send_time: x.send_time as f64 / 1e9,
                        recv_time: recv_time as f64 / 1e9,
                        msg_size: x.size(),
                        is_inter_process,
                    };

                    std::iter::once(latency_sample)
                })
            },
            || {},
            OperatorConfig::new().node(assigned_node),
            &data_stream,
        );
        sample_streams.push(sample_stream);
        assigned_node += node_increment;
    }

    // Merging streams and writing to CSV can occur on the same node
    // (defaults to node 0).
    let init = sample_streams.pop().unwrap();
    let all_samples = sample_streams.into_iter().fold(init, |x, y| x.concat(&y));

    // Receive a top watermark when compute finishes.
    let csv_filename = csv_filename.to_string();
    let done_stream = erdos::connect_one_in_one_out(
        move || CSVSink::new(&csv_filename),
        || {},
        OperatorConfig::new().name("CSV Sink"),
        &all_samples,
    );

    if node_id == 0 {
        Some(ExtractStream::new(&done_stream))
    } else {
        None
    }
}

fn main() {
    let args = erdos::new_app("broadcast-latency")
        .arg(
            Arg::with_name("frequency")
                .short("f")
                .long("frequency")
                .takes_value(true)
                .default_value("10"),
        )
        .arg(
            Arg::with_name("samples")
                .short("n")
                .long("samples")
                .takes_value(true)
                .default_value("10000"),
        )
        .arg(
            Arg::with_name("warmup-samples")
                .short("w")
                .long("warmup-samples")
                .takes_value(true)
                .default_value("100"),
        )
        .arg(
            Arg::with_name("output")
                .short("o")
                .long("output")
                .help("Output CSV filename")
                .takes_value(true)
                .required(true),
        )
        .arg(
            Arg::with_name("num-receivers")
                .short("r")
                .long("num-receivers")
                .takes_value(true)
                .required(true),
        )
        .arg(
            Arg::with_name("msg-size")
                .short("m")
                .long("msg-size")
                .takes_value(true)
                .required(true),
        )
        .get_matches();

    let num_receivers: usize = args.value_of("num-receivers").unwrap().parse().unwrap();
    let msg_size: usize = args.value_of("msg-size").unwrap().parse().unwrap();
    let send_frequency: f32 = args.value_of("frequency").unwrap().parse().unwrap();
    let samples: usize = args.value_of("samples").unwrap().parse().unwrap();
    let warmup_samples: usize = args.value_of("warmup-samples").unwrap().parse().unwrap();
    let output: String = args.value_of("output").unwrap().parse().unwrap();
    let node_id: NodeId = args.value_of("index").unwrap().parse().unwrap();
    let data_addresses: Vec<_> = args
        .value_of("data-addresses")
        .unwrap()
        .split(",")
        .collect();
    let is_inter_process = data_addresses.len() > 1;

    if is_inter_process {
        assert_eq!(
            data_addresses.len(),
            num_receivers + 1,
            "Must launch num_receiver + 1 nodes to run in inter-process mode!",
        );
    }

    let mut node = Node::new(Configuration::from_args(&args));

    let extract_stream_opt = setup_dataflow(
        num_receivers,
        msg_size,
        send_frequency,
        samples,
        warmup_samples,
        output,
        node_id,
        is_inter_process,
    );

    if let Some(mut extract_stream) = extract_stream_opt {
        let node_handle = node.run_async();

        loop {
            let msg = extract_stream.read().unwrap();
            if msg.is_top_watermark() {
                tracing::info!("Shutting down ERDOS.");
                break;
            }
        }

        node_handle.shutdown().unwrap();
    } else {
        node.run();
    }
}
