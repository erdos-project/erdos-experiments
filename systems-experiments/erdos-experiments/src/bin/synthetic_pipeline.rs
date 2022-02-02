use clap::Arg;
use erdos::{
    dataflow::{
        operators::{Concat, FlatMapOperator},
        stream::{ExtractStream, Stream},
    },
    node::{Node, NodeId},
    Configuration, OperatorConfig,
};
use erdos_experiments::{
    messages::{BinaryData, LatencySample},
    operators::{binary_source::BinarySource, csv_sink::CSVSink, join::JoinOperator},
};

// Typical sizes of messages, taken from a trace of a Pylot run.
const CAMERA_MSG_SIZE: usize = 921600; // 6222274;
const LIDAR_MSG_SIZE: usize = 600_000;
const LOCALIZATION_MSG_SIZE: usize = 1_000;
const DETECTION_MSG_SIZE: usize = 4433;
const TRAFFIC_LIGHT_MSG_SIZE: usize = 246;
const LANE_DETECTION_MSG_SIZE: usize = 358;
const TRACKER_MSG_SIZE: usize = 123;
const PREDICTION_MSG_SIZE: usize = 600;
const PLANNING_MSG_SIZE: usize = 13099;
const CONTROL_MSG_SIZE: usize = 140;
const FUSION_MSG_SIZE: usize = 10_000;

fn connect_map(
    name: &str,
    msg_size: usize,
    assigned_node: NodeId,
    is_sensor: bool,
    stream: &Stream<BinaryData>,
) -> Stream<BinaryData> {
    erdos::connect_one_in_one_out(
        move || {
            FlatMapOperator::new(move |input_data: &BinaryData| {
                let mut output_data = BinaryData::new(msg_size);
                if !is_sensor {
                    output_data.send_time = input_data.send_time
                }
                std::iter::once(output_data)
            })
        },
        || {},
        OperatorConfig::new().name(name).node(assigned_node),
        stream,
    )
}

fn connect_join(
    name: &str,
    msg_size: usize,
    assigned_node: NodeId,
    left_stream: &Stream<BinaryData>,
    right_stream: &Stream<BinaryData>,
) -> Stream<BinaryData> {
    erdos::connect_two_in_one_out(
        move || {
            JoinOperator::new(move |x: &[BinaryData], y: &[BinaryData]| {
                if x.len() == 0 || y.len() == 0 {
                    return vec![];
                }
                assert_eq!(x.len(), 1);
                assert_eq!(y.len(), 1);
                let mut data = BinaryData::new(msg_size);
                data.send_time = x[0].send_time.min(y[0].send_time);
                vec![data]
            })
        },
        || {},
        OperatorConfig::new().node(assigned_node).name(name),
        left_stream,
        right_stream,
    )
}

fn setup_dataflow(
    pipeline_copies: usize,
    send_frequency: f32,
    samples: usize,
    warmup_samples: usize,
    csv_filename: String,
    node_id: NodeId,
    is_inter_process: bool,
) -> Option<ExtractStream<()>> {
    let node_increment = if is_inter_process { 1 } else { 0 };

    let mut assigned_node: NodeId = 0;
    let mut sample_streams = vec![];

    // Synchronously signals other operators to produce data.
    let source_stream = erdos::connect_source(
        move || BinarySource::new(0, warmup_samples, samples, send_frequency),
        || {},
        OperatorConfig::new().node(assigned_node),
    );
    for _ in 0..pipeline_copies {
        let camera_1_stream = connect_map(
            "Camera 1",
            CAMERA_MSG_SIZE,
            assigned_node,
            true,
            &source_stream,
        );
        assigned_node += node_increment;

        let camera_2_stream = connect_map(
            "Camera 2",
            CAMERA_MSG_SIZE,
            assigned_node,
            true,
            &source_stream,
        );
        assigned_node += node_increment;

        let lidar_stream =
            connect_map("LIDAR", LIDAR_MSG_SIZE, assigned_node, true, &source_stream);
        assigned_node += node_increment;

        let localization_stream = connect_map(
            "Localization",
            LOCALIZATION_MSG_SIZE,
            assigned_node,
            true,
            &source_stream,
        );
        assigned_node += node_increment;

        let _segmentation_stream = connect_map(
            "Segmentation",
            CAMERA_MSG_SIZE,
            assigned_node,
            false,
            &camera_1_stream,
        );
        assigned_node += node_increment;

        let detection_stream = connect_map(
            "Obstacle detection",
            DETECTION_MSG_SIZE,
            assigned_node,
            false,
            &camera_1_stream,
        );
        assigned_node += node_increment;

        let traffic_light_detection_stream = connect_map(
            "Traffic light detection",
            TRAFFIC_LIGHT_MSG_SIZE,
            assigned_node,
            false,
            &camera_2_stream,
        );
        assigned_node += node_increment;

        let lane_detection_stream = connect_map(
            "Lane detection",
            LANE_DETECTION_MSG_SIZE,
            assigned_node,
            false,
            &camera_2_stream,
        );
        assigned_node += node_increment;

        let tracker_stream = connect_join(
            "Tracker",
            TRACKER_MSG_SIZE,
            assigned_node,
            &camera_1_stream,
            &detection_stream,
        );
        assigned_node += node_increment;

        let lidar_traffic_light_fusion_stream = connect_join(
            "LIDAR traffic light fusion",
            FUSION_MSG_SIZE,
            assigned_node,
            &lidar_stream,
            &traffic_light_detection_stream,
        );
        assigned_node += node_increment;

        let tmp_fusion_stream = connect_join(
            "tmp fusion",
            FUSION_MSG_SIZE,
            assigned_node,
            &tracker_stream,
            &lidar_stream,
        );
        let main_fusion_stream = connect_join(
            "Fusion",
            FUSION_MSG_SIZE,
            assigned_node,
            &tmp_fusion_stream,
            &localization_stream,
        );
        assigned_node += node_increment;

        let prediction_stream = connect_join(
            "Prediction",
            PREDICTION_MSG_SIZE,
            assigned_node,
            &main_fusion_stream,
            &localization_stream,
        );
        assigned_node += node_increment;

        let tmp_planning_stream = connect_join(
            "tmp planning",
            PLANNING_MSG_SIZE,
            assigned_node,
            &prediction_stream,
            &lane_detection_stream,
        );
        let planning_stream = connect_join(
            "Planning",
            PLANNING_MSG_SIZE,
            assigned_node,
            &tmp_planning_stream,
            &lidar_traffic_light_fusion_stream,
        );
        assigned_node += node_increment;

        let control_stream = connect_join(
            "Control",
            CONTROL_MSG_SIZE,
            assigned_node,
            &planning_stream,
            &localization_stream,
        );

        assigned_node += node_increment;
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
            &control_stream,
        );
        sample_streams.push(sample_stream);
        assigned_node += node_increment;
    }

    // Merging streams and writing to CSV can occur on the same node
    // (defaults to node 0).
    let init = sample_streams.pop().unwrap();
    let all_samples = sample_streams.into_iter().fold(init, |x, y| x.concat(&y));

    // operators::connect_csv_sink(assigned_node, warmup_samples, csv_filename, sample_stream);
    let done_stream = erdos::connect_one_in_one_out(
        move || CSVSink::new(&csv_filename),
        || {},
        OperatorConfig::new().node(assigned_node),
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
            Arg::with_name("pipeline-copies")
                .short("p")
                .long("pipeline-copies")
                .takes_value(true)
                .default_value("1"),
        )
        .arg(
            Arg::with_name("output")
                .short("o")
                .long("output")
                .help("Output CSV filename")
                .takes_value(true)
                .required(true),
        )
        .get_matches();

    let send_frequency: f32 = args.value_of("frequency").unwrap().parse().unwrap();
    let samples: usize = args.value_of("samples").unwrap().parse().unwrap();
    let warmup_samples: usize = args.value_of("warmup-samples").unwrap().parse().unwrap();
    let pipeline_copies: usize = args.value_of("pipeline-copies").unwrap().parse().unwrap();
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
            15 * pipeline_copies + 1,
            "Must launch 15 * pipeline_copies + 1 nodes to run in inter-process mode!",
        );
    }

    let mut node = Node::new(Configuration::from_args(&args));

    let extract_stream_opt = setup_dataflow(
        pipeline_copies,
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
