package flink_experiments;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Vector;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import org.apache.flink.api.common.functions.MapFunction;
import org.apache.flink.streaming.api.datastream.DataStream;
import org.apache.flink.streaming.api.environment.StreamExecutionEnvironment;
import org.apache.flink.streaming.api.functions.co.CoFlatMapFunction;
import org.apache.flink.util.Collector;

import flink_experiments.messages.*;
import flink_experiments.operators.*;

public class SyntheticPipeline {
    final static int CAMERA_MSG_SIZE = 921600; // 6222274;
    final static int LIDAR_MSG_SIZE = 600_000;
    final static int LOCALIZATION_MSG_SIZE = 1_000;
    final static int DETECTION_MSG_SIZE = 4433;
    final static int TRAFFIC_LIGHT_MSG_SIZE = 246;
    final static int LANE_DETECTION_MSG_SIZE = 358;
    final static int TRACKER_MSG_SIZE = 123;
    final static int PREDICTION_MSG_SIZE = 600;
    final static int PLANNING_MSG_SIZE = 13099;
    final static int CONTROL_MSG_SIZE = 140;
    final static int FUSION_MSG_SIZE = 10_000;

    public static void main(String[] args) throws Exception {
        Options options = new Options();

        Option pipeline_copies_arg = new Option("c", "pipeline-copies", true, "Number of pipelines to run.");
        pipeline_copies_arg.setRequired(true);
        options.addOption(pipeline_copies_arg);

        Option frequency_arg = new Option("f", "frequency", true, "Frequency");
        frequency_arg.setRequired(true);
        options.addOption(frequency_arg);

        Option samples_arg = new Option("n", "samples", true, "Number of samples to take.");
        samples_arg.setRequired(true);
        options.addOption(samples_arg);

        Option warmup_samples_arg = new Option("w", "warmup-samples", true, "Number of samples used to warm up");
        warmup_samples_arg.setRequired(true);
        options.addOption(warmup_samples_arg);

        Option is_inter_process_arg = new Option("i", "inter-process", false,
                "Signals that messages should be sent across processes");
        options.addOption(is_inter_process_arg);

        Option output_arg = new Option("o", "output", true, "Output CSV filename");
        output_arg.setRequired(true);
        options.addOption(output_arg);

        CommandLineParser parser = new DefaultParser();
        HelpFormatter formatter = new HelpFormatter();
        CommandLine cmd = null;

        try {
            cmd = parser.parse(options, args);
        } catch (ParseException e) {
            System.out.println(e.getMessage());
            formatter.printHelp("SyntheticPipeline", options);

            System.exit(1);
        }

        System.out.println(cmd.getOptionValue("pipeline-copies"));
        final int pipeline_copies = Integer.parseInt(cmd.getOptionValue("pipeline-copies"));
        final double send_frequency = Double.parseDouble(cmd.getOptionValue("frequency"));
        final int samples = Integer.parseInt(cmd.getOptionValue("samples"));
        final int warmup_samples = Integer.parseInt(cmd.getOptionValue("warmup-samples"));
        final String csv_filename = cmd.getOptionValue("output");
        final boolean is_inter_process = cmd.hasOption("inter-process");

        SyntheticPipeline.run_experiment(pipeline_copies, send_frequency, samples, warmup_samples,
                csv_filename, is_inter_process);
    }

    private static DataStream<BinaryData> connect_map(DataStream<BinaryData> stream, final String name,
            final int msg_size,
            final boolean is_sensor,
            boolean is_inter_process) {
        String slot_sharing_group = is_inter_process ? name : "Source";
        return stream.map(new MapFunction<BinaryData, BinaryData>() {
            @Override
            public BinaryData map(BinaryData value) throws Exception {
                BinaryData data = new BinaryData(msg_size);
                if (!is_sensor) {
                    data.send_time_ns = value.send_time_ns;
                }
                return data;
            }
        }).slotSharingGroup(slot_sharing_group);
    }

    private static DataStream<BinaryData> connect_join(DataStream<BinaryData> left_stream,
            DataStream<BinaryData> right_stream, final String name, final int msg_size, boolean is_inter_process) {
        String slot_sharing_group = is_inter_process ? name : "Source";
        return left_stream.connect(right_stream).flatMap(new CoFlatMapFunction<BinaryData, BinaryData, BinaryData>() {
            Deque<BinaryData> left_msgs = new ArrayDeque<BinaryData>();
            Deque<BinaryData> right_msgs = new ArrayDeque<BinaryData>();

            @Override
            public void flatMap1(BinaryData left_msg, Collector<BinaryData> out) throws Exception {
                BinaryData right_msg = this.right_msgs.pollFirst();
                if (right_msg != null) {
                    BinaryData data = new BinaryData(msg_size);
                    data.send_time_ns = Math.min(left_msg.send_time_ns, right_msg.send_time_ns);
                    out.collect(data);
                } else {
                    this.left_msgs.addLast(left_msg);
                }
            }

            @Override
            public void flatMap2(BinaryData right_msg, Collector<BinaryData> out) throws Exception {
                BinaryData left_msg = this.left_msgs.pollFirst();
                if (left_msg != null) {
                    BinaryData data = new BinaryData(msg_size);
                    data.send_time_ns = Math.min(left_msg.send_time_ns, right_msg.send_time_ns);
                    out.collect(data);
                } else {
                    this.right_msgs.addLast(right_msg);
                }
            }
        }).slotSharingGroup(slot_sharing_group);
    }

    private static void run_experiment(int pipeline_copies, double send_frequency, int num_msgs,
            int num_warmup_msgs, String csv_filename, final boolean is_inter_process) {

        final StreamExecutionEnvironment env = StreamExecutionEnvironment.getExecutionEnvironment();
        env.setBufferTimeout(0L);

        // Single source to synchronize sensors.
        DataStream<BinaryData> source_stream = env
                .addSource(new BinarySource(0, num_warmup_msgs, num_msgs, send_frequency), "Source")
                .slotSharingGroup("Source");

        Vector<DataStream<LatencySample>> latency_streams = new Vector<DataStream<LatencySample>>();

        for (int pipeline_copy = 0; pipeline_copy < pipeline_copies; pipeline_copy++) {

            DataStream<BinaryData> camera_1_stream = connect_map(source_stream,
                    String.format("Pipeline %d: Camera 1", pipeline_copy), CAMERA_MSG_SIZE, true, is_inter_process);

            DataStream<BinaryData> camera_2_stream = connect_map(source_stream,
                    String.format("Pipeline %d: Camera 2", pipeline_copy), CAMERA_MSG_SIZE, true, is_inter_process);

            DataStream<BinaryData> lidar_stream = connect_map(source_stream,
                    String.format("Pipeline %d: LIDAR", pipeline_copy), LIDAR_MSG_SIZE, true, is_inter_process);

            DataStream<BinaryData> localization_stream = connect_map(source_stream,
                    String.format("Pipeline %d: Localization", pipeline_copy), LOCALIZATION_MSG_SIZE, true,
                    is_inter_process);

            DataStream<BinaryData> segmentation_stream = connect_map(camera_1_stream,
                    String.format("Pipeline %d: Segmentation", pipeline_copy), CAMERA_MSG_SIZE, false,
                    is_inter_process);

            DataStream<BinaryData> detection_stream = connect_map(camera_1_stream,
                    String.format("Pipeline %d: Obstacle detection", pipeline_copy), DETECTION_MSG_SIZE, false,
                    is_inter_process);

            DataStream<BinaryData> traffic_light_detection_stream = connect_map(camera_2_stream,
                    String.format("Pipeline %d: Traffic light detection", pipeline_copy), TRAFFIC_LIGHT_MSG_SIZE, false,
                    is_inter_process);

            DataStream<BinaryData> lane_detection_stream = connect_map(camera_2_stream,
                    String.format("Pipeline %d: Lane detection", pipeline_copy), LANE_DETECTION_MSG_SIZE, false,
                    is_inter_process);

            DataStream<BinaryData> tracker_stream = connect_join(camera_1_stream, detection_stream,
                    String.format("Pipeline %d: Tracker", pipeline_copy), TRACKER_MSG_SIZE, is_inter_process);

            DataStream<BinaryData> lidar_traffic_light_fusion_stream = connect_join(lidar_stream,
                    traffic_light_detection_stream,
                    String.format("Pipeline %d: LIDAR traffic light_fusion", pipeline_copy), FUSION_MSG_SIZE,
                    is_inter_process);

            DataStream<BinaryData> tmp_fusion_stream = connect_join(tracker_stream, lidar_stream,
                    String.format("Pipeline %d: Fusion", pipeline_copy), FUSION_MSG_SIZE, is_inter_process);

            DataStream<BinaryData> main_fusion_stream = connect_join(tmp_fusion_stream, localization_stream,
                    String.format("Pipeline %d: Fusion", pipeline_copy), FUSION_MSG_SIZE, is_inter_process);

            DataStream<BinaryData> prediction_stream = connect_join(main_fusion_stream, localization_stream,
                    String.format("Pipeline %d: Prediction", pipeline_copy), PREDICTION_MSG_SIZE, is_inter_process);

            DataStream<BinaryData> tmp_planning_stream = connect_join(prediction_stream, lane_detection_stream,
                    String.format("Pipeline %d: Planning", pipeline_copy), PLANNING_MSG_SIZE, is_inter_process);

            DataStream<BinaryData> planning_stream = connect_join(tmp_planning_stream,
                    lidar_traffic_light_fusion_stream, String.format("Pipeline %d: Planning", pipeline_copy),
                    PLANNING_MSG_SIZE, is_inter_process);

            DataStream<BinaryData> control_stream = connect_join(planning_stream, localization_stream,
                    String.format("Pipeline %d: Control", pipeline_copy), CONTROL_MSG_SIZE, is_inter_process);

            String slot_sharing_group = is_inter_process ? String.format("Pipeline %d: receiver", pipeline_copy)
                    : "Source";
            DataStream<LatencySample> latency_stream = control_stream.map(new MapFunction<BinaryData, LatencySample>() {
                @Override
                public LatencySample map(BinaryData data) throws Exception {
                    double recv_time = (double) System.nanoTime() / 1e9;
                    double send_time = (double) data.send_time_ns / 1e9;
                    double serialize_start_time = (double) data.serialize_start_time_ns / 1e9;
                    double serialize_end_time = (double) data.serialize_end_time_ns / 1e9;
                    double deserialize_start_time = (double) data.deserialize_start_time_ns / 1e9;
                    double deserialize_end_time = (double) data.deserialize_end_time_ns / 1e9;
                    return new LatencySample(recv_time - send_time, send_time, recv_time, serialize_start_time,
                            serialize_end_time, deserialize_start_time, deserialize_end_time, data.data.length,
                            is_inter_process);
                }
            }).slotSharingGroup(slot_sharing_group);
            latency_streams.add(latency_stream);
        }

        DataStream<LatencySample> merged_latency_stream = latency_streams.get(0);
        for (int i = 1; i < latency_streams.size(); i++) {
            merged_latency_stream = merged_latency_stream.union(latency_streams.get(i));
        }

        String slot_sharing_group = is_inter_process ? "Sink" : "Source";
        merged_latency_stream.addSink(new CSVSink(num_warmup_msgs, csv_filename)).slotSharingGroup(slot_sharing_group);

        try {
            env.execute();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
