package flink_experiments;

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

import flink_experiments.messages.*;
import flink_experiments.messages.BinaryData.BinaryDataSerializer;
import flink_experiments.operators.*;

public class BroadcastExperiment {
    public static void main(String[] args) throws Exception {
        Options options = new Options();

        Option msg_size_arg = new Option("m", "msg-size", true, "Message size");
        msg_size_arg.setRequired(true);
        options.addOption(msg_size_arg);

        Option num_receivers_arg = new Option("r", "num-receivers", true, "Number of receivers to broadcast to.");
        num_receivers_arg.setRequired(true);
        options.addOption(num_receivers_arg);

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

        Option analyze_performance_arg = new Option("a", "analyze-performance", false,
                "Whether to use a custom serializer to measure serialization costs and FLink overhead.");
        options.addOption(analyze_performance_arg);

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
            formatter.printHelp("BroadcastExperiment", options);

            System.exit(1);
        }

        final int msg_size = Integer.parseInt(cmd.getOptionValue("msg-size"));
        final int num_receivers = Integer.parseInt(cmd.getOptionValue("num-receivers"));
        final double send_frequency = Double.parseDouble(cmd.getOptionValue("frequency"));
        final int samples = Integer.parseInt(cmd.getOptionValue("samples"));
        final int warmup_samples = Integer.parseInt(cmd.getOptionValue("warmup-samples"));
        final String csv_filename = cmd.getOptionValue("output");
        final boolean is_inter_process = cmd.hasOption("inter-process");
        final boolean analyze_performance = cmd.hasOption("analyze-performance");

        BroadcastExperiment.run_experiment(num_receivers, msg_size, send_frequency, samples, warmup_samples,
                csv_filename, is_inter_process, analyze_performance);
    }

    private static void run_experiment(int num_receivers, final int msg_size, double send_frequency, int num_msgs,
            int num_warmup_msgs, String csv_filename, final boolean is_inter_process,
            final boolean analyze_performance) {

        final StreamExecutionEnvironment env = StreamExecutionEnvironment.getExecutionEnvironment();
        env.setBufferTimeout(0L);

        if (analyze_performance) {
            env.registerTypeWithKryoSerializer(BinaryData.class, BinaryDataSerializer.class);
        }

        DataStream<BinaryData> source_stream = env
                .addSource(new BinarySource(msg_size, num_warmup_msgs, num_msgs, send_frequency), "Source")
                .slotSharingGroup("Source");

        // Mirror ERDOS experiment by updating time in a map operator.
        DataStream<BinaryData> data_stream = source_stream.map(new MapFunction<BinaryData, BinaryData>() {
            @Override
            public BinaryData map(BinaryData value) throws Exception {
                value.update_send_time();
                return value;
            }
        }).slotSharingGroup("Source");

        Vector<DataStream<LatencySample>> latency_streams = new Vector<DataStream<LatencySample>>();
        for (int i = 0; i < num_receivers; i++) {
            String slot_sharing_group = is_inter_process ? "Receiver " + Integer.toString(i) : "Source";
            DataStream<LatencySample> latency_stream = data_stream.map(new MapFunction<BinaryData, LatencySample>() {
                @Override
                public LatencySample map(BinaryData data) throws Exception {
                    double recv_time = (double) System.nanoTime() / 1e9;
                    double send_time = (double) data.send_time_ns / 1e9;
                    double serialize_start_time = (double) data.serialize_start_time_ns / 1e9;
                    double serialize_end_time = (double) data.serialize_end_time_ns / 1e9;
                    double deserialize_start_time = (double) data.deserialize_start_time_ns / 1e9;
                    double deserialize_end_time = (double) data.deserialize_end_time_ns / 1e9;
                    return new LatencySample(recv_time - send_time, send_time, recv_time, serialize_start_time,
                            serialize_end_time, deserialize_start_time, deserialize_end_time, msg_size,
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
