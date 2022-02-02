package flink_experiments.operators;

import java.io.File;
import java.io.FileWriter;

import org.apache.flink.streaming.api.functions.sink.SinkFunction;

import flink_experiments.messages.LatencySample;

public class CSVSink implements SinkFunction<LatencySample> {
    private final int warmup_samples;
    private final String filename;

    public CSVSink(final int warmup_samples, final String filename) {
        this.warmup_samples = warmup_samples;
        this.filename = filename;
    }

    private void setup_file() {
        File file = new File(this.filename);
        if (!file.exists()) {
            try {
                FileWriter csv_writer = new FileWriter(file);
                csv_writer.write(
                        "latency_secs,send_time,recv_time,serialize_start_time,serialize_end_time,deserialize_start_time,deserialize_end_time,msg_size,is_inter_process\n");
                csv_writer.flush();
                csv_writer.close();

            } catch (final Exception e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void invoke(LatencySample sample, Context ctx) {
        if (ctx.timestamp() >= (long) this.warmup_samples) {
            try {
                this.setup_file();
                FileWriter csv_writer = new FileWriter(this.filename, true);

                csv_writer.write(Double.toString(sample.latency_secs));
                csv_writer.write(",");
                csv_writer.write(Double.toString(sample.send_time));
                csv_writer.write(",");
                csv_writer.write(Double.toString(sample.recv_time));
                csv_writer.write(",");
                csv_writer.write(Double.toString(sample.serialize_start_time));
                csv_writer.write(",");
                csv_writer.write(Double.toString(sample.serialize_end_time));
                csv_writer.write(",");
                csv_writer.write(Double.toString(sample.deserialize_start_time));
                csv_writer.write(",");
                csv_writer.write(Double.toString(sample.deserialize_end_time));
                csv_writer.write(",");
                csv_writer.write(Integer.toString(sample.msg_size));
                csv_writer.write(",");
                csv_writer.write(Boolean.toString(sample.is_inter_process));
                csv_writer.write("\n");

                csv_writer.flush();
                csv_writer.close();

            } catch (final Exception e) {
                e.printStackTrace();
            }

        }
    }

    public String filename() {
        return this.filename;
    }
}
