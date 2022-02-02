package flink_experiments.operators;

import java.util.concurrent.TimeUnit;

import org.apache.flink.streaming.api.functions.source.SourceFunction;

import flink_experiments.messages.BinaryData;

public class BinarySource implements SourceFunction<BinaryData> {
    private boolean is_running = true;

    private final int msg_size;
    private final int num_warmup_msgs;
    private final int num_msgs;
    private final double send_frequency;

    public BinarySource(final int msg_size, final int num_warmup_msgs, final int num_msgs,
            final double send_frequency) {
        this.msg_size = msg_size;
        this.num_warmup_msgs = num_warmup_msgs;
        this.num_msgs = num_msgs;
        this.send_frequency = send_frequency;
    }

    private void send(long timestamp, SourceContext<BinaryData> ctx) {
        BinaryData data = new BinaryData(this.msg_size);
        data.update_send_time();
        ctx.collectWithTimestamp(data, timestamp);
    }

    private void send_messages(int num_msgs, long start_timestamp, SourceContext<BinaryData> ctx) throws Exception {
        double period = 1.0 / send_frequency;

        long send_time = System.nanoTime();
        for (long timestamp = start_timestamp; this.is_running && timestamp < start_timestamp + num_msgs; timestamp++) {
            this.send(timestamp, ctx);

            long elapsed_time = System.nanoTime() - send_time;
            if (elapsed_time < (long) (period * 1e9)) {
                // Sleep until the next send time.
                TimeUnit.NANOSECONDS.sleep((long) (period * 1e9) - elapsed_time);
            } else {
                System.out.print("BinarySource: fell behind by ");
                System.out.print(Long.toString(elapsed_time - (long) (period * 1e9)));
                System.out.println(" ns");
            }

            send_time += (long) (period * 1e9);
        }

    }

    @Override
    public void run(SourceContext<BinaryData> ctx) throws Exception {

        System.out.println("Starting warmup");
        this.send_messages(this.num_warmup_msgs, 0, ctx);
        System.out.println("Ended warmup");

        System.out.println("Sending messages");
        this.send_messages(this.num_msgs, (long) this.num_warmup_msgs, ctx);
        System.out.println("Done sending messages");
    }

    @Override
    public void cancel() {
        this.is_running = false;
    }
}
