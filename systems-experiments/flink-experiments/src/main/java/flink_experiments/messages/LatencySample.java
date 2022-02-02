package flink_experiments.messages;

public class LatencySample {
    public final double latency_secs;
    public final double send_time;
    public final double recv_time;
    public final double serialize_start_time;
    public final double serialize_end_time;
    public final double deserialize_start_time;
    public final double deserialize_end_time;
    public final int msg_size;
    public final boolean is_inter_process;

    public LatencySample(final double latency_secs, final double send_time, final double recv_time,
            final double serialize_start_time, final double serialize_end_time, final double deserialize_start_time,
            final double deserialize_end_time, final int msg_size, final boolean is_inter_process) {
        this.latency_secs = latency_secs;
        this.send_time = send_time;
        this.recv_time = recv_time;
        this.serialize_start_time = serialize_start_time;
        this.serialize_end_time = serialize_end_time;
        this.deserialize_start_time = deserialize_start_time;
        this.deserialize_end_time = deserialize_end_time;
        this.msg_size = msg_size;
        this.is_inter_process = is_inter_process;
    }
}
