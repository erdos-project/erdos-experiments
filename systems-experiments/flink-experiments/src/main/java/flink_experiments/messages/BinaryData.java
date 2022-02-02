package flink_experiments.messages;

import java.io.IOException;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

public class BinaryData {
    public byte[] data;
    public long send_time_ns;
    public long serialize_start_time_ns;
    public long serialize_end_time_ns;
    public long deserialize_start_time_ns;
    public long deserialize_end_time_ns;

    public BinaryData(int msg_size) {
        this.data = new byte[msg_size];
        this.send_time_ns = System.nanoTime();
        this.serialize_start_time_ns = 0;
        this.serialize_end_time_ns = 0;
        this.deserialize_start_time_ns = 0;
        this.deserialize_end_time_ns = 0;
    }

    // Updates the send time to the current time.
    public void update_send_time() {
        this.send_time_ns = System.nanoTime();
    }

    // Used to compare serialization overhead.
    public static final class BinaryDataSerializer extends Serializer<BinaryData> implements java.io.Serializable {
        private static final long serialVersionUID = 1L;

        @Override
        public BinaryData read(Kryo kyro, Input input, Class<BinaryData> aClass) {
            long deserialize_start_time_ns = System.nanoTime();

            BinaryData data = new BinaryData(0);
            data.send_time_ns = input.readLong();

            data.serialize_start_time_ns = input.readLong();
            input.readLong(); // deserialize_start_time_ns
            input.readLong(); // deserialize_end_time_ns
            data.deserialize_start_time_ns = deserialize_start_time_ns;

            int data_size = input.readInt();
            data.data = input.readBytes(data_size);

            data.serialize_end_time_ns = input.readLong();
            data.deserialize_end_time_ns = System.nanoTime();

            return data;
        }

        @Override
        public void write(Kryo kyro, Output output, BinaryData data) {
            data.serialize_start_time_ns = System.nanoTime();

            output.writeLong(data.send_time_ns);
            output.writeLong(data.serialize_start_time_ns);
            output.writeLong(data.deserialize_start_time_ns);
            output.writeLong(data.deserialize_end_time_ns);

            output.writeInt(data.data.length);
            output.write(data.data);

            data.serialize_end_time_ns = System.nanoTime();
            output.writeLong(data.serialize_end_time_ns);
        }
    }
}
