use std::{
    thread,
    time::{Duration, Instant},
};

use erdos::{
    dataflow::{
        message::*, operator::Source, stream::errors::SendError, stream::WriteStreamT, Timestamp,
        WriteStream,
    },
    OperatorConfig,
};

use crate::messages::BinaryData;

pub struct BinarySource {
    msg_size: usize,
    num_warmup_msgs: usize,
    num_msgs: usize,
    send_frequency: f32,
}

impl BinarySource {
    pub fn new(
        msg_size: usize,
        num_warmup_msgs: usize,
        num_msgs: usize,
        send_frequency: f32,
    ) -> Self {
        Self {
            msg_size,
            num_warmup_msgs,
            num_msgs,
            send_frequency,
        }
    }

    fn send(
        &self,
        t: Vec<u64>,
        write_stream: &mut WriteStream<BinaryData>,
    ) -> Result<(), SendError> {
        let timestamp = Timestamp::Time(t);
        let data = BinaryData::new(self.msg_size);
        let msg = Message::new_message(timestamp.clone(), data);
        write_stream.send(msg).unwrap();
        write_stream.send(Message::new_watermark(timestamp))
    }
}

impl Source<(), BinaryData> for BinarySource {
    fn run(&mut self, config: &OperatorConfig, write_stream: &mut WriteStream<BinaryData>) {
        let period = Duration::from_secs_f32(1.0 / self.send_frequency);

        tracing::info!("{}: starting warmup", config.get_name());

        let mut send_time = Instant::now();
        for t in 0..self.num_warmup_msgs {
            self.send(vec![0, t as u64], write_stream).unwrap();
            if send_time.elapsed() < period {
                // Sleep until the next send time.
                // This should be fine for high frequencies if the OS supports nanosleep.
                thread::sleep(period - send_time.elapsed());
            } else {
                tracing::warn!(
                    "{}: fell behind by {:?}",
                    config.get_name(),
                    send_time.elapsed() - period
                )
            }
            send_time += period;
        }
        tracing::info!("{}: ended warmup", config.get_name());

        tracing::info!("{}: sending messages", config.get_name());
        for t in 1..self.num_msgs + 1 {
            self.send(vec![t as u64, self.num_warmup_msgs as u64], write_stream)
                .unwrap();
            if send_time.elapsed() < period {
                // Sleep until the next send time.
                // This should be fine for high frequencies if the OS supports nanosleep.
                thread::sleep(period - send_time.elapsed());
            } else {
                tracing::warn!(
                    "{}: fell behind by {:?}",
                    config.get_name(),
                    send_time.elapsed() - period
                )
            }
            send_time += period;
        }

        tracing::debug!("Done sending messages.");

        // Send top watermark to indicate throughput measurements are done.
        write_stream
            .send(Message::new_watermark(Timestamp::Top))
            .unwrap();
    }
}
