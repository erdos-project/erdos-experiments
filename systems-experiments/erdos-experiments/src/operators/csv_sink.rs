use std::marker::PhantomData;

use csv::Writer;

use serde::{Deserialize, Serialize};

use erdos::{
    dataflow::{
        context::OneInOneOutContext, message::*, operator::OneInOneOut, stream::WriteStreamT,
        ReadStream, Timestamp, WriteStream,
    },
    OperatorConfig,
};

pub struct CSVSink<T: Serialize + Data> {
    filename: String,
    phantom: PhantomData<T>,
}

impl<T: Data> CSVSink<T> {
    pub fn new(filename: &str) -> Self {
        Self {
            filename: filename.to_string(),
            phantom: PhantomData,
        }
    }
}

impl<T> OneInOneOut<(), T, ()> for CSVSink<T>
where
    T: Data + for<'a> Deserialize<'a>,
{
    fn on_data(&mut self, _ctx: &mut OneInOneOutContext<(), ()>, _data: &T) {}

    fn on_watermark(&mut self, _ctx: &mut OneInOneOutContext<(), ()>) {}

    fn run(
        &mut self,
        config: &OperatorConfig,
        read_stream: &mut ReadStream<T>,
        write_stream: &mut WriteStream<()>,
    ) {
        let mut csv_writer = Writer::from_path(self.filename.clone()).unwrap();

        while let Ok(msg) = read_stream.read() {
            if msg.is_top_watermark() {
                tracing::info!("{}: done writing to CSV", config.get_name());
                write_stream
                    .send(Message::new_watermark(Timestamp::Top))
                    .unwrap();
                return;
            }
            if let (Some(sample), Timestamp::Time(t)) = (msg.data(), msg.timestamp()) {
                if t[0] > 0 {
                    csv_writer.serialize(sample).unwrap();
                }
            }
        }
    }
}
