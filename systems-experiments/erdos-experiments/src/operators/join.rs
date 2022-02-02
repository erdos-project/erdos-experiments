use std::{collections::HashMap, sync::Arc};

use erdos::dataflow::{operator::TwoInOneOut, stream::WriteStreamT, Data, Message, Timestamp};
use serde::Deserialize;

pub struct JoinOperator<D1, D2, I>
where
    for<'a> D1: Data + Deserialize<'a>,
    for<'a> D2: Data + Deserialize<'a>,
    I: IntoIterator,
    for<'a> I::Item: Data + Deserialize<'a>,
{
    left_values: HashMap<Timestamp, Vec<D1>>,
    right_values: HashMap<Timestamp, Vec<D2>>,
    join_fn: Arc<dyn Fn(&[D1], &[D2]) -> I + Send + Sync>,
}

impl<D1, D2, I> JoinOperator<D1, D2, I>
where
    for<'a> D1: Data + Deserialize<'a>,
    for<'a> D2: Data + Deserialize<'a>,
    I: IntoIterator,
    for<'a> I::Item: Data + Deserialize<'a>,
{
    pub fn new<F>(join_fn: F) -> Self
    where
        F: 'static + Fn(&[D1], &[D2]) -> I + Send + Sync,
    {
        Self {
            left_values: HashMap::new(),
            right_values: HashMap::new(),
            join_fn: Arc::new(join_fn),
        }
    }
}

impl<D1, D2, I> TwoInOneOut<(), D1, D2, I::Item> for JoinOperator<D1, D2, I>
where
    for<'a> D1: Data + Deserialize<'a>,
    for<'a> D2: Data + Deserialize<'a>,
    I: IntoIterator,
    for<'a> I::Item: Data + Deserialize<'a>,
{
    fn on_left_data(
        &mut self,
        ctx: &mut erdos::dataflow::context::TwoInOneOutContext<(), I::Item>,
        data: &D1,
    ) {
        self.left_values
            .entry(ctx.get_timestamp().clone())
            .or_default()
            .push(data.clone())
    }

    fn on_right_data(
        &mut self,
        ctx: &mut erdos::dataflow::context::TwoInOneOutContext<(), I::Item>,
        data: &D2,
    ) {
        self.right_values
            .entry(ctx.get_timestamp().clone())
            .or_default()
            .push(data.clone())
    }

    fn on_watermark(
        &mut self,
        ctx: &mut erdos::dataflow::context::TwoInOneOutContext<(), I::Item>,
    ) {
        let left_values = self
            .left_values
            .remove(ctx.get_timestamp())
            .unwrap_or_default();
        let right_values = self
            .right_values
            .remove(ctx.get_timestamp())
            .unwrap_or_default();

        let results = (self.join_fn)(&left_values[..], &right_values[..]);
        for result in results {
            let timestamp = ctx.get_timestamp().clone();
            ctx.get_write_stream()
                .send(Message::new_message(timestamp, result))
                .unwrap();
        }
    }
}
