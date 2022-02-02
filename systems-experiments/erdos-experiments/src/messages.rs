use abomonation_derive::Abomonation;
use serde::{Deserialize, Serialize};

#[derive(Abomonation, Clone, Debug, Serialize, Deserialize)]
pub struct BinaryData {
    /// nanoseconds since the Unix epoch.
    pub send_time: u128,
    pub data: Vec<u8>,
}

impl BinaryData {
    pub fn new(size: usize) -> Self {
        let data = vec![0u8; size];

        Self {
            data,
            send_time: crate::ns_since_unix_epoch(),
        }
    }

    pub fn size(&self) -> usize {
        self.data.len()
    }
}

#[derive(Abomonation, Clone, Debug, Serialize, Deserialize)]
pub struct LatencySample {
    pub latency_secs: f64,
    pub send_time: f64,
    pub recv_time: f64,
    pub msg_size: usize,
    pub is_inter_process: bool,
}

/// The structure to serialize the results from the deadline microbenchmarks.
#[derive(Serialize)]
pub struct DeadlineSample {
    pub invocation_latency: u128,
    pub timestamp: u64,
}

#[derive(Abomonation, Clone, Debug, Serialize, Deserialize)]
pub struct RichDeadlineSample {
    pub timestamp: u64,
    pub missed_deadline: bool,
    pub invocation_delay_secs: f64,
    pub frequency: f64,
    pub callback_duration_secs: f64,
    pub deadline_secs: f64,
    pub callback_delay_after_deadline_start_cond_secs: f64,
}
