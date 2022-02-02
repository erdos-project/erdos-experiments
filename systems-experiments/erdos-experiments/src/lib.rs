use std::time::{SystemTime, UNIX_EPOCH};

pub mod messages;
pub mod operators;

pub fn ns_since_unix_epoch() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos()
}
