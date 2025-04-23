use std::time::Instant;

use once_cell::sync::Lazy;

pub mod io;
pub mod discrete_line;
pub mod discrete_item;
pub mod sdr_config;
pub mod sdr_optimizer;
pub mod sdr_parse;

pub static EPOCH: Lazy<Instant> = Lazy::new(Instant::now);
