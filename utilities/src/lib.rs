use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub fn format_number(value: f64) -> String {
    if value.abs() >= 10000.0 || value.abs() < 0.0001 {
        format!("{: >10.4e}", value)
    } else {
        format!("{: >10}", value)
    }
}

pub fn generate_unique_id() -> String {
    // Get the current time since Unix epoch
    let now = SystemTime::now();
    let duration_since_epoch = now.duration_since(UNIX_EPOCH).expect("Time went backwards");
    let timestamp = duration_since_epoch.as_secs();
    // Convert the timestamp to base-36
    let unique_id = base36_encode(timestamp);
    unique_id
}

// Function to encode a number to base-36
fn base36_encode(mut num: u64) -> String {
    let mut chars = Vec::new();
    while num > 0 {
        let remainder = (num % 36) as u8;
        let char = match remainder {
            0..=9 => (b'0' + remainder) as char,
            10..=35 => (b'a' + (remainder - 10)) as char,
            _ => unreachable!(),
        };
        chars.push(char);
        num /= 36;
    }
    chars.iter().rev().collect()
}

pub fn format_duration(duration: Duration) -> String {
    if duration.as_secs() >= 3600 {
        let hours = duration.as_secs() / 3600;
        let minutes = (duration.as_secs() % 3600) / 60;
        let seconds = duration.as_secs() % 60;
        format!("{} hrs, {} min, {} sec", hours, minutes, seconds)
    } else if duration.as_secs() >= 60 {
        let minutes = duration.as_secs() / 60;
        let seconds = duration.as_secs() % 60;
        format!("{} min, {} sec", minutes, seconds)
    } else if duration.as_secs() >= 1 {
        format!("{} sec", duration.as_secs())
    } else if duration.as_millis() >= 1 {
        format!("{} ms", duration.as_millis())
    } else {
        format!("{} us", duration.as_micros())
    }
}
