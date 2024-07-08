use std::collections::HashSet;
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

/// Combines two vectors of strings and returns a vector containing only the unique strings,
/// preserving the order of their first appearance.
///
/// This function takes two `Vec<String>` as input, combines them, and returns a new `Vec<String>`
/// that contains only unique strings, maintaining the order of their first appearance across both
/// input vectors.
///
/// # Arguments
///
/// * `vec1` - The first vector of strings.
/// * `vec2` - The second vector of strings.
///
/// # Returns
///
/// A vector containing the unique strings from both input vectors, in the order of their first appearance.
///
/// # Examples
///
/// ```
/// let vec1 = vec![
///     "apple".to_string(),
///     "banana".to_string(),
///     "cherry".to_string(),
/// ];
/// let vec2 = vec![
///     "banana".to_string(),
///     "date".to_string(),
///     "fig".to_string(),
/// ];
///
/// let unique = unique_strings_ordered(vec1, vec2);
///
/// assert_eq!(unique, vec![
///     "apple".to_string(),
///     "banana".to_string(),
///     "cherry".to_string(),
///     "date".to_string(),
///     "fig".to_string(),
/// ]);
/// ```
pub fn unique_strings(vec1: Vec<String>, vec2: Vec<String>) -> Vec<String> {
    let mut set = HashSet::new();
    let mut result = Vec::new();

    for s in vec1.into_iter().chain(vec2.into_iter()) {
        if set.insert(s.clone()) {
            result.push(s);
        }
    }

    result
}

pub fn unique_strings_alphabetical(vec1: Vec<String>, vec2: Vec<String>) -> Vec<String> {
    let mut result = unique_strings(vec1, vec2);
    result.sort();
    result 
}
