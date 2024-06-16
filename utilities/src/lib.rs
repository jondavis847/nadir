pub fn format_number(value: f64) -> String {
    if value.abs() >= 10000.0 || value.abs() < 0.0001 {
        format!("{: >10.4e}", value)
    } else {
        format!("{: >10}", value)
    }
}