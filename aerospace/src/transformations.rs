use std::f64::consts::PI;

pub enum OrbitalFrame {
    Icrf,
    Itrf,
    J2000,
}

/// Converts a time in Julian Date to Greenwich Sidereal Time (GST) in radians.
fn julian_to_gst(julian_date: f64) -> f64 {
    let j2000 = 2451545.0;
    let t = (julian_date - j2000) / 36525.0;

    let gst_deg = 280.46061837 + 360.98564736629 * (julian_date - j2000) +
        0.000387933 * t.powi(2) - t.powi(3) / 38710000.0;

    let gst_deg = gst_deg % 360.0;
    let gst_deg = if gst_deg < 0.0 { gst_deg + 360.0 } else { gst_deg };

    gst_deg.to_radians()
}