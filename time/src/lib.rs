use std::ops::Add;

use chrono::{Duration, NaiveDate, NaiveDateTime, Utc};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct EphemerisTime(f64);
impl EphemerisTime {
    pub fn now() -> Self {
        let utc = UTC(Utc::now().naive_utc());
        EphemerisTime::from(&utc)
    }
}
impl Add<f64> for EphemerisTime {
    type Output = Self;
    fn add(self, rhs: f64) -> Self::Output {
        EphemerisTime(self.0 + rhs)
    }
}


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GPS(NaiveDateTime);
impl GPS {
    pub fn now() -> Self {
        let utc = UTC(Utc::now().naive_utc());
        GPS::from(&utc)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JulianDate(f64);
impl JulianDate {
    pub fn now() -> Self {
        let utc = UTC(Utc::now().naive_utc());
        JulianDate::from(&utc)
    }
}
impl Add<f64> for JulianDate {
    type Output = Self;
    fn add(self, rhs: f64) -> Self::Output {
        JulianDate(self.0 + rhs)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TAI(NaiveDateTime);
impl TAI {
    pub fn now() -> Self {
        let utc = UTC(Utc::now().naive_utc());
        TAI::from(&utc)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UTC(NaiveDateTime);
impl UTC {
    pub fn now() -> Self {
        UTC(Utc::now().naive_utc())
    }
}

impl From<&JulianDate> for EphemerisTime {
    fn from(jd: &JulianDate) -> Self {
        let et = (jd.0 - 2451545.0) * 86400.0;
        Self(et)
    }
}

impl From<&EphemerisTime> for JulianDate {
    fn from(et: &EphemerisTime) -> Self {
        let jd = et.0 / 86400.0 + 2451545.0;
        Self(jd)
    }
}

impl From<&UTC> for JulianDate {
    fn from(utc: &UTC) -> Self {
        // Reference time in the Gregorian calendar (January 1, 2000, 12:00)
        let reference_date = NaiveDate::from_ymd_opt(2000, 1, 1)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();

        // Difference in days between the input date and J2000 epoch
        let duration = utc.0.signed_duration_since(reference_date);
        let days_since_j2000 = duration.num_seconds() as f64 / 86400.0; // Convert to days

        // Julian Date is days since J2000 epoch plus 2451545.0
        Self(2451545.0 + days_since_j2000)
    }
}

impl From<&UTC> for EphemerisTime {
    fn from(utc: &UTC) -> Self {
        let jd = JulianDate::from(utc);
        EphemerisTime::from(&jd)
    }
}

impl From<&UTC> for GPS {
    fn from(utc: &UTC) -> Self {
        Self(utc.0 + Duration::seconds(18))
    }
}

impl From<&UTC> for TAI {
    fn from(utc: &UTC) -> Self {
        Self(utc.0 + Duration::seconds(37))
    }
}

impl From<&TAI> for UTC {
    fn from(tai: &TAI) -> Self {
        Self(tai.0 - Duration::seconds(37))
    }
}

impl From<&GPS> for UTC {
    fn from(gps: &GPS) -> Self {
        Self(gps.0 - Duration::seconds(18))
    }
}

impl From<&GPS> for TAI {
    fn from(gps: &GPS) -> Self {
        Self(gps.0 + Duration::seconds(19))
    }
}

impl From<&TAI> for GPS {
    fn from(tai: &TAI) -> Self {
        Self(tai.0 - Duration::seconds(19))
    }
}
