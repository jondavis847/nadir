use std::{
    ops::{Add, Sub},
    time::Duration,
};
use chrono::{NaiveDate, NaiveDateTime, Utc};
use serde::{Deserialize, Serialize};

pub const JD_J2000: f64 = 2451545.0;
pub const SEC_PER_DAY: f64 = 86400.0;
pub const DAYS_PER_CENTURY: f64 = 36525.0;

pub mod prelude {
    pub use crate::{Time,TimeSystem,TimeFormat,TimeErrors};
}

//https://naif.jpl.nasa.gov/pub/naif/generic_kernels/lsk/latest_leapseconds.tls
pub const LEAPSECONDS: [(f64, f64); 28] = [
    (10.0, -883656000.0), // @1972-JAN-1
    (11.0, -867931200.0), // @1972-JUL-1
    (12.0, -852033600.0), // @1973-JAN-1
    (13.0, -820497600.0), // @1974-JAN-1
    (14.0, -788961600.0), // @1975-JAN-1
    (15.0, -757425600.0), // @1976-JAN-1
    (16.0, -725803200.0), // @1977-JAN-1
    (17.0, -694267200.0), // @1978-JAN-1
    (18.0, -662731200.0), // @1979-JAN-1
    (19.0, -631195200.0), // @1980-JAN-1
    (20.0, -583934400.0), // @1981-JUL-1
    (21.0, -552398400.0), // @1982-JUL-1
    (22.0, -520862400.0), // @1983-JUL-1
    (23.0, -457704000.0), // @1985-JUL-1
    (24.0, -378734400.0), // @1988-JAN-1
    (25.0, -315576000.0), // @1990-JAN-1
    (26.0, -284040000.0), // @1991-JAN-1
    (27.0, -236779200.0), // @1992-JUL-1
    (28.0, -205243200.0), // @1993-JUL-1
    (29.0, -173707200.0), // @1994-JUL-1
    (30.0, -126273600.0), // @1996-JAN-1
    (31.0, -79012800.0),  // @1997-JUL-1
    (32.0, -31579200.0),  // @1999-JAN-1
    (33.0, 189345600.0),  // @2006-JAN-1
    (34.0, 284040000.0),  // @2009-JAN-1
    (35.0, 394372800.0),  // @2012-JUL-1
    (36.0, 488980800.0),  // @2015-JUL-1
    (37.0, 536500800.0),  // @2017-JAN-1
];

#[derive(Debug, Clone, Copy)]
pub enum TimeErrors {
    InvalidEpoch,
    InvalidUtcTime,
    NaiveDateTimeError,
    UseNewUtc,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
pub enum TimeSystem {
    GPS,
    TAI,
    UTC,
    TDB,
    TT,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
pub enum TimeFormat {
    DateTime,
    JulianDate,
    SecondsSinceJ2000,
    SecondsSinceTAI,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
pub struct Time {
    system: TimeSystem,
    value: SecondsSinceJ2000,
}

impl Time {
    pub fn now() -> Result<Self, TimeErrors> {
        let datetime = Utc::now().naive_utc();
        Time::from_datetime(datetime, TimeSystem::UTC)
    }

    pub fn from_jd(jd: f64, system: TimeSystem) -> Self {
        let value = SecondsSinceJ2000::new((jd - JD_J2000) * SEC_PER_DAY);
        Self { system, value }
    }
    pub fn from_datetime(dt: NaiveDateTime, system: TimeSystem) -> Result<Self, TimeErrors> {
        let value = SecondsSinceJ2000::try_from(dt)?;
        Ok(Self { system, value })
    }

    pub fn from_sec_j2k(value: f64, system: TimeSystem) -> Self {
        let value = SecondsSinceJ2000::new(value);
        Self { system, value }
    }

    pub fn from_ymdhms(
        year: i32,
        month: u32,
        day: u32,
        hour: u32,
        minute: u32,
        second: f64,
        system: TimeSystem,
    ) -> Result<Self, TimeErrors> {
        let sec = second.floor();
        let nano = ((second - sec) * 1e9).round();
        let dt = NaiveDate::from_ymd_opt(year, month, day)
            .ok_or(TimeErrors::NaiveDateTimeError)?
            .and_hms_nano_opt(hour, minute, sec as u32, nano as u32)
            .ok_or(TimeErrors::NaiveDateTimeError)?;
        Time::from_datetime(dt, system)
    }

    pub fn to_system(&self, target_system: TimeSystem) -> Time {
        // return if converting to the same system
        if self.system == target_system {
            return *self;
        }

        // get leap seconds if either system is UTC
        let leap_seconds = if self.system == TimeSystem::UTC {
            find_leap_seconds_utc(self.value)
        } else if target_system == TimeSystem::UTC {
            find_leap_seconds_tai(self.value)
        } else {
            // doesn't matter
            0.0
        };

        // convert to tai first
        // we don't have to do this technically but the code is much
        // cleaner without writing out every permutation, and its simple addition

        let tai_value = match self.system {
            TimeSystem::UTC => self.value + leap_seconds,
            TimeSystem::GPS => self.value + 19.0,
            TimeSystem::TT => self.value - 32.184,
            TimeSystem::TDB => todo!("add tdb calcs"),
            TimeSystem::TAI => self.value,
        };

        // convert to target system
        let value = match target_system {
            TimeSystem::UTC => tai_value - leap_seconds,
            TimeSystem::GPS => tai_value - 19.0,
            TimeSystem::TT => tai_value + 32.184,
            TimeSystem::TDB => todo!("add tdb calcs"),
            TimeSystem::TAI => tai_value,
        };

        Time {
            system: target_system,
            value,
        }
    }

    pub fn get_jd(&self) -> f64 {
        self.value.0 / SEC_PER_DAY + JD_J2000
    }

    pub fn get_jd_centuries(&self) -> f64 {
        self.value.0 / SEC_PER_DAY / DAYS_PER_CENTURY
    }

    pub fn get_datetime(&self) -> NaiveDateTime {
        self.value.to_datetime()
    }

    pub fn get_seconds_j2k(&self) -> f64 {
        self.value.0
    }
}

fn find_leap_seconds_utc(epoch: SecondsSinceJ2000) -> f64 {
    for &(leap_seconds, leap_epoch) in LEAPSECONDS.iter().rev() {
        if epoch.0 >= leap_epoch {
            return leap_seconds;
        }
    }
    // not greater than the first entry, must be less than which is 10.0
    10.0
}

pub fn find_leap_seconds_tai(epoch_tai: SecondsSinceJ2000) -> f64 {
    let mut current_leap_seconds;

    for &(leap_seconds, leap_epoch) in LEAPSECONDS.iter().rev() {
        current_leap_seconds = leap_seconds; // Update the number of leap seconds at each step

        // Convert the TAI epoch to the corresponding UTC epoch by subtracting the current leap seconds
        let epoch_utc = epoch_tai - current_leap_seconds;

        // If the adjusted UTC epoch is greater than or equal to the current leap epoch, return the leap seconds
        if epoch_utc.0 >= leap_epoch {
            return leap_seconds;
        }
    }
    // not greater than the first entry, must be less than which is 10.0
    10.0
}

impl Add<f64> for Time {
    type Output = Self;
    fn add(self, rhs: f64) -> Self::Output {
        Time::from_sec_j2k(self.value.0 + rhs, self.system)
    }
}

// will be accuruate for 285 million years at f64 precision
#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
pub struct SecondsSinceJ2000(f64);

impl SecondsSinceJ2000 {
    pub fn to_datetime(&self) -> NaiveDateTime {
        let j2k = NaiveDate::from_ymd_opt(2000, 1, 1)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        let seconds = self.0.floor();
        let nanos = ((self.0 - seconds) * 1e9).round();
        let duration = Duration::new(seconds as u64, nanos as u32);
        j2k + duration
    }
}

impl SecondsSinceJ2000 {
    pub fn new(sec: f64) -> Self {
        SecondsSinceJ2000(sec)
    }
}
impl Add<f64> for SecondsSinceJ2000 {
    type Output = Self;
    fn add(self, rhs: f64) -> Self::Output {
        SecondsSinceJ2000(self.0 + rhs)
    }
}

impl Sub<f64> for SecondsSinceJ2000 {
    type Output = Self;
    fn sub(self, rhs: f64) -> Self::Output {
        SecondsSinceJ2000(self.0 - rhs)
    }
}

impl TryFrom<NaiveDateTime> for SecondsSinceJ2000 {
    type Error = TimeErrors;
    fn try_from(datetime: NaiveDateTime) -> Result<Self, TimeErrors> {
        // Define J2000 epoch as January 1, 2000, 12:00:00.000
        let j2k = NaiveDate::from_ymd_opt(2000, 1, 1)
            .ok_or(TimeErrors::NaiveDateTimeError)?
            .and_hms_nano_opt(12, 0, 0, 0)
            .ok_or(TimeErrors::NaiveDateTimeError)?;

        // Calculate duration between the provided datetime and the J2000 epoch
        let delta = datetime.signed_duration_since(j2k);

        // Compute seconds as f64
        let seconds = delta.num_seconds() as f64 + delta.subsec_nanos() as f64 * 1e-9;

        Ok(SecondsSinceJ2000(seconds))
    }
}

/*
impl From<&NaiveDateTime> for JulianDate {
    fn from(dt: &NaiveDateTime) -> Self {
        // reference: Meeus, Jean. Astronomical Algorithms, 2nd Edition, 1998. Willmann-Bell, Inc
        let year = dt.year();
        let month = dt.month();
        let day = dt.day() as f64;
        let hour = dt.hour() as f64;
        let minute = dt.minute() as f64;
        let second = dt.second() as f64;
        let ns = dt.nanosecond() as f64;

        let second = second + ns * 1e-9;

        let mut y = year;
        let mut m = month;

        // shift gregorian calendar to julian calendar, which starts in march
        if m <= 2 {
            y -= 1;
            m += 12;
        }

        // corrections for leap years
        let a = (y as f64 / 100.0).floor();
        let b = 2.0 - a + (a / 4.0).floor();

        let jd = (365.25 * (y as f64 + 4716.0)).floor()
            + (30.6001 * (m as f64 + 1.0)).floor()
            + day
            + ((hour + minute / 60.0 + second / 3600.0) / 24.0)
            + b
            - 1524.5;

        JulianDate(jd)
    }
*/

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_jd_from_datetime_utc_0() {
        let dt = Time::from_ymdhms(2000, 1, 1, 12, 0, 0.0, TimeSystem::UTC).unwrap();
        let jd = dt.get_jd();
        assert_abs_diff_eq!(jd, 2451545.0, epsilon = 1e-9); //julia Dates
    }

    #[test]
    fn test_jd_from_datetime_utc_1() {
        let dt = Time::from_ymdhms(2024, 1, 31, 6, 16, 30.5, TimeSystem::UTC).unwrap();
        let jd = dt.get_jd();
        assert_abs_diff_eq!(jd, 2.4603407614641204e6, epsilon = 1e-9); //julia Dates
    }

    #[test]
    fn test_jd_from_datetime_utc_2() {
        let dt = Time::from_ymdhms(2028, 2, 29, 16, 59, 59.999, TimeSystem::UTC).unwrap();
        let jd = dt.get_jd();
        assert_abs_diff_eq!(jd, 2.461831208333322e6, epsilon = 1e-9); //julia Dates
    }

    #[test]
    fn test_utc_to_tai_1() {
        let utc = Time::from_ymdhms(2000, 1, 1, 12, 0, 0.0, TimeSystem::UTC).unwrap();
        let tai = utc.to_system(TimeSystem::TAI);
        assert_abs_diff_eq!(tai.value.0, 32.0, epsilon = 1e-9); //julia Dates
    }

    #[test]
    fn test_leap_year_2024_feb_29() {
        let dt = Time::from_ymdhms(2024, 2, 29, 23, 59, 59.999, TimeSystem::UTC).unwrap();
        let jd = dt.get_jd();
        assert_abs_diff_eq!(jd, 2460370.4999999884, epsilon = 1e-9); // Expected JD
    }

    #[test]
    fn test_non_leap_year_2023_feb_28() {
        let dt = Time::from_ymdhms(2023, 2, 28, 23, 59, 59.999, TimeSystem::UTC).unwrap();
        let jd = dt.get_jd();
        assert_abs_diff_eq!(jd, 2.4600044999999884e6, epsilon = 1e-9); // Expected JD
    }

    #[test]
    fn test_end_of_january() {
        let dt = Time::from_ymdhms(2024, 1, 31, 23, 59, 59.999, TimeSystem::UTC).unwrap();
        let jd = dt.get_jd();
        assert_abs_diff_eq!(jd, 2460341.4999999884, epsilon = 1e-9); // Expected JD
    }

    #[test]
    fn test_end_of_december() {
        let dt = Time::from_ymdhms(2023, 12, 31, 23, 59, 59.999, TimeSystem::UTC).unwrap();
        let jd = dt.get_jd();
        assert_abs_diff_eq!(jd, 2.4603104999999884e6, epsilon = 1e-9); // Expected JD
    }

    #[test]
    fn test_utc_to_tai_with_leap_second() {
        let utc = Time::from_ymdhms(2017, 1, 1, 0, 0, 0.0, TimeSystem::UTC).unwrap();
        let tai = utc.to_system(TimeSystem::TAI);
        assert_abs_diff_eq!(tai.value.0 - utc.value.0, 37.0, epsilon = 1e-9); // 37 leap seconds by 2017
    }

    #[test]
    fn test_tai_to_utc_with_leap_second() {
        let tai = Time::from_ymdhms(2017, 1, 1, 0, 0, 37.0, TimeSystem::TAI).unwrap();
        let utc = tai.to_system(TimeSystem::UTC);
        assert_abs_diff_eq!(tai.value.0 - utc.value.0, 37.0, epsilon = 1e-9); // At the moment of leap second correction
    }

    #[test]
    fn test_far_future_date() {
        let dt = Time::from_ymdhms(3000, 1, 1, 0, 0, 0.0, TimeSystem::UTC).unwrap();
        let jd = dt.get_jd();
        assert_abs_diff_eq!(jd, 2816787.5, epsilon = 1e-9); // Expected JD in the year 3000
    }

    #[test]
    fn test_far_past_date() {
        let dt = Time::from_ymdhms(1000, 1, 1, 0, 0, 0.0, TimeSystem::UTC).unwrap();
        let jd = dt.get_jd();
        assert_abs_diff_eq!(jd, 2.0863025e6, epsilon = 1e-9); // Expected JD in the year 1000
    }
}
