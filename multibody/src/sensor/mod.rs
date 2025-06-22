pub mod noise;
use crate::{
    HardwareBuffer,
    body::{BodyConnection, BodyConnectionBuilder},
    system::Id,
};

use gps::{Gps, GpsBuilder, GpsErrors};
use magnetometer::{Magnetometer, MagnetometerBuilder, MagnetometerErrors};
use nadir_diffeq::saving::{StateWriter, StateWriterBuilder, WriterId, WriterManager};
use rand::rngs::SmallRng;
use rate_gyro::{RateGyro, RateGyroBuilder, RateGyroErrors};
use serde::{Deserialize, Serialize};
use star_tracker::{StarTracker, StarTrackerBuilder, StarTrackerErrors};
use std::{fmt::Debug, path::PathBuf};
use thiserror::Error;
use transforms::Transform;
use uncertainty::{Uncertainty, UncertaintyErrors};

pub mod gps;
pub mod magnetometer;
pub mod rate_gyro;
pub mod star_tracker;

#[derive(Debug, Error)]
pub enum SensorErrors {
    #[error("sensor '{0}' is already connected to body '{1}'")]
    AlreadyConnectedToAnotherBody(String, String),
    #[error("sensor '{0}' is already connected to that body")]
    AlreadyConnectedToThisBody(String),
    #[error("{0}")]
    Gps(#[from] GpsErrors),
    #[error("{0}")]
    Magnetometer(#[from] MagnetometerErrors),
    #[error("{0}")]
    RateGyro(#[from] RateGyroErrors),
    #[error("{0}")]
    StarTracker(#[from] StarTrackerErrors),
    #[error("{0}")]
    Uncertainty(#[from] UncertaintyErrors),
}

pub trait SensorModel {
    fn update(&mut self, t: f64, connection: &BodyConnection);
    fn writer_headers(&self) -> &[&str];
    fn writer_save_fn(&self, writer: &mut StateWriter);
    fn write_buffer(&self, buffer: &mut HardwareBuffer) -> Result<(), SensorErrors>;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SensorBuilder {
    pub name: String,
    pub model: SensorModelBuilders,
    pub connection: Option<BodyConnectionBuilder>,
}

impl SensorBuilder {
    pub fn connect_body(&mut self, body: Id, transform: Transform) -> Result<(), SensorErrors> {
        self.connection = Some(BodyConnectionBuilder::new(
            body.clone(),
            transform,
        ));
        Ok(())
    }

    pub fn new(name: &str, model: SensorModelBuilders) -> Self {
        Self { name: name.to_string(), model, connection: None }
    }

    pub fn sample(
        &self,
        nominal: bool,
        rng: &mut SmallRng,
        connection: BodyConnection,
    ) -> Result<Sensor, SensorErrors> {
        let model = self.model.sample(nominal, rng)?;
        Ok(Sensor {
            name: self.name.clone(),
            model,
            connection,
            writer_id: None,
            telemetry_buffer: HardwareBuffer::new(),
        })
    }
}

#[derive(Debug)]
pub struct Sensor {
    pub name: String,
    pub model: SensorModels,
    pub connection: BodyConnection,
    pub telemetry_buffer: HardwareBuffer,
    writer_id: Option<WriterId>,
}

impl Sensor {
    pub fn update(&mut self, t: f64) -> Result<(), SensorErrors> {
        self.model.update(t, &self.connection);
        self.model
            .write_buffer(&mut self.telemetry_buffer)?;
        Ok(())
    }

    pub fn writer_init_fn(&mut self, manager: &mut WriterManager) {
        let rel_path = PathBuf::new()
            .join("sensors")
            .join(format!("{}.csv", self.name));
        let headers = self.model.writer_headers();
        let writer = StateWriterBuilder::new(headers.len(), rel_path)
            .with_headers(headers)
            .unwrap();
        self.writer_id = Some(manager.add_writer(writer));
    }

    pub fn writer_save_fn(&self, manager: &mut WriterManager) {
        if let Some(id) = &self.writer_id {
            if let Some(writer) = manager.writers.get_mut(id) {
                self.model.writer_save_fn(writer);
            }
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum SensorModelBuilders {
    Gps(GpsBuilder),
    Magnetometer(MagnetometerBuilder),
    RateGyro(RateGyroBuilder),
    StarTracker(StarTrackerBuilder),
}

impl SensorModelBuilders {
    pub fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<SensorModels, SensorErrors> {
        match self {
            SensorModelBuilders::Gps(builder) => {
                Ok(SensorModels::Gps(builder.sample(nominal, rng)?))
            }
            SensorModelBuilders::Magnetometer(builder) => Ok(SensorModels::Magnetometer(
                builder.sample(nominal, rng)?,
            )),
            SensorModelBuilders::RateGyro(builder) => Ok(SensorModels::RateGyro(
                builder.sample(nominal, rng)?,
            )),
            SensorModelBuilders::StarTracker(builder) => Ok(SensorModels::StarTracker(
                builder.sample(nominal, rng)?,
            )),
        }
    }
}

impl From<GpsBuilder> for SensorModelBuilders {
    fn from(builder: GpsBuilder) -> Self {
        SensorModelBuilders::Gps(builder)
    }
}
impl From<MagnetometerBuilder> for SensorModelBuilders {
    fn from(builder: MagnetometerBuilder) -> Self {
        SensorModelBuilders::Magnetometer(builder)
    }
}
impl From<RateGyroBuilder> for SensorModelBuilders {
    fn from(builder: RateGyroBuilder) -> Self {
        SensorModelBuilders::RateGyro(builder)
    }
}
impl From<StarTrackerBuilder> for SensorModelBuilders {
    fn from(builder: StarTrackerBuilder) -> Self {
        SensorModelBuilders::StarTracker(builder)
    }
}

#[derive(Debug)]
pub enum SensorModels {
    Gps(Gps),
    Magnetometer(Magnetometer),
    RateGyro(RateGyro),
    StarTracker(StarTracker),
}

impl SensorModel for SensorModels {
    fn writer_save_fn(&self, writer: &mut StateWriter) {
        match self {
            SensorModels::Gps(sensor) => sensor.writer_save_fn(writer),
            SensorModels::Magnetometer(sensor) => sensor.writer_save_fn(writer),
            SensorModels::RateGyro(sensor) => sensor.writer_save_fn(writer),
            SensorModels::StarTracker(sensor) => sensor.writer_save_fn(writer),
        }
    }

    fn writer_headers(&self) -> &[&str] {
        match self {
            SensorModels::Gps(sensor) => sensor.writer_headers(),
            SensorModels::Magnetometer(sensor) => sensor.writer_headers(),
            SensorModels::RateGyro(sensor) => sensor.writer_headers(),
            SensorModels::StarTracker(sensor) => sensor.writer_headers(),
        }
    }

    fn update(&mut self, t: f64, connection: &BodyConnection) {
        match self {
            SensorModels::Gps(sensor) => sensor.update(t, connection),
            SensorModels::Magnetometer(sensor) => sensor.update(t, connection),
            SensorModels::RateGyro(sensor) => sensor.update(t, connection),
            SensorModels::StarTracker(sensor) => sensor.update(t, connection),
        }
    }

    fn write_buffer(&self, buffer: &mut HardwareBuffer) -> Result<(), SensorErrors> {
        match self {
            SensorModels::Gps(sensor) => sensor.write_buffer(buffer),
            SensorModels::Magnetometer(sensor) => sensor.write_buffer(buffer),
            SensorModels::RateGyro(sensor) => sensor.write_buffer(buffer),
            SensorModels::StarTracker(sensor) => sensor.write_buffer(buffer),
        }
    }
}
