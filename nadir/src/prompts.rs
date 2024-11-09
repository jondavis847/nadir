use aerospace::{
    celestial_system::{CelestialBodies, CelestialSystem},
    gravity::{ConstantGravity, Gravity, NewtownianGravity},
    orbit::KeplerianElements,
};

use cliclack::{input, log::info, multiselect, select};
use color::{Color, ColorMode};
use coordinate_systems::{cartesian::Cartesian, CoordinateSystem};
use nadir_3d::{
    geometry::{
        cuboid::Cuboid,
        ellipsoid::{Ellipsoid, Ellipsoid16, Ellipsoid32, Ellipsoid64},
        Geometry, GeometryState,
    },
    material::Material,
    mesh::Mesh,
};
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    body::Body,
    joint::{
        floating::{Floating, FloatingState},
        prismatic::{Prismatic, PrismaticState},
        revolute::{Revolute, RevoluteState},
        JointParameters,
    },
    sensor::{
        noise::{gaussian::GaussianNoise, uniform::UniformNoise, Noise, NoiseModels},
        simple::{rate::RateSensor, rate3::Rate3Sensor, SimpleSensor},
        Sensor, SensorModel,
    },
    solver::SimulationConfig,
};
use nalgebra::Vector3;
use rotations::{
    axes::{AlignedAxes, Axis, AxisPair},
    prelude::{EulerAngles, EulerSequence, RotationMatrix},
    quaternion::Quaternion,
    Rotation,
};

use std::error::Error;
use time::{Time, TimeFormat, TimeSystem};
use transforms::{
    prelude::{Cylindrical, Spherical},
    Transform,
};

pub trait PromptAction {
    type Output;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>>;
}

pub struct PromptAngle;
impl PromptAction for PromptAngle {
    type Output = f64;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let angle = input("Initial Angle <rad>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        Ok(angle)
    }
}

pub struct PromptAngularRate;
impl PromptAction for PromptAngularRate {
    type Output = f64;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let rate = input("Initial Angular Rate <rad/s>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        Ok(rate)
    }
}

pub struct PromptAngularRate3;
impl PromptAction for PromptAngularRate3 {
    type Output = Vector3<f64>;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let x = input("Initial Angular Rate [x] <rad/s>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let y = input("Initial Angular Rate [y] <rad/s>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let z = input("Initial Angular Rate [z] <rad/s>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        Ok(Vector3::new(x, y, z))
    }
}

pub struct PromptAlignedAxes;
impl PromptAction for PromptAlignedAxes {
    type Output = Rotation;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        info("Old Primary Axis")?;
        let op = PromptAxis.prompt()?;
        info("New Primary Axis")?;
        let np = PromptAxis.prompt()?;
        info("Old Secondary Axis")?;
        let os = PromptAxis.prompt()?;
        info("New Secondary Axis")?;
        let ns = PromptAxis.prompt()?;

        let pri = AxisPair::new(op, np);
        let sec = AxisPair::new(os, ns);
        Ok(Rotation::from(AlignedAxes::new(pri, sec)?))
    }
}

pub struct PromptAxis;
impl PromptAction for PromptAxis {
    type Output = Axis;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let axis = match select("Axis")
            .item("+x", "+x", "")
            .item("+y", "+y", "")
            .item("+z", "+z", "")
            .item("-x", "-x", "")
            .item("-y", "-y", "")
            .item("-z", "-z", "")
            .interact()?
        {
            "+x" => Axis::Xp,
            "+y" => Axis::Yp,
            "+z" => Axis::Zp,
            "-x" => Axis::Xn,
            "-y" => Axis::Yn,
            "-z" => Axis::Zn,
            _ => unreachable!("picked in a select"),
        };
        Ok(axis)
    }
}

pub struct PromptBody;
impl PromptAction for PromptBody {
    type Output = Body;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let name = PromptName.prompt()?;
        let mass_properties = PromptMassProperties.prompt()?;

        let mesh_option = PromptMesh.prompt()?;
        let body = match mesh_option {
            true => {
                let geometry = PromptGeometry.prompt()?;
                let material = PromptMaterial.prompt()?;
                let mesh = Mesh {
                    name: name.clone(),
                    geometry,
                    material,
                    state: GeometryState::default(),
                    texture: None,
                };
                Body::new(&name, mass_properties)?.with_mesh(mesh)
            }
            false => Body::new(&name, mass_properties)?,
        };
        Ok(body)
    }
}

pub struct PromptCartesian;
impl PromptAction for PromptCartesian {
    type Output = Cartesian;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let x = input("Cartesian Coordinate [x] <m>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let y = input("Cartesian Coordinate [y] <m>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let z = input("Cartesian Coordinate [z] <m>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        Ok(Cartesian::new(x, y, z))
    }
}

pub struct PromptCelestialBodies;
impl PromptAction for PromptCelestialBodies {
    type Output = CelestialBodies;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        Ok(select("Select Celestial Body")
            .item(CelestialBodies::Pluto, "Sun", "")
            .item(CelestialBodies::Mercury, "Mercury", "")
            .item(CelestialBodies::Venus, "Venus", "")
            .item(CelestialBodies::Earth, "Earth", "")
            .item(CelestialBodies::Moon, "Moon", "")
            .item(CelestialBodies::Mars, "Mars", "")
            .item(CelestialBodies::Jupiter, "Jupiter", "")
            .item(CelestialBodies::Saturn, "Saturn", "")
            .item(CelestialBodies::Uranus, "Uranus", "")
            .item(CelestialBodies::Neptune, "Neptune", "")
            .item(CelestialBodies::Pluto, "Pluto", "")
            .interact()?)
    }
}

pub struct PromptCelestial;
impl PromptAction for PromptCelestial {
    type Output = CelestialSystem;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let epoch = PromptEpoch.prompt()?;

        let bodies = multiselect("Select Celestial Objects")
            .item(CelestialBodies::Mercury, "Mercury", "")
            .item(CelestialBodies::Venus, "Venus", "")
            .item(CelestialBodies::Earth, "Earth", "")
            .item(CelestialBodies::Moon, "Moon", "")
            .item(CelestialBodies::Mars, "Mars", "")
            .item(CelestialBodies::Jupiter, "Jupiter", "")
            .item(CelestialBodies::Saturn, "Saturn", "")
            .item(CelestialBodies::Uranus, "Uranus", "")
            .item(CelestialBodies::Neptune, "Neptune", "")
            .item(CelestialBodies::Pluto, "Pluto", "")
            .interact()?;
        let mut celestial = CelestialSystem::new(epoch)?;
        for body in bodies {
            let (g, b) = match body {
                CelestialBodies::Earth => {
                    info("Earth Options")?;
                    (                        
                        PromptCelestialGravityOption.prompt()?,
                        PromptCelestialMagneticOption.prompt()?,
                    )
                }
                CelestialBodies::Jupiter => {
                    info("Jupiter Options")?;
                    (PromptCelestialGravityOption.prompt()?, false)
                }
                CelestialBodies::Mars => {
                    info("Mars Options")?;
                    (PromptCelestialGravityOption.prompt()?, false)
                }
                CelestialBodies::Mercury => {
                    info("Mercury Options")?;
                    (PromptCelestialGravityOption.prompt()?, false)
                }
                CelestialBodies::Moon => {
                    info("Moon Options")?;
                    (PromptCelestialGravityOption.prompt()?, false)
                }
                CelestialBodies::Neptune => {
                    info("Neptune Options")?;
                    (PromptCelestialGravityOption.prompt()?, false)
                }
                CelestialBodies::Pluto => {
                    info("Pluto Options")?;
                    (PromptCelestialGravityOption.prompt()?, false)
                }
                CelestialBodies::Saturn => {
                    info("Saturn Options")?;
                    (PromptCelestialGravityOption.prompt()?, false)
                }
                CelestialBodies::Sun => {
                    info("Sun Options")?;
                    (PromptCelestialGravityOption.prompt()?, false)
                }
                CelestialBodies::Uranus => {
                    info("Uranus Options")?;
                    (PromptCelestialGravityOption.prompt()?, false)
                }
                CelestialBodies::Venus => {
                    info("Venus Options")?;
                    (PromptCelestialGravityOption.prompt()?, false)
                }
            };
            celestial.add_body(body, g, b)?;
        }
        Ok(celestial)
    }
}

pub struct PromptCelestialGravityOption;
impl PromptAction for PromptCelestialGravityOption {
    type Output = bool;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let g = select("Add gravity model?")
            .item(false, "No", "newtonian")
            .item(true, "Yes", "newtonian")
            .interact()?;
        Ok(g)
    }
}

pub struct PromptCelestialMagneticOption;
impl PromptAction for PromptCelestialMagneticOption {
    type Output = bool;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let g = select("Add magnetic field model?")
            .item(false, "No", "")
            .item(true, "Yes", "")
            .interact()?;
        Ok(g)
    }
}

pub struct PromptColor;
impl PromptAction for PromptColor {
    type Output = Color;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let mode = select("Color Mode")
            .item(ColorMode::Preset, "Preset", "")
            .item(ColorMode::Rgba, "RGBA", "")
            .interact()?;

        let color = match mode {
            ColorMode::Preset => {
                let color = select("Select Color")
                    .item(0, "White", "")
                    .item(1, "Black", "")
                    .item(2, "Red", "")
                    .item(3, "Green", "")
                    .item(4, "Blue", "")
                    .item(5, "Yellow", "")
                    .item(6, "Orange", "")
                    .interact()?;

                match color {
                    0 => Color::WHITE,
                    1 => Color::BLACK,
                    2 => Color::RED,
                    3 => Color::GREEN,
                    4 => Color::BLUE,
                    5 => Color::YELLOW,
                    6 => Color::ORANGE,
                    _ => unreachable!("picked in select"),
                }
            }
            ColorMode::Rgba => {
                let r = input("Red [0-1]")
                    .default_input("1.0")
                    .validate(PromptValidator::BetweenZeroAndOne)
                    .interact()?;

                let g = input("Green [0-1]")
                    .default_input("1.0")
                    .validate(PromptValidator::BetweenZeroAndOne)
                    .interact()?;

                let b = input("Blue [0-1]")
                    .default_input("1.0")
                    .validate(PromptValidator::BetweenZeroAndOne)
                    .interact()?;

                let a = input("Alpha [0-1]")
                    .default_input("1.0")
                    .validate(PromptValidator::BetweenZeroAndOne)
                    .interact()?;
                Color::new(r, g, b, a)
            }
        };

        Ok(color)
    }
}

pub struct PromptCuboid;
impl PromptAction for PromptCuboid {
    type Output = Cuboid;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let x = input("Cuboid Length [x] <m>")
            .default_input("1.0")
            .validate(PromptValidator::Positive)
            .interact()?;

        let y = input("Cuboid Length [y] <m>")
            .default_input("1.0")
            .validate(PromptValidator::Positive)
            .interact()?;

        let z = input("Cuboid Length [z] <m>")
            .default_input("1.0")
            .validate(PromptValidator::Positive)
            .interact()?;

        Ok(Cuboid::new(x, y, z))
    }
}

pub struct PromptCylindrical;
impl PromptAction for PromptCylindrical {
    type Output = Cylindrical;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let radius = input("Radius <m>")
            .default_input("0.0")
            .validate(PromptValidator::Positive)
            .interact()?;

        let azimuth = input("Azimuth <rad>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let height = input("Height <m>")
            .default_input("0.0")
            .validate(PromptValidator::Positive)
            .interact()?;

        Ok(Cylindrical::new(radius, azimuth, height))
    }
}

pub struct PromptDelay;
impl PromptAction for PromptDelay {
    type Output = f64;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let delay = input("Delay <seconds>")
            .default_input("0.0")
            .validate(PromptValidator::Positive)
            .interact()?;
        Ok(delay)
    }
}

//TODO: More generic ellipsoid pub structs, for now have to conpub struct in geometry
// pub struct PromptEllipsoid;
// impl PromptAction for PromptEllipsoid {
//     type Output = Ellipsoid;
//     fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
//         let radius_x = input("Ellipsoid Radius [x] <m>")
//             .default_input("1.0")
//             .validate(PromptValidator::Positive)
//             .interact()?;

//         let radius_y = input("Ellipsoid Radius [y] <m>")
//             .default_input("1.0")
//             .validate(PromptValidator::Positive)
//             .interact()?;

//         let radius_z = input("Ellipsoid Radius [z] <m>")
//             .default_input("1.0")
//             .validate(PromptValidator::Positive)
//             .interact()?;

//         let bands = input("Number of Latitude Bands")
//             .default_input("16")
//             .validate(PromptValidator::Positive)
//             .interact()?;
//         Ok(radius_x)
//     }
// }

pub struct PromptEpoch;
impl PromptAction for PromptEpoch {
    type Output = Time;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        info("Celestial Epoch")?;
        let system = select("Time System")
            .item(TimeSystem::UTC, "UTC", "")
            .item(TimeSystem::TAI, "TAI", "")
            .item(TimeSystem::GPS, "GPS", "")
            .item(TimeSystem::TT, "TT/TDB", "")
            .interact()?;

        let format = select("Time Format")
            .item(TimeFormat::DateTime, "DateTime", "")
            .item(TimeFormat::JulianDate, "JulianDate", "")
            .item(TimeFormat::SecondsSinceJ2000, "Seconds since J2000", "")
            .interact()?;

        let epoch = match format {
            TimeFormat::DateTime => {
                let year = input("Year")
                    .default_input("2000")
                    .validate(PromptValidator::ValidYear)
                    .interact()?;

                let month = input("Month")
                    .default_input("1")
                    .validate(PromptValidator::Integer)
                    .validate(PromptValidator::ValidMonth)
                    .interact()?;

                let day = input("Day")
                    .default_input("1")
                    .validate(PromptValidator::Integer)
                    .validate(PromptValidator::ValidDay)
                    .interact()?;

                let hour = input("Hour")
                    .default_input("0")
                    .validate(PromptValidator::Integer)
                    .validate(PromptValidator::ValidHour)
                    .interact()?;

                let minute = input("Minute")
                    .default_input("0")
                    .validate(PromptValidator::Integer)
                    .validate(PromptValidator::ValidMinute)
                    .interact()?;

                let second = input("Second")
                    .default_input("0.0")
                    .validate(PromptValidator::Integer)
                    .validate(PromptValidator::ValidSecond)
                    .interact()?;

                Time::from_ymdhms(year, month, day, hour, minute, second, system)?
            }
            TimeFormat::JulianDate => {
                let jd = input("Julian Date")
                    .validate(PromptValidator::Numeric)
                    .default_input("2451545.0")
                    .interact()?;
                Time::from_jd(jd, system)
            }
            TimeFormat::SecondsSinceJ2000 => {
                let s = input("Seconds since J2000")
                    .validate(PromptValidator::Numeric)
                    .default_input("0.0")
                    .interact()?;
                Time::from_sec_j2k(s, system)
            }
        };
        Ok(epoch)
    }
}

pub struct PromptEulerAngles;
impl PromptAction for PromptEulerAngles {
    type Output = EulerAngles;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let sequence = select("Euler Sequence")
            .item(EulerSequence::ZYX, "ZYX", "")
            .item(EulerSequence::XYZ, "XYZ", "")
            .item(EulerSequence::XZY, "XZY", "")
            .item(EulerSequence::YXZ, "YXZ", "")
            .item(EulerSequence::YZX, "YZX", "")
            .item(EulerSequence::ZXY, "ZXY", "")
            .item(EulerSequence::XYX, "XYX", "")
            .item(EulerSequence::YXY, "YXY", "")
            .item(EulerSequence::XZX, "XZX", "")
            .item(EulerSequence::ZXZ, "ZXZ", "")
            .item(EulerSequence::YZY, "YZY", "")
            .item(EulerSequence::ZYZ, "ZYZ", "")
            .interact()?;

        let phi = input("Phi <rad>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let psi = input("Psi <rad>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let theta = input("Theta <rad>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        Ok(EulerAngles::new(phi, theta, psi, sequence))
    }
}

pub struct PromptKeplerian;
impl PromptAction for PromptKeplerian {
    type Output = KeplerianElements;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let central_body = PromptCelestialBodies.prompt()?;
        let epoch = PromptEpoch.prompt()?;

        let a = input("Semimajor Axis <m>")
            .validate(PromptValidator::Positive)
            .interact()?;
        let e = input("Eccentricity")
            .validate(PromptValidator::Positive)
            .interact()?;

        let i = input("Inclination <rad>")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let raan = input("Right Ascension of the Ascending Node (RAAN) <rad>")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let argp = input("Argument of Periapsis <rad>")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let f = input("True Anomaly <rad>")
            .validate(PromptValidator::Numeric)
            .interact()?;

        Ok(KeplerianElements::new(
            a,
            e,
            i,
            raan,
            argp,
            f,
            epoch,
            central_body,
        ))
    }
}

pub struct PromptJointFloating;
impl PromptAction for PromptJointFloating {
    type Output = Floating;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let name = PromptName.prompt()?;
        info("Initial Orientation")?;
        let rotation = PromptRotation.prompt()?;
        let rate = PromptAngularRate3.prompt()?;

        let translation_type = select("How would you like to specify translation?")
            .item(0, "Position & Velocity", "")
            .item(1, "Keplerian Elements", "")
            .interact()?;

        let (position, velocity) = match translation_type {
            0 => (PromptPosition.prompt()?, PromptVelocity.prompt()?),
            1 => {
                let kep = PromptKeplerian.prompt()?;
                kep.get_rv()
            },
            _ => unreachable!("picked in a select")
        };

        let state = FloatingState::new(Quaternion::from(rotation), rate, position, velocity);

        let parameters = if PromptJointMechanicsOption.prompt()? {
            info("Rotational Mechanics [x]")?;
            let rx = PromptJointMechanics.prompt()?;
            info("Rotational Mechanics [y]")?;
            let ry = PromptJointMechanics.prompt()?;
            info("Rotational Mechanics [z]")?;
            let rz = PromptJointMechanics.prompt()?;
            info("Translational Mechanics [x]")?;
            let tx = PromptJointMechanics.prompt()?;
            info("Translational Mechanics [y]")?;
            let ty = PromptJointMechanics.prompt()?;
            info("Translational Mechanics [z]")?;
            let tz = PromptJointMechanics.prompt()?;
            [rx, ry, rz, tx, ty, tz]
        } else {
            let p = JointParameters::default();
            [p, p, p, p, p, p]
        };
        Ok(Floating::new(&name, parameters, state))
    }
}

pub struct PromptJointPrismatic;
impl PromptAction for PromptJointPrismatic {
    type Output = Prismatic;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let name = PromptName.prompt()?;
        let position = input("Initial Position <m>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        let velocity = input("Initial Velocity <m/s>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        let state = PrismaticState::new(position, velocity);

        let parameters = if PromptJointMechanicsOption.prompt()? {
            PromptJointMechanics.prompt()?
        } else {
            JointParameters::default()
        };
        Ok(Prismatic::new(&name, parameters, state))
    }
}

pub struct PromptJointRevolute;
impl PromptAction for PromptJointRevolute {
    type Output = Revolute;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let name = PromptName.prompt()?;
        let position = input("Initial Angle <rad>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        let velocity = input("Initial Angular Rate <rad/s>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        let state = RevoluteState::new(position, velocity);

        let parameters = if PromptJointMechanicsOption.prompt()? {
            PromptJointMechanics.prompt()?
        } else {
            JointParameters::default()
        };
        Ok(Revolute::new(&name, parameters, state))
    }
}

pub struct PromptGeometry;
impl PromptAction for PromptGeometry {
    type Output = Geometry;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let geometry = select("Select Geometry Type")
            .item(0, "Cuboid", "")
            .item(1, "Ellipsoid", "")
            .interact()?;
        let geometry = match geometry {
            0 => Geometry::Cuboid(PromptCuboid.prompt()?),
            1 => {
                let radius_x = input("Ellipsoid Radius [x] <m>")
                    .default_input("1.0")
                    .validate(PromptValidator::Positive)
                    .interact()?;

                let radius_y = input("Ellipsoid Radius [y] <m>")
                    .default_input("1.0")
                    .validate(PromptValidator::Positive)
                    .interact()?;

                let radius_z = input("Ellipsoid Radius [z] <m>")
                    .default_input("1.0")
                    .validate(PromptValidator::Positive)
                    .interact()?;

                let bands = input("Number of Latitude Bands [16,32,64]")
                    .default_input("64")
                    .validate(PromptValidator::Positive)
                    .interact()?;

                let ellipsoid = Ellipsoid::new(radius_x, radius_y, radius_z);
                match bands {
                    16 => Geometry::Ellipsoid16(Ellipsoid16(ellipsoid)),
                    32 => Geometry::Ellipsoid32(Ellipsoid32(ellipsoid)),
                    64 => Geometry::Ellipsoid64(Ellipsoid64(ellipsoid)),
                    _ => unreachable!("caught in validator"),
                }
            }
            _ => unreachable!("picked in a select"),
        };
        Ok(geometry)
    }
}

pub struct PromptGravity;
impl PromptAction for PromptGravity {
    type Output = Gravity;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let gravity_type = select("Select Gravity Type")
            .item(0, "Constant", "")
            .item(1, "Newtonian", "")
            .interact()?;

        let gravity = match gravity_type {
            0 => PromptGravityConstant.prompt()?,
            1 => PromptGravityNewtownian.prompt()?,
            _ => unreachable!("picked in a select"),
        };

        Ok(gravity)
    }
}

pub struct PromptGravityConstant;
impl PromptAction for PromptGravityConstant {
    type Output = Gravity;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let x = input("Constant Gravity [x] <m/s^2>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let y = input("Constant Gravity [y] <m/s^2>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let z = input("Constant Gravity [z] <m/s^2>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        Ok(Gravity::Constant(ConstantGravity::new(x, y, z)))
    }
}

pub struct PromptGravityNewtownian;
impl PromptAction for PromptGravityNewtownian {
    type Output = Gravity;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let mu = input("Newtonian Gravity Mu <m^3/s^2>")
            .default_input("398600.4418") // Default is Earth's standard gravitational parameter
            .validate(PromptValidator::Positive)
            .interact()?;
        Ok(Gravity::Newtownian(NewtownianGravity::new(mu)))
    }
}

pub struct PromptJointMechanicsOption;
impl PromptAction for PromptJointMechanicsOption {
    type Output = bool;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let mechanics = select("Add Joint Mechanics?")
            .item(false, "No", "")
            .item(true, "Yes", "")
            .interact()?;
        Ok(mechanics)
    }
}

pub struct PromptJointMechanics;
impl PromptAction for PromptJointMechanics {
    type Output = JointParameters;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let force = input("Joint Constant Force <N>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let spring = input("Joint Spring Constant <N/m>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let damping = input("Joint Damping")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        Ok(JointParameters::new(force, damping, spring))
    }
}

pub struct PromptMain;
impl PromptAction for PromptMain {
    type Output = String;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let main_option = input("NADIR")
            .default_input("default")
            .validate(PromptValidator::NonEmpty)
            .interact()?;
        Ok(main_option)
    }
}

pub struct PromptMassProperties;
impl PromptAction for PromptMassProperties {
    type Output = MassProperties;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let mass = input("Mass <kg>")
            .default_input("1.0")
            .validate(PromptValidator::Positive)
            .interact()?;

        let cmx = input("Center of Mass [x] <m>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let cmy = input("Center of Mass [y] <m>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let cmz = input("Center of Mass [z] <m>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let ixx = input("Ixx <kg*m^2>")
            .default_input("1.0")
            .validate(PromptValidator::Positive)
            .interact()?;

        let iyy = input("Iyy <kg*m^2>")
            .default_input("1.0")
            .validate(PromptValidator::Positive)
            .interact()?;

        let izz = input("Izz <kg*m^2>")
            .default_input("1.0")
            .validate(PromptValidator::Positive)
            .interact()?;

        let ixy = input("Ixy <kg*m^2>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let ixz = input("Ixz <kg*m^2>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let iyz = input("Iyz <kg*m^2>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let cm = CenterOfMass::new(cmx, cmy, cmz);
        let inertia = Inertia::new(ixx, iyy, izz, ixy, ixz, iyz)?;

        Ok(MassProperties::new(mass, cm, inertia)?)
    }
}

pub struct PromptMaterial;
impl PromptAction for PromptMaterial {
    type Output = Material;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let material = select("Material")
            .item(0, "Basic", "no lighting effects - flat color")
            .item(1, "Phong", "diffuse and specular lighting")
            .interact()?;
        let color = PromptColor.prompt()?;
        let material = match material {
            0 => Material::Basic { color },
            1 => {
                let specular_power = PromptSpecularPower.prompt()?;
                Material::Phong {
                    color,
                    specular_power,
                }
            }
            _ => unreachable!("picked in a select"),
        };
        Ok(material)
    }
}

pub struct PromptMax;
impl PromptAction for PromptMax {
    type Output = f64;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let max = input("Maximum Value")
            .default_input("1.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        Ok(max)
    }
}

pub struct PromptMean;
impl PromptAction for PromptMean {
    type Output = f64;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let mean = input("Mean Value")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        Ok(mean)
    }
}

pub struct PromptMesh;
impl PromptAction for PromptMesh {
    type Output = bool;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let mesh = select("Add mesh for animation?")
            .item(true, "yes", "")
            .item(false, "no", "")
            .interact()?;
        Ok(mesh)
    }
}

pub struct PromptMin;
impl PromptAction for PromptMin {
    type Output = f64;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let min = input("Enter Minimum Value")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        Ok(min)
    }
}

pub struct PromptName;
impl PromptAction for PromptName {
    type Output = String;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let name = input("Name")
            .default_input("default")
            .validate(PromptValidator::NonEmpty)
            .interact()?;
        Ok(name)
    }
}

pub struct PromptNoiseModel;
impl PromptAction for PromptNoiseModel {
    type Output = NoiseModels;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let noise_model = select("Noise Model")
            .item(0, "None", "")
            .item(1, "Uniform", "")
            .item(2, "Gaussian", "")
            .interact()?;

        let noise_model = match noise_model {
            0 => NoiseModels::None,
            1 => {
                let min = PromptMin.prompt()?;
                let max = PromptMax.prompt()?;
                NoiseModels::Uniform(UniformNoise::new(min, max))
            }
            2 => {
                let mean = PromptMean.prompt()?;
                let sigma = PromptStd.prompt()?;
                NoiseModels::Gaussian(GaussianNoise::new(mean, sigma))
            }
            _ => unreachable!("picked in a select"),
        };
        Ok(noise_model)
    }
}

pub struct PromptPosition;
impl PromptAction for PromptPosition {
    type Output = Vector3<f64>;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let x = input("Initial Position [x] <m>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        let y = input("Initial Position [y] <m>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        let z = input("Initial Position [z] <m>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        Ok(Vector3::new(x, y, z))
    }
}

pub struct PromptQuaternion;
impl PromptAction for PromptQuaternion {
    type Output = Quaternion;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let x = input("Quaternion [x]")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let y = input("Quaternion [y]")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let z = input("Quaternion [z]")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let w = input("Quaternion [w]")
            .default_input("1.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        Ok(Quaternion::new(x, y, z, w))
    }
}

pub struct PromptSensor;
impl PromptAction for PromptSensor {
    type Output = Sensor;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let name = PromptName.prompt()?;
        let model = select("Sensor Model")
            .item(0, "Body Rate 3-axis", "")
            .item(1, "Body Rate 1-axis", "")
            .interact()?;
        let model = match model {
            0 => {
                let delay = PromptDelay.prompt()?;
                let noise_model = PromptNoiseModel.prompt()?;
                let noise = Noise::new(noise_model);
                SensorModel::Simple(SimpleSensor::Rate3(Rate3Sensor::new(delay, noise)))
            }
            1 => {
                let delay = PromptDelay.prompt()?;
                let noise_model = PromptNoiseModel.prompt()?;
                let noise = Noise::new(noise_model);
                SensorModel::Simple(SimpleSensor::Rate(RateSensor::new(delay, noise)))
            }
            _ => unreachable!("picked in a select"),
        };
        Ok(Sensor::new(name, model))
    }
}

pub struct PromptSimulation;
impl PromptAction for PromptSimulation {
    type Output = SimulationConfig;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let name = PromptName.prompt()?;

        let start = input("Simulation Start Time <seconds>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let stop = input("Simulation Stop Time <seconds>")
            .default_input("10.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let dt = input("Simulation Time Step <seconds>")
            .default_input("0.1")
            .validate(PromptValidator::Positive)
            .interact()?;
        Ok(SimulationConfig {
            name,
            start,
            stop,
            dt,
        })
    }
}

pub struct PromptSpherical;
impl PromptAction for PromptSpherical {
    type Output = Spherical;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let radius = input("Spherical Radius <m>")
            .default_input("1.0")
            .validate(PromptValidator::Positive)
            .interact()?;

        let azimuth = input("Spherical Azimuth <rad>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let inclination = input("Spherical Inclination <rad>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        Ok(Spherical::new(radius, azimuth, inclination))
    }
}

pub struct PromptSpecularPower;
impl PromptAction for PromptSpecularPower {
    type Output = f32;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let power = input("Specular Power")
            .default_input("32.0")
            .validate(PromptValidator::Positive)
            .interact()?;
        Ok(power)
    }
}

pub struct PromptStd;
impl PromptAction for PromptStd {
    type Output = f64;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let std = input("Standard Deviation")
            .default_input("1.0")
            .validate(PromptValidator::Positive)
            .interact()?;
        Ok(std)
    }
}

pub struct PromptTransform;
impl PromptAction for PromptTransform {
    type Output = Transform;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let transform = select("Transform")
            .item(0, "Identity", "")
            .item(1, "Custom", "")
            .interact()?;
        let transform = match transform {
            0 => Transform::IDENTITY,
            1 => {
                let rotation = PromptRotation.prompt()?;
                let translation = PromptTranslation.prompt()?;
                Transform::new(rotation, translation)
            }
            _ => unreachable!("picked in a select"),
        };
        Ok(transform)
    }
}

pub struct PromptRotation;
impl PromptAction for PromptRotation {
    type Output = Rotation;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let rotation = select("Rotation")
            .item(0, "Identity", "")
            .item(1, "Quaternion", "")
            .item(2, "Rotation Matrix", "")
            .item(3, "Euler Angles", "")
            .item(4, "Aligned Axes", "")
            .interact()?;

        let rotation = match rotation {
            0 => Rotation::IDENTITY,
            1 => {
                let quaternion = PromptQuaternion.prompt()?;
                Rotation::from(quaternion)
            }
            2 => {
                let rotation_matrix = PromptRotationMatrix.prompt()?;
                Rotation::from(rotation_matrix)
            }
            3 => {
                let euler_angles = PromptEulerAngles.prompt()?;
                Rotation::from(euler_angles)
            }
            4 => {
                let aligned_axes = PromptAlignedAxes.prompt()?;
                Rotation::from(aligned_axes)
            }
            _ => unreachable!("picked in a select"),
        };
        Ok(rotation)
    }
}

pub struct PromptRotationMatrix;
impl PromptAction for PromptRotationMatrix {
    type Output = RotationMatrix;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let e11 = input("[0,0]")
            .default_input("1.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let e21 = input("[1,0]")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let e31 = input("[2,0]")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let e12 = input("[0,1]")
            .default_input("1.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let e22 = input("[1,1]")
            .default_input("1.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let e32 = input("[2,1]")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let e13 = input("[0,2]")
            .default_input("1.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let e23 = input("[1,2]")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        let e33 = input("[2,2]")
            .default_input("1.0")
            .validate(PromptValidator::Numeric)
            .interact()?;

        Ok(RotationMatrix::new(
            e11, e12, e13, e21, e22, e23, e31, e32, e33,
        )?)
    }
}

pub struct PromptTranslation;
impl PromptAction for PromptTranslation {
    type Output = CoordinateSystem;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let translation = select("Translation")
            .item(0, "Zero", "")
            .item(1, "Cartesian", "")
            .item(2, "Cylindrical", "")
            .item(3, "Spherical", "")
            .interact()?;
        let translation = match translation {
            0 => CoordinateSystem::ZERO,
            1 => CoordinateSystem::from(PromptCartesian.prompt()?),
            2 => CoordinateSystem::from(PromptCylindrical.prompt()?),
            3 => CoordinateSystem::from(PromptSpherical.prompt()?),
            _ => unreachable!("picked in a select"),
        };

        Ok(CoordinateSystem::from(translation))
    }
}

pub struct PromptVelocity;
impl PromptAction for PromptVelocity {
    type Output = Vector3<f64>;
    fn prompt(&self) -> Result<Self::Output, Box<dyn Error>> {
        let vx = input("Initial Velocity X <m/s>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        let vy = input("Initial Velocity Y <m/s>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        let vz = input("Initial Velocity Z <m/s>")
            .default_input("0.0")
            .validate(PromptValidator::Numeric)
            .interact()?;
        Ok(Vector3::new(vx, vy, vz))
    }
}

enum PromptValidator {
    BetweenZeroAndOne,
    Integer,
    NonEmpty,
    Numeric,
    Positive,
    ValidYear,
    ValidMonth,
    ValidDay,
    ValidHour,
    ValidMinute,
    ValidSecond,
}

impl cliclack::Validate<String> for PromptValidator {
    type Err = String;

    fn validate(&self, input: &String) -> Result<(), Self::Err> {
        match self {
            // BetweenZeroAndOne validator: checks if the input is a number between 0 and 1 inclusive
            PromptValidator::BetweenZeroAndOne => {
                match input.parse::<f64>() {
                    Ok(num) if (0.0..=1.0).contains(&num) => Ok(()), // Number is between 0 and 1 inclusive
                    Ok(_) => Err(format!("'{}' is not between 0 and 1 inclusive", input)), // Outside range
                    Err(_) => Err(format!("'{}' is not a valid number", input)), // Parsing error
                }
            }
            // Integer validator: checks if the input is a valid integer
            PromptValidator::Integer => match input.parse::<i64>() {
                Ok(_) => Ok(()),
                Err(_) => Err(format!("'{}' is not a valid integer", input)),
            },
            // Numeric validator: checks if the input is a valid number
            PromptValidator::Numeric => input
                .parse::<f64>()
                .map(|_| ())
                .map_err(|_| format!("'{}' is not a valid number", input)),
            // Non-empty validator: checks if the input is not empty
            PromptValidator::NonEmpty => {
                if input.is_empty() {
                    Err("Input cannot be empty".to_string())
                } else {
                    Ok(())
                }
            }
            // Positive validator: checks if the input is a number and >= 0
            PromptValidator::Positive => {
                match input.parse::<f64>() {
                    Ok(num) if num >= 0.0 => Ok(()), // Valid positive number
                    Ok(_) => Err(format!("'{}' is not a positive number", input)), // Negative number
                    Err(_) => Err(format!("'{}' is not a valid number", input)),   // Parsing error
                }
            }
            // ValidDay validator: checks if the input is a valid day (1 to 31)
            PromptValidator::ValidDay => {
                match input.parse::<u32>() {
                    Ok(day) if (1..=31).contains(&day) => Ok(()), // Day is between 1 and 31
                    Ok(_) => Err(format!(
                        "'{}' is not a valid day (must be between 1 and 31)",
                        input
                    )),
                    Err(_) => Err(format!("'{}' is not a valid number", input)), // Parsing error
                }
            }
            // ValidHour validator: checks if the input is a valid hour (0 to 23)
            PromptValidator::ValidHour => {
                match input.parse::<u32>() {
                    Ok(hour) if (0..=23).contains(&hour) => Ok(()), // Hour is between 0 and 23
                    Ok(_) => Err(format!(
                        "'{}' is not a valid hour (must be between 0 and 23)",
                        input
                    )),
                    Err(_) => Err(format!("'{}' is not a valid number", input)), // Parsing error
                }
            }
            // ValidMinute validator: checks if the input is a valid minute (0 to 59)
            PromptValidator::ValidMinute => {
                match input.parse::<u32>() {
                    Ok(minute) if (0..=59).contains(&minute) => Ok(()), // Minute is between 0 and 59
                    Ok(_) => Err(format!(
                        "'{}' is not a valid minute (must be between 0 and 59)",
                        input
                    )),
                    Err(_) => Err(format!("'{}' is not a valid number", input)), // Parsing error
                }
            }
            // ValidMonth validator: checks if the input is a valid month (1 to 12)
            PromptValidator::ValidMonth => {
                match input.parse::<u32>() {
                    Ok(month) if (1..=12).contains(&month) => Ok(()), // Month is between 1 and 12
                    Ok(_) => Err(format!(
                        "'{}' is not a valid month (must be between 1 and 12)",
                        input
                    )),
                    Err(_) => Err(format!("'{}' is not a valid number", input)), // Parsing error
                }
            }
            // ValidSecond validator: checks if the input is a valid second (0 to 59)
            PromptValidator::ValidSecond => {
                match input.parse::<f64>() {
                    Ok(second) if (0.0..60.0).contains(&second) => Ok(()), // Second is between 0 and 59
                    Ok(_) => Err(format!(
                        "'{}' is not a valid second (must be between 0 and <60)",
                        input
                    )),
                    Err(_) => Err(format!("'{}' is not a valid number", input)), // Parsing error
                }
            }
            // ValidYear validator: checks if the input is a valid year (let's assume 1900 to 2100 for example)
            PromptValidator::ValidYear => {
                match input.parse::<i32>() {
                    Ok(year) if (1900..=2100).contains(&year) => Ok(()), // Year is within a reasonable range
                    Ok(_) => Err(format!(
                        "'{}' is not a valid year (must be between 1900 and 2100)",
                        input
                    )),
                    Err(_) => Err(format!("'{}' is not a valid number", input)), // Parsing error
                }
            }
        }
    }
}
