use std::collections::HashMap;

use aerospace::celestial_system::CelestialBodies;
use color::Color;
use gadgt_3d::mesh::{Mesh, MeshPrimitive};
use gadgt_3d::{
    geometry::{ellipsoid::Ellipsoid64, Geometry, GeometryState},
    material::Material,
};
use glam::{DQuat, DVec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CelestialMeshes {
    Earth,
    EarthAtmosphere,
    Jupiter,
    Mars,
    Mercury,
    Moon,
    Neptune,
    Pluto,
    Saturn,
    //SaturnRings,
    Sun,
    SunCorona,
    Uranus,
    Venus,
}

impl CelestialMeshes {
    pub fn to_body(&self) -> CelestialBodies {
        match self {
            CelestialMeshes::Earth => CelestialBodies::Earth,
            CelestialMeshes::EarthAtmosphere => CelestialBodies::Earth,
            CelestialMeshes::Jupiter => CelestialBodies::Jupiter,
            CelestialMeshes::Mars => CelestialBodies::Mars,
            CelestialMeshes::Mercury => CelestialBodies::Mercury,
            CelestialMeshes::Moon => CelestialBodies::Moon,
            CelestialMeshes::Neptune => CelestialBodies::Neptune,
            CelestialMeshes::Pluto => CelestialBodies::Pluto,
            CelestialMeshes::Saturn => CelestialBodies::Saturn,
            //      CelestialMeshes::SaturnRings => CelestialBodies::Saturn,
            CelestialMeshes::Sun => CelestialBodies::Sun,
            CelestialMeshes::SunCorona => CelestialBodies::Sun,
            CelestialMeshes::Uranus => CelestialBodies::Uranus,
            CelestialMeshes::Venus => CelestialBodies::Venus,
        }
    }
}
#[derive(Debug, Default)]
pub struct CelestialAnimation {
    pub meshes: HashMap<CelestialMeshes, Mesh>,
}

impl CelestialAnimation {
    pub fn add_body(&mut self, body: CelestialBodies) {
        match body {
            CelestialBodies::Earth => {
                let earth_polar_radius = 6356752.3;
                let earth_equatorial_radius = 6378137.0;
                let atmosphere_height = 1e5; //100km
                let earth_mesh = celestial_mesh(
                    "earth",
                    earth_equatorial_radius,
                    earth_polar_radius,
                    Material::Basic {
                        color: Color::WHITE,
                    },
                );
                self.meshes.insert(CelestialMeshes::Earth, earth_mesh);
                let atmosphere_mesh = celestial_mesh(
                    "earth_atmosphere",
                    earth_equatorial_radius + atmosphere_height,
                    earth_polar_radius + atmosphere_height,
                    Material::Basic {
                        color: Color::WHITE,
                    },
                );
                self.meshes
                    .insert(CelestialMeshes::EarthAtmosphere, atmosphere_mesh)
            }
            CelestialBodies::Jupiter => self.meshes.insert(
                CelestialMeshes::Jupiter,
                celestial_mesh(
                    "jupiter",
                    71492000.0,
                    66854000.0,
                    Material::Phong {
                        color: Color::ORANGE,
                        specular_power: 32.0,
                    },
                ),
            ),
            CelestialBodies::Mars => self.meshes.insert(
                CelestialMeshes::Mars,
                celestial_mesh(
                    "mars",
                    3396200.0,
                    3376200.0,
                    Material::Phong {
                        color: Color::ORANGE,
                        specular_power: 32.0,
                    },
                ),
            ),
            CelestialBodies::Mercury => self.meshes.insert(
                CelestialMeshes::Mercury,
                celestial_mesh(
                    "mercury",
                    2440500.0,
                    2438300.0,
                    Material::Phong {
                        color: Color::WHITE,
                        specular_power: 32.0,
                    },
                ),
            ),
            CelestialBodies::Moon => self.meshes.insert(
                CelestialMeshes::Moon,
                celestial_mesh(
                    "moon",
                    1738100.0,
                    1736000.0,
                    Material::Phong {
                        color: Color::WHITE,
                        specular_power: 32.0,
                    },
                ),
            ),
            CelestialBodies::Neptune => self.meshes.insert(
                CelestialMeshes::Neptune,
                celestial_mesh(
                    "neptune",
                    24764000.0,
                    24341000.0,
                    Material::Phong {
                        color: Color::WHITE,
                        specular_power: 32.0,
                    },
                ),
            ),
            CelestialBodies::Pluto => self.meshes.insert(
                CelestialMeshes::Pluto,
                celestial_mesh(
                    "pluto",
                    1188000.0,
                    1188000.0,
                    Material::Phong {
                        color: Color::WHITE,
                        specular_power: 32.0,
                    },
                ),
            ),
            CelestialBodies::Saturn => self.meshes.insert(
                CelestialMeshes::Saturn,
                celestial_mesh(
                    "saturn",
                    60268000.0,
                    54364000.0,
                    Material::Phong {
                        color: Color::WHITE,
                        specular_power: 32.0,
                    },
                ),
            ),
            CelestialBodies::Sun => {
                let sun_radius = 696340000.0;
                let corona_radius = 3e10;
                self.meshes.insert(
                    CelestialMeshes::Sun,
                    celestial_mesh(
                        "sun",
                        sun_radius,
                        sun_radius,
                        Material::Phong {
                            color: Color::WHITE,
                            specular_power: 32.0,
                        },
                    ),
                );
                let corona_mesh = celestial_mesh(
                    "sun_corona",
                    sun_radius + corona_radius,
                    sun_radius + corona_radius,
                    Material::Basic {
                        color: Color::WHITE,
                    },
                );
                self.meshes
                    .insert(CelestialMeshes::SunCorona, corona_mesh)
            }
            CelestialBodies::Uranus => self.meshes.insert(
                CelestialMeshes::Uranus,
                celestial_mesh(
                    "uranus",
                    25559000.0,
                    24973000.0,
                    Material::Phong {
                        color: Color::WHITE,
                        specular_power: 32.0,
                    },
                ),
            ),
            CelestialBodies::Venus => self.meshes.insert(
                CelestialMeshes::Venus,
                celestial_mesh(
                    "venus",
                    6051800.0,
                    6051800.0,
                    Material::Phong {
                        color: Color::WHITE,
                        specular_power: 32.0,
                    },
                ),
            ),
        };
    }

    pub fn update_body(&mut self, body: CelestialBodies, position: DVec3, rotation: DQuat) {
        match body {
            CelestialBodies::Earth => {
                if let Some(earth) = self.meshes.get_mut(&CelestialMeshes::Earth) {
                    earth.update(position, rotation);
                }

                if let Some(atmosphere) = self.meshes.get_mut(&CelestialMeshes::EarthAtmosphere) {
                    atmosphere.update(position, rotation);
                }
            }
            CelestialBodies::Jupiter => {
                if let Some(body) = self.meshes.get_mut(&CelestialMeshes::Jupiter) {
                    body.update(position, rotation);
                }
            }
            CelestialBodies::Mars => {
                if let Some(body) = self.meshes.get_mut(&CelestialMeshes::Mars) {
                    body.update(position, rotation);
                }
            }
            CelestialBodies::Mercury => {
                if let Some(body) = self.meshes.get_mut(&CelestialMeshes::Mercury) {
                    body.update(position, rotation);
                }
            }
            CelestialBodies::Moon => {
                if let Some(body) = self.meshes.get_mut(&CelestialMeshes::Moon) {
                    body.update(position, rotation);
                }
            }
            CelestialBodies::Neptune => {
                if let Some(body) = self.meshes.get_mut(&CelestialMeshes::Neptune) {
                    body.update(position, rotation);
                }
            }
            CelestialBodies::Pluto => {
                if let Some(body) = self.meshes.get_mut(&CelestialMeshes::Pluto) {
                    body.update(position, rotation);
                }
            }
            CelestialBodies::Saturn => {
                if let Some(body) = self.meshes.get_mut(&CelestialMeshes::Saturn) {
                    body.update(position, rotation);
                }
            }
            CelestialBodies::Sun => {
                if let Some(body) = self.meshes.get_mut(&CelestialMeshes::Sun) {
                    body.update(position, rotation);
                }
            }
            CelestialBodies::Uranus => {
                if let Some(body) = self.meshes.get_mut(&CelestialMeshes::Uranus) {
                    body.update(position, rotation);
                }
            }
            CelestialBodies::Venus => {
                if let Some(body) = self.meshes.get_mut(&CelestialMeshes::Venus) {
                    body.update(position, rotation);
                }
            }
        };
    }
}

fn celestial_mesh(
    name: &str,
    equatorial_radius: f64,
    polar_radius: f64,
    material: Material,
) -> Mesh {
    Mesh {
        name: name.to_string(),
        geometry: Geometry::Ellipsoid64(Ellipsoid64::new(
            equatorial_radius,
            equatorial_radius,
            polar_radius,
        )),
        material,
        state: GeometryState {
            position: DVec3::ZERO,     //placeholder
            rotation: DQuat::IDENTITY, //placeholder
        },
        texture: None,
    }
}

#[derive(Debug)]
pub struct CelestialPrimitives {
    pub meshes: HashMap<CelestialMeshes, MeshPrimitive>,
}

impl From<&CelestialAnimation> for CelestialPrimitives {
    fn from(value: &CelestialAnimation) -> Self {
        let mut meshes = HashMap::new();
        for (body, mesh) in &value.meshes {
            meshes.insert(*body, MeshPrimitive::from(mesh));
        }
        Self { meshes }
    }
}
