use coordinate_systems::CoordinateSystem;
use rotations::Rotation;
use sim_value::SimValue;

#[derive(Debug, Default, Copy, Clone)]
pub struct Transform<T>
where
    T: SimValue,
{
    translation: CoordinateSystem<T>,
    rotation: Rotation<T>,
}
