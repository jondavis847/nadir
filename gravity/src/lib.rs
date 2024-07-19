use linear_algebra::vector3::Vector3;

#[derive(Debug, Clone)]
pub struct ConstantGravity{
    value:Vector3
}
impl ConstantGravity {
    pub fn new (x:f64, y:f64, z:f64)->Self{
        Self{value:Vector3::new(x,y,z)}
    }   
}

#[derive(Debug, Clone)]
pub struct TwoBodyGravity{
    mu:f64
}

pub trait Gravity {
    fn calculate(&self, position:Vector3)->Vector3;
}

impl Gravity for ConstantGravity{
    fn calculate(&self, _position:Vector3)->Vector3{
        self.value 
    }
}

impl Gravity for TwoBodyGravity{
    fn calculate(&self, position:Vector3)->Vector3 {
        let position_mag = position.magnitude();
        position*self.mu/position_mag.powi(3)     
    }    
}

#[derive(Debug, Clone)]
pub enum GravityEnum {
    Constant(ConstantGravity), 
    TwoBody(TwoBodyGravity),  
}
    
