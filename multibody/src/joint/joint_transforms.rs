
use serde::{Deserialize,Serialize};
use spatial_algebra::SpatialTransform;



/// We use the terminology B_from_A rather than A_to_B so that notation matches matrix multiplication
/// i.e. v_C = C_from_B * B_from_A * v_A instead of
///      v_C = (A_to_B * B_to_C) * v_A
/// base: the reference frame that is the base
/// inner_body: the "body frame" of the body on the base side of the joint
/// outer_body: the "body frame" of the body on the tip side of the joint
/// jif: the "joint inner frame"
/// jof: the "joint outer frame"
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct JointTransforms {
    // joint frame
    pub jif_from_jof: SpatialTransform, // my-joint-inner-frame from my-joint-outer-frame
    pub jof_from_jif: SpatialTransform, // my-joint-outer-frame from my-joint-inner-frame

    // body to joint frames
    pub jif_from_ib: SpatialTransform, // my-joint-inner-frame from my-inner-body-frame
    pub ib_from_jif: SpatialTransform, // my-inner-body-frame from my-joint-inner-frame

    pub jof_from_ob: SpatialTransform, // my-joint-outer-frame from my-outer-body-frame
    pub ob_from_jof: SpatialTransform, // my-outer-body-frame from my-joint-outer-frame

    // joint to joint frames
    pub jof_from_ij_jof: SpatialTransform, // my-joint-outer-frame from inner-joint-outer-frame
    pub ij_jof_from_jof: SpatialTransform, // inner-joint-outer-frame from my-joint-outer-frame

    // base to joint frames - only need outer really
    pub jof_from_base: SpatialTransform,
    pub base_from_jof: SpatialTransform,

    //base to outer body
    pub base_from_ob: SpatialTransform,
    pub ob_from_base: SpatialTransform,
}

impl JointTransforms {
    pub fn update(&mut self, ij_transforms: Option<(SpatialTransform, SpatialTransform)>) {
        // transforms are multiplied like matrices from right to left.
        // i.e. if you want to express v from frame A in frame C
        // you would use vC = C_to_B * B_to_A * vA
        // this means that the transform outer_body_to_inner_body is actually a
        // transform from the inner body to the outer body
        // I just like this notation better

        let jof_from_ij_jof;
        let ij_jof_from_jof;
        let jof_from_base;

        // get relevant transforms from the parent for calculations to the base, if the inner body is not the base
        if let Some((ij_ob_from_ij_jof, ij_jof_from_base)) = ij_transforms {
            // this joints inner body is the parent joints outer body            
            jof_from_ij_jof = self.jof_from_jif * self.jif_from_ib * ij_ob_from_ij_jof;
            ij_jof_from_jof = jof_from_ij_jof.inv();
            jof_from_base = jof_from_ij_jof * ij_jof_from_base;
        } else {
            // inner joint is the base, so base transform is the inner joint transform
            // note that the base to outer joint transform is still accounted for
            jof_from_ij_jof = self.jof_from_jif * self.jif_from_ib;
            ij_jof_from_jof = jof_from_ij_jof.inv();
            jof_from_base = jof_from_ij_jof;
        }
        self.jof_from_ij_jof = jof_from_ij_jof;
        self.ij_jof_from_jof = ij_jof_from_jof;
        self.jof_from_base = jof_from_base;
        self.base_from_jof = jof_from_base.inv();
        self.ob_from_base = self.ob_from_jof * jof_from_base;
        self.base_from_ob = self.ob_from_base.inv();
    }
}
