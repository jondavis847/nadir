pub struct ButcherTableau<const STAGES: usize> {
    pub a: [[f64; STAGES]; STAGES],
    pub b: [f64; STAGES],
    pub b2: Option<[f64; STAGES]>,
    pub c: [f64; STAGES],
    pub use_higher_order: bool,
}

impl ButcherTableau<4> {
    // usage is ButcherTableau::<4>::RK4
    pub const RK4: Self = Self {
        a: [
            [0., 0., 0., 0.],
            [1. / 2., 0., 0., 0.],
            [0., 1. / 2., 0., 0.],
            [0., 0., 1., 0.],
        ],
        b: [1. / 6., 1. / 3., 1. / 3., 1. / 6.],
        b2: None,
        c: [0., 1.0 / 2.0, 1.0 / 2.0, 1.0],
        use_higher_order: false,
    };
}
impl ButcherTableau<7> {
    // usage is ButcherTableau::<7>::DORMANDRINCE45
    pub const DORMANDPRINCE45: Self = Self {
        a: [
            [0., 0., 0., 0., 0., 0., 0.],
            [1. / 5., 0., 0., 0., 0., 0., 0.],
            [3. / 40., 9. / 40., 0., 0., 0., 0., 0.],
            [44. / 45., -56. / 15., 32. / 9., 0., 0., 0., 0.],
            [
                19372. / 6561.,
                -25360. / 2187.,
                64448. / 6561.,
                -212. / 729.,
                0.,
                0.,
                0.,
            ],
            [
                9017. / 3168.,
                -355. / 33.,
                46732. / 5247.,
                49. / 176.,
                -5103. / 18656.,
                0.,
                0.,
            ],
            [
                35. / 384.,
                0.,
                500. / 1113.,
                125. / 192.,
                -2187. / 6784.,
                11. / 84.,
                0.,
            ],
        ],
        b: [
            35. / 384.,
            0.,
            500. / 1113.,
            125. / 192.,
            -2187. / 6784.,
            11. / 84.,
            0.,
        ],
        b2: Some([
            5179. / 57600.,
            0.,
            7571. / 16695.,
            393. / 640.,
            -92097. / 339200.,
            187. / 2100.,
            1. / 40.,
        ]),
        c: [0., 1. / 5., 3. / 10., 4. / 5., 8. / 9., 1.0, 1.0],
        use_higher_order: true,
    };
}
