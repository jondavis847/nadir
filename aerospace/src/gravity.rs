use nalgebra::{Matrix5, SimdBool, Vector3};
use std::f64::consts::PI;
use serde::{Serialize, Deserialize};


pub const EARTH: f64 = 3.986004418e14; // mu (m^3/s^2)
pub const EARTH_RE: f64 = 6378137.0; // (m)  TODO: implement WGS84
pub const MAX_DEG: u8 = 4; //16; // degrees of spherical harmonics
pub const EGM96_C:Matrix5<f64> = Matrix5::new(0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0,
    -0.000484165371736000, -0.000000000186987636, 0.000002439143523980, 0.0, 0.0,
    0.000000957254173792, 0.000002029988821840, 0.000000904627768605, 0.000000721072657057, 0.0,
    0.000000539873863789, -0.000000536321616971, 0.000000350694105785, 0.000000990771803829, -0.000000188560802735);

pub const EGM96_S:Matrix5<f64> = Matrix5::new(0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.000119528012031e-5, -0.140016683654000e-5, 0.0, 0.0,
    0.0, 0.024851315871600e-5, -0.061902594420500e-5, 0.141435626958000e-5, 0.0,
    0.0, -0.047344026585300e-5, 0.066267157254000e-5, -0.020092836917700e-5, 0.030885316933300e-5);

/*
pub const EGM96_C:Array2<f64>= arr2(&[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-0.484165371736000e-3, -0.000000186987636e-3, 0.002439143523980e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0],
    [0.095725417379200e-5, 0.202998882184000e-5, 0.090462776860500e-5, 0.072107265705700e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.539873863789000e-6, -0.536321616971000e-6, 0.350694105785000e-6, 0.990771803829000e-6, -0.188560802735000e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
    [0.068532347563000e-6, -0.062101212852800e-6, 0.652438297612000e-6, -0.451955406071000e-6, -0.295301647654000e-6, 0.174971983203000e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-0.149957994714000e-6, -0.076087938494700e-6, 0.048173244283200e-6, 0.057173099051600e-6, -0.086214266010900e-6, -0.267133325490000e-6, 0.009676161210920e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.090978937145000e-6, 0.279872910488000e-6, 0.329743816488000e-6, 0.250398657706000e-6, -0.275114355257000e-6, 0.001937655072430e-6, -0.358856860645000e-6, 0.001091851480450e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.049671166732400e-6, 0.023342204789300e-6, 0.080297872261500e-6, -0.019187775700900e-6, -0.244600105471000e-6, -0.025535240303700e-6, -0.065736161096100e-6, 0.067281158007200e-6, -0.124092493016000e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.027671430085300e-6, 0.143387502749000e-6, 0.022228831856400e-6, -0.160811502143000e-6, -0.009001792253360e-6, -0.016616509292400e-6, 0.062694193824800e-6, -0.118366323475000e-6, 0.188436022794000e-6, -0.047747538613200e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.052622248856900e-6, 0.083511577565200e-6, -0.094241388208100e-6, -0.006898950481760e-6, -0.084076454971600e-6, -0.049339593818500e-6, -0.037588523659800e-6, 0.008114605409250e-6, 0.040492798169400e-6, 0.125491334939000e-6, 0.100538634409000e-6,0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
    [-0.509613707522000e-7, 0.151687209933000e-7, 0.186309749878000e-7, -0.309871239854000e-7, -0.389580205051000e-7, 0.377848029452000e-7, -0.011867659239500e-7, 0.041156518807400e-7, -0.059841084130000e-7, -0.314231072723000e-7, -0.521882681927000e-7, 0.460344448746000e-7, 0.0, 0.0, 0.0, 0.0, 0.0], 
    [0.377252636558000e-7, -0.540654977836000e-7, 0.142979642253000e-7, 0.393995876403000e-7, -0.686908127934000e-7, 0.309411128730000e-7, 0.034152327520800e-7, -0.186909958587000e-7, -0.253769398865000e-7, 0.422880630662000e-7, -0.061761965490200e-7, 0.112502994122000e-7, -0.024953260739000e-7, 0.0, 0.0, 0.0, 0.0],   
    [0.422982206413000e-7, -0.513569699124000e-7, 0.559217667099000e-7, -0.219360927945000e-7, -0.031376259966600e-7, 0.590049394905000e-7, -0.359038073075000e-7, 0.025300214708700e-7, -0.098315082269500e-7, 0.247325771791000e-7, 0.410324653930000e-7, -0.443869677399000e-7, -0.312622200222000e-7, -0.612759553199000e-7, 0.0, 0.0, 0.0],   
    [-0.242786502921000e-7, -0.186968616381000e-7, -0.367789379502000e-7, 0.358875097333000e-7, 0.018386561779200e-7, 0.287344273542000e-7, -0.194810485574000e-7, 0.375003839415000e-7, -0.350946485865000e-7, 0.320284939341000e-7, 0.390329180008000e-7, 0.153970516502000e-7, 0.084082916386900e-7, 0.322147043964000e-7, -0.518980794309000e-7, 0.0, 0.0],
    [0.014791006870800e-7, 0.100817268177000e-7, -0.213942673775000e-7, 0.521392929041000e-7, -0.408150084078000e-7, 0.124935723108000e-7, 0.331211643896000e-7, 0.596210699259000e-7, -0.322428691498000e-7, 0.128788268085000e-7, 0.104688722521000e-7, -0.011167506193400e-7, -0.323962134415000e-7, -0.283933019117000e-7, 0.051916885933000e-7, -0.190930538322000e-7, 0.0],
    [-0.031532298672200e-7, 0.258360856231000e-7, -0.233671404512000e-7, -0.336019429391000e-7, 0.402316284314000e-7, -0.129501939245000e-7, 0.140239252323000e-7, -0.070841263513600e-7, -0.209018868094000e-7, -0.218588720643000e-7, -0.117529900814000e-7,  0.187574042592000e-7, 0.195400194038000e-7, 0.138196369576000e-7, -0.193182168856000e-7, -0.145149060142000e-7, -0.379671710746000e-7]]);
    
pub const EGM96_S:Matrix17<f64> = Matrix17::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.000119528012031e-5, -0.140016683654000e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.024851315871600e-5, -0.061902594420500e-5, 0.141435626958000e-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.473440265853000e-6, 0.662671572540000e-6, -0.200928369177000e-6, 0.308853169333000e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.094422612752500e-6, -0.323349612668000e-6, -0.214847190624000e-6, 0.049665887676900e-6, -0.669384278219000e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.026289054550100e-6, -0.373728201347000e-6, 0.009026945171630e-6, -0.471408154267000e-6, -0.536488432483000e-6, -0.237192006935000e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.095433691186700e-6, 0.093066759604200e-6, -0.217198608738000e-6, -0.123800392323000e-6, 0.017737771987200e-6, 0.151789817739000e-6, 0.024441570799300e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.059006049341100e-6, 0.065417542585900e-6, -0.086345444502100e-6, 0.070023301693400e-6, 0.089146216478800e-6, 0.309238461807000e-6, 0.074744047363300e-6, 0.120533165603000e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.021683494761800e-6, -0.032219664711600e-6, -0.074228740946200e-6, 0.019466677947500e-6, -0.054111319148300e-6, 0.222903525945000e-6, -0.096515266788600e-6, -0.003085662204210e-6, 0.096641284771400e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.131314331796000e-6, -0.051579165739000e-6, -0.153768828694000e-6, -0.079280625533100e-6, -0.050537022189700e-6, -0.079566705387200e-6, -0.003366296413140e-6, -0.091870597592200e-6, -0.037651622239200e-6, -0.024014844952000e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.026860414616600e-6, -0.099069386204700e-6, -0.148131804260000e-6, -0.063666651198000e-6, 0.049473623816900e-6, 0.034476958459300e-6, -0.089825280897700e-6, 0.024398961223700e-6, 0.041773182982900e-6, -0.018336456178800e-6, -0.069666230818500e-6, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -0.435675748979000e-7, 0.320975937619000e-7, 0.244264863505000e-7, 0.041508110901100e-7, 0.078253627903300e-7, 0.391765484449000e-7, 0.356131849382000e-7, 0.169361024629000e-7, 0.252692598301000e-7, 0.308375794212000e-7, -0.063794650155800e-7, -0.111780601900000e-7, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.390510386685000e-7, -0.627337565381000e-7, 0.974829362237000e-7, -0.119627874492000e-7, 0.664975958036000e-7, -0.065728061368600e-7, -0.062147082233100e-7, -0.104740222825000e-7, 0.452870369936000e-7, -0.368121029480000e-7, -0.047650780428800e-7, 0.878405809267000e-7, 0.685261488594000e-7, 0.0, 0.0, 0.0,
    0.0, 0.294747542249000e-7, -0.051677939205500e-7, 0.204618827833000e-7, -0.226780613566000e-7, -0.163882249728000e-7, 0.024783127278100e-7, -0.041729131942900e-7, -0.153515265203000e-7, 0.288804922064000e-7, -0.014430845246900e-7, -0.390548173245000e-7, -0.311327189117000e-7, 0.451897224960000e-7, -0.048150663674800e-7, 0.0, 0.0,
    0.0, 0.109773066324000e-7, -0.308914875777000e-7, 0.172892926103000e-7, 0.065017470779400e-7, 0.080837556399600e-7, -0.368246004304000e-7, 0.053184117187900e-7, 0.221523579587000e-7, 0.375629820829000e-7, 0.147222147015000e-7, 0.180996198432000e-7, 0.155243104746000e-7, -0.042206679110300e-7, -0.243752739666000e-7, -0.047113942155800e-7, 0.0,
    0.0, 0.325447560859000e-7, 0.288799363439000e-7, -0.220418988010000e-7, 0.483837716909000e-7, -0.031945857812900e-7, -0.350760208303000e-7, -0.088158156113100e-7, 0.050052739053000e-7, -0.395012419994000e-7, 0.114211582961000e-7, -0.030316191992500e-7, 0.066698357407100e-7, 0.010277849950800e-7, -0.386174893776000e-7, -0.327443078739000e-7, 0.030215537265500e-7);
*/
pub const MOON: f64 = 4.9048695e12;

pub const SUN: f64 = 1.32712440018e20;

pub const MERCURY: f64 = 2.2032e13;

pub const VENUS: f64 = 3.24859e14;

pub const MARS: f64 = 4.282837e13;

pub const JUPITER: f64 = 1.26686534e17;

pub const SATURN: f64 = 3.7931187e16;

pub const URANUS: f64 = 5.793939e15;

pub const NEPTUNE: f64 = 6.836529e15;

pub const PLUTO: f64 = 8.71e11;

pub trait GravityTrait {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64>;
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Gravity {
    Constant(ConstantGravity),
    TwoBody(TwoBodyGravity),
    EGM96(EGM96Gravity),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConstantGravity {
    pub value: Vector3<f64>,
}
impl ConstantGravity {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            value: Vector3::new(x, y, z),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TwoBodyGravity {
    pub mu: f64,
}

impl TwoBodyGravity {
    pub const EARTH: Self = Self { mu: EARTH };

    pub const MOON: Self = Self { mu: MOON };

    pub const SUN: Self = Self { mu: SUN };

    pub const MERCURY: Self = Self { mu: MERCURY };

    pub const VENUS: Self = Self { mu: VENUS };

    pub const MARS: Self = Self { mu: MARS };

    pub const JUPITER: Self = Self { mu: JUPITER };

    pub const SATURN: Self = Self { mu: SATURN };

    pub const URANUS: Self = Self { mu: URANUS };

    pub const NEPTUNE: Self = Self { mu: NEPTUNE };

    pub const PLUTO: Self = Self { mu: PLUTO };

    pub fn new(mu: f64) -> Self {
        Self { mu }
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct EGM96Gravity {
}
impl EGM96Gravity {
    pub const EGM96_COEFF: Self = Self {
        //mu: EARTH,
        //earth_re: EARTH_RE,
        //maxdeg: MAX_DEG,
        //c: EGM96_C,
        //s: EGM96_S,
    
    //pub fn new(deg:u8)-> Self{
        //let C: File = file.dataset("C").unwrap().read().unwrap();
        //let c = file. dataset("C").unwrap().read().unwrap();
        /*
        let file = std::fs::File::open(&"egm96.mat").expect("Unable to open file");
        let mat_file = matfile::MatFile::parse(file).unwrap();
        let c = mat_file.find_by_name("C").unwrap().data();
        let s = mat_file.find_by_name("S").unwrap();
        */  
};
}

impl GravityTrait for ConstantGravity {
    fn calculate(&self, _position: Vector3<f64>) -> Vector3<f64> {
        self.value
    }
}

impl GravityTrait for TwoBodyGravity {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64> {
        let position_mag = position.magnitude();
        if position_mag < 0.1 {
            println!("WARNING! division by zero on two body gravity!");
        }
        -position * self.mu / position_mag.powi(3) // point mass two body model
    }
}

impl GravityTrait for EGM96Gravity {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64> {
        let px = position.x;
        let py = position.y;
        let pz = position.z;
        let position_mag = position.magnitude();
        let lat: f64 = (pz / position_mag).asin(); // latitude (rad)
        let lambda: f64 = py.atan2(px); // longitude (rad)
        let n = MAX_DEG as usize; //self.deg as usize;
        let sm_lambda: Vec<f64> = lambda_coeff(n, lambda).0;
        let cm_lambda: Vec<f64> = lambda_coeff(n, lambda).1;
        let p_coeff: Vec<Vec<f64>> = legendre_func(lat, n).0;
        let scale_factor: Vec<Vec<f64>> = legendre_func(lat, n).1;
        let g_ecef: Vector3<f64> = loc_gravity_ecef(
            position,
            n,
            p_coeff,
            EGM96_C, //self.c,
            EGM96_S, //self.s,
            sm_lambda,
            cm_lambda,
            position_mag,
            scale_factor,
        );        
        g_ecef
    }
}

fn legendre_func(phi: f64, maxdeg: usize) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
    let mut p = vec![vec![0.0; maxdeg + 3]; maxdeg + 3];
    let mut scale_factor = vec![vec![0.0; maxdeg + 3]; maxdeg + 3];
    let mut cphi: f64 = (PI / 2.0 - phi).cos();
    let mut sphi: f64 = (PI / 2.0 - phi).sin();
    // Force numerically zero values to be exactly zero
    if cphi.abs() <= f64::EPSILON {
        cphi = 0.0;
    }
    if sphi.abs() <= f64::EPSILON {
        sphi = 0.0;
    }

    // Seeds for recursion formula
    p[0][0] = 1.0; // n = 0, m = 0
    p[1][0] = (3f64).sqrt() * cphi; // n = 1, m = 0
    scale_factor[0][0] = 0.0;
    scale_factor[1][0] = 1.0;
    p[1][1] = (3f64).sqrt() * sphi; // n = 1, m = 1
    scale_factor[1][1] = 0.0;

    for n in 2..=maxdeg + 2 {
//            for n in 2..=maxdeg + 1 {
//              let k = n + 1;
        let k = n;
        for m in 0..=n {
            //let p_index = m + 1;
            let p_index = m;
            // Compute normalized associated legendre polynomials, P, via recursion relations
            // Scale Factor needed for normalization of dUdphi partial derivative
            if n == m {
                p[k][k] = ((2.0 * n as f64 + 1.0).sqrt() / (2.0 * n as f64).sqrt())
                    * sphi
                    * p[k - 1][k - 1];
                scale_factor[k][k] = 0.0;
            } else if m == 0 {
                p[k][p_index] = ((2.0 * n as f64 + 1.0).sqrt() / n as f64) *
                    ((2.0 * n as f64 - 1.0).sqrt() * cphi * p[k - 1][p_index]
                        - (n as f64 - 1.0) / ((2.0 * n as f64 - 3.0).sqrt())
                            * p[k - 2][p_index]);
                scale_factor[k][p_index] = ((n as f64 + 1.0) * n as f64 / 2.0).sqrt();
            } else {
                p[k][p_index] = ((2.0 * n as f64 + 1.0).sqrt()
                    / ((n + m) as f64).sqrt()
                    / ((n - m) as f64).sqrt())
                    * ((2.0 * n as f64 - 1.0).sqrt() * cphi * p[k - 1][p_index]
                        - ((n + m - 1) as f64).sqrt() * ((n - m - 1) as f64).sqrt()
                            / ((2.0 * n as f64 - 3.0).sqrt())
                            * p[k - 2][p_index]);
                scale_factor[k][p_index] = ((n + m + 1) as f64 * (n - m) as f64).sqrt();
            }
        }
    }
    (p, scale_factor)
}

fn loc_gravity_ecef(
    pos: Vector3<f64>, // position
    maxdeg: usize,
    p: Vec<Vec<f64>>,
    c: Matrix5<f64>, 
    s: Matrix5<f64>,
    smlambda: Vec<f64>,
    cmlambda: Vec<f64>,
    r: f64,  // position_mag
    scale_factor: Vec<Vec<f64>>,
) -> Vector3<f64> {
    let re = EARTH_RE;
    let mu = EARTH;
    let r_ratio: f64 = re / r;
    let mut r_ratio_n = r_ratio;

    // Initialize summation of gravity in radial coordinates
    let mut du_dr_sum_n = 1.0;
    let mut du_dphi_sum_n = 0.0;
    let mut du_dlambda_sum_n = 0.0;

    // Summation of gravity in radial coordinates
    for n in 2..=maxdeg {

        //let k = n + 1;
        let k = n;
        r_ratio_n *= r_ratio;
        let mut du_dr_sum_m = 0.0;
        let mut du_dphi_sum_m = 0.0;
        let mut du_dlambda_sum_m = 0.0;

        for m in 0..=n {
            //let j = m + 1;
            let j = m;
            du_dr_sum_m += p[k][j] * (c[(k, j)] * cmlambda[j] + s[(k, j)] * smlambda[j]);
            du_dphi_sum_m += (p[k][j + 1] * scale_factor[k][j]
                - pos[2] / (pos[0].powi(2) + pos[1].powi(2)).sqrt()
                    * m as f64
                    * p[k][j])
                    * (c[(k, j)] * cmlambda[j] + s[(k, j)] * smlambda[j]);
            du_dlambda_sum_m +=
                m as f64 * p[k][j] * (s[(k, j)] * cmlambda[j] - c[(k, j)] * smlambda[j]);                    
            }
        du_dr_sum_n += du_dr_sum_m * r_ratio_n * (k as f64 + 1.0);
        du_dphi_sum_n += du_dphi_sum_m * r_ratio_n;
        du_dlambda_sum_n += du_dlambda_sum_m * r_ratio_n;
    }

    // Gravity in spherical coordinates
    let du_dr = -mu / r.powi(2) * du_dr_sum_n;
    let du_dphi = mu / r * du_dphi_sum_n;
    let du_dlambda = mu / r * du_dlambda_sum_n;

    // Gravity in ECEF coordinates
    let mut gx = (1.0 / r * du_dr
        - pos[2] / (r.powi(2) * (pos[0].powi(2) + pos[1].powi(2)).sqrt()) * du_dphi)
        * pos[0]
        - (du_dlambda / (pos[0].powi(2) + pos[1].powi(2))) * pos[1];
    let mut gy = (1.0 / r * du_dr
        - pos[2] / (r.powi(2) * (pos[0].powi(2) + pos[1].powi(2)).sqrt()) * du_dphi)
        * pos[1]
        + (du_dlambda / (pos[0].powi(2) + pos[1].powi(2))) * pos[0];
    let mut gz = 1.0 / r * du_dr * pos[2]
        + ((pos[0].powi(2) + pos[1].powi(2)).sqrt() / r.powi(2)) * du_dphi;

    // Special case for poles
    let at_pole = (pos[2].atan2((pos[0].powi(2) + pos[1].powi(2)).sqrt())).abs()
        == std::f64::consts::PI / 2.0;
     if at_pole.any() {
        gx = 0.0;
        gy = 0.0;
        gz = 1.0 / r * du_dr * pos[2];
    }
    let g = Vector3::new(gx, gy, gz);
    g
}

fn lambda_coeff(max_deg: usize, lambda: f64) -> (Vec<f64>, Vec<f64>) {
    //let num_rows = pos_ecef.len();
    let mut sm_lambda = vec![0.0; max_deg + 1];
    let mut cm_lambda = vec![0.0; max_deg + 1];

    let slambda = lambda.sin();
    let clambda = lambda.cos();
    sm_lambda[0] = 0.0;
    cm_lambda[0] = 1.0;
    sm_lambda[1] = slambda;
    cm_lambda[1] = clambda;
    for m in 2..=max_deg {
        sm_lambda[m] = 2.0 * clambda * sm_lambda[m - 1] - sm_lambda[m - 2];
        cm_lambda[m] = 2.0 * clambda * cm_lambda[m - 1] - cm_lambda[m - 2];
    }

    (sm_lambda, cm_lambda)
}

impl GravityTrait for Gravity {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64> {
        match self {
            Gravity::Constant(gravity) => gravity.calculate(position),
            Gravity::TwoBody(gravity) => gravity.calculate(position),
            Gravity::EGM96(gravity) => gravity.calculate(position),
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Unittest for EGM96
    #[test]
    fn grav_egm96() {
        let grav = EGM96Gravity{};
        let pos_ecef = Vector3::new(
            -821562.9892,
            -906648.2064,
            -6954665.433);
        
        let g_rust = grav.calculate(pos_ecef);
        let g_pace_model = Vector3::new(
            0.925349412278864,
            1.021116998885220,
            7.853405068561626);

        assert_relative_eq!(g_rust.x, g_pace_model.x, max_relative = 1e-3);
        assert_relative_eq!(g_rust.y, g_pace_model.y, max_relative = 1e-3);
        assert_relative_eq!(g_rust.z, g_pace_model.z, max_relative = 1e-3);
        }
} 
