use bincode;
use byteorder::{LittleEndian, ReadBytesExt};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs::File;
use std::io::{self, Error as IoError, Read, Seek, SeekFrom, Write}; // To read values in big-endian format

#[derive(Debug, Deserialize, Serialize)]
pub struct Spice {
    file_record: FileRecord,
    segments: HashMap<Bodies, Segment>,
}

impl Spice {
    pub fn new(file: &str) -> Result<Self, IoError> {
        let mut segments = HashMap::new();
        let mut file = File::open(file)?;
        let mut header = [0u8; 1024];
        file.read_exact(&mut header)?;
        let file_record = FileRecord::from_bytes(&header).unwrap();

        // Seek to the start of the 1st record
        let first_summary_location = (1024 * (file_record.fward - 1)) as u64;
        file.seek(SeekFrom::Start(first_summary_location))?;

        let mut record_data = [0u8; 1024];
        file.read_exact(&mut record_data)?;

        // Parse the first 3 double precision control items (each 8 bytes)
        let next_record = f64::from_le_bytes(record_data[0..8].try_into().unwrap()) as u32;
        let prev_record = f64::from_le_bytes(record_data[8..16].try_into().unwrap()) as u32;
        let num_segments = f64::from_le_bytes(record_data[16..24].try_into().unwrap()) as u32;
        let summary_record = SummaryRecord {
            next_record,
            prev_record,
            num_segments,
        };

        // for now we're just using de440s.bps, which only has 1 summary record

        for i in 0..summary_record.num_segments {
            let segment_begin = (24 + i * 8 * file_record.ss) as usize;
            let segment_end = segment_begin + (file_record.ss * 8) as usize;
            // Extract the corresponding byte slice for the summary
            let segment_bytes = &record_data[segment_begin..segment_end];
            let segment = Segment::from_bytes(segment_bytes).unwrap();
            if let Some(mut segment) = segment {
                // get the element data from the daf file
                segment.get_data_from_file(&mut file).unwrap();
                segments.insert(segment.target, segment);
            }
        }
        Ok(Self {
            file_record,
            segments,
        })
    }

    pub fn get_position(&mut self, t: f64, body: Bodies) -> Result<[f64; 3], SpiceErrors> {
        if let Some(segment) = self.segments.get_mut(&body) {
            // earth and moon are in reference to the earth barycenter, adjust to solar system barycenter to match others
            match segment.target {
                Bodies::Earth  | Bodies::Moon => {
                    let body_pos = segment.evaluate(t)?;
                    let barycenter = self.segments.get_mut(&Bodies::EarthBarycenter).unwrap();
                    let barycenter_pos = barycenter.evaluate(t)?;
                    Ok([
                        body_pos[0] + barycenter_pos[0],
                        body_pos[1] + barycenter_pos[1],
                        body_pos[2] + barycenter_pos[2],
                    ])
                }
                _ => Ok(segment.evaluate(t)?)
            }
            
        } else {
            return Err(SpiceErrors::BodyNotFound);
        }
    }

    pub fn save_spice_data(&self) -> std::io::Result<()> {
        let encoded: Vec<u8> = bincode::serialize(self).unwrap(); // Serialize the struct
        let mut file = File::create("resources/spice.dat")?;
        file.write_all(&encoded)?; // Write the serialized data to a file
        Ok(())
    }
}

#[derive(Debug)]
pub enum SpiceErrors {
    BodyNotFound,
    RecordNotFound,
}

/// Note that for most of these, we use the Barycenter since it's all that's available in the de440s.bsp
/// The only ones that are not barycenter are the earth, moon, and sun. Change this in the future if needed
#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy, Serialize, Deserialize)]
pub enum Bodies {
    SolarSystemBarycenter,
    Mercury,
    MercuryBarycenter,
    Venus,
    VenusBarycenter,
    Earth,
    EarthBarycenter,
    Moon,
    Mars,
    MarsBarycenter,
    Jupiter,
    JupiterBarycenter,
    Saturn,
    SaturnBarycenter,
    Neptune,
    NeptuneBarycenter,
    Uranus,
    UranusBarycenter,
    Pluto,
    PlutoBarycenter,
    Sun,
}

impl Bodies {
    fn from_id(id: i32) -> Option<Self> {
        match id {
            0 => Some(Bodies::SolarSystemBarycenter),
            1 => Some(Bodies::MercuryBarycenter),
            199 => Some(Bodies::Mercury),
            2 => Some(Bodies::VenusBarycenter),
            299 => Some(Bodies::Venus),
            3 => Some(Bodies::EarthBarycenter),
            399 => Some(Bodies::Earth),
            301 => Some(Bodies::Moon),
            4 => Some(Bodies::MarsBarycenter),
            499 => Some(Bodies::Mars),
            5 => Some(Bodies::JupiterBarycenter),
            599 => Some(Bodies::Jupiter),
            6 => Some(Bodies::SaturnBarycenter),
            699 => Some(Bodies::Saturn),
            7 => Some(Bodies::UranusBarycenter),
            799 => Some(Bodies::Uranus),
            8 => Some(Bodies::NeptuneBarycenter),
            899 => Some(Bodies::Neptune),
            9 => Some(Bodies::PlutoBarycenter),
            999 => Some(Bodies::Pluto),
            10 => Some(Bodies::Sun),
            _ => {
                dbg!(id);
                None
            }
        }
    }

    pub fn from_string(str: &String) -> Option<Self> {
        match str.to_lowercase().as_str() {
            "solar system barycenter" => Some(Bodies::SolarSystemBarycenter),
            "mercury barycenter" => Some(Bodies::MercuryBarycenter),
            "mercury" => Some(Bodies::Mercury),
            "venus barycenter" => Some(Bodies::VenusBarycenter),
            "venus" => Some(Bodies::Venus),
            "earth barycenter" => Some(Bodies::EarthBarycenter),
            "earth" => Some(Bodies::Earth),
            "moon" => Some(Bodies::Moon),
            "mars barycenter" => Some(Bodies::MarsBarycenter),
            "mars" => Some(Bodies::Mars),
            "jupiter barycenter" => Some(Bodies::JupiterBarycenter),
            "jupiter" => Some(Bodies::Jupiter),
            "saturn barycenter" => Some(Bodies::SaturnBarycenter),
            "saturn" => Some(Bodies::Saturn),
            "uranus barycenter" => Some(Bodies::UranusBarycenter),
            "uranus" => Some(Bodies::Uranus),
            "neptune barycenter" => Some(Bodies::NeptuneBarycenter),
            "neptune" => Some(Bodies::Neptune),
            "pluto barycenter" => Some(Bodies::PlutoBarycenter),
            "pluto" => Some(Bodies::Pluto),
            "sun" => Some(Bodies::Sun),
            _ => {
                println!("Error: body '{}' not found", str);
                println!("Note: you may need to use quotes like 'earth barycenter'");
                println!("Options:");
                println!("solar system barycenter");
                println!("mercury barycenter");
                println!("mercury");
                println!("venus barycenter");
                println!("venus");
                println!("earth barycenter");
                println!("earth");
                println!("moon");
                println!("mars barycenter");
                println!("mars");
                println!("jupiter barycenter");
                println!("jupiter");
                println!("saturn barycenter");
                println!("saturn");
                println!("uranus barycenter");
                println!("uranus");
                println!("neptune barycenter");
                println!("neptune");
                println!("pluto barycenter");
                println!("pluto");
                println!("sun");
                None
            }
        }
    }
}

/// Lets just stick with J2000 for now
#[derive(Debug, Deserialize, Serialize)]
enum Frames {
    J2000,
}

impl Frames {
    fn from_id(id: i32) -> Option<Self> {
        match id {
            1 => Some(Frames::J2000),
            _ => None,
        }
    }
}

#[derive(Debug, Deserialize, Serialize)]
enum DataTypes {
    ModifiedDifferenceArrays,
    ChebyshevPolyPosition,
    ChebyshevPolyPositionVelocity,
    DiscretePositionVelocity,
    TabularPositionVelocity,
    HighPrecisionChevyshevPoly,
    EquinocatalOrbitalElements,
}

impl DataTypes {
    fn from_id(id: i32) -> Option<Self> {
        match id {
            1 => Some(DataTypes::ModifiedDifferenceArrays),
            2 => Some(DataTypes::ChebyshevPolyPosition),
            3 => Some(DataTypes::ChebyshevPolyPositionVelocity),
            5 => Some(DataTypes::DiscretePositionVelocity),
            9 => Some(DataTypes::TabularPositionVelocity),
            13 => Some(DataTypes::HighPrecisionChevyshevPoly),
            15 => Some(DataTypes::EquinocatalOrbitalElements),
            _ => None,
        }
    }
}

#[derive(Debug, Deserialize, Serialize)]
struct ChebyshevRecord {
    mid: f64,              // The midpoint of the time interval
    radius: f64,           // The radius of the time interval
    pos_x_coefs: Vec<f64>, // Coefficients for X axis
    pos_y_coefs: Vec<f64>, // Coefficients for Y axis
    pos_z_coefs: Vec<f64>, // Coefficients for Z axis
}

#[derive(Debug, Deserialize, Serialize)]
struct SegmentMeta {
    init: f64,
    init_len: f64,
    rsize: usize,
    n_records: usize,
}

#[derive(Debug, Deserialize, Serialize)]
struct FileRecord {
    locidw: String, // An identification word (DAF/xxxx)
    nd: u32,        // The number of double precision components in each array summary
    ni: u32,        // The number of integer components in each array summary
    nc: u32,        // Alpanumeric information, usually names
    ss: u32,        // size of a single summary
    ns: u32,        // number of summaries that can fit into a summary record
    locifn: String, // Internal name or description of the array file
    fward: u32,     // Record number of the initial summary record
    bward: u32,     // Record number of the final summary record
    free: u32,      // First free address in the file
    locfmt: String, // Indicates numeric binary format ("LTL-IEEE" or "BIG-IEEE")
}

impl FileRecord {
    fn from_bytes(bytes: &[u8]) -> io::Result<Self> {
        // Ensure the byte slice is the correct size
        assert!(
            bytes.len() == 1024,
            "DAF file record must be exactly 1024 bytes long."
        );
        let mut cursor = std::io::Cursor::new(bytes);

        // Read the LOCIDW (8 bytes as string)
        let mut locidw_bytes = [0u8; 8];
        cursor.read_exact(&mut locidw_bytes)?;
        let locidw = String::from_utf8_lossy(&locidw_bytes).to_string();

        // Read ND (4 bytes as u32)
        let nd = cursor.read_u32::<LittleEndian>()?;

        // Read NI (4 bytes as u32)
        let ni = cursor.read_u32::<LittleEndian>()?;

        let nc = 8 * (nd + (ni + 1) / 2);

        let ss = nd + (ni + 1) / 2;
        let ns = 125 / ss;

        // Read LOCIFN (60 bytes as string)
        let mut locifn_bytes = [0u8; 60];
        cursor.read_exact(&mut locifn_bytes)?;
        let locifn = String::from_utf8_lossy(&locifn_bytes).to_string();

        // Read FWARD (4 bytes as u32)
        let fward = cursor.read_u32::<LittleEndian>()?;

        // Read BWARD (4 bytes as u32)
        let bward = cursor.read_u32::<LittleEndian>()?;

        // Read FREE (4 bytes as u32)
        let free = cursor.read_u32::<LittleEndian>()?;

        // Read LOCFMT (8 bytes as string)
        let mut locfmt_bytes = [0u8; 8];
        cursor.read_exact(&mut locfmt_bytes)?;
        let locfmt = String::from_utf8_lossy(&locfmt_bytes).to_string();

        // Return the parsed DAF file record
        Ok(FileRecord {
            locidw,
            nd,
            ni,
            nc,
            ss,
            ns,
            locifn,
            fward,
            bward,
            free,
            locfmt,
        })
    }
}

#[derive(Debug, Deserialize, Serialize)]
struct SummaryRecord {
    next_record: u32,  // Record number of the next summary record
    prev_record: u32,  // Record number of the previous summary record
    num_segments: u32, // Number of summaries in this record
}

#[derive(Debug, Deserialize, Serialize)]
struct Segment {
    start_time: f64,
    end_time: f64,
    target: Bodies,
    center: Bodies,
    frame: Frames,
    data_type: DataTypes,
    initial_address: i32,
    final_address: i32,
    meta: Option<SegmentMeta>,
    records: Vec<ChebyshevRecord>,
    current_record: usize, //searches occur from the current_record
}

impl Segment {
    fn get_record_from_t(&mut self, t: f64) -> Result<&ChebyshevRecord, SpiceErrors> {
        if let Some((i, record)) =
            self.records[self.current_record..]
                .iter()
                .enumerate()
                .find(|(_, record)| {
                    (t > (record.mid - record.radius)) && (t < (record.mid + record.radius))
                })
        {
            self.current_record = i;
            Ok(record)
        } else {
            Err(SpiceErrors::RecordNotFound)
        }
    }

    /// t is the julian date
    fn evaluate(&mut self, t: f64) -> Result<[f64; 3], SpiceErrors> {
        // find the record with the right time
        let record = self.get_record_from_t(t)?;
        //let t_begin = record.mid - record.radius;
        //let t_end = record.mid + record.radius;

        // apparently this reduces to the equation below, and seems to work, thanks chatgpt
        //let tau = 2.0 * (t - t_begin)/(t_end-t_begin) - 1.0;
        let tau = (t - record.mid) / record.radius;

        let x = evaluate_chebyshev(&record.pos_x_coefs, tau);
        let y = evaluate_chebyshev(&record.pos_y_coefs, tau);
        let z = evaluate_chebyshev(&record.pos_z_coefs, tau);

        Ok([x, y, z])
    }

    fn from_bytes(bytes: &[u8]) -> Result<Option<Self>, std::io::Error> {
        let mut cursor = std::io::Cursor::new(bytes);

        // Read the 2 double precision words (f64 values)
        let start_time = cursor.read_f64::<LittleEndian>()?;
        let end_time = cursor.read_f64::<LittleEndian>()?;

        // Read the packed integers from 3 f64 values
        let target_and_center = cursor.read_f64::<LittleEndian>()?;
        let frame_and_data_type = cursor.read_f64::<LittleEndian>()?;
        let initial_and_final = cursor.read_f64::<LittleEndian>()?;

        // Convert the f64 values to bytes
        let target_and_center_bytes = target_and_center.to_le_bytes();
        let frame_and_data_type_bytes = frame_and_data_type.to_le_bytes();
        let initial_and_final_bytes = initial_and_final.to_le_bytes();

        // Extract the integers from the packed f64 byte representations
        let target_id = i32::from_le_bytes([
            target_and_center_bytes[0],
            target_and_center_bytes[1],
            target_and_center_bytes[2],
            target_and_center_bytes[3],
        ]);
        let center_id = i32::from_le_bytes([
            target_and_center_bytes[4],
            target_and_center_bytes[5],
            target_and_center_bytes[6],
            target_and_center_bytes[7],
        ]);

        let frame_id = i32::from_le_bytes([
            frame_and_data_type_bytes[0],
            frame_and_data_type_bytes[1],
            frame_and_data_type_bytes[2],
            frame_and_data_type_bytes[3],
        ]);
        let data_type_id = i32::from_le_bytes([
            frame_and_data_type_bytes[4],
            frame_and_data_type_bytes[5],
            frame_and_data_type_bytes[6],
            frame_and_data_type_bytes[7],
        ]);

        let initial_address = i32::from_le_bytes([
            initial_and_final_bytes[0],
            initial_and_final_bytes[1],
            initial_and_final_bytes[2],
            initial_and_final_bytes[3],
        ]);
        let final_address = i32::from_le_bytes([
            initial_and_final_bytes[4],
            initial_and_final_bytes[5],
            initial_and_final_bytes[6],
            initial_and_final_bytes[7],
        ]);

        if let (Some(target), Some(center), Some(frame), Some(data_type)) = (
            Bodies::from_id(target_id),
            Bodies::from_id(center_id),
            Frames::from_id(frame_id),
            DataTypes::from_id(data_type_id),
        ) {
            Ok(Some(Self {
                start_time,
                end_time,
                target,
                center,
                frame,
                data_type,
                initial_address,
                final_address,
                meta: None,
                records: Vec::new(),
                current_record: 0,
            }))
        } else {
            dbg!(target_id);
            dbg!(center_id);
            dbg!(frame_id);
            dbg!(data_type_id);
            Ok(None)
        }
    }

    fn get_data_from_file(&mut self, file: &mut File) -> Result<(), std::io::Error> {
        // get the meta data at the end of the file first
        // see https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/C/req/spk.html#Segments--The%20Fundamental%20SPK%20Building%20Blocks

        // Convert final_address to byte offset
        let final_byte_offset = (self.final_address as u64) * 8;

        // The metadata is 32 bytes before the final address
        let metadata_offset = final_byte_offset - 32;

        // Seek to the metadata location
        file.seek(SeekFrom::Start(metadata_offset))?;

        // Read the metadata (INIT, INTLEN, RSIZE, N)
        let mut buffer = [0u8; 32];
        file.read_exact(&mut buffer)?;

        // Interpret the values as f64
        let init = f64::from_le_bytes(buffer[0..8].try_into().unwrap());
        let init_len = f64::from_le_bytes(buffer[8..16].try_into().unwrap());
        let rsize = f64::from_le_bytes(buffer[16..24].try_into().unwrap()) as usize;
        let n_records = f64::from_le_bytes(buffer[24..32].try_into().unwrap()) as usize;
        let meta = SegmentMeta {
            init,
            init_len,
            rsize,
            n_records,
        };
        self.meta = Some(meta);

        let n_coeffs = (rsize - 2) / 3;

        // now go back to beginning and get the records
        file.seek(SeekFrom::Start(((self.initial_address - 1) * 8) as u64))?;
        // Read the metadata (INIT, INTLEN, RSIZE, N)
        for _ in 0..n_records {
            // Read the midpoint (mid) and radius (radius) of the time interval
            let mid = file.read_f64::<LittleEndian>()?;
            let radius = file.read_f64::<LittleEndian>()?;

            // Read the X, Y, and Z coefficients
            let mut pos_x_coefs = Vec::new();
            let mut pos_y_coefs = Vec::new();
            let mut pos_z_coefs = Vec::new();

            for _ in 0..n_coeffs {
                pos_x_coefs.push(file.read_f64::<LittleEndian>()?);
            }
            for _ in 0..n_coeffs {
                pos_y_coefs.push(file.read_f64::<LittleEndian>()?);
            }
            for _ in 0..n_coeffs {
                pos_z_coefs.push(file.read_f64::<LittleEndian>()?);
            }

            // Create the ChebyshevRecord and push it to the vector
            self.records.push(ChebyshevRecord {
                mid,
                radius,
                pos_x_coefs,
                pos_y_coefs,
                pos_z_coefs,
            });
        }

        Ok(())
    }
}

fn evaluate_chebyshev(coeffs: &[f64], tau: f64) -> f64 {
    let n = coeffs.len();
    if n == 0 {
        return 0.0;
    } else if n == 1 {
        return coeffs[0];
    }

    let mut t_0 = 1.0;
    let mut t_1 = tau;
    let mut result = coeffs[0] + coeffs[1] * tau;

    for k in 2..n {
        let t_k = 2.0 * tau * t_1 - t_0;
        result += coeffs[k] * t_k;
        t_0 = t_1;
        t_1 = t_k;
    }

    result
}
