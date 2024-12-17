use super::{DataTypes, Frames, SpiceBodies, SpiceErrors};
use crate::SpiceFileTypes;
use byteorder::{LittleEndian, ReadBytesExt};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::HashMap;
use std::io::{self, Read};

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct DafData {
    pub file_record: FileRecord,
    pub segments: HashMap<SpiceBodies, Vec<Segment>>,
    pub current_segment: usize,
}

impl DafData {
    pub fn new(data: &[u8], file_type: SpiceFileTypes) -> Result<DafData, SpiceErrors> {
        let mut header = [0u8; 1024];
        header.copy_from_slice(&data[0..1024]);
        let file_record = FileRecord::from_bytes(&header).unwrap();

        // Initialize HashMap to store segments
        let mut segments = HashMap::new();

        // Initialize the first summary record location from the file header
        let mut current_record_location = (1024 * (file_record.fward - 1)) as usize;

        // Continue processing while there are more summary records
        while current_record_location != 0 {
            // Read the current summary record data
            let record_data = &data[current_record_location..current_record_location + 1024];

            // Parse the first 3 double precision control items (each 8 bytes)
            let next_record = f64::from_le_bytes(record_data[0..8].try_into().unwrap()) as u32;
            let _prev_record = f64::from_le_bytes(record_data[8..16].try_into().unwrap()) as u32;
            let num_segments = f64::from_le_bytes(record_data[16..24].try_into().unwrap()) as u32;

            let summary_record = SummaryRecord {
                next_record,
                prev_record: _prev_record,
                num_segments,
            };
            //dbg!(&summary_record);
            // Loop through each segment in the current summary record
            for i in 0..summary_record.num_segments {
                let segment_begin = (24 + i * 8 * file_record.ss) as usize;
                let segment_end = segment_begin + (file_record.ss * 8) as usize;

                // Extract the corresponding byte slice for the segment summary
                let segment_bytes = &record_data[segment_begin..segment_end];

                let segment = match file_type {
                    SpiceFileTypes::Spk => Segment::from_spk_bytes(segment_bytes).unwrap(),
                    SpiceFileTypes::Pck => Segment::from_pck_bytes(segment_bytes).unwrap(),
                };

                if let Some(mut segment) = segment {
                    // Get metadata and Chebyshev data for the segment
                    segment.get_record_metadata(data)?;
                    segment.get_chebyshev_data(data)?;
                    let target = segment.target;
                    segments
                        .entry(segment.target)
                        .or_insert_with(Vec::new) // If the key doesn't exist, insert a new empty Vec
                        .push(segment);
                    // ensure the Vec<Segment> is always sorted by segment.start_time
                    segments
                        .get_mut(&target)
                        .unwrap()
                        .sort_by(|a, b| a.start_time.partial_cmp(&b.start_time).unwrap());
                }
            }

            // Move to the next summary record
            if next_record != 0 {
                current_record_location = (1024 * (next_record - 1)) as usize;
            } else {
                break; // Exit the loop when there are no more records
            }
        }

        Ok(Self {
            file_record,
            segments,
            current_segment: 0,
        })
    }

    pub fn get_segment(
        &mut self,
        body: &SpiceBodies,
        t: f64,
    ) -> Result<Option<&mut Segment>, SpiceErrors> {
        if let Some(segments) = self.segments.get_mut(body) {
            Ok(segments
                .binary_search_by(|segment| {
                    if t < segment.start_time {
                        Ordering::Greater
                    } else if t > segment.end_time {
                        Ordering::Less
                    } else {
                        Ordering::Equal
                    }
                })
                .ok()
                .map(move |index| &mut segments[index]))
        } else {
            Err(SpiceErrors::BodyNotFound.into())
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct FileRecord {
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

#[derive(Clone, Debug, Deserialize, Serialize)]
struct SegmentMeta {
    init: f64,
    init_len: f64,
    rsize: usize,
    n_records: usize,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Segment {
    pub start_time: f64,
    pub end_time: f64,
    pub target: SpiceBodies,
    center: Option<SpiceBodies>,
    frame: Frames,
    data_type: DataTypes,
    initial_address: i32,
    final_address: i32,
    meta: Option<SegmentMeta>,
    records: Vec<ChebyshevRecord>, // both spk and pck/eop files are type 2 chebyshev poly position, change this in the future if needed
    current_record: usize,         //searches occur from the current_record
}

impl Segment {
    fn get_record_from_t(&mut self, t: f64) -> Result<&ChebyshevRecord, SpiceErrors> {
        // First, check from current_record to the end, start from current record for speed!
        if let Some((i, record)) =
            self.records[self.current_record..]
                .iter()
                .enumerate()
                .find(|(_, record)| {
                    (t > (record.mid - record.radius)) && (t < (record.mid + record.radius))
                })
        {
            // Update current_record to the found index
            self.current_record += i;
            Ok(record)
        }
        // If not found, search from the beginning up to current_record
        else if let Some((i, record)) = self.records[..self.current_record]
            .iter()
            .enumerate()
            .find(|(_, record)| {
                (t > (record.mid - record.radius)) && (t < (record.mid + record.radius))
            })
        {
            // Update current_record to the new index (which is in the early part)
            self.current_record = i;
            Ok(record)
        } else {
            // If no record is found in both ranges, return an error
            Err(SpiceErrors::RecordNotFound)
        }
    }

    /// t is the ephemeris time (et)
    pub fn evaluate(&mut self, t: f64) -> Result<Vector3<f64>, SpiceErrors> {
        // find the record with the right time
        let record = self.get_record_from_t(t)?;
        //let t_begin = record.mid - record.radius;
        //let t_end = record.mid + record.radius;

        // apparently this reduces to the equation below, and seems to work, thanks chatgpt
        //let tau = 2.0 * (t - t_begin)/(t_end-t_begin) - 1.0;
        let tau = (t - record.mid) / record.radius;

        let result = record.evaluate(tau);

        Ok(result)
    }

    fn from_pck_bytes(bytes: &[u8]) -> Result<Option<Self>, std::io::Error> {
        let mut cursor = std::io::Cursor::new(bytes);

        let start_time = cursor.read_f64::<LittleEndian>()?;
        let end_time = cursor.read_f64::<LittleEndian>()?;
        let target_id = cursor.read_i32::<LittleEndian>()?;
        let frame_id = cursor.read_i32::<LittleEndian>()?;
        let data_type_id = cursor.read_i32::<LittleEndian>()?;
        let initial_address = cursor.read_i32::<LittleEndian>()?;
        let final_address = cursor.read_i32::<LittleEndian>()?;

        if let (Some(target), Some(frame), Some(data_type)) = (
            SpiceBodies::from_pck_id(target_id),
            Frames::from_id(frame_id),
            DataTypes::from_id(data_type_id),
        ) {
            Ok(Some(Self {
                start_time,
                end_time,
                target,
                center: None,
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
            dbg!(frame_id);
            dbg!(data_type_id);
            Ok(None)
        }
    }

    fn from_spk_bytes(bytes: &[u8]) -> Result<Option<Self>, std::io::Error> {
        let mut cursor = std::io::Cursor::new(bytes);

        // Read the 2 double precision words (f64 values)
        let start_time = cursor.read_f64::<LittleEndian>()?;
        let end_time = cursor.read_f64::<LittleEndian>()?;

        let target_id = cursor.read_i32::<LittleEndian>()?;
        let center_id = cursor.read_i32::<LittleEndian>()?;
        let frame_id = cursor.read_i32::<LittleEndian>()?;
        let data_type_id = cursor.read_i32::<LittleEndian>()?;
        let initial_address = cursor.read_i32::<LittleEndian>()?;
        let final_address = cursor.read_i32::<LittleEndian>()?;

        if let (Some(target), Some(center), Some(frame), Some(data_type)) = (
            SpiceBodies::from_spk_id(target_id),
            SpiceBodies::from_spk_id(center_id),
            Frames::from_id(frame_id),
            DataTypes::from_id(data_type_id),
        ) {
            Ok(Some(Self {
                start_time,
                end_time,
                target,
                center: Some(center),
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

    fn get_record_metadata(&mut self, data: &[u8]) -> Result<(), SpiceErrors> {
        // Convert final_address to byte offset
        let final_byte_offset = (self.final_address as usize) * 8;

        // The metadata is 32 bytes before the final address
        let metadata_offset = final_byte_offset - 32;

        // Read the metadata (INIT, INTLEN, RSIZE, N)
        let metadata = &data[metadata_offset..metadata_offset + 32];

        // Interpret the values as f64
        let init = f64::from_le_bytes(metadata[0..8].try_into().unwrap());
        let init_len = f64::from_le_bytes(metadata[8..16].try_into().unwrap());
        let rsize = f64::from_le_bytes(metadata[16..24].try_into().unwrap()) as usize;
        let n_records = f64::from_le_bytes(metadata[24..32].try_into().unwrap()) as usize;

        let meta = SegmentMeta {
            init,
            init_len,
            rsize,
            n_records,
        };
        self.meta = Some(meta);
        Ok(())
    }

    fn get_chebyshev_data(&mut self, data: &[u8]) -> Result<(), SpiceErrors> {
        if let Some(meta) = &self.meta {
            let n_coeffs = (meta.rsize - 2) / 3;

            // Now go back to beginning and get the records
            let mut offset = (self.initial_address as usize - 1) * 8;

            for _ in 0..meta.n_records {
                // Read the midpoint (mid) and radius (radius) of the time interval
                let mid = f64::from_le_bytes(data[offset..offset + 8].try_into().unwrap());
                offset += 8;
                let radius = f64::from_le_bytes(data[offset..offset + 8].try_into().unwrap());
                offset += 8;

                // Read the X, Y, and Z coefficients
                let mut coefs1 = Vec::new();
                let mut coefs2 = Vec::new();
                let mut coefs3 = Vec::new();

                for _ in 0..n_coeffs {
                    coefs1.push(f64::from_le_bytes(
                        data[offset..offset + 8].try_into().unwrap(),
                    ));
                    offset += 8;
                }
                for _ in 0..n_coeffs {
                    coefs2.push(f64::from_le_bytes(
                        data[offset..offset + 8].try_into().unwrap(),
                    ));
                    offset += 8;
                }
                for _ in 0..n_coeffs {
                    coefs3.push(f64::from_le_bytes(
                        data[offset..offset + 8].try_into().unwrap(),
                    ));
                    offset += 8;
                }

                // Create the ChebyshevRecord and push it to the vector
                self.records.push(ChebyshevRecord {
                    mid,
                    radius,
                    coefs1,
                    coefs2,
                    coefs3,
                });
            }
            Ok(())
        } else {
            Err(SpiceErrors::RecordMetaNotFound)
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct ChebyshevRecord {
    mid: f64,         // The midpoint of the time interval
    radius: f64,      // The radius of the time interval
    coefs1: Vec<f64>, // Coefficients for 1st index (x spk, ra pck)
    coefs2: Vec<f64>, // Coefficients for 2nd index (y spk, dec pck)
    coefs3: Vec<f64>, // Coefficients for 3rd index (z spk, w pck)
}

impl ChebyshevRecord {
    fn evaluate(&self, tau: f64) -> Vector3<f64> {
        Vector3::new(
            evaluate_chebyshev(&self.coefs1, tau),
            evaluate_chebyshev(&self.coefs2, tau),
            evaluate_chebyshev(&self.coefs3, tau),
        )
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
