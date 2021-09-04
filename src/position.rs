use std::{fs::File, path::PathBuf};

use anyhow::{ensure, Result};
use nalgebra as na;
use yaml_rust::Yaml;

use crate::{load_yaml, Timestamp};

const DATA_CSV: &str = "data.csv";
const SENSOR_YAML: &str = "sensor.yaml";

#[derive(Debug, Clone)]
pub struct PositionData {
    path: PathBuf,
}

impl PositionData {
    pub fn new(path: PathBuf) -> Result<Self> {
        ensure!(path.is_dir());
        ensure!(path.join(DATA_CSV).is_file());
        ensure!(path.join(SENSOR_YAML).is_file());

        Ok(Self { path })
    }

    #[inline]
    fn read_sensor_yaml(&self) -> Result<Vec<Yaml>> {
        load_yaml(self.path.join(SENSOR_YAML))
    }

    /// Return extrinsics wrt. the body-frame.
    pub fn extrinsics(&self) -> Result<na::Matrix4<f64>> {
        let data: Vec<_> = self.read_sensor_yaml()?[0]["T_BS"]["data"]
            .as_vec()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();

        assert!(data.len() == 16);

        Ok(na::Matrix4::from_row_slice(&data))
    }

    pub fn records(&self) -> Result<PositionIterator> {
        let f = File::open(self.path.join(DATA_CSV))?;

        Ok(PositionIterator {
            reader: csv::Reader::from_reader(f).into_records(),
        })
    }
}

#[derive(Debug, Clone)]
pub struct PositionRecord {
    pub timestamp: Timestamp,
    /// position (m)
    pub position: na::Vector3<f64>,
}

pub struct PositionIterator {
    reader: csv::StringRecordsIntoIter<File>,
}

impl Iterator for PositionIterator {
    type Item = Result<PositionRecord>;

    fn next(&mut self) -> Option<Self::Item> {
        let row = match self.reader.next()? {
            Ok(v) => v,
            Err(e) => return Some(Err(e.into())),
        };

        let parse = || {
            Ok(PositionRecord {
                timestamp: row[0].parse::<u64>()?.into(),
                position: na::Vector3::new(
                    row[1].parse::<f64>()?,
                    row[2].parse::<f64>()?,
                    row[3].parse::<f64>()?,
                ),
            })
        };

        Some(parse())
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::EuRoC;

    #[test]
    fn extrinsics() -> Result<()> {
        let data = EuRoC::new("test_data")?.position()?;
        assert_eq!(
            data.extrinsics()?,
            na::Matrix4::from_rows(&[
                na::RowVector4::new(1.0, 0.0, 0.0, 7.48903e-02),
                na::RowVector4::new(0.0, 1.0, 0.0, -1.84772e-02),
                na::RowVector4::new(0.0, 0.0, 1.0, -1.20209e-01),
                na::RowVector4::new(0.0, 0.0, 0.0, 1.0)
            ])
        );

        Ok(())
    }

    #[test]
    fn records() -> Result<()> {
        let data = EuRoC::new("test_data")?.position()?;
        let record = data.records()?.skip(2).next().unwrap()?;

        assert_eq!(record.timestamp, 1403636579022881280.into());
        assert_eq!(
            record.position,
            na::Vector3::new(4.7807530761485442, -1.8131922179613229, 0.87462386853895402)
        );
        assert_eq!(data.records()?.count(), 5);

        Ok(())
    }
}
