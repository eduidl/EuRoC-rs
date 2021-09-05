use std::{fs::File, path::PathBuf};

use anyhow::{ensure, Result};
use nalgebra as na;
use yaml_rust::Yaml;

use crate::{load_yaml, Timestamp};

const DATA_CSV: &str = "data.csv";
const SENSOR_YAML: &str = "sensor.yaml";

#[derive(Debug, Clone)]
pub struct GroundTruthData {
    path: PathBuf,
}

impl GroundTruthData {
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

    pub fn records(&self) -> Result<GroundTruthIterator> {
        let f = File::open(self.path.join(DATA_CSV))?;

        Ok(GroundTruthIterator {
            reader: csv::Reader::from_reader(f).into_records(),
        })
    }
}

#[derive(Debug, Clone)]
pub struct GroundTruthRecord {
    pub timestamp: Timestamp,
    /// position (m)
    pub position: na::Vector3<f64>,
    /// quaternion
    pub quaternion: na::Quaternion<f64>,
    /// linear velocity (m/s)
    pub velocity: na::Vector3<f64>,
    /// angular velocity (rad/s)
    pub gyro: na::Vector3<f64>,
    /// linear acceleration (m/s^2)
    pub accel: na::Vector3<f64>,
}

pub struct GroundTruthIterator {
    reader: csv::StringRecordsIntoIter<File>,
}

impl Iterator for GroundTruthIterator {
    type Item = Result<GroundTruthRecord>;

    fn next(&mut self) -> Option<Self::Item> {
        self.reader.next().map(|row| {
            let row = row?;
            Ok(GroundTruthRecord {
                timestamp: row[0].parse::<u64>()?.into(),
                position: na::Vector3::new(
                    row[1].parse::<f64>()?,
                    row[2].parse::<f64>()?,
                    row[3].parse::<f64>()?,
                ),
                quaternion: na::Quaternion::new(
                    row[4].parse::<f64>()?,
                    row[5].parse::<f64>()?,
                    row[6].parse::<f64>()?,
                    row[7].parse::<f64>()?,
                ),
                velocity: na::Vector3::new(
                    row[8].parse::<f64>()?,
                    row[9].parse::<f64>()?,
                    row[10].parse::<f64>()?,
                ),
                gyro: na::Vector3::new(
                    row[11].parse::<f64>()?,
                    row[12].parse::<f64>()?,
                    row[13].parse::<f64>()?,
                ),
                accel: na::Vector3::new(
                    row[14].parse::<f64>()?,
                    row[15].parse::<f64>()?,
                    row[16].parse::<f64>()?,
                ),
            })
        })
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::EuRoC;

    #[test]
    fn extrinsics() -> Result<()> {
        let data = EuRoC::new("test_data")?.ground_truth()?;
        assert_eq!(data.extrinsics()?, na::Matrix4::identity());

        Ok(())
    }

    #[test]
    fn records() -> Result<()> {
        let data = EuRoC::new("test_data")?.ground_truth()?;
        let record = data.records()?.skip(2).next().unwrap()?;

        assert_eq!(record.timestamp, 1403636580848555520.into());
        assert_eq!(
            record.position,
            na::Vector3::new(4.688028, -1.786598, 0.791382)
        );
        assert_eq!(
            record.quaternion,
            na::Quaternion::new(0.535178, -0.152945, -0.826562, -0.083605)
        );
        assert_eq!(
            record.velocity,
            na::Vector3::new(-0.030043, 0.034999, 0.808240)
        );
        assert_eq!(record.gyro, na::Vector3::new(-0.003172, 0.021267, 0.078502));
        assert_eq!(
            record.accel,
            na::Vector3::new(-0.025266, 0.136696, 0.075593)
        );

        assert_eq!(data.records()?.count(), 5);

        Ok(())
    }
}
