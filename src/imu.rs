use std::{fs::File, path::PathBuf};

use anyhow::{ensure, Result};
use nalgebra as na;
use yaml_rust::Yaml;

use crate::{load_yaml, Timestamp};

const DATA_CSV: &str = "data.csv";
const SENSOR_YAML: &str = "sensor.yaml";

#[derive(Debug)]
pub struct ImuData {
    path: PathBuf,
}

impl ImuData {
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

    /// Return gyroscope "white noise" (rad/s/√Hz)
    pub fn gyro_noise_density(&self) -> Result<f64> {
        Ok(self.read_sensor_yaml()?[0]["gyroscope_noise_density"]
            .as_f64()
            .unwrap())
    }

    /// Return gyroscope "random walk" (rad/s^2/√Hz)
    pub fn gyro_random_walk(&self) -> Result<f64> {
        Ok(self.read_sensor_yaml()?[0]["gyroscope_random_walk"]
            .as_f64()
            .unwrap())
    }

    /// Return accelerometer "white noise" (m/s^2/√Hz)
    pub fn accel_noise_density(&self) -> Result<f64> {
        Ok(self.read_sensor_yaml()?[0]["accelerometer_noise_density"]
            .as_f64()
            .unwrap())
    }

    /// Return accelerometer "random walk" (m/s^3/√Hz)
    pub fn accel_random_walk(&self) -> Result<f64> {
        Ok(self.read_sensor_yaml()?[0]["accelerometer_random_walk"]
            .as_f64()
            .unwrap())
    }

    pub fn records(&self) -> Result<ImuIterator> {
        let f = File::open(self.path.join(DATA_CSV))?;

        Ok(ImuIterator {
            reader: csv::Reader::from_reader(f).into_records(),
        })
    }
}

#[derive(Debug, Clone)]
pub struct ImuRecord {
    pub timestamp: Timestamp,
    /// angular velocity [rad/s]
    pub gyro: na::Vector3<f64>,
    /// linear acceleration [m/s^2]
    pub accel: na::Vector3<f64>,
}

pub struct ImuIterator {
    reader: csv::StringRecordsIntoIter<File>,
}

impl Iterator for ImuIterator {
    type Item = Result<ImuRecord>;

    fn next(&mut self) -> Option<Self::Item> {
        let row = self.reader.next()?;

        let parse = || -> Self::Item {
            let row = row?;
            Ok(ImuRecord {
                timestamp: row[0].parse::<u64>()?.into(),
                gyro: na::Vector3::new(
                    row[1].parse::<f64>()?,
                    row[2].parse::<f64>()?,
                    row[3].parse::<f64>()?,
                ),
                accel: na::Vector3::new(
                    row[4].parse::<f64>()?,
                    row[5].parse::<f64>()?,
                    row[6].parse::<f64>()?,
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
        let data = EuRoC::new("test_data")?.imu()?;
        assert_eq!(data.extrinsics()?, na::Matrix4::identity());

        Ok(())
    }

    #[test]
    fn gyro_noise_density() -> Result<()> {
        let data = EuRoC::new("test_data")?.imu()?;
        assert_eq!(data.gyro_noise_density()?, 1.6968e-04);

        Ok(())
    }

    #[test]
    fn gyro_random_walk() -> Result<()> {
        let data = EuRoC::new("test_data")?.imu()?;
        assert_eq!(data.gyro_random_walk()?, 1.9393e-05);

        Ok(())
    }

    #[test]
    fn accel_noise_density() -> Result<()> {
        let data = EuRoC::new("test_data")?.imu()?;
        assert_eq!(data.accel_noise_density()?, 2.0000e-3);

        Ok(())
    }

    #[test]
    fn accel_random_walk() -> Result<()> {
        let data = EuRoC::new("test_data")?.imu()?;
        assert_eq!(data.accel_random_walk()?, 3.0000e-3);

        Ok(())
    }

    #[test]
    fn records() -> Result<()> {
        let data = EuRoC::new("test_data")?.imu()?;
        let record = data.records()?.skip(2).next().unwrap()?;

        assert_eq!(record.timestamp, 1403636579768555520.into());
        assert_eq!(
            record.gyro,
            na::Vector3::new(
                -0.098436569812480182,
                0.12775810124598494,
                0.037699111843077518
            )
        );
        assert_eq!(
            record.accel,
            na::Vector3::new(
                7.8861810416666662,
                -0.42495483333333334,
                -2.4353180833333332
            )
        );
        assert_eq!(data.records()?.count(), 5);

        Ok(())
    }
}
