use std::{convert::TryInto, fs::File, path::PathBuf};

use anyhow::{ensure, Result};
use image::DynamicImage;
use nalgebra as na;
use yaml_rust::Yaml;

use crate::{load_yaml, Timestamp};

const DATA: &str = "data";
const DATA_CSV: &str = "data.csv";
const SENSOR_YAML: &str = "sensor.yaml";

#[derive(Debug, Clone)]
pub struct CameraRecords {
    path: PathBuf,
}

impl CameraRecords {
    pub fn new(path: PathBuf) -> Result<Self> {
        ensure!(path.is_dir());
        ensure!(path.join(DATA).is_dir());
        ensure!(path.join(DATA_CSV).is_file());
        ensure!(path.join(SENSOR_YAML).is_file());

        Ok(Self { path })
    }

    #[inline]
    fn read_sensor_yaml(&self) -> Result<Vec<Yaml>> {
        load_yaml(self.path.join(SENSOR_YAML))
    }

    /// Return image size (width, height)
    pub fn image_size(&self) -> Result<(u32, u32)> {
        let data: Vec<_> = self.read_sensor_yaml()?[0]["resolution"]
            .as_vec()
            .unwrap()
            .iter()
            .map(|v| v.as_i64().unwrap().try_into().unwrap())
            .collect();

        assert!(data.len() == 2);

        Ok((data[0], data[1]))
    }

    /// Return intrinsics (fu, fv, cu, cv)
    pub fn intrinsics(&self) -> Result<(f64, f64, f64, f64)> {
        let data: Vec<_> = self.read_sensor_yaml()?[0]["intrinsics"]
            .as_vec()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();

        assert!(data.len() == 4);

        Ok((data[0], data[1], data[2], data[3]))
    }

    /// Return camera matrix
    pub fn camera_matrix(&self) -> Result<na::Matrix3<f64>> {
        let (fu, fv, cu, cv) = self.intrinsics()?;

        Ok(na::Matrix3::from_rows(&[
            na::RowVector3::new(fu, 0., cu),
            na::RowVector3::new(0., fv, cv),
            na::RowVector3::new(0.0, 0.0, 1.0),
        ]))
    }

    /// Return Distortion coefficients
    pub fn distrotion_coeff(&self) -> Result<na::Vector4<f64>> {
        let data: Vec<_> = self.read_sensor_yaml()?[0]["distortion_coefficients"]
            .as_vec()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();

        assert!(data.len() == 4);

        Ok(na::Vector4::from_column_slice(&data))
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

    pub fn records(&self) -> Result<ImageIterator> {
        let f = File::open(self.path.join(DATA_CSV))?;

        Ok(ImageIterator {
            path: self.path.join(DATA),
            reader: csv::Reader::from_reader(f).into_records(),
        })
    }
}

#[derive(Debug, Clone)]
pub struct ImageRecord {
    pub timestamp: Timestamp,
    pub image: DynamicImage,
}

pub struct ImageIterator {
    path: PathBuf,
    reader: csv::StringRecordsIntoIter<File>,
}

impl Iterator for ImageIterator {
    type Item = Result<ImageRecord>;

    fn next(&mut self) -> Option<Self::Item> {
        let row = match self.reader.next()? {
            Ok(v) => v,
            Err(e) => return Some(Err(e.into())),
        };

        let parse = || {
            Ok(ImageRecord {
                timestamp: row[0].parse::<u64>()?.into(),
                image: image::open(self.path.join(&row[1]))?,
            })
        };

        Some(parse())
    }
}

#[cfg(test)]
mod test {
    use image::GenericImageView;

    use super::*;
    use crate::EuRoC;

    #[test]
    fn image_size() -> Result<()> {
        let data = EuRoC::new("test_data")?.left_camera()?;
        assert_eq!(data.image_size()?, (752, 480));

        Ok(())
    }

    #[test]
    fn intrinsics() -> Result<()> {
        let data = EuRoC::new("test_data")?.left_camera()?;
        assert_eq!(data.intrinsics()?, (458.654, 457.296, 367.215, 248.375));

        Ok(())
    }

    #[test]
    fn camera_matrix() -> Result<()> {
        let data = EuRoC::new("test_data")?.left_camera()?;
        assert_eq!(
            data.camera_matrix()?,
            na::Matrix3::from_rows(&[
                na::RowVector3::new(458.654, 0., 367.215),
                na::RowVector3::new(0., 457.296, 248.375),
                na::RowVector3::new(0.0, 0.0, 1.0),
            ])
        );

        Ok(())
    }

    #[test]
    fn distrotion_coeff() -> Result<()> {
        let data = EuRoC::new("test_data")?.left_camera()?;
        assert_eq!(
            data.distrotion_coeff()?,
            na::Vector4::new(-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05)
        );

        Ok(())
    }

    #[test]
    fn extrinsics() -> Result<()> {
        let data = EuRoC::new("test_data")?.left_camera()?;
        assert_eq!(
            data.extrinsics()?,
            na::Matrix4::from_rows(&[
                na::RowVector4::new(
                    0.0148655429818,
                    -0.999880929698,
                    0.00414029679422,
                    -0.0216401454975
                ),
                na::RowVector4::new(
                    0.999557249008,
                    0.0149672133247,
                    0.025715529948,
                    -0.064676986768,
                ),
                na::RowVector4::new(
                    -0.0257744366974,
                    0.00375618835797,
                    0.999660727178,
                    0.00981073058949
                ),
                na::RowVector4::new(0.0, 0.0, 0.0, 1.0)
            ])
        );

        Ok(())
    }

    #[test]
    fn records() -> Result<()> {
        let data = EuRoC::new("test_data")?.left_camera()?;
        let record = data.records()?.skip(2).next().unwrap()?;

        assert_eq!(record.timestamp, 1403636579863555584.into());
        assert_eq!(record.image.dimensions(), (752, 480));

        assert_eq!(data.records()?.count(), 5);

        Ok(())
    }
}
