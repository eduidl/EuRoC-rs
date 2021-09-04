#![warn(
    rust_2018_idioms,
    elided_lifetimes_in_paths,
    clippy::all,
    clippy::nursery
)]

mod camera;
mod common;
mod ground_truth;
mod imu;
mod position;

use std::path::{Path, PathBuf};

use anyhow::{ensure, Result};

pub use self::{camera::*, common::*, ground_truth::*, imu::*, position::*};

#[derive(Debug)]
pub struct EuRoC {
    root: PathBuf,
}

impl EuRoC {
    pub fn new<P: AsRef<Path>>(root: P) -> Result<Self> {
        ensure!(root.as_ref().is_dir());

        Ok(Self {
            root: root.as_ref().to_owned(),
        })
    }

    pub fn left_camera(&self) -> Result<CameraRecords> {
        CameraRecords::new(self.root.join("cam0"))
    }

    pub fn right_camera(&self) -> Result<CameraRecords> {
        CameraRecords::new(self.root.join("cam1"))
    }

    pub fn imu(&self) -> Result<ImuData> {
        ImuData::new(self.root.join("imu0"))
    }

    pub fn position(&self) -> Result<PositionData> {
        PositionData::new(self.root.join("leica0"))
    }

    pub fn ground_truth(&self) -> Result<GroundTruthData> {
        GroundTruthData::new(self.root.join("state_groundtruth_estimate0"))
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn right_camera() -> Result<()> {
        let _ = EuRoC::new("test_data")?.right_camera()?;

        Ok(())
    }
}
