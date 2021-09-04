use std::{fs, path::Path};

use anyhow::Result;
use yaml_rust::YamlLoader;

pub fn load_yaml<P: AsRef<Path>>(path: P) -> Result<Vec<yaml_rust::Yaml>> {
    let f = fs::read_to_string(path)?;
    Ok(YamlLoader::load_from_str(&f)?)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Timestamp(u64);

impl Timestamp {
    pub const fn nsecs(self) -> u64 {
        self.0
    }
}

impl From<u64> for Timestamp {
    fn from(v: u64) -> Self {
        Self(v)
    }
}
