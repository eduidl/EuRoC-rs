#![warn(
    rust_2018_idioms,
    elided_lifetimes_in_paths,
    clippy::all,
    clippy::nursery
)]

#[cfg(test)]
mod test {
    #[test]
    fn test() {
        assert_eq!(1 + 1, 2);
    }
}
