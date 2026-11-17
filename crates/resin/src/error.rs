//! Errors for resin output.

/// Anything that can go wrong while slicing, encoding, or reading a `.pm7m`.
#[derive(Debug, thiserror::Error)]
pub enum ResinError {
    #[error("invalid input: {0}")]
    Invalid(String),
    #[error("layer image: {0}")]
    Image(String),
    #[error("zip: {0}")]
    Zip(String),
    #[error("pm7m: {0}")]
    Format(String),
}

impl ResinError {
    pub(crate) fn invalid(message: impl Into<String>) -> Self {
        Self::Invalid(message.into())
    }
}

pub type Result<T> = std::result::Result<T, ResinError>;
