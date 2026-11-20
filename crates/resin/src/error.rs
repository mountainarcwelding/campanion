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
    /// A long-running job was stopped at the caller's request.
    #[error("cancelled")]
    Cancelled,
}

impl ResinError {
    pub(crate) fn invalid(message: impl Into<String>) -> Self {
        Self::Invalid(message.into())
    }

    pub(crate) fn cancelled() -> Self {
        Self::Cancelled
    }

    /// True when the work was stopped because the caller cancelled it.
    #[must_use]
    pub fn is_cancelled(&self) -> bool {
        matches!(self, Self::Cancelled)
    }
}

pub type Result<T> = std::result::Result<T, ResinError>;
