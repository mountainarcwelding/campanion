//! Failures while loading a post or writing G-code.

/// Result type for post operations.
pub type Result<T> = std::result::Result<T, PostError>;

/// A post definition or a program the definition cannot write.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum PostError {
    /// A field in the post JSON is missing, out of range, or inconsistent.
    #[error("invalid post field {field}: {message}")]
    InvalidField {
        /// Dotted field path, such as `arcs.max_sweep_deg`.
        field: String,
        /// What is wrong with that field.
        message: String,
    },
    /// The program or the options cannot be posted.
    #[error("invalid input: {0}")]
    InvalidInput(String),
    /// The program uses a feature this post does not write.
    #[error("not supported: {0}")]
    NotSupported(String),
    /// The JSON text is not a post definition.
    #[error("json: {0}")]
    Json(String),
}

pub(crate) fn field(field: impl Into<String>, message: impl Into<String>) -> PostError {
    PostError::InvalidField { field: field.into(), message: message.into() }
}

pub(crate) fn invalid(message: impl Into<String>) -> PostError {
    PostError::InvalidInput(message.into())
}

pub(crate) fn unsupported(message: impl Into<String>) -> PostError {
    PostError::NotSupported(message.into())
}
