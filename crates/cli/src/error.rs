//! Failures from argument parsing, job files and IR IO.

use std::fmt;
use std::io;
use std::process::ExitCode;

use campanion_ir::IrError;

/// Same byte cap `campanion-ir` uses when it loads a document.
pub const MAX_JSON_BYTES: usize = 32 * 1024 * 1024;

/// A command failed, or the arguments were not a known command.
pub enum CliError {
    /// The arguments do not match a command. The process exits 2.
    Usage(String),
    /// The command ran and failed. The process exits 1.
    Run(String),
}

impl CliError {
    pub fn usage(message: impl Into<String>) -> Self {
        Self::Usage(message.into())
    }

    pub fn run(message: impl Into<String>) -> Self {
        Self::Run(message.into())
    }

    pub fn invalid(message: impl fmt::Display) -> Self {
        Self::Run(format!("invalid input: {message}"))
    }

    pub fn code(&self) -> ExitCode {
        match self {
            Self::Usage(_) => ExitCode::from(2),
            Self::Run(_) => ExitCode::FAILURE,
        }
    }
}

impl fmt::Display for CliError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Usage(message) | Self::Run(message) => f.write_str(message),
        }
    }
}

impl From<IrError> for CliError {
    fn from(err: IrError) -> Self {
        Self::Run(err.to_string())
    }
}

impl From<io::Error> for CliError {
    fn from(err: io::Error) -> Self {
        Self::Run(err.to_string())
    }
}
