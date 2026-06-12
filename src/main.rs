// This file is part of ts_mtm2_controller.
//
// Developed for the Vera Rubin Observatory Systems.
// This product includes software developed by the LSST Project
// (https://www.lsst.org).
// See the COPYRIGHT file at the top-level directory of this distribution
// for details of code ownership.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

use clap::{value_parser, Arg, ArgAction, Command};
use flexi_logger::{
    Age, Cleanup, Criterion, DeferredNow, Duplicate, FileSpec, Logger, LoggerHandle, Naming,
    WriteMode,
};
use log::{info, Record};
use std::io::Write;
use std::path::Path;

use run_m2::application;
use ts_control_utils::utility::get_parameter;

#[cfg(feature = "realtime")]
use libc::{mlockall, MCL_CURRENT, MCL_FUTURE};

#[cfg(feature = "realtime")]
use std::io::Error;

#[cfg(feature = "realtime")]
use log::error;

fn main() {
    // Parse the command line arguments
    let matches = Command::new("control system")
        .about("M2 mirror control system.")
        .arg(
            Arg::new("ports")
                .short('p')
                .long("ports")
                .value_names(["GUI command port", "GUI telemetry port", "CSC command port", "CSC telemetry port"])
                .help("Command and telemetry ports to override the configuration file. Default is 0, which means no override.")
                .number_of_values(4)
                .default_values(["0", "0", "0", "0"])
                .value_parser(value_parser!(i32)),
        )
        .arg(
            Arg::new("simulate")
                .short('s')
                .long("simulate")
                .action(ArgAction::SetTrue)
                .help("Run the simulation mode"),
        )
        .get_matches();

    // Check the ports
    let ports: Vec<i32> = matches
        .get_many("ports")
        .expect("There should be four ports.")
        .copied()
        .collect();

    // Check the simulation mode
    let is_simulation_mode = matches.get_flag("simulate");

    // Initiate the logger
    let config_file = Path::new("config/parameters_app.yaml");
    let logger_handle = initiate_logger(
        get_parameter(config_file, "log_directory"),
        get_parameter(config_file, "log_basename"),
        get_parameter(config_file, "log_suffix"),
        get_parameter(config_file, "log_size"),
        get_parameter(config_file, "log_file_keep_days"),
    );

    if let Ok(level_filter) = logger_handle.current_max_level() {
        info!("Log level: {}.", level_filter);
    }

    // Lock memory to prevent latency spikes from paging
    #[cfg(feature = "realtime")]
    {
        unsafe {
            if mlockall(MCL_CURRENT | MCL_FUTURE) != 0 {
                let message = format!("Failed to lock the memory: {}.", Error::last_os_error());
                error!("{}", message);
                panic!("{}", message);
            }
        }
    }

    // Run the application
    application::run(
        ports[0],
        ports[1],
        ports[2],
        ports[3],
        is_simulation_mode,
        Some(logger_handle),
    );
}

/// Initiate the logger.
///
/// # Arguments
/// * `directory` - Log file directory.
/// * `basename` - Log file basename.
/// * `suffix` - Log file suffix.
/// * `size` - Log file size to rotate in bytes.
/// * `keep_days` - Log file keep days.
fn initiate_logger(
    directory: String,
    basename: String,
    suffix: String,
    size: u64,
    keep_days: usize,
) -> LoggerHandle {
    let file_spec = FileSpec::default()
        .directory(directory)
        .basename(basename)
        .suffix(suffix);

    match Logger::try_with_str("info").and_then(|logger| {
        logger
            .format(log_format)
            .log_to_file(file_spec)
            .duplicate_to_stdout(Duplicate::All)
            .rotate(
                Criterion::AgeOrSize(Age::Day, size),
                Naming::TimestampsDirect,
                Cleanup::KeepForDays(keep_days),
            )
            .write_mode(WriteMode::Async)
            .start_with_specfile(Path::new("config/logspecification.toml"))
    }) {
        Ok(logger_handle) => logger_handle,
        Err(error) => panic!("Failed to initialize logger: {error}."),
    }
}

/// Log format for the logger.
///
/// # Arguments
/// * `writer` - Log writer.
/// * `now` - Current time.
/// * `record` - Log record.
///
/// # Returns
/// Result of the log format operation.
fn log_format(
    writer: &mut dyn Write,
    now: &mut DeferredNow,
    record: &Record,
) -> std::io::Result<()> {
    write!(
        writer,
        "{} [{}] ({}) {}",
        now.format("%Y/%m/%d %H:%M:%S%.3f"),
        record.level(),
        record.target(),
        record.args(),
    )
}
