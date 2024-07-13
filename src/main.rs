use clap::{value_parser, Arg, ArgAction, Command};
use log::info;
use simplelog::{
    format_description, ColorChoice, CombinedLogger, ConfigBuilder, LevelFilter, TermLogger,
    TerminalMode, WriteLogger,
};
use std::fs::File;

use run_m2_controller::application;

fn main() {
    // Parse the command line arguments
    let matches = Command::new("control system")
        .about("M2 mirror control system.")
        .arg(
            Arg::new("ports")
                .short('p')
                .long("ports")
                .value_names(&["GUI command port", "GUI telemetry port", "CSC command port", "CSC telemetry port"])
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
        .arg(
            Arg::new("level")
                .short('l')
                .long("log-level")
                .help("Log level: 0 (Off), 1 (Error), 2 (Warn), 3 (Info), 4 (Debug), 5 (Trace)")
                .default_value("3")
                .value_parser(value_parser!(u32)),
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

    // Check the log filter
    let log_filter = get_log_filter(matches.get_one::<u32>("level"));

    // Initiate the logger
    initiate_logger(log_filter, "application.log");
    info!("Log level: {log_filter}.");

    // Run the application
    application::run(ports[0], ports[1], ports[2], ports[3], is_simulation_mode);
}

/// Get the log filter.
///
/// # Arguments
/// * `log_level` - Log level.
///
/// # Returns
/// Log filter.
fn get_log_filter(log_level: Option<&u32>) -> LevelFilter {
    match log_level {
        Some(level) => match level {
            0 => LevelFilter::Off,
            1 => LevelFilter::Error,
            2 => LevelFilter::Warn,
            3 => LevelFilter::Info,
            4 => LevelFilter::Debug,
            5 => LevelFilter::Trace,
            _ => LevelFilter::Info,
        },
        None => LevelFilter::Info,
    }
}

/// Initiate the logger.
///
/// # Arguments
/// * `level` - Log level.
/// * `filepath` - Log file path.
fn initiate_logger(level: LevelFilter, filepath: &str) {
    let config = ConfigBuilder::new()
        .set_time_format_custom(format_description!(
            "[year]/[month]/[day] [hour]:[minute]:[second].[subsecond]"
        ))
        .build();

    // Log to the terminal
    let logger_terminal = TermLogger::new(
        level,
        config.clone(),
        TerminalMode::Mixed,
        ColorChoice::Auto,
    );

    // Log to the file
    let logger_file: Option<Box<WriteLogger<File>>>;
    match File::create(filepath) {
        Ok(file) => {
            logger_file = Some(WriteLogger::new(level, config.clone(), file));
        }
        Err(error) => {
            logger_file = None;
            eprintln!("Failed to create the log file: {error}.");
        }
    }

    if logger_file.is_some() {
        let _ = CombinedLogger::init(vec![logger_terminal, logger_file.unwrap()]);
    } else {
        let _ = CombinedLogger::init(vec![logger_terminal]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_log_filter() {
        assert_eq!(get_log_filter(Some(&0)), LevelFilter::Off);
        assert_eq!(get_log_filter(Some(&1)), LevelFilter::Error);
        assert_eq!(get_log_filter(Some(&2)), LevelFilter::Warn);
        assert_eq!(get_log_filter(Some(&3)), LevelFilter::Info);
        assert_eq!(get_log_filter(Some(&4)), LevelFilter::Debug);
        assert_eq!(get_log_filter(Some(&5)), LevelFilter::Trace);

        assert_eq!(get_log_filter(Some(&6)), LevelFilter::Info);

        assert_eq!(get_log_filter(None), LevelFilter::Info);
    }
}
