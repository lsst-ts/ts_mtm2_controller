use approx::assert_relative_eq;
use config::Config;
use csv::ReaderBuilder;
use serde_json::{json, Value};
use std::fs::File;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::Path;
use std::thread::sleep;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::constants::{NUM_AXIAL_ACTUATOR, NUM_COLUMN_LUT_GRAVITY};
use crate::enums::CommandStatus;

/// Trait for parsing the configuration value.
///
/// # Parameters
/// * `Self` - Type of the configuration value.
pub trait ConfigValue: Sized {
    /// Parse the configuration value.
    ///
    /// # Parameters
    /// * `s` - String to parse.
    ///
    /// # Returns
    /// The parsed configuration value.
    fn parse_value(s: &str) -> Self;
}

/// Implement the trait ConfigValue for String.
///
/// # Parameters
/// * `String` - Type of the configuration value.
impl ConfigValue for String {
    fn parse_value(s: &str) -> Self {
        s.to_string()
    }
}

/// Implement the trait ConfigValue for f64.
///
/// # Parameters
/// * `f64` - Type of the configuration value.
impl ConfigValue for f64 {
    fn parse_value(s: &str) -> Self {
        s.parse::<f64>().expect(&format!("{s} should parse as f64"))
    }
}

/// Implement the trait ConfigValue for usize.
///
/// # Parameters
/// * `usize` - Type of the configuration value.
impl ConfigValue for usize {
    fn parse_value(s: &str) -> Self {
        s.parse::<usize>()
            .expect(&format!("{s} should parse as usize"))
    }
}

/// Implement the trait ConfigValue for i32.
///
/// # Parameters
/// * `i32` - Type of the configuration value.
impl ConfigValue for i32 {
    fn parse_value(s: &str) -> Self {
        s.parse::<i32>().expect(&format!("{s} should parse as i32"))
    }
}

/// Implement the trait ConfigValue for u64.
///
/// # Parameters
/// * `u64` - Type of the configuration value.
///
/// # Panics
/// If the hex string does not start with 0x or 0X.
impl ConfigValue for u64 {
    fn parse_value(s: &str) -> Self {
        if !s.starts_with("0x") && !s.starts_with("0X") {
            panic!("Hex string {s} should start with 0x or 0X");
        }

        u64::from_str_radix(&s[2..], 16).expect(&format!("Hex string {s} should parse as u64"))
    }
}

/// Implement the trait ConfigValue for bool.
///
/// # Parameters
/// * `bool` - Type of the configuration value.
impl ConfigValue for bool {
    fn parse_value(s: &str) -> Self {
        s.parse::<bool>()
            .expect(&format!("{s} should parse as bool"))
    }
}

/// Get the configuation from the file.
///
/// # Parameters
/// * `filepath` - Path to the config file.
///
/// # Returns
/// The configuration.
pub fn get_config(filepath: &Path) -> Config {
    let name = filepath
        .to_str()
        .expect(&format!("Should have the file name in the {:?}", filepath));

    Config::builder()
        .add_source(config::File::with_name(name))
        .build()
        .expect(&format!("Should be able to read the {name}"))
}

/// Get the parameter from the file.
///
/// # Parameters
/// * `filepath` - Path to the config file.
/// * `key` - Key to find the parameter in the config file.
///
/// # Returns
/// The parameter.
pub fn get_parameter<T: ConfigValue>(filepath: &Path, key: &str) -> T {
    let config = get_config(filepath);

    config
        .get_string(key)
        .map(|v| T::parse_value(&v))
        .expect(&format!("Should find the {key} in the {:?}", filepath))
}

/// Get the array parameter from the file.
///
/// # Parameters
/// * `filepath` - Path to the config file.
/// * `key` - Key to find the parameter in the config file.
///
/// # Returns
/// The array parameter.
pub fn get_parameter_array<T: ConfigValue>(filepath: &Path, key: &str) -> Vec<T> {
    let config = get_config(filepath);
    let config_array = config
        .get_array(key)
        .expect(&format!("Should find the {key} in the {:?}", filepath));

    config_array
        .iter()
        .map(|x| T::parse_value(&x.clone().into_string().expect("Should be a string")))
        .collect()
}

/// Get the matrix parameter from the file.
///
/// # Parameters
/// * `filepath` - Path to the config file.
/// * `key` - Key to find the parameter in the config file.
///
/// # Returns
/// The matrix parameter.
pub fn get_parameter_matrix<T: ConfigValue>(filepath: &Path, key: &str) -> Vec<Vec<T>> {
    let config = get_config(filepath);
    let config_array = config
        .get_array(key)
        .expect(&format!("Should find the {key} in the {:?}", filepath));

    let matrix = config_array
        .iter()
        .map(|x| {
            x.clone()
                .into_array()
                .unwrap()
                .iter()
                .map(|y| T::parse_value(&y.clone().into_string().unwrap()))
                .collect()
        })
        .collect();

    matrix
}

/// Read the file of cell geometry.
///
/// # Parameters
/// * `filepath` - Path to the cell geometry file.
///
/// # Returns
/// * `loc_act_axial` - Location (x, y) of the axial actuators in m.
/// * `loc_act_tangent` - Location of the tangential actuators in degree.
/// * `radius_act_tangent` - Radius of the tangential actuators in m.
pub fn read_file_cell_geom(filepath: &Path) -> (Vec<Vec<f64>>, Vec<f64>, f64) {
    let loc_act_axial = get_parameter_matrix(filepath, "loc_act_axial");
    let loc_act_tangent = get_parameter_array(filepath, "loc_act_tangent");

    let radius_act_tangent = get_parameter(filepath, "radius_act_tangent");

    (loc_act_axial, loc_act_tangent, radius_act_tangent)
}

/// Read the file of stiffness matrix.
///
/// # Parameters
/// * `filepath` - Path to the stiffness matrix file.
///
/// # Returns
/// Stiffness matrix.
pub fn read_file_stiffness(filepath: &Path) -> Vec<Vec<f64>> {
    get_parameter_matrix(filepath, "stiff")
}

/// Read the file of displacement independent measurement system (IMS).
///
/// # Parameters
/// * `filepath` - Path to the displacement IMS file.
///
/// # Returns
/// * `matrix` - Matrix to calculate the rigid body position.
/// * `offset` - Offset to calculate the rigid body position.
pub fn read_file_disp_ims(filepath: &Path) -> (Vec<Vec<f64>>, Vec<f64>) {
    (
        get_parameter_matrix(filepath, "matrix"),
        get_parameter_array(filepath, "offset"),
    )
}

/// Read the file of temperature look-up table (LUT).
///
/// # Parameters
/// * `filepath` - Path to the LUT temperature file.
///
/// # Returns
/// Vector of temperature.
///
/// # Panics
/// If failed to read the LUT temperature file.
pub fn read_file_lut_temperature(filepath: &Path) -> Vec<f64> {
    let file = File::open(filepath).expect(&format!("Should be able to read the {:?}", filepath));
    let mut reader = ReaderBuilder::new().has_headers(false).from_reader(file);

    let mut vector: Vec<f64> = Vec::new();
    for result in reader.records() {
        if let Ok(record) = result {
            if let Ok(num) = record[0].parse::<f64>() {
                vector.push(num);
            }
        }
    }

    assert!(
        vector.len() == NUM_AXIAL_ACTUATOR,
        "Failed to read the LUT temperature file."
    );

    vector
}

/// Read the file of gravity look-up table (LUT).
///
/// # Parameters
/// * `filepath` - Path to the LUT gravity file.
///
/// # Returns
/// Matrix of gravity.
///
/// # Panics
/// If failed to read the LUT gravity file.
pub fn read_file_lut_gravity(filepath: &Path) -> Vec<Vec<f64>> {
    let file = File::open(filepath).expect(&format!("Should be able to read the {:?}", filepath));
    let mut reader = ReaderBuilder::new().has_headers(false).from_reader(file);

    let mut matrix: Vec<Vec<f64>> = Vec::new();
    for result in reader.records() {
        if let Ok(record) = result {
            let row: Vec<f64> = record.iter().map(|x| x.parse::<f64>().unwrap()).collect();
            matrix.push(row);
        }
    }

    // We will hold the first row as the header of the LUT. Therefore,
    // we have the length of the matrix to be at least:
    // (NUM_AXIAL_ACTUATOR + 1).

    assert!(
        (matrix.len() >= (NUM_AXIAL_ACTUATOR + 1)) && (matrix[0].len() == NUM_COLUMN_LUT_GRAVITY),
        "Failed to read the LUT gravity file."
    );

    matrix
}

/// Assert that two vectors are equal within a relative tolerance.
///
/// # Parameters
/// * `v1` - First vector.
/// * `v2` - Second vector.
/// * `epsilon` - Relative tolerance.
///
/// # Panics
/// If the two vectors are not equal within the relative tolerance.
pub fn assert_relative_eq_vector(v1: &[f64], v2: &[f64], epsilon: f64) {
    assert_eq!(v1.len(), v2.len());
    for (a, b) in v1.iter().zip(v2.iter()) {
        assert_relative_eq!(a, b, epsilon = epsilon);
    }
}

/// Check if the message is a telemetry.
///
/// # Arguments
/// * `name` - Name of the message.
///
/// # Returns
/// True if the message is a telemetry, false otherwise.
pub fn is_telemetry(name: &str) -> bool {
    name.starts_with("tel_")
}

/// Check if the message is an event.
///
/// # Arguments
/// * `name` - Name of the message.
///
/// # Returns
/// True if the message is an event, false otherwise.
pub fn is_event(name: &str) -> bool {
    name.starts_with("evt_")
}

/// Check if the message is a command.
///
/// # Arguments
/// * `name` - Name of the message.
///
/// # Returns
/// True if the message is a command, false otherwise.
pub fn is_command(name: &str) -> bool {
    name.starts_with("cmd_")
}

/// Acknowledge the command.
///
/// # Arguments
/// * `command_status` - Command status.
/// * `sequence_id` - Sequence ID.
///
/// # Returns
/// Acknowledged command.
pub fn acknowledge_command(command_status: CommandStatus, sequence_id: i64) -> Value {
    json!({"id": command_status.as_ref().to_lowercase(), "sequence_id": sequence_id})
}

/// Get the message name.
///
/// # Arguments
/// * `message` - Message that should have the "id" field.
///
/// # Returns
/// Message name. Return an empty string if the name is not found.
pub fn get_message_name(message: &Value) -> String {
    match message["id"].as_str() {
        Some(id) => String::from(id),
        None => String::new(),
    }
}

/// Get the message sequence ID.
///
/// # Arguments
/// * `message` - Message.
///
/// # Returns
/// Message sequence ID. Return -1 if the sequence ID is not found.
pub fn get_message_sequence_id(message: &Value) -> i64 {
    match message["sequence_id"].as_i64() {
        Some(sequence_id) => sequence_id,
        None => -1,
    }
}

/// TCP/IP client writes the message and sleep.
///
/// # Arguments
/// * `client` - TCP/IP client.
/// * `message` - Message to write.
/// * `sleep_time` - Sleep time in milliseconds.
///
/// # Panics
/// If the TCP stream of the client cannot write or flush.
pub fn client_write_and_sleep(client: &mut TcpStream, message: &str, sleep_time: u64) {
    client
        .write_all(message.as_bytes())
        .expect("Tcp stream should write.");
    client.flush().expect("Tcp stream should flush.");

    sleep(Duration::from_millis(sleep_time));
}

/// TCP/IP client reads the message and assert.
///
/// # Arguments
/// * `client` - TCP/IP client.
/// * `expected` - Expected message.
///
/// # Panics
/// If the TCP stream of the client cannot read.
pub fn client_read_and_assert(client: &mut TcpStream, expected: &str) {
    let mut buffer = vec![0; expected.len()];
    client
        .read(&mut buffer)
        .expect("Tcp stream of the client should read.");

    assert_eq!(std::str::from_utf8(&buffer).unwrap(), expected);
}

/// TCP/IP client reads the JSON message.
///
/// # Arguments
/// * `client` - TCP/IP client.
/// * `terminator` - Terminator of the message.
///
/// # Returns
/// JSON message.
pub fn client_read_json(client: &mut TcpStream, terminator: &[u8]) -> Value {
    let mut buffer = Vec::new();
    loop {
        let mut byte = [0; 1];
        client
            .read_exact(&mut byte)
            .expect("Tcp stream of the client should read.");

        buffer.push(byte[0]);
        if buffer.ends_with(terminator) {
            break;
        }
    }

    serde_json::from_slice(&buffer[0..(buffer.len() - terminator.len())])
        .expect("Should be able to convert to JSON.")
}

/// TCP/IP client reads the specific JSON message.
///
/// # Arguments
/// * `client` - TCP/IP client.
/// * `terminator` - Terminator of the message.
/// * `name` - Name of the message.
///
/// # Returns
/// JSON message.
pub fn client_read_specific_json(client: &mut TcpStream, terminator: &[u8], name: &str) -> Value {
    loop {
        let message = client_read_json(client, terminator);
        if get_message_name(&message) == name {
            return message;
        }
    }
}

/// Get the system time in milliseconds.
///
/// # Returns
/// System time in milliseconds since the UNIX epoch. Return 0 if fail.
pub fn get_system_time_ms() -> u64 {
    let now = SystemTime::now();
    match now.duration_since(UNIX_EPOCH) {
        Ok(duration) => duration.as_millis() as u64,
        Err(_) => 0,
    }
}

#[cfg(test)]
mod tests {
    use std::f64::EPSILON;

    use super::*;
    use approx::assert_relative_eq;

    use crate::constants::{NUM_ACTUATOR, NUM_HARDPOINTS};

    #[test]
    fn test_get_config() {
        let filepath = Path::new("config/parameters_control.yaml");
        let inclinometer_offset = get_config(filepath)
            .get_float("inclinometer_offset")
            .unwrap();

        assert_relative_eq!(inclinometer_offset, 0.94, epsilon = EPSILON);
    }

    #[test]
    fn test_get_parameter() {
        let lut: String = get_parameter(Path::new("config/parameters_app.yaml"), "lut");

        assert_eq!(lut, "optical");

        let inclinometer_offset: f64 = get_parameter(
            Path::new("config/parameters_control.yaml"),
            "inclinometer_offset",
        );

        assert_relative_eq!(inclinometer_offset, 0.94, epsilon = EPSILON);

        let local_telemetry_file: bool = get_parameter(
            Path::new("config/parameters_app.yaml"),
            "local_telemetry_file",
        );

        assert!(!local_telemetry_file);

        let enabled_faults_mask: u64 = get_parameter(
            Path::new("config/parameters_control.yaml"),
            "enabled_faults_mask",
        );

        assert_eq!(enabled_faults_mask, 0xff800003fffffff8);
    }

    #[test]
    #[should_panic(expected = "Should be able to read the wrong.yaml")]
    fn test_get_config_panic() {
        get_config(Path::new("wrong.yaml"));
    }

    #[test]
    fn test_read_file_cell_geom() {
        let filepath = Path::new("config/cell_geom.yaml");

        let (loc_act_axial, loc_act_tangent, radius_act_tangent) = read_file_cell_geom(filepath);

        assert_eq!(loc_act_axial.len(), NUM_AXIAL_ACTUATOR);
        assert_eq!(loc_act_axial[0], vec![0.0, 1.601]);
        assert_eq!(
            loc_act_axial[loc_act_axial.len() - 1],
            vec![-0.3427, 0.94157]
        );

        assert_eq!(loc_act_tangent, vec![0.0, 60.0, 120.0, 180.0, 240.0, 300.0]);
        assert_eq!(radius_act_tangent, 1.780189734);
    }

    #[test]
    fn test_get_parameter_array() {
        let hardpoints: Vec<usize> =
            get_parameter_array(Path::new("config/parameters_control.yaml"), "hardpoints");

        assert_eq!(hardpoints.len(), NUM_HARDPOINTS);
    }

    #[test]
    fn test_read_file_stiffness() {
        let filepath = Path::new("config/stiff_matrix_m2.yaml");

        let stiffness = read_file_stiffness(filepath);

        assert_eq!(stiffness.len(), NUM_ACTUATOR);
        assert_eq!(stiffness[0].len(), NUM_ACTUATOR);
    }

    #[test]
    fn test_read_file_disp_ims() {
        let filepath = Path::new("config/disp_ims.yaml");

        let (matrix, offset) = read_file_disp_ims(filepath);

        assert_eq!(matrix.len(), 6);
        assert_eq!(matrix[0].len(), 12);

        assert_eq!(offset.len(), 12);
    }

    #[test]
    fn test_read_file_lut_temperature() {
        let filepath = Path::new("config/lut/handling/Tu.csv");

        let vector = read_file_lut_temperature(filepath);

        assert_eq!(vector.len(), NUM_AXIAL_ACTUATOR);
    }

    #[test]
    #[should_panic(
        expected = "Should be able to read the \"wrong.csv\": Os { code: 2, kind: NotFound, message: \"No such file or directory\" }"
    )]
    fn test_read_file_lut_temperature_panic() {
        read_file_lut_temperature(Path::new("wrong.csv"));
    }

    #[test]
    fn test_read_file_lut_gravity() {
        let filepath = Path::new("config/lut/handling/F_0.csv");

        let matrix = read_file_lut_gravity(filepath);

        assert_eq!(matrix.len(), NUM_AXIAL_ACTUATOR + 1);
        assert_eq!(matrix[0].len(), NUM_COLUMN_LUT_GRAVITY);
    }

    #[test]
    #[should_panic(expected = "Failed to read the LUT gravity file.")]
    fn test_read_file_lut_gravity_panic() {
        let filepath = Path::new("config/lut/handling/Tu.csv");

        read_file_lut_gravity(filepath);
    }

    #[test]
    fn test_assert_relative_eq_vector() {
        assert_relative_eq_vector(&vec![1.0, 2.0, 3.0], &vec![1.0, 2.0, 3.0], EPSILON);
    }

    #[test]
    #[should_panic(expected = "`left == right` failed")]
    fn test_assert_relative_eq_vector_panic_1() {
        assert_relative_eq_vector(&vec![0.0, 0.0], &vec![0.0, 1.0, 0.0], EPSILON);
    }

    #[test]
    #[should_panic(
        expected = "assert_relative_eq!(a, b, epsilon = epsilon)\n\n    left  = 0.1\n    right = 1.1\n\n"
    )]
    fn test_assert_relative_eq_vector_panic_2() {
        assert_relative_eq_vector(&vec![0.0, 0.1, 0.0], &vec![0.0, 1.1, 0.0], EPSILON);
    }

    #[test]
    fn test_is_telemetry() {
        assert_eq!(is_telemetry("tel_test"), true);
        assert_eq!(is_telemetry("evt_test"), false);
        assert_eq!(is_telemetry("cmd_test"), false);
        assert_eq!(is_telemetry("test"), false);
    }

    #[test]
    fn test_is_event() {
        assert_eq!(is_event("evt_test"), true);
        assert_eq!(is_event("tel_test"), false);
        assert_eq!(is_event("cmd_test"), false);
        assert_eq!(is_event("test"), false);
    }

    #[test]
    fn test_is_command() {
        assert_eq!(is_command("cmd_test"), true);
        assert_eq!(is_command("evt_test"), false);
        assert_eq!(is_command("tel_test"), false);
        assert_eq!(is_command("test"), false);
    }

    #[test]
    fn test_acknowledge_command() {
        assert_eq!(
            acknowledge_command(CommandStatus::Success, 1),
            json!({"id": "success", "sequence_id": 1})
        );
        assert_eq!(
            acknowledge_command(CommandStatus::Fail, 2),
            json!({"id": "fail", "sequence_id": 2})
        );
        assert_eq!(
            acknowledge_command(CommandStatus::Ack, 3),
            json!({"id": "ack", "sequence_id": 3})
        );
        assert_eq!(
            acknowledge_command(CommandStatus::NoAck, 4),
            json!({"id": "noack", "sequence_id": 4})
        );
    }

    #[test]
    fn test_get_message_name() {
        // There is the "id" field.
        assert_eq!(get_message_name(&json!({"id": "cmd_test"})), "cmd_test");
        assert_eq!(get_message_name(&json!({"id": 1})), "");

        // There is no "id" field.
        assert_eq!(get_message_name(&json!({})), "");
        assert_eq!(get_message_name(&json!({"test": 1})), "");
    }

    #[test]
    fn test_get_message_sequence_id() {
        // There is the "sequence_id" field.
        assert_eq!(get_message_sequence_id(&json!({"sequence_id": 1})), 1);
        assert_eq!(get_message_sequence_id(&json!({"sequence_id": 1.0})), -1);

        // There is no "sequence_id" field.
        assert_eq!(get_message_sequence_id(&json!({})), -1);
        assert_eq!(get_message_sequence_id(&json!({"test": 1})), -1);
    }

    #[test]
    fn test_get_system_time_ms() {
        assert!(get_system_time_ms() > 0);
    }
}
