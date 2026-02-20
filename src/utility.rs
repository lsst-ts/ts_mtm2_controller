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

use csv::ReaderBuilder;
use serde_json::{json, Value};
use std::fs::File;
use std::net::TcpStream;
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use crate::constants::{NUM_AXIAL_ACTUATOR, NUM_COLUMN_LUT_GRAVITY};
use crate::enums::CommandStatus;
use ts_control_utils::utility::{
    client_read_json, get_parameter, get_parameter_array, get_parameter_matrix,
};

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
    let file = File::open(filepath)
        .unwrap_or_else(|_| panic!("Should be able to read the {:?}", filepath));
    let mut reader = ReaderBuilder::new().has_headers(false).from_reader(file);

    let mut vector: Vec<f64> = Vec::new();
    for record in reader.records().flatten() {
        if let Ok(num) = record[0].parse::<f64>() {
            vector.push(num);
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
    let file = File::open(filepath)
        .unwrap_or_else(|_| panic!("Should be able to read the {:?}", filepath));
    let mut reader = ReaderBuilder::new().has_headers(false).from_reader(file);

    let mut matrix: Vec<Vec<f64>> = Vec::new();
    for record in reader.records().flatten() {
        let row: Vec<f64> = record.iter().map(|x| x.parse::<f64>().unwrap()).collect();
        matrix.push(row);
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
    message["sequence_id"].as_i64().unwrap_or(-1)
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
    use super::*;

    use crate::constants::NUM_ACTUATOR;

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
    #[should_panic(expected = "Should be able to read the \"wrong.csv\"")]
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
