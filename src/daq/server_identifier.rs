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

#[derive(Debug, PartialEq, Eq)]
pub struct ServerIdentifier {
    // Unique identifier. Note in the frame, only the last 6 bytes (48 bits)
    // are used to represent the unique ID.
    pub unique_id: u64,
    // Application/firmware type:
    // 1 = Electrical Actuator
    // 4 = Temperature Monitor
    // 5 = Displacement Monitor
    // 6 = Inclinometer Monitor
    // 10 = Bootloader
    pub application_type: u8,
    // Network node (1-wire) type.
    pub network_node_type: u8,
    // Inner-loop controller (ILC) selected options:
    // Electrical Actuator ILC: 0x00 = Gray Code Encoder, 0x01 = Binary Encoder
    // Monitor ILCs: None
    // Bootloader: None
    pub selected_options: u8,
    // Network node (1-wire) options.
    pub network_node_options: u8,
    // Revision number of the ILC firmware. In the frame, the revision is
    // represented as two bytes: major (0-255) and minor (0-255).
    pub firmware_revision: String,
    // ASCII firmware name.
    pub firmware_name: String,
}

impl ServerIdentifier {
    /// Create a new instance of ServerIdentifier from a byte array.
    ///
    /// # Arguments
    /// * `frame` - The received frame containing the server identifier data.
    ///   The first element is the total bytes without the byte count and CRC,
    ///   followed by the unique ID, application type, network node type,
    ///   selected options, network node options, firmware revision (
    ///   major.minor), and firmware name.
    ///
    /// # Returns
    /// * `Option<ServerIdentifier>` - Some(ServerIdentifier) if the frame
    ///   contains valid data, None otherwise.
    pub fn from_frame(frame: &[u8]) -> Option<ServerIdentifier> {
        if frame.is_empty() {
            return None;
        }

        // 13 is the minimum payload and 1 here is the bytes of byte count.
        let bytes_without_byte_count_and_crc = frame[0] as usize;
        if (frame.len() < 13) || (frame.len() < (bytes_without_byte_count_and_crc + 1)) {
            return None;
        }

        // Trim the firmware name.
        let name = String::from_utf8_lossy(&frame[13..(bytes_without_byte_count_and_crc + 1)]);
        let name_trim_end = name.trim_end();

        Some(Self {
            unique_id: u64::from_be_bytes([
                0, 0, frame[1], frame[2], frame[3], frame[4], frame[5], frame[6],
            ]),
            application_type: frame[7],
            network_node_type: frame[8],
            selected_options: frame[9],
            network_node_options: frame[10],
            firmware_revision: format!("{}.{}", frame[11], frame[12]),
            firmware_name: name_trim_end.to_string(),
        })
    }

    /// Convert the ServerIdentifier instance into a byte array.
    ///
    /// Returns
    /// A byte array representing the server identifier data. The first element
    /// is the total bytes without the byte count and CRC, followed by the
    /// unique ID, application type, network node type, selected options,
    /// network node options, firmware revision (major.minor), and firmware
    /// name.
    pub fn to_frame(&self) -> Vec<u8> {
        let name_bytes = self.firmware_name.len();
        let mut frame = vec![0; 13 + name_bytes];

        frame[0] = frame.len() as u8 - 1;

        // For the unique_id, we only use the last 6 bytes (48 bits) to fit
        // into the frame.
        frame[1..7].copy_from_slice(&self.unique_id.to_be_bytes()[2..]);

        frame[7] = self.application_type;
        frame[8] = self.network_node_type;
        frame[9] = self.selected_options;
        frame[10] = self.network_node_options;

        // The format of firmware revision is "major.minor" (e.g., "1.0").
        // We split it into two parts and store them as u8 values.
        let revision_parts: Vec<&str> = self.firmware_revision.split('.').collect();
        frame[11] = revision_parts
            .first()
            .and_then(|s| s.parse::<u8>().ok())
            .unwrap_or(0);
        frame[12] = revision_parts
            .last()
            .and_then(|s| s.parse::<u8>().ok())
            .unwrap_or(0);

        frame[13..].copy_from_slice(self.firmware_name.as_bytes());

        frame
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_from_frame() {
        // Test data
        // Valid frame with server identifier data
        let frame_test = [
            0x10, 0x12, 0x34, 0x56, 0x78, 0x90, 0xF1, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, b't',
            b'e', b's', b't',
        ];

        let server_id_test = ServerIdentifier::from_frame(&frame_test).unwrap();

        assert_eq!(server_id_test.unique_id, 0x1234567890F1);
        assert_eq!(server_id_test.application_type, 0x01);
        assert_eq!(server_id_test.network_node_type, 0x02);
        assert_eq!(server_id_test.selected_options, 0x03);
        assert_eq!(server_id_test.network_node_options, 0x04);
        assert_eq!(server_id_test.firmware_revision, "5.6");
        assert_eq!(server_id_test.firmware_name, "test");

        // Invalid frame length
        assert!(ServerIdentifier::from_frame(&[]).is_none());
        assert!(ServerIdentifier::from_frame(&[0x0F]).is_none());
        assert!(ServerIdentifier::from_frame(&frame_test[0..(frame_test.len() - 1)]).is_none());

        // Real data (ILC 0)
        let frame_ilc_0 = [
            53, 0, 0, 23, 133, 83, 133, 1, 1, 1, 1, 7, 1, 69, 108, 101, 99, 116, 114, 111, 109,
            101, 99, 104, 97, 110, 105, 99, 97, 108, 32, 73, 76, 67, 32, 40, 99, 41, 50, 48, 49,
            55, 32, 65, 85, 82, 65, 45, 76, 83, 83, 84, 32, 32, 17, 204,
        ];

        let server_id_ilc_0 = ServerIdentifier::from_frame(&frame_ilc_0).unwrap();

        assert_eq!(server_id_ilc_0.unique_id, 0x17855385);
        assert_eq!(server_id_ilc_0.application_type, 0x01);
        assert_eq!(server_id_ilc_0.network_node_type, 0x01);
        assert_eq!(server_id_ilc_0.selected_options, 0x01);
        assert_eq!(server_id_ilc_0.network_node_options, 0x01);
        assert_eq!(server_id_ilc_0.firmware_revision, "7.1");
        assert_eq!(
            server_id_ilc_0.firmware_name,
            "Electromechanical ILC (c)2017 AURA-LSST"
        );

        // Real data (ILC 78)
        let frame_ilc_78 = [
            56, 0, 0, 23, 132, 188, 248, 4, 4, 0, 0, 10, 0, 84, 101, 109, 112, 101, 114, 97, 116,
            117, 114, 101, 32, 77, 111, 110, 105, 116, 111, 114, 105, 110, 103, 32, 73, 76, 67, 32,
            40, 99, 41, 50, 48, 50, 52, 32, 65, 85, 82, 65, 45, 76, 83, 83, 84, 192, 106,
        ];

        let server_id_ilc_78 = ServerIdentifier::from_frame(&frame_ilc_78).unwrap();

        assert_eq!(server_id_ilc_78.unique_id, 0x1784bcf8);
        assert_eq!(server_id_ilc_78.application_type, 0x04);
        assert_eq!(server_id_ilc_78.network_node_type, 0x04);
        assert_eq!(server_id_ilc_78.selected_options, 0x0);
        assert_eq!(server_id_ilc_78.network_node_options, 0x0);
        assert_eq!(server_id_ilc_78.firmware_revision, "10.0");
        assert_eq!(
            server_id_ilc_78.firmware_name,
            "Temperature Monitoring ILC (c)2024 AURA-LSST"
        );

        // Real data (ILC 83)
        let frame_ilc_83 = [
            57, 0, 0, 23, 132, 192, 79, 6, 6, 0, 0, 10, 0, 73, 110, 99, 108, 105, 110, 111, 109,
            101, 116, 101, 114, 32, 77, 111, 110, 105, 116, 111, 114, 105, 110, 103, 32, 73, 76,
            67, 32, 40, 99, 41, 50, 48, 50, 52, 32, 65, 85, 82, 65, 45, 76, 83, 83, 84, 187, 76,
        ];

        let server_id_ilc_83 = ServerIdentifier::from_frame(&frame_ilc_83).unwrap();

        assert_eq!(server_id_ilc_83.unique_id, 0x1784c04f);
        assert_eq!(server_id_ilc_83.application_type, 0x06);
        assert_eq!(server_id_ilc_83.network_node_type, 0x06);
        assert_eq!(server_id_ilc_83.selected_options, 0x0);
        assert_eq!(server_id_ilc_83.network_node_options, 0x0);
        assert_eq!(server_id_ilc_83.firmware_revision, "10.0");
        assert_eq!(
            server_id_ilc_83.firmware_name,
            "Inclinometer Monitoring ILC (c)2024 AURA-LSST"
        );
    }

    #[test]
    fn test_to_frame() {
        let server_id = ServerIdentifier {
            unique_id: 0x1234567890F1,
            application_type: 0x01,
            network_node_type: 0x02,
            selected_options: 0x03,
            network_node_options: 0x04,
            firmware_revision: String::from("5.6"),
            firmware_name: String::from("test"),
        };

        let frame = server_id.to_frame();

        assert_eq!(ServerIdentifier::from_frame(&frame).unwrap(), server_id);
    }
}
