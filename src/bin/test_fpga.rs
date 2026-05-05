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

use nifpga_dll::{NifpgaError, Session};
use std::path::Path;

// The NI C shim is produced by this package build script on Linux targets.
// Explicitly linking here ensures test_fpga pulls in NiFpga_* symbols.
#[cfg(target_os = "linux")]
#[link(name = "nifpga_c_api", kind = "static")]
unsafe extern "C" {}

#[cfg(target_os = "linux")]
#[link(name = "dl")]
unsafe extern "C" {}

fn main() -> Result<(), NifpgaError> {
    println!("Begin to test the FPGA.");

    // Open the session, it will be closed when it goes out of scope
    let bitfile = Path::new("fpga/NiFpga_portSerialMasterSlave.lvbitx");

    // Check the bitfile exists
    if !bitfile.exists() {
        println!("Bitfile {} does not exist.", bitfile.display());
    }

    // Print the abosolute path (as a string) of the bitfile
    let bitfile_path = bitfile
        .canonicalize()
        .unwrap()
        .to_str()
        .unwrap()
        .to_string();
    println!("Using bitfile: {}", bitfile_path);

    // For the signature, search <SignatureRegister> in the .lvbitx
    let signature = "BCA48EBD4CE9E2D0FA6A1B6BC2636EC6";
    let resource = "RIO0";
    let session = Session::open(&bitfile_path, signature, resource, true, false)?;

    println!("Session opened.");

    // Read the NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On 0x18056
    let mut control_bool_ilc_comm_power_on = session.read::<bool>(0x18056)?;

    println!(
        "NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On (init): {}",
        control_bool_ilc_comm_power_on
    );

    // Set the value to true
    session.write::<bool>(0x18056, true)?;

    // Read the value back
    control_bool_ilc_comm_power_on = session.read::<bool>(0x18056)?;
    println!(
        "NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On (after write): {}",
        control_bool_ilc_comm_power_on
    );

    // Set the value back to false
    session.write::<bool>(0x18056, false)?;
    // Read the value back
    control_bool_ilc_comm_power_on = session.read::<bool>(0x18056)?;
    println!(
        "NiFpga_portSerialMasterSlave_ControlBool_ILC_Comm_Power_On (after reset): {}",
        control_bool_ilc_comm_power_on
    );

    println!("End of FPGA test.");

    Ok(())
}
