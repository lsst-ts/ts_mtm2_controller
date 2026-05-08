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

fn main() {
    // Cargo sets CARGO_FEATURE_<NAME> for enabled features (uppercased).
    if std::env::var_os("CARGO_FEATURE_FPGA").is_some() {
        println!("cargo:rerun-if-changed=fpga/NiFpga.c");
        println!("cargo:rerun-if-changed=fpga/NiFpga.h");

        cc::Build::new()
            .file("fpga/NiFpga.c")
            .include("fpga")
            .warnings(true)
            .compile("nifpga_c_api");

        // Emit explicit link directives to make sure NiFpga_* symbols are resolved.
        let out_dir = std::env::var("OUT_DIR").unwrap();
        println!("cargo:rustc-link-search=native={out_dir}");
        println!("cargo:rustc-link-lib=static=nifpga_c_api");
        println!("cargo:rustc-link-lib=dylib=dl");
    }
}
