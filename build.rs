fn main() {
    println!("cargo:rerun-if-changed=fpga/NiFpga.c");
    println!("cargo:rerun-if-changed=fpga/NiFpga.h");

    // NI's generated C API shim is used on Linux targets (for cRIO builds).
    let target = std::env::var("TARGET").unwrap_or_default();
    if target.contains("-linux-") {
        cc::Build::new()
            .file("fpga/NiFpga.c")
            .include("fpga")
            .warnings(false)
            .compile("nifpga_c_api");

        // Emit explicit link directives to make sure NiFpga_* symbols are resolved.
        let out_dir = std::env::var("OUT_DIR").unwrap();
        println!("cargo:rustc-link-search=native={out_dir}");
        println!("cargo:rustc-link-lib=static=nifpga_c_api");
        println!("cargo:rustc-link-lib=dylib=dl");
    }
}
