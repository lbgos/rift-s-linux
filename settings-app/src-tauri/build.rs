fn main() {
    cc::Build::new()
        .cpp(true)
        .file("native/bridge.cpp")
        .flag("-std=c++17")
        .compile("openvr_bridge");
    println!("cargo:rustc-link-lib=dl");
    println!("cargo:rerun-if-changed=native/bridge.cpp");
    tauri_build::build();
}
