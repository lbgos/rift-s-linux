fn main() {
    cc::Build::new()
        .cpp(true)
        .file("native/bridge.cpp")
        .file("native/mirror.cpp")
        .flag("-std=c++17")
        .compile("openvr_bridge");
    println!("cargo:rustc-link-lib=dl");
    println!("cargo:rustc-link-lib=GL");
    println!("cargo:rustc-link-lib=X11");
    println!("cargo:rerun-if-changed=native/bridge.cpp");
    println!("cargo:rerun-if-changed=native/mirror.cpp");
    tauri_build::build();
}
