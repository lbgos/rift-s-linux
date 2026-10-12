fn main() {
    let out = std::path::PathBuf::from(std::env::var_os("OUT_DIR").expect("Missing OUT_DIR"));
    let shader = std::process::Command::new("glslc")
        .args([
            "-fshader-stage=compute",
            "-mfmt=c",
            "native/overlay_preview.comp",
            "-o",
        ])
        .arg(out.join("overlay_preview_shader.inc"))
        .status()
        .expect("Vulkan shader compiler glslc is required");
    assert!(shader.success(), "Unable to compile headset preview shader");
    cc::Build::new()
        .cpp(true)
        .file("native/bridge.cpp")
        .file("native/overlay_preview.cpp")
        .include(&out)
        .flag("-std=c++17")
        .compile("openvr_bridge");
    println!("cargo:rustc-link-lib=dl");
    println!("cargo:rustc-link-lib=vulkan");
    println!("cargo:rerun-if-changed=native/overlay_preview.cpp");
    println!("cargo:rerun-if-changed=native/overlay_preview.hpp");
    println!("cargo:rerun-if-changed=native/openvr.h");
    println!("cargo:rerun-if-changed=native/overlay_preview.comp");
    println!("cargo:rerun-if-changed=native/bridge.cpp");
    tauri_build::build();
}
