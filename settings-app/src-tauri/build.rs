fn main() {
    let out = std::path::PathBuf::from(std::env::var_os("OUT_DIR").expect("Missing OUT_DIR"));
    for stage in ["vert", "frag"] {
        let status = std::process::Command::new("glslc")
            .arg(format!("-fshader-stage={stage}"))
            .arg("-mfmt=c")
            .arg(format!("native/headset_preview.{stage}"))
            .arg("-o")
            .arg(out.join(format!("headset_preview_{stage}.inc")))
            .status()
            .expect("Vulkan shader compiler glslc is required");
        assert!(status.success(), "Unable to compile headset preview shaders");
    }
    cc::Build::new()
        .cpp(true)
        .file("native/bridge.cpp")
        .file("native/headset_preview.cpp")
        .include(&out)
        .flag("-std=c++17")
        .compile("openvr_bridge");
    println!("cargo:rustc-link-lib=dl");
    println!("cargo:rustc-link-lib=vulkan");
    println!("cargo:rustc-link-lib=wayland-client");
    // headset_preview.cpp finds the toplevel wl_surface through GTK/GDK.
    for lib in ["gtk-3", "gdk-3", "gobject-2.0"] {
        println!("cargo:rustc-link-lib={lib}");
    }
    println!("cargo:rerun-if-changed=native/openvr.h");
    println!("cargo:rerun-if-changed=native/bridge.cpp");
    for file in ["cpp", "hpp", "vert", "frag"] {
        println!("cargo:rerun-if-changed=native/headset_preview.{file}");
    }
    tauri_build::build();
}
