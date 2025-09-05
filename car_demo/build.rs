use std::{env, path::PathBuf};

// build the c library from source
#[cfg(not(feature = "link-system"))]
fn build_or_link() {
    let mut build = cc::Build::new();
    build.file("../build_libccar.c").include("..");
    build.compile("ccar");
}

// link the pre-compiled library
#[cfg(feature = "link-system")]
fn build_or_link() {
    println!("cargo:rerun-if-env-changed=LIBCCAR_LIB_DIR");
    println!("cargo:rerun-if-env-changed=LIBCCAR_LINK_NAME");

    let link_name = env::var("LIBCCAR_LINK_NAME").unwrap_or_else(|_| "ccar".to_string());
    let lib_dir = env::var("LIBCCAR_LIB_DIR").unwrap_or_else(|_| "lib".to_string());

    // Make an absolute path (best effort)
    let abs = fs::canonicalize(&lib_dir).unwrap_or_else(|_| PathBuf::from(&lib_dir));

    // Linker search path + link name
    println!("cargo:rustc-link-search=native={}", abs.display());
    println!("cargo:rustc-link-lib=dylib={}", link_name);

    // Embed rpath so the binary finds libccar.so at runtime without env vars
    // Absolute path:
    #[cfg(target_os = "linux")]
    {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", abs.display());
        // Dev-friendly relative path: from target/release to project/lib and cwd
        println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/../../lib");
        println!("cargo:rustc-link-arg=-Wl,-rpath,.");
    }
    #[cfg(target_os = "macos")]
    {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", abs.display());
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", abs.display());
        // Dev-friendly relative path: from target/release to project/lib and cwd
        println!("cargo:rustc-link-arg=-Wl,-rpath,@executable_path/../../lib");
        println!("cargo:rustc-link-arg=-Wl,-rpath,.");
    }
    #[cfg(target_os = "windows")]
    {
        // Windows does not have rpath so the so must be in PATH or next to binary
        // This is just so the compiler sees the .dll
        println!("cargo:rustc-link-search=native={}", abs.display());
        println!("cargo:rustc-link-lib=dylib=ccar");
    }
}

fn main() {
    println!("cargo:rerun-if-changed=../libccar.h");

    build_or_link();

    let bindings = bindgen::Builder::default()
        .header("../libccar.h")
        .allowlist_type("lcc_.*")
        .allowlist_function("lcc_.*")
        .allowlist_var("LCC_.*")
        .generate_comments(true)
        .derive_default(true)
        .constified_enum("lcc_.*")
        .generate()
        .expect("bindgen failed");
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out.join("bindings.rs"))
        .expect("write bindings");
}
