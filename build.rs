//! Build-time checks.

fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    // Exactly one board feature. Counting the enabled `board-*` features,
    // rather than writing a guard per pair of boards, keeps this correct as
    // boards are added.
    let mut boards: Vec<String> = std::env::vars()
        .filter_map(|(k, _)| {
            k.strip_prefix("CARGO_FEATURE_BOARD_")
                .map(|b| format!("board-{}", b.to_lowercase().replace('_', "-")))
        })
        .collect();
    boards.sort();
    let error = match boards.len() {
        1 => return,
        0 => "no board selected".to_string(),
        _ => format!("more than one board selected: {}", boards.join(", ")),
    };
    eprintln!(
        "error: {error}. Enable exactly one board-* feature, e.g.\n  \
         cargo build --release --no-default-features --features board-xiao-c6\n\
         Boards are listed in Cargo.toml [features]."
    );
    std::process::exit(1);
}
