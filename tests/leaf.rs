//! **The crate boundary, enforced.**
//!
//! `bevy_orca` is the 2-D linear program over discs and nothing else. It takes `bevy_math` for
//! `Vec2` and, behind its `serde`/`reflect` features, `serde` and `bevy_reflect` for derives on data
//! that already exists — and nothing more. No ECS, no app, no renderer, and above all no game.
//!
//! That matters for a specific reason. Avoidance is only reusable if the CALLER decides when it runs
//! and what counts as a neighbour: those are gameplay decisions. The moment this crate takes
//! `bevy_app` it starts making them, and the moment it takes a game type it stops being a library at
//! all.
//!
//! These two tests are the ratchet: cheap, and they fail at the door rather than at link time.
//! Widening the list is a design decision, so it should cost a deliberate edit here.

use std::path::{Path, PathBuf};

/// Everything `bevy_orca` is allowed to depend on. Data and arithmetic, nothing that draws.
const ALLOWED_DEPS: &[&str] = &["bevy_math", "serde", "bevy_reflect"];

/// Crate names that would mean the boundary has been crossed, checked as substrings so
/// `bevy_math`/`bevy_ecs` are caught as readily as `bevy`.
/// Crate names that would mean the boundary has been crossed. `bevy_math` is glam types and is
/// allowed above; anything that draws, schedules, or knows what a game is, is not.
const FORBIDDEN_DEP_MARKERS: &[&str] =
    &["bevy_ecs", "bevy_app", "bevy_render", "avian", "wgpu", "winit", "emerge", "foundation_vs_slop"];

fn crate_root() -> PathBuf {
    // Cargo runs a test binary with the cwd set to its own package root.
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn rust_sources(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            rust_sources(&path, out);
        } else if path.extension().is_some_and(|e| e == "rs") {
            out.push(path);
        }
    }
}

/// Strip `//` line comments and `/* */` blocks so a doc comment *about* Bevy — of which this crate has
/// several, deliberately — is not mistaken for an import of it.
///
/// Deliberately simple: it does not track string literals, so a `"//"` inside a string would end the
/// line early. That is acceptable here because the only thing this feeds is a substring search for
/// crate names, and an over-eager strip can only ever cause a false *pass* on a line that already
/// contains no dependency.
fn strip_comments(src: &str) -> String {
    let mut out = String::with_capacity(src.len());
    let mut chars = src.chars().peekable();
    let mut in_block = false;
    while let Some(c) = chars.next() {
        if in_block {
            if c == '*' && chars.peek() == Some(&'/') {
                chars.next();
                in_block = false;
            }
            continue;
        }
        if c == '/' && chars.peek() == Some(&'/') {
            for n in chars.by_ref() {
                if n == '\n' {
                    out.push('\n');
                    break;
                }
            }
            continue;
        }
        if c == '/' && chars.peek() == Some(&'*') {
            chars.next();
            in_block = true;
            continue;
        }
        out.push(c);
    }
    out
}

#[test]
fn no_source_file_reaches_for_an_engine() {
    let src = crate_root().join("src");
    let mut files = Vec::new();
    rust_sources(&src, &mut files);
    assert!(
        !files.is_empty(),
        "expected to scan the crate's sources, found no .rs file under {}",
        src.display()
    );

    let mut offenders = Vec::new();
    for path in &files {
        let Ok(text) = std::fs::read_to_string(path) else {
            continue;
        };
        let code = strip_comments(&text);
        for (n, line) in code.lines().enumerate() {
            for marker in FORBIDDEN_DEP_MARKERS {
                if line.contains(marker) {
                    offenders.push(format!(
                        "{}:{} — {}",
                        path.strip_prefix(crate_root()).unwrap_or(path).display(),
                        n + 1,
                        line.trim()
                    ));
                }
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "bevy_orca must stay engine-free, but {} line(s) reference an engine crate.\n  {}\n\n\
         This crate is the avoidance layer the game, a simulation and any standalone host can share \
         without agreeing on a renderer or a schedule. If a type genuinely needs to cross the \
         boundary, the answer is a plain-data type here and a conversion on the far side — not a \
         dependency.",
        offenders.len(),
        offenders.join("\n  ")
    );
}

#[test]
fn the_dependency_list_stays_closed() {
    let manifest =
        std::fs::read_to_string(crate_root().join("Cargo.toml")).expect("bevy_orca must have a Cargo.toml");

    // Only the `[dependencies]` table — a dev-dependency on something heavier would be a different
    // (and much less alarming) conversation.
    let deps =
        manifest.split("[dependencies]").nth(1).expect("bevy_orca must declare a [dependencies] table");
    let deps = deps.split("\n[").next().unwrap_or(deps);

    for line in deps.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        // `.` is a separator too, so an inherited dependency written as the dotted key
        // `serde.workspace = true` reads as `serde` rather than as a crate called
        // `serde.workspace`. A crate name cannot contain a dot, so nothing real is truncated —
        // and the check keeps biting on the part that matters, the name before it.
        let name = line.split(['=', ' ', '.']).next().unwrap_or("");
        // A dependency line is `name = ...` or `name.workspace = true`. Anything else inside this
        // table is a CONTINUATION of a multi-line value — a feature array's entries, a closing
        // bracket — and is not a crate name.
        if name.is_empty() || !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-') {
            continue;
        }
        assert!(
            ALLOWED_DEPS.contains(&name),
            "bevy_orca declares `{name}`, which is not in its allowed set {ALLOWED_DEPS:?}.\n\
             Widening this is a design decision, not a convenience. If it is genuinely warranted, add \
             it to ALLOWED_DEPS in this test in the same commit, so the change is visible in review."
        );
        for marker in FORBIDDEN_DEP_MARKERS {
            assert!(
                !name.contains(marker),
                "bevy_orca declares `{name}` — the crate exists precisely so it does not depend on \
                 an engine."
            );
        }
    }
}
