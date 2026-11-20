//! Embeds the git commit in the firmware as `GIT_VERSION` (e.g. "ffbcdd2", or "ffbcdd2-dirty"
//! when built with uncommitted changes), shown on screen next to the package version.
//!
//! Uses `git describe` (from PATH, or the default Git for Windows install: on this PC cargo
//! usually runs from PowerShell, where git is not on PATH). Without git it reads the commit from
//! `.git` directly (no dirty flag then).

use std::fs;
use std::process::Command;

/// `git describe`, trying PATH first, then the default Git for Windows location.
fn from_git() -> Option<String> {
    ["git", "C:/Program Files/Git/cmd/git.exe"].iter().find_map(|git| {
        let o = Command::new(git).args(["describe", "--always", "--dirty", "--abbrev=7"]).output().ok()?;
        o.status.success().then(|| String::from_utf8_lossy(&o.stdout).trim().to_string())
    })
}

/// Commit hash from .git/HEAD, following a branch ref (loose or in packed-refs).
fn from_files() -> Option<String> {
    let head = fs::read_to_string(".git/HEAD").ok()?;
    let head = head.trim();
    let hash = match head.strip_prefix("ref: ") {
        None => head.to_string(),
        Some(r) => match fs::read_to_string(format!(".git/{r}")) {
            Ok(h) => h.trim().to_string(),
            Err(_) => fs::read_to_string(".git/packed-refs")
                .ok()?
                .lines()
                .find(|l| l.ends_with(&format!(" {r}")))?
                .split(' ')
                .next()?
                .to_string(),
        },
    };
    (hash.len() >= 7).then(|| hash[..7].to_string())
}

fn main() {
    let version = from_git().or_else(from_files).unwrap_or_else(|| "unknown".into());
    println!("cargo:rustc-env=GIT_VERSION={version}");
    // New commits move HEAD or the branch ref; edits change the dirty flag.
    println!("cargo:rerun-if-changed=.git/HEAD");
    println!("cargo:rerun-if-changed=.git/refs");
    println!("cargo:rerun-if-changed=.git/packed-refs");
    println!("cargo:rerun-if-changed=.git/index");
    println!("cargo:rerun-if-changed=src");
    println!("cargo:rerun-if-changed=Cargo.toml");
}
