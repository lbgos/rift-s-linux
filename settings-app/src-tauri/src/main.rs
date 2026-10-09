#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use chrono::{DateTime, Local};
use serde::Serialize;
use serde_json::{json, Value};
use std::{
    ffi::{CStr, CString},
    fs,
    path::{Path, PathBuf},
    process::{Command, Stdio},
    sync::{
        atomic::{AtomicU32, Ordering},
        Mutex,
    },
    time::{Duration, SystemTime},
};
use tungstenite::{client::IntoClientRequest, Message};

unsafe extern "C" {
    fn rifts_snapshot(path: *const std::ffi::c_char) -> *const std::ffi::c_char;
    fn rifts_action(action: i32) -> i32;
    fn rifts_shutdown();
}
static VR: Mutex<()> = Mutex::new(());
static SERVER_PID: AtomicU32 = AtomicU32::new(0);

fn home() -> Result<PathBuf, String> {
    std::env::var_os("HOME")
        .map(PathBuf::from)
        .ok_or("Home directory unavailable".into())
}
fn steam_home() -> Result<PathBuf, String> {
    let h = home()?;
    [h.join(".local/share/Steam"), h.join(".steam/steam")]
        .into_iter()
        .find(|p| p.join("steamapps/common/SteamVR").is_dir())
        .ok_or("SteamVR is not installed".into())
}
fn process_pid(name: &str) -> Option<String> {
    let out = Command::new("pgrep")
        .args(["-u", &unsafe { libc_uid() }.to_string(), "-x", name])
        .output()
        .ok()?;
    String::from_utf8(out.stdout)
        .ok()?
        .split_whitespace()
        .next()
        .map(str::to_owned)
}
fn server_pid() -> Option<String> {
    process_pid("vrserver")
}
// Avoid matching another user's SteamVR session.
unsafe fn libc_uid() -> u32 {
    unsafe extern "C" {
        fn getuid() -> u32;
    }
    unsafe { getuid() }
}
fn vr_snapshot() -> Result<Value, String> {
    let pid = server_pid()
        .and_then(|p| p.parse::<u32>().ok())
        .unwrap_or(0);
    if SERVER_PID.swap(pid, Ordering::Relaxed) != pid {
        unsafe { rifts_shutdown() };
    }
    if pid == 0 {
        unsafe { rifts_shutdown() };
        return Ok(json!({}));
    }
    let path = steam_home()?.join("steamapps/common/SteamVR/bin/linux64/libopenvr_api.so");
    let path = CString::new(path.to_string_lossy().as_bytes()).map_err(|e| e.to_string())?;
    let value = unsafe { CStr::from_ptr(rifts_snapshot(path.as_ptr())) };
    serde_json::from_slice(value.to_bytes()).map_err(|e| e.to_string())
}

fn bench_mode() -> bool {
    server_pid()
        .and_then(|pid| fs::read(format!("/proc/{pid}/environ")).ok())
        .is_some_and(|env| {
            env.split(|b| *b == 0).any(|item| {
                item.strip_prefix(b"MONADO_STEAMVR_BENCH=")
                    .is_some_and(|value| !matches!(value, b"" | b"0" | b"false" | b"off"))
            })
        })
}

#[derive(Debug, Serialize, PartialEq)]
#[serde(rename_all = "camelCase")]
struct Dimensions {
    width: f64,
    depth: f64,
    area: f64,
}

// Boundary dimensions and area use the live collision polygon. With no walls,
// fall back to the ordered play-area edges, including a rotated rectangle.
fn dimensions(rect: &[[f64; 3]], walls: &[[[f64; 3]; 4]]) -> Option<Dimensions> {
    if rect.len() != 4 || !rect.iter().flatten().all(|v| v.is_finite()) {
        return None;
    }
    let distance = |a: [f64; 3], b: [f64; 3]| (a[0] - b[0]).hypot(a[2] - b[2]);
    let a = distance(rect[0], rect[1]);
    let b = distance(rect[1], rect[2]);
    if a <= 0.0 || b <= 0.0 {
        return None;
    }
    let (width, depth) = if walls.is_empty() {
        (a, b)
    } else {
        let xs = walls.iter().flat_map(|q| [q[0][0], q[3][0]]);
        let zs = walls.iter().flat_map(|q| [q[0][2], q[3][2]]);
        let extent = |values: Vec<f64>| {
            values.iter().copied().fold(f64::NEG_INFINITY, f64::max)
                - values.iter().copied().fold(f64::INFINITY, f64::min)
        };
        (extent(xs.collect()), extent(zs.collect()))
    };
    let area = if walls.is_empty() {
        a * b
    } else {
        if !walls.iter().flatten().flatten().all(|v| v.is_finite()) {
            return None;
        }
        walls
            .iter()
            .map(|q| q[0][0] * q[3][2] - q[3][0] * q[0][2])
            .sum::<f64>()
            .abs()
            / 2.0
    };
    Some(Dimensions { width, depth, area })
}

#[derive(Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct LogFile {
    label: String,
    path: PathBuf,
    size: Option<u64>,
}
fn list_logs(candidates: &[(String, PathBuf)]) -> Vec<LogFile> {
    candidates
        .iter()
        .map(|(label, path)| LogFile {
            label: label.clone(),
            path: path.clone(),
            size: fs::metadata(path)
                .ok()
                .filter(|m| m.is_file())
                .map(|m| m.len()),
        })
        .collect()
}
fn log_files(steam: &Path) -> Vec<LogFile> {
    list_logs(&[
        ("Driver / Monado".into(), steam.join("logs/vrserver.txt")),
        (
            "Room setup guard".into(),
            steam.join("logs/vrclient_monado-roomsetup-guard.txt"),
        ),
        (
            "SteamVR compositor".into(),
            steam.join("logs/vrcompositor.txt"),
        ),
    ])
}
fn journal_size() -> Option<u64> {
    let machine = fs::read_to_string("/etc/machine-id").ok()?;
    let mut total = None;
    for root in ["/var/log/journal", "/run/log/journal"] {
        let Ok(entries) = fs::read_dir(Path::new(root).join(machine.trim())) else {
            continue;
        };
        for file in entries
            .flatten()
            .filter(|f| f.path().extension().is_some_and(|x| x == "journal"))
        {
            if let Ok(m) = file.metadata() {
                total = Some(total.unwrap_or(0) + m.len());
            }
        }
    }
    total
}
fn driver_version() -> Option<String> {
    let pid = server_pid()?;
    let maps = fs::read_to_string(format!("/proc/{pid}/maps")).ok()?;
    let path = maps
        .lines()
        .filter_map(|l| l.split_whitespace().last())
        .find(|p| p.ends_with("/driver_monado.so"))?;
    let sum = Command::new("sha256sum").arg(path).output().ok()?;
    let sum = String::from_utf8(sum.stdout)
        .ok()?
        .split_whitespace()
        .next()?
        .to_owned();
    // Short hash of the driver SteamVR actually loaded.
    Some(sum.chars().take(12).collect())
}
fn steam_version(steam: &Path) -> Option<String> {
    let output = Command::new(steam.join("steamapps/common/SteamVR/bin/linux64/vrcmd"))
        .arg("--version")
        .output()
        .ok();
    if let Some(output) = output.filter(|o| o.status.success()) {
        let text = String::from_utf8_lossy(&output.stdout);
        if let Some(version) = text.lines().find(|line| {
            let parts: Vec<_> = line.trim().split('.').collect();
            parts.len() == 3
                && parts
                    .iter()
                    .all(|p| !p.is_empty() && p.chars().all(|c| c.is_ascii_digit()))
        }) {
            return Some(version.trim().to_owned());
        }
    }
    // Version is reported by the installed lighthouse driver at every startup.
    let bytes = fs::read(steam.join("logs/vrserver.txt")).ok()?;
    let text = String::from_utf8_lossy(&bytes);
    text.lines().rev().find_map(|line| {
        line.split("lighthouse: version ")
            .nth(1)
            .map(|v| v.trim().to_owned())
    })
}
fn physical_proximity() -> Option<bool> {
    let out = Command::new("journalctl")
        .args([
            "--user",
            "-u",
            "steam-ui",
            "--since",
            "10 seconds ago",
            "--no-pager",
            "-o",
            "cat",
        ])
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    String::from_utf8(out.stdout)
        .ok()?
        .lines()
        .rev()
        .find_map(|l| {
            l.split("HMD physical proximity=")
                .nth(1)
                .and_then(|s| match s.chars().next() {
                    Some('0') => Some(false),
                    Some('1') => Some(true),
                    _ => None,
                })
        })
}

#[tauri::command]
async fn snapshot() -> Result<Value, String> {
    tauri::async_runtime::spawn_blocking(|| {
        let _lock = VR.lock().map_err(|e| e.to_string())?;
        let mut value = vr_snapshot()?;
        let steam = steam_home().ok();
        let rect: Vec<[f64; 3]> = serde_json::from_value(value["rect"].clone()).unwrap_or_default();
        let walls: Vec<[[f64; 3]; 4]> =
            serde_json::from_value(value["walls"].clone()).unwrap_or_default();
        value["dimensions"] =
            serde_json::to_value(dimensions(&rect, &walls)).map_err(|e| e.to_string())?;
        value["saved"] = steam
            .as_ref()
            .and_then(|s| fs::metadata(s.join("config/chaperone_info.vrchap")).ok())
            .and_then(|m| m.modified().ok())
            .map(|t| {
                DateTime::<Local>::from(t)
                    .format("%-d %b %Y, %H:%M")
                    .to_string()
            })
            .into();
        value["logs"] = steam
            .as_ref()
            .map(|s| serde_json::to_value(log_files(s)).unwrap_or_default())
            .unwrap_or(json!([]));
        value["journalSize"] = journal_size().into();
        value["driver"] = driver_version().into();
        value["steamVersion"] = steam.as_ref().and_then(|s| steam_version(s)).into();
        let ipd = home()
            .ok()
            .and_then(|h| fs::read_to_string(h.join("rift-s-ipd.conf")).ok())
            .and_then(|s| s.trim().parse::<f64>().ok())
            .filter(|x| x.is_finite() && (50.0..=80.0).contains(x));
        value["ipdSetting"] = ipd.into();
        value["proximity"] = physical_proximity().into();
        let bench = bench_mode();
        value["bench"] = bench.into();
        if bench {
            // Bench poses are synthetic. Never present them as physical tracking.
            if let Some(devices) = value["devices"].as_array_mut() {
                for device in devices {
                    device["tracking"] = Value::Null;
                }
            }
            value["head"] = Value::Null;
            value["idle"] = false.into();
        }
        Ok(value)
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
fn set_ipd(mm: f64) -> Result<(), String> {
    if !mm.is_finite() || !(50.0..=80.0).contains(&mm) {
        return Err("IPD must be between 50 and 80 mm".into());
    }
    // The active Rift S driver polls this file every 500 ms.
    let h = home()?;
    let tmp = h.join(".rift-s-ipd.conf.settings-tmp");
    fs::write(&tmp, format!("{mm:.1}\n")).map_err(|e| e.to_string())?;
    fs::rename(tmp, h.join("rift-s-ipd.conf")).map_err(|e| e.to_string())
}

#[tauri::command]
fn vr_action(action: &str) -> Result<String, String> {
    let _lock = VR.lock().map_err(|e| e.to_string())?;
    vr_snapshot()?;
    if bench_mode() && matches!(action, "room" | "recenter") {
        return Err("Unavailable while SteamVR is in bench mode".into());
    }
    let (id, response) = match action {
        "mirror" => (1, "SteamVR mirror opened."),
        "room" => (2, "Room setup is running in the headset."),
        "recenter" => (3, "Recentered."),
        _ => return Err("Unknown action".into()),
    };
    if unsafe { rifts_action(id) } == 0 {
        Err("SteamVR interface unavailable".into())
    } else {
        if action == "room" {
            start_native_room_setup()?;
        }
        Ok(response.into())
    }
}

fn start_native_room_setup() -> Result<(), String> {
    // SteamVR 2.17's dashboard handler enters its own native room setup at
    // step 7. No CEF debugger, copied web assets or configuration edits.
    let mut request = "ws://127.0.0.1:27062"
        .into_client_request()
        .map_err(|_| "Unable to construct dashboard request")?;
    request.headers_mut().insert(
        "Origin",
        "http://127.0.0.1:27062"
            .parse()
            .map_err(|_| "Invalid local origin")?,
    );
    let (mut socket, _) =
        tungstenite::connect(request).map_err(|_| "SteamVR dashboard connection unavailable")?;
    if let tungstenite::stream::MaybeTlsStream::Plain(stream) = socket.get_mut() {
        stream
            .set_read_timeout(Some(Duration::from_secs(3)))
            .map_err(|_| "Unable to set dashboard timeout")?;
        stream
            .set_write_timeout(Some(Duration::from_secs(3)))
            .map_err(|_| "Unable to set dashboard timeout")?;
    }
    let address = format!("rifts_settings/{}", std::process::id());
    socket
        .send(Message::Text(format!("mailbox_open {address}").into()))
        .map_err(|_| "Unable to register dashboard client")?;
    let wait = json!({"type":"request_mailbox_registration_notification", "mailbox_name":"vrwebui_dashboardstore", "returnAddress":address, "message_id":1});
    socket
        .send(Message::Text(
            format!("mailbox_send web_server_mailbox {wait}").into(),
        ))
        .map_err(|_| "Unable to query dashboard")?;
    let deadline = std::time::Instant::now() + Duration::from_secs(3);
    loop {
        if std::time::Instant::now() > deadline {
            return Err("SteamVR dashboard is not ready".into());
        }
        let message = socket
            .read()
            .map_err(|_| "SteamVR dashboard is not ready")?;
        if let Message::Text(text) = message {
            let value: Value =
                serde_json::from_str(&text).map_err(|_| "Invalid dashboard response")?;
            if value["type"] == "mailbox_registered" {
                break;
            }
        }
    }
    socket
        .send(Message::Text(
            "mailbox_send vrwebui_dashboardstore {\"type\":\"guided_tour_room_setup\"}".into(),
        ))
        .map_err(|_| "Unable to start native room setup")?;
    let _ = socket.close(None);
    Ok(())
}

fn checked(command: &mut Command) -> Result<(), String> {
    let status = command
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map_err(|e| e.to_string())?;
    if status.success() {
        Ok(())
    } else {
        Err(format!("Command failed with {status}"))
    }
}

#[tauri::command]
async fn export_logs() -> Result<String, String> {
    tauri::async_runtime::spawn_blocking(|| {
        let h = home()?;
        let steam = steam_home()?;
        let stamp = Local::now().format("%Y%m%d-%H%M%S-%f");
        let staging = h.join(format!(".rift-s-logs-{stamp}"));
        fs::create_dir(&staging).map_err(|e| e.to_string())?;
        let archive = h.join(format!("rift-s-logs-{stamp}.tar.gz"));
        let result = (|| {
            for log in log_files(&steam).into_iter().filter(|l| l.size.is_some()) {
                if let Some(name) = log.path.file_name() {
                    fs::copy(&log.path, staging.join(name)).map_err(|e| e.to_string())?;
                }
            }
            let file = fs::File::create(staging.join("journal.txt")).map_err(|e| e.to_string())?;
            let status = Command::new("journalctl")
                .args(["--user", "-b", "-u", "steam-ui", "-u", "monado-service", "--since", "1 hour ago", "-n", "10000", "--no-pager", "-o", "short-iso"])
                .stdout(file)
                .stderr(Stdio::null())
                .status()
                .map_err(|e| e.to_string())?;
            if !status.success() {
                return Err("Unable to export the user journal".into());
            }
            fs::write(staging.join("sources.txt"), "SteamVR server and compositor logs; Monado room setup guard log.\nJournal: current boot, steam-ui and monado-service user units, last hour, up to 10000 entries.\n").map_err(|e| e.to_string())?;
            checked(
                Command::new("tar")
                    .arg("-czf")
                    .arg(&archive)
                    .arg("-C")
                    .arg(&staging)
                    .arg("."),
            )?;
            Ok(archive.to_string_lossy().into_owned())
        })();
        let _ = fs::remove_dir_all(staging);
        if result.is_err() {
            let _ = fs::remove_file(archive);
        }
        result
    })
    .await
    .map_err(|e| e.to_string())?
}

#[tauri::command]
async fn restart_steamvr(confirmed: bool) -> Result<String, String> {
    if !confirmed {
        return Err("Confirmation required".into());
    }
    tauri::async_runtime::spawn_blocking(|| {
        if bench_mode() {
            return Err("Restart blocked: SteamVR is in bench mode".into());
        }
        let checked_pid = server_pid().ok_or("SteamVR is not running")?;
        {
            let _lock = VR.lock().map_err(|e| e.to_string())?;
            // Five fresh samples. Unknown proximity or invalid/moving pose blocks restart.
            for _ in 0..5 {
                if bench_mode() || server_pid().as_ref() != Some(&checked_pid) {
                    return Err("Restart blocked: SteamVR session changed".into());
                }
                let value = vr_snapshot()?;
                if physical_proximity() != Some(false) || value["idle"] != true {
                    return Err(
                        "Restart blocked: headset is worn, moving, or state is unavailable".into(),
                    );
                }
                std::thread::sleep(Duration::from_millis(200));
            }
            unsafe { rifts_shutdown() };
        }
        // Ask the runtime to quit gracefully while Steam stays available. Never
        // kill Steam or the compositor, and never rewrite runtime settings.
        if server_pid().as_ref() != Some(&checked_pid) {
            return Err("Restart blocked: SteamVR session changed".into());
        }
        checked(Command::new("kill").args(["-TERM", &checked_pid]))?;
        let deadline = SystemTime::now() + Duration::from_secs(20);
        while server_pid().is_some() || process_pid("vrcompositor").is_some() {
            if SystemTime::now() > deadline {
                return Err("SteamVR did not stop within 20 seconds".into());
            }
            std::thread::sleep(Duration::from_millis(250));
        }
        let launcher = home()?.join("src/steamvr-fixes-tools/start-steamvr.py");
        if launcher.is_file() {
            checked(Command::new("python").arg(launcher))?;
        } else {
            checked(Command::new("steam").arg("steam://rungameid/250820"))?;
        }
        Ok("SteamVR restarted.".into())
    })
    .await
    .map_err(|e| e.to_string())?
}

fn main() {
    // Read-only evidence mode uses exactly the same backend as the application.
    if std::env::args().any(|a| a == "--snapshot") {
        let result = tauri::async_runtime::block_on(snapshot());
        match result {
            Ok(v) => println!("{v}"),
            Err(e) => {
                eprintln!("{e}");
                std::process::exit(1);
            }
        }
        unsafe { rifts_shutdown() };
        return;
    }
    let mut context = tauri::generate_context!();
    let args: Vec<String> = std::env::args().collect();
    let arg = |name: &str| {
        args.windows(2)
            .find(|a| a[0] == name)
            .map(|a| a[1].as_str())
    };
    let window = &mut context.config_mut().app.windows[0];
    if let Some(view @ ("headset" | "room" | "settings")) = arg("--view") {
        window.url = tauri::WebviewUrl::App(format!("index.html#{view}").into());
    }
    if let Some(width) = arg("--width")
        .and_then(|s| s.parse::<f64>().ok())
        .filter(|x| x.is_finite() && *x >= 1000.0)
    {
        window.width = width;
    }
    if let Some(height) = arg("--height")
        .and_then(|s| s.parse::<f64>().ok())
        .filter(|x| x.is_finite() && *x >= 650.0)
    {
        window.height = height;
    }
    tauri::Builder::default()
        .invoke_handler(tauri::generate_handler![
            snapshot,
            set_ipd,
            vr_action,
            export_logs,
            restart_steamvr
        ])
        .build(context)
        .expect("Unable to start Rift S settings")
        .run(|_, event| {
            if matches!(event, tauri::RunEvent::Exit) {
                if let Ok(_lock) = VR.lock() {
                    unsafe { rifts_shutdown() };
                }
            }
        });
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::UNIX_EPOCH;
    #[test]
    fn rotated_play_area_and_irregular_boundary() {
        let rect = [[0., 0., 0.], [2., 0., 2.], [1., 0., 3.], [-1., 0., 1.]];
        let polygon = [
            [0., 0., 0.],
            [3., 0., 0.],
            [3., 0., 2.],
            [1., 0., 2.],
            [0., 0., 1.],
        ];
        let walls: Vec<_> = (0..polygon.len())
            .map(|i| {
                let a = polygon[i];
                let b = polygon[(i + 1) % polygon.len()];
                [a, [a[0], 2., a[2]], [b[0], 2., b[2]], b]
            })
            .collect();
        let d = dimensions(&rect, &walls).unwrap();
        let play_area = dimensions(&rect, &[]).unwrap();
        assert!((play_area.width - 8_f64.sqrt()).abs() < 1e-9);
        assert!((play_area.depth - 2_f64.sqrt()).abs() < 1e-9);
        assert_eq!(d.width, 3.0);
        assert_eq!(d.depth, 2.0);
        assert!((d.area - 5.5).abs() < 1e-9);
        let reversed: Vec<_> = walls.iter().map(|q| [q[3], q[2], q[1], q[0]]).collect();
        assert_eq!(dimensions(&rect, &reversed), Some(d));
        assert!(dimensions(&[], &walls).is_none());
        assert!(dimensions(&[[f64::NAN; 3]; 4], &[]).is_none());
    }
    #[test]
    fn logs_report_actual_bytes_and_missing_files() {
        let p = std::env::temp_dir().join(format!(
            "rifts-log-test-{}-{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        fs::create_dir(&p).unwrap();
        fs::write(p.join("driver.log"), b"actual log\n").unwrap();
        let rows = list_logs(&[
            ("Driver".into(), p.join("driver.log")),
            ("Missing".into(), p.join("missing")),
            ("Directory".into(), p.clone()),
        ]);
        assert_eq!(rows[0].size, Some(11));
        assert_eq!(rows[1].size, None);
        assert_eq!(rows[2].size, None);
        assert_eq!(rows[0].path, p.join("driver.log"));
        fs::remove_dir_all(p).unwrap();
    }
}
