// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
use crate::pairing_protocol::{Device, Radio, Transport, LEFT, RIGHT};
use serde::Serialize;
use std::{
    fs::{self, File, OpenOptions},
    os::fd::AsRawFd,
    sync::{mpsc, Mutex},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

#[derive(Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Status {
    stage: String,
    message: String,
    candidates: Vec<Candidate>,
    hand: String,
}

#[derive(Clone, Serialize)]
struct Candidate {
    id: String,
}

enum Request {
    Pair(u64),
    Cancel,
}

#[derive(Default)]
struct Session {
    status: Status,
    sender: Option<mpsc::Sender<Request>>,
    worker: Option<JoinHandle<()>>,
}
static SESSION: Mutex<Session> = Mutex::new(Session {
    status: Status {
        stage: String::new(),
        message: String::new(),
        candidates: Vec::new(),
        hand: String::new(),
    },
    sender: None,
    worker: None,
});

fn publish(stage: &str, message: impl Into<String>, devices: &[Device]) {
    if let Ok(mut session) = SESSION.lock() {
        session.status = Status {
            stage: stage.into(),
            message: message.into(),
            candidates: devices
                .iter()
                .map(|device| Candidate {
                    id: format!("{:016x}", device.id),
                })
                .collect(),
            hand: session.status.hand.clone(),
        };
    }
}

struct Hid(File);
impl Hid {
    fn open() -> Result<Self, String> {
        if radio_in_use() {
            return Err("Close SteamVR and Monado before pairing a controller.".into());
        }
        let mut paths = Vec::new();
        for entry in fs::read_dir("/sys/class/hidraw")
            .map_err(|e| e.to_string())?
            .flatten()
        {
            let Ok(device) = fs::canonicalize(entry.path().join("device")) else {
                continue;
            };
            let interface = device.ancestors().any(|path| {
                fs::read_to_string(path.join("bInterfaceNumber")).is_ok_and(|s| s.trim() == "06")
            });
            let headset = device.ancestors().any(|path| {
                fs::read_to_string(path.join("idVendor")).is_ok_and(|s| s.trim() == "2833")
                    && fs::read_to_string(path.join("idProduct")).is_ok_and(|s| s.trim() == "0051")
            });
            if interface && headset {
                paths.push(std::path::Path::new("/dev").join(entry.file_name()));
            }
        }
        if paths.len() != 1 {
            return Err(if paths.is_empty() {
                "Rift S headset is not connected."
            } else {
                "Connect only one Rift S headset before pairing."
            }
            .into());
        }
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(&paths[0])
            .map_err(|e| {
                format!("Cannot open headset radio. Check Rift S device permissions: {e}")
            })?;
        unsafe extern "C" {
            fn flock(fd: i32, operation: i32) -> i32;
        }
        if unsafe { flock(file.as_raw_fd(), 2 | 4) } < 0 {
            return Err("Another pairing session owns the headset radio.".into());
        }
        Ok(Self(file))
    }
}

impl Transport for Hid {
    fn get(&mut self, report: u8) -> Result<Vec<u8>, String> {
        let mut data = vec![0u8; 200];
        data[0] = report;
        let size = feature(&self.0, 0x07, &mut data)?;
        data.truncate(size);
        Ok(data)
    }
    fn set(&mut self, data: &[u8]) -> Result<(), String> {
        // hidraw locks are advisory; the driver does not participate in them.
        // Check again before every write if SteamVR was started during the scan.
        if radio_in_use() {
            return Err("A VR runtime started during pairing. Close it before retrying.".into());
        }
        let mut data = data.to_vec();
        let size = feature(&self.0, 0x06, &mut data)?;
        if size != data.len() {
            return Err("Incomplete headset radio write. Pairing state may have changed.".into());
        }
        Ok(())
    }
}

fn feature(file: &File, operation: u32, data: &mut [u8]) -> Result<usize, String> {
    unsafe extern "C" {
        fn ioctl(fd: i32, request: std::ffi::c_ulong, ...) -> i32;
    }
    // Linux HIDIOCGFEATURE/HIDIOCSFEATURE, both _IOC_READ | _IOC_WRITE.
    let request = (3u32 << 30) | ((data.len() as u32) << 16) | (u32::from(b'H') << 8) | operation;
    let result = unsafe {
        ioctl(
            file.as_raw_fd(),
            request as std::ffi::c_ulong,
            data.as_mut_ptr(),
        )
    };
    if result < 0 {
        Err(format!(
            "Headset USB operation failed: {}",
            std::io::Error::last_os_error()
        ))
    } else {
        Ok(result as usize)
    }
}

fn run(kind: u32, receiver: mpsc::Receiver<Request>) {
    let result = (|| {
        let mut radio = Radio(Hid::open()?);
        let previous = radio.mode()?;
        // Restore the original mode even when entering discovery only partly succeeds.
        let outcome = (|| {
            radio.set_mode(1)?;
            let deadline = Instant::now() + Duration::from_secs(90);
            loop {
                let paired = radio.list(0x11)?;
                let candidates: Vec<_> = radio
                    .list(0x0c)?
                    .into_iter()
                    .filter(|device| {
                        device.kind == kind
                            && device.id != 0
                            && !device.corrupt
                            && !paired.iter().any(|old| old.id == device.id)
                    })
                    .collect();
                publish(
                    if candidates.is_empty() {
                        "searching"
                    } else {
                        "ready"
                    },
                    if candidates.is_empty() {
                        "Waiting for a controller."
                    } else {
                        "Select the controller to pair."
                    },
                    &candidates,
                );
                match receiver.recv_timeout(Duration::from_millis(500)) {
                    Ok(Request::Cancel) | Err(mpsc::RecvTimeoutError::Disconnected) => {
                        return Ok(false)
                    }
                    Ok(Request::Pair(id)) => {
                        let selected = candidates
                            .iter()
                            .find(|device| device.id == id)
                            .ok_or("Controller is no longer available. Scan again.")?;
                        publish(
                            "pairing",
                            "Pairing controller. Keep Settings open and SteamVR closed.",
                            &[],
                        );
                        radio.pair(selected).map_err(|error| format!("{error}. Pairing did not finish; the saved pairing list may have changed."))?;
                        return Ok(true);
                    }
                    Err(mpsc::RecvTimeoutError::Timeout) => {}
                }
                if Instant::now() >= deadline {
                    return Err("No controller selected within 90 seconds. Try again.".into());
                }
            }
        })();
        let restore = radio.set_mode(previous);
        match (outcome, restore) {
            (Ok(paired), Ok(())) => Ok(paired),
            (Err(error), Ok(())) => Err(error),
            (outcome, Err(error)) => Err(format!("{} Radio mode could not be restored: {error}. Reconnect the headset before starting SteamVR.",
                match outcome { Ok(true) => "Controller was added to the pairing list.".into(), Ok(false) => "Scan cancelled.".into(), Err(error) => error })),
        }
    })();
    match result {
        Ok(true) => publish(
            "complete",
            "Controller added to the headset pairing list. You can start SteamVR.",
            &[],
        ),
        Ok(false) => publish("cancelled", "Pairing scan cancelled.", &[]),
        Err(error) => publish("failed", error, &[]),
    }
}

#[tauri::command]
pub fn controller_pairing_status() -> Result<Status, String> {
    Ok(SESSION.lock().map_err(|e| e.to_string())?.status.clone())
}

#[tauri::command]
pub fn controller_pairing_start(hand: String) -> Result<(), String> {
    let kind = match hand.as_str() {
        "left" => LEFT,
        "right" => RIGHT,
        _ => return Err("Invalid controller hand".into()),
    };
    if radio_in_use() {
        return Err("Close SteamVR and Monado before pairing a controller.".into());
    }
    let mut session = SESSION.lock().map_err(|e| e.to_string())?;
    if session
        .worker
        .as_ref()
        .is_some_and(|worker| !worker.is_finished())
    {
        return Err("A controller pairing session is already running.".into());
    }
    let (sender, receiver) = mpsc::channel();
    session.sender = Some(sender);
    session.status = Status {
        stage: "searching".into(),
        message: "Opening headset radio.".into(),
        candidates: vec![],
        hand,
    };
    session.worker = Some(thread::spawn(move || run(kind, receiver)));
    Ok(())
}

fn radio_in_use() -> bool {
    ["vrserver", "monado-service", "monado-cli"]
        .iter()
        .any(|name| super::process_pid(name).is_some())
}

#[tauri::command]
pub fn controller_pairing_select(id: String) -> Result<(), String> {
    let mut session = SESSION.lock().map_err(|e| e.to_string())?;
    if session.status.stage != "ready"
        || !session
            .status
            .candidates
            .iter()
            .any(|candidate| candidate.id == id)
    {
        return Err("Controller is no longer available.".into());
    }
    let id = u64::from_str_radix(&id, 16).map_err(|e| e.to_string())?;
    session
        .sender
        .as_ref()
        .ok_or("No pairing session")?
        .send(Request::Pair(id))
        .map_err(|e| e.to_string())?;
    session.status.stage = "pairing".into();
    session.status.message = "Pairing controller.".into();
    Ok(())
}

#[tauri::command]
pub fn controller_pairing_cancel() -> Result<(), String> {
    let session = SESSION.lock().map_err(|e| e.to_string())?;
    if !["searching", "ready"].contains(&session.status.stage.as_str()) {
        return Err("Wait for the pairing transaction to finish.".into());
    }
    session
        .sender
        .as_ref()
        .ok_or("No pairing session")?
        .send(Request::Cancel)
        .map_err(|e| e.to_string())
}

pub fn shutdown() {
    let worker = if let Ok(mut session) = SESSION.lock() {
        if let Some(sender) = session.sender.take() {
            let _ = sender.send(Request::Cancel);
        }
        session.worker.take()
    } else {
        None
    };
    if let Some(worker) = worker {
        let _ = worker.join();
    }
}
