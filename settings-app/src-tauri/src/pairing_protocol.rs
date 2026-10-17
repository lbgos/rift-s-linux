// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
//! Rift S wireless-host HID protocol. Lengths include the report ID.
//! Pairing uses discovery mode, a temporary connection, then an app restart.
//! This module has no firmware-update or unpair commands.

use std::time::{Duration, Instant};

pub const LEFT: u32 = 0x13001101;
pub const RIGHT: u32 = 0x13011101;

#[derive(Clone, Debug, PartialEq)]
pub struct Device {
    pub id: u64,
    pub kind: u32,
    pub corrupt: bool,
}

pub trait Transport {
    fn get(&mut self, report: u8) -> Result<Vec<u8>, String>;
    fn set(&mut self, data: &[u8]) -> Result<(), String>;
}

pub fn devices(data: &[u8], report: u8) -> Result<Vec<Device>, String> {
    let count_offset = if report == 0x11 { 1 } else { 2 };
    if data.first() != Some(&report) || data.len() <= count_offset {
        return Err("Invalid controller list report".into());
    }
    let count = usize::from(data[count_offset]);
    let start = count_offset + 1;
    if count > 4 || data.len() < start + count * 28 {
        return Err("Truncated or oversized controller list".into());
    }
    data[start..start + count * 28]
        .chunks_exact(28)
        .map(|record| {
            Ok(Device {
                id: u64::from_le_bytes(record[..8].try_into().unwrap()),
                kind: u32::from_le_bytes(record[8..12].try_into().unwrap()),
                corrupt: record[27] & 1 != 0,
            })
        })
        .collect()
}

pub struct Radio<T>(pub T);

impl<T: Transport> Radio<T> {
    fn idle(&mut self) -> Result<Vec<u8>, String> {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let data = self.0.get(0x0b)?;
            if data.len() < 7 || data[0] != 0x0b {
                return Err("Truncated wireless status".into());
            }
            if data[2] == 0 {
                return Ok(data);
            }
            if Instant::now() >= deadline {
                return Err("Headset radio is busy".into());
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    pub fn command(&mut self, data: &[u8]) -> Result<Vec<u8>, String> {
        let sequence = self.idle()?[1];
        // Never retry a write: an ambiguous USB failure may have committed it.
        self.0.set(data)?;
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let reply = self.idle()?;
            if reply[1] != sequence {
                let status = u32::from_le_bytes(reply[3..7].try_into().unwrap());
                return if status == 0 {
                    Ok(reply)
                } else {
                    Err(format!(
                        "Headset radio rejected command {:#04x}: {status:#x}",
                        data[0]
                    ))
                };
            }
            if Instant::now() >= deadline {
                return Err("Headset radio did not acknowledge the command".into());
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    pub fn mode(&mut self) -> Result<u8, String> {
        let data = self.0.get(0x0a)?;
        match data.as_slice() {
            [0x0a, mode @ 0..=2, ..] => Ok(*mode),
            _ => Err("Invalid headset radio mode".into()),
        }
    }

    pub fn set_mode(&mut self, mode: u8) -> Result<(), String> {
        self.command(&[0x0a, mode])?;
        if self.mode()? != mode {
            return Err("Headset radio mode did not change".into());
        }
        Ok(())
    }

    pub fn list(&mut self, report: u8) -> Result<Vec<Device>, String> {
        devices(&self.0.get(report)?, report)
    }

    pub fn pair(&mut self, selected: &Device) -> Result<(), String> {
        if ![LEFT, RIGHT].contains(&selected.kind) || selected.id == 0 || selected.corrupt {
            return Err("Controller type or app firmware is unsupported".into());
        }
        let whitelist = self.list(0x11)?;
        if whitelist.iter().any(|device| device.id == selected.id) {
            return Err("This controller is already paired".into());
        }
        if whitelist.len() == 4 {
            return Err("Headset pairing list is full. No existing pairing was removed.".into());
        }
        if !self.list(0x0c)?.contains(selected) {
            return Err(
                "Controller stopped advertising. Hold its pairing buttons and try again.".into(),
            );
        }
        let mut connect = vec![0x0d];
        connect.extend(selected.id.to_le_bytes());
        connect.extend(4000u16.to_le_bytes());
        if let Err(error) = self.command(&connect) {
            let cleanup = self.command(&[0x0e, 0]);
            return Err(match cleanup {
                Ok(_) => error,
                Err(cleanup) => format!("{error}. Temporary connection cleanup failed: {cleanup}"),
            });
        }
        let result: Result<(), String> = (|| {
            let mut pair = vec![0x0f];
            pair.extend(selected.kind.to_le_bytes());
            self.command(&pair)?;
            // Register 0x15 restarts the existing app, with no flash data.
            let mut restart = vec![0u8; 61];
            restart[0] = 0x13;
            restart[1..9].copy_from_slice(&selected.id.to_le_bytes());
            restart[9] = 0x15;
            restart[11..13].copy_from_slice(&1000u16.to_le_bytes());
            self.command(&restart)?;
            Ok(())
        })();
        // End the temporary radio connection on success and failure.
        // The disconnect report carries only the low byte of the target ID.
        let disconnect = self.command(&[0x0e, selected.id as u8]);
        match (result, disconnect) {
            (Err(error), Err(cleanup)) => {
                return Err(format!(
                    "{error}. Temporary connection cleanup failed: {cleanup}"
                ))
            }
            (Err(error), _) => return Err(error),
            (_, Err(error)) => return Err(error),
            _ => {}
        }
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            let saved = self.list(0x11)?;
            if whitelist.iter().any(|old| {
                !saved
                    .iter()
                    .any(|device| device.id == old.id && device.kind == old.kind)
            }) {
                return Err("An existing controller is missing from the pairing list. Stop and check the headset.".into());
            }
            if saved
                .iter()
                .any(|device| device.id == selected.id && device.kind == selected.kind)
            {
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(
                    "Pair command completed but the controller is absent from the pairing list"
                        .into(),
                );
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[test]
    fn rejects_malformed_lists() {
        for data in [vec![], vec![0x0c, 0], vec![0x0c, 0, 5], vec![0x0c, 0, 1]] {
            assert!(devices(&data, 0x0c).is_err());
        }
        let mut data = vec![0x11, 1];
        data.extend(42u64.to_le_bytes());
        data.extend(LEFT.to_le_bytes());
        data.extend([0; 16]);
        assert_eq!(
            devices(&data, 0x11).unwrap()[0],
            Device {
                id: 42,
                kind: LEFT,
                corrupt: false
            }
        );
    }

    struct Fake {
        reads: VecDeque<Vec<u8>>,
        writes: Vec<Vec<u8>>,
        fail_write: bool,
    }
    impl Transport for Fake {
        fn get(&mut self, report: u8) -> Result<Vec<u8>, String> {
            let data = self.reads.pop_front().expect("unexpected read");
            assert_eq!(data[0], report);
            Ok(data)
        }
        fn set(&mut self, data: &[u8]) -> Result<(), String> {
            self.writes.push(data.to_vec());
            if self.fail_write {
                Err("USB disconnected".into())
            } else {
                Ok(())
            }
        }
    }

    #[test]
    fn command_requires_new_sequence_and_checks_result() {
        let mut radio = Radio(Fake {
            reads: [vec![11, 255, 0, 0, 0, 0, 0], vec![11, 0, 0, 3, 0, 0, 0]].into(),
            writes: vec![],
            fail_write: false,
        });
        assert!(radio.command(&[10, 1]).unwrap_err().contains("rejected"));
        assert_eq!(radio.0.writes, [vec![10, 1]]);
    }

    #[test]
    fn ambiguous_write_failure_is_never_retried() {
        let mut radio = Radio(Fake {
            reads: [vec![11, 1, 0, 0, 0, 0, 0]].into(),
            writes: vec![],
            fail_write: true,
        });
        assert!(radio.command(&[10, 1]).is_err());
        assert_eq!(radio.0.writes.len(), 1);
    }

    fn list(report: u8, entries: &[Device]) -> Vec<u8> {
        let mut data = if report == 0x11 {
            vec![report, entries.len() as u8]
        } else {
            vec![report, 1, entries.len() as u8]
        };
        for device in entries {
            data.extend(device.id.to_le_bytes());
            data.extend(device.kind.to_le_bytes());
            data.extend([0; 15]);
            data.push(u8::from(device.corrupt));
        }
        data
    }

    #[test]
    fn pair_preserves_existing_entries_and_verifies_new_identity() {
        let old = Device {
            id: 3,
            kind: RIGHT,
            corrupt: false,
        };
        let new = Device {
            id: 0x12345678abcdef01,
            kind: LEFT,
            corrupt: false,
        };
        let mut reads = VecDeque::from([list(0x11, &[old.clone()]), list(0x0c, &[new.clone()])]);
        for seq in 0..4 {
            reads.push_back(vec![11, seq, 0, 0, 0, 0, 0]);
            reads.push_back(vec![11, seq + 1, 0, 0, 0, 0, 0]);
        }
        reads.push_back(list(0x11, &[old, new.clone()]));
        let mut radio = Radio(Fake {
            reads,
            writes: vec![],
            fail_write: false,
        });
        radio.pair(&new).unwrap();
        assert!(radio.0.reads.is_empty());
        let writes = &radio.0.writes;
        assert_eq!(
            writes.iter().map(|data| data[0]).collect::<Vec<_>>(),
            [0x0d, 0x0f, 0x13, 0x0e]
        );
        assert_eq!(&writes[0][1..9], &new.id.to_le_bytes());
        assert_eq!(&writes[1][1..], &LEFT.to_le_bytes());
        assert_eq!(writes[2].len(), 61);
        assert_eq!(writes[2][9], 0x15);
        assert_eq!(writes[2][10], 0);
        assert_eq!(writes[3], [0x0e, new.id as u8]);
    }

    #[test]
    fn full_whitelist_never_removes_existing_pairing() {
        let entries: Vec<_> = (1..=4)
            .map(|id| Device {
                id,
                kind: LEFT,
                corrupt: false,
            })
            .collect();
        let mut radio = Radio(Fake {
            reads: [list(0x11, &entries)].into(),
            writes: vec![],
            fail_write: false,
        });
        assert!(radio
            .pair(&Device {
                id: 5,
                kind: LEFT,
                corrupt: false
            })
            .unwrap_err()
            .contains("full"));
        assert!(radio.0.writes.is_empty());
    }

    #[test]
    fn rejected_pair_still_disconnects_without_restarting_controller() {
        let new = Device {
            id: 42,
            kind: RIGHT,
            corrupt: false,
        };
        let mut radio = Radio(Fake {
            reads: [
                list(0x11, &[]),
                list(0x0c, &[new.clone()]),
                vec![11, 0, 0, 0, 0, 0, 0],
                vec![11, 1, 0, 0, 0, 0, 0],
                vec![11, 1, 0, 0, 0, 0, 0],
                vec![11, 2, 0, 1, 0, 0, 0],
                vec![11, 2, 0, 0, 0, 0, 0],
                vec![11, 3, 0, 0, 0, 0, 0],
            ]
            .into(),
            writes: vec![],
            fail_write: false,
        });
        assert!(radio.pair(&new).unwrap_err().contains("rejected"));
        assert_eq!(
            radio
                .0
                .writes
                .iter()
                .map(|data| data[0])
                .collect::<Vec<_>>(),
            [0x0d, 0x0f, 0x0e]
        );
        assert!(radio.0.reads.is_empty());
    }
}
