// This file is part of ts_mtm2_controller.
//
// Developed for the Vera Rubin Observatory Systems.
// This product includes software developed by the LSST Project
// (https://www.lsst.org).
// See the COPYRIGHT file at the top-level directory of this distribution
// for details of code ownership.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

use file_rotate::{
    compression::Compression,
    suffix::{AppendTimestamp, FileLimit},
    ContentLimit, FileRotate,
};
use log::info;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    mpsc::{sync_channel, Receiver, SyncSender},
    Arc,
};
use std::time::{Duration, Instant};

use crate::constants::BOUND_SYNC_CHANNEL;
use crate::telemetry::telemetry_file::TelemetryFile;

pub struct TelemetryFileProcess {
    // Writer to the file
    _writer: BufWriter<FileRotate<AppendTimestamp>>,
    // Sender to the telemetry file process
    _sender_to_telemetry_file_process: SyncSender<TelemetryFile>,
    // Receiver to the telemetry file process
    _receiver_to_telemetry_file_process: Receiver<TelemetryFile>,
    // Timeout to wait for new telemetry data in milliseconds
    _timeout: u64,
    // Stop the loop
    _stop: Arc<AtomicBool>,
}

impl TelemetryFileProcess {
    /// Creates a new TelemetryFileProcess.
    ///
    /// # Arguments
    /// * `path` - The path to the telemetry file.
    /// * `max_files` - The maximum number of telemetry files to keep.
    /// * `bytes_limit` - Cut the log file after surpassing size in bytes (but
    ///   having written a complete buffer from a write call.)
    /// * `timeout` - The timeout to wait for new telemetry data in
    ///   milliseconds.
    /// * `stop` - An Arc instance that holds the AtomicBool instance to stop
    ///   the loop.
    ///
    /// # Returns
    /// New instance of the telemetry file process.
    pub fn new(
        path: &Path,
        max_files: usize,
        bytes_limit: usize,
        timeout: u64,
        stop: &Arc<AtomicBool>,
    ) -> Self {
        let rotator = FileRotate::new(
            path,
            AppendTimestamp::default(FileLimit::MaxFiles(max_files)),
            ContentLimit::BytesSurpassed(bytes_limit),
            Compression::OnRotate(0),
            None,
        );

        let (sender, receiver) = sync_channel(BOUND_SYNC_CHANNEL);

        Self {
            _writer: BufWriter::new(rotator),

            _sender_to_telemetry_file_process: sender,
            _receiver_to_telemetry_file_process: receiver,

            _timeout: timeout,

            _stop: stop.clone(),
        }
    }

    /// Get the sender to the telemetry file process.
    ///
    /// # Returns
    /// The sender to the telemetry file process.
    pub fn get_sender_to_telemetry_file_process(&self) -> SyncSender<TelemetryFile> {
        self._sender_to_telemetry_file_process.clone()
    }

    /// Run the telemetry file process.
    pub fn run(&mut self) {
        info!("Telemetry file process is running.");

        let time_start = Instant::now();
        while !self._stop.load(Ordering::Relaxed) {
            if let Ok(mut telemetry) = self
                ._receiver_to_telemetry_file_process
                .recv_timeout(Duration::from_millis(self._timeout))
            {
                telemetry.timestamp = time_start.elapsed().as_millis();

                // Write the data to the buffer. Ignore the possible error.
                if let Ok(value) = telemetry.serialize() {
                    let _ = self._writer.write(&value);
                }
            }
        }

        let _ = self._writer.flush();

        info!("Telemetry file process is stopped.");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use rmp_serde::from_read;
    use std::fs::File;
    use std::io::BufReader;
    use std::thread::{sleep, spawn};
    use tempfile::NamedTempFile;

    use crate::telemetry::{
        telemetry_control_loop::TelemetryControlLoop, telemetry_power::TelemetryPower,
    };

    #[test]
    fn test_run() {
        // Run the process to log the telemetry data to a file.
        let temp_file = NamedTempFile::new().unwrap();
        let stop = Arc::new(AtomicBool::new(false));

        let timeout = 50;
        let mut telemetry_file_process =
            TelemetryFileProcess::new(temp_file.path(), 1, 10000, timeout, &stop);

        let sender = telemetry_file_process.get_sender_to_telemetry_file_process();

        let handle = spawn(move || {
            telemetry_file_process.run();
        });

        let telemetry = TelemetryFile::new(&TelemetryPower::new(), &TelemetryControlLoop::new());

        for _ in 0..3 {
            let _ = sender.try_send(telemetry.clone()).unwrap();
            sleep(Duration::from_millis(timeout));
        }

        sleep(Duration::from_millis(500));

        stop.store(true, Ordering::Relaxed);
        let _ = handle.join();

        // Read the file and check if it contains the telemetry data.
        let file = File::open(temp_file.path()).unwrap();
        let mut reader = BufReader::new(file);

        let mut timestamps = Vec::new();
        for _ in 0..3 {
            let telemetry: TelemetryFile = from_read(&mut reader).unwrap();
            timestamps.push(telemetry.timestamp);
        }

        assert!((timestamps[1] - timestamps[0]) >= (timeout as u128));
        assert!((timestamps[2] - timestamps[1]) >= (timeout as u128));

        temp_file.close().unwrap();
    }
}
