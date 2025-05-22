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

//! # M2 Control System
//!
//! This library is a collection of control system algorithms for the M2
//! mirror control system.
pub mod application;
pub mod command;
pub mod config;
pub mod constants;
pub mod control;
pub mod controller;
pub mod enums;
pub mod error_handler;
pub mod event_queue;
pub mod interface;
pub mod mock;
pub mod model;
pub mod power;
pub mod status;
pub mod telemetry;
pub mod utility;
