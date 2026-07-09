# This file is part of ts_mtm2_controller.
#
# Developed for the Vera Rubin Observatory Systems.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from dataclasses import dataclass
from msgpack import Unpacker
from typing import Self

@dataclass
class TelemetryFile:
    """Telemetry data file structure. This should be the same as the
    TelemetryFile struct in telemetry_file.rs."""

    # Elapsed timestamp of the telemetry data in milliseconds from the
    # beginning of the telemetry file process.
    timestamp: int
    # Processed communication voltage in volt.
    processed_voltage_communication: float
    # Processed motor voltage in volt.
    processed_voltage_motor: float
    # Processed communication current in ampere.
    processed_current_communication: float
    # Processed motor current in ampere.
    processed_current_motor: float
    # Mirror is in position or not.
    is_in_position: bool
    # Actuator steps.
    actuator_steps: list[int]
    # Actuator positions in millimeter.
    actuator_positions: list[float]
    # Gravity look-up table (LUT) forces in Newton.
    forces_lut_gravity: list[float]
    # Temperature LUT forces in Newton.
    forces_lut_temperature: list[float]
    # Applied actuator forces in Newton.
    forces_applied: list[float]
    # Measured actuator forces in Newton.
    forces_measured: list[float]
    # Hardpoint correction forces in Newton.
    forces_hardpoint_correction: list[float]
    # Inner loop controller (ILC) status.
    ilc_status: list[int]
    # ILC encoder values.
    ilc_encoders: list[int]
    # Ring temperature in degree C.
    temperature_ring: list[float]
    # Intake temperature in degree C.
    temperature_intake: list[float]
    # Exhaust temperature in degree C.
    temperature_exhaust: list[float]
    # Theta-Z displacement sensors in micron.
    displacement_sensors_theta_z: list[float]
    # Delta-Z displacement sensors in micron.
    displacement_sensors_delta_z: list[float]
    # Mirror position x based on the hardpoints in um.
    mirror_position_x: float
    # Mirror position y based on the hardpoints in um.
    mirror_position_y: float
    # Mirror position z based on the hardpoints in um.
    mirror_position_z: float
    # Mirror rotation x based on the hardpoints in arcsec.
    mirror_position_rx: float
    # Mirror rotation y based on the hardpoints in arcsec.
    mirror_position_ry: float
    # Mirror rotation z based on the hardpoints in arcsec.
    mirror_position_rz: float
    # Raw inclinometer angle in degree.
    inclinometer_raw: float
    # Processed inclinometer angle in degree.
    inclinometer_processed: float
    # Cycle time in milliseconds.
    cycle_time: int

    def deserialize_from_file(filepath: str) -> list[Self]:
        """Deserialize a telemetry data file into a list of TelemetryFile
        objects.

        Parameters
        ----------
        filepath : `str`
            The path to the telemetry data file to deserialize.

        Returns
        -------
        `list` [`TelemetryFile`]
            A list of deserialized TelemetryFile objects.
        """

        with open(filepath, "rb") as file:
            unpacker = Unpacker(file)
            result = list()
            for data in unpacker:
                try:
                    result.append(TelemetryFile.deserialize(data))
                except Exception as err:
                    print(f"Failed to deserialize data: {err}")
                    break

            print(f"Deserialized {len(result)} telemetry data entries.")

            return result

    def deserialize(data: list) -> Self:
        """Deserialize a list of data into a TelemetryFile object.

        Parameters
        ----------
        data : `list`
            A list of data to deserialize into a TelemetryFile object.

        Returns
        -------
        `TelemetryFile`
            The deserialized TelemetryFile object.
        """
        return TelemetryFile(
            timestamp=int.from_bytes(data[0], byteorder='big', signed=False),
            processed_voltage_communication=data[1],
            processed_voltage_motor=data[2],
            processed_current_communication=data[3],
            processed_current_motor=data[4],
            is_in_position=data[5],
            actuator_steps=data[6],
            actuator_positions=data[7],
            forces_lut_gravity=data[8],
            forces_lut_temperature=data[9],
            forces_applied=data[10],
            forces_measured=data[11],
            forces_hardpoint_correction=data[12],
            ilc_status=data[13],
            ilc_encoders=data[14],
            temperature_ring=data[15],
            temperature_intake=data[16],
            temperature_exhaust=data[17],
            displacement_sensors_theta_z=data[18],
            displacement_sensors_delta_z=data[19],
            mirror_position_x=data[20],
            mirror_position_y=data[21],
            mirror_position_z=data[22],
            mirror_position_rx=data[23],
            mirror_position_ry=data[24],
            mirror_position_rz=data[25],
            inclinometer_raw=data[26],
            inclinometer_processed=data[27],
            cycle_time=data[28]
        )
