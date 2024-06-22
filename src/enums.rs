use num_traits::PrimInt;
use strum::IntoEnumIterator;
use strum_macros::{AsRefStr, EnumIter, FromRepr, VariantNames};

/// A trait to provide value and bit value methods for the bit enum.
pub trait BitEnum<T: PrimInt> {
    /// Get the value.
    ///
    /// # Returns
    /// Value.
    fn value(&self) -> T;

    /// Get the bit value.
    ///
    /// # Returns
    /// Bit value. If the value is not defined, it returns 0.
    fn bit_value(&self) -> T {
        match self.value().to_usize() {
            Some(value) => T::one() << value,
            None => T::zero(),
        }
    }
}

impl BitEnum<u64> for ErrorCode {
    fn value(&self) -> u64 {
        *self as u64
    }
}

impl BitEnum<u8> for DigitalOutput {
    fn value(&self) -> u8 {
        *self as u8
    }
}

impl BitEnum<u32> for DigitalInput {
    fn value(&self) -> u32 {
        *self as u32
    }
}

/// Commander status.
#[derive(FromRepr, Debug, PartialEq, Clone, Copy, Hash, Eq, VariantNames, AsRefStr)]
#[repr(u8)]
pub enum Commander {
    CSC,
    GUI,
}

/// Command status.
#[derive(Debug, VariantNames, AsRefStr)]
pub enum CommandStatus {
    Success,
    Fail,
    Ack,
    NoAck,
}

/// Inclination telemetry source.
#[derive(FromRepr, Debug, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum InclinationTelemetrySource {
    OnBoard = 1,
    MtMount = 2,
}

/// Type of the power.
#[derive(FromRepr, Debug, PartialEq, Clone, Copy, Hash, Eq)]
#[repr(u8)]
pub enum PowerType {
    Motor = 1,
    Communication = 2,
}

/// State of the power system.
#[derive(FromRepr, Debug, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum PowerSystemState {
    Init = 1,
    PoweredOff = 2,
    PoweringOn = 3,
    ResettingBreakers = 4,
    PoweredOn = 5,
    PoweringOff = 6,
}

/// Closed loop control mode.
#[derive(FromRepr, Debug, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum ClosedLoopControlMode {
    Idle = 1,
    TelemetryOnly = 2,
    OpenLoop = 3,
    ClosedLoop = 4,
}

/// Inner-loop control mode.
#[derive(FromRepr, Debug, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum InnerLoopControlMode {
    Standby = 1,
    Disabled = 2,
    Enabled = 3,
    FirmwareUpdate = 4,
    Fault = 5,
    ClearFaults = 6,
    NoChange = 7,
    Unknown = 8,
}

/// Unit of the actuator displacement.
#[derive(FromRepr, Debug, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum ActuatorDisplacementUnit {
    None = 0,
    Millimeter = 1,
    Step = 2,
}

/// Action to command the actuators.
#[derive(FromRepr, Debug, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum CommandActuator {
    Start = 1,
    Stop = 2,
    Pause = 3,
    Resume = 4,
}

/// Error code. If the name begins from the `Warn`, it is a warning. If the name
/// begins from the `Fault`, it is a fault. For the spares, the name begins from
/// the `Spare`. Some of the error codes are ignored at the moment and the name
/// begins with the `Ignore`.
#[derive(Debug, Clone, Copy, EnumIter)]
pub enum ErrorCode {
    IgnoreWarnStaleData,
    IgnoreFaultStaleData,
    IgnoreWarnBroadcast,
    FaultActuatorIlcRead,
    FaultExcessiveForce,
    WarnActuatorLimitOL,
    FaultActuatorLimitCL,
    FaultInclinometerWLut,
    WarnInclinometerWoLut,
    FaultMotorVoltage,
    WarnMotorVoltage,
    FaultCommVoltage,
    WarnCommVoltage,
    FaultMotorOverCurrent,
    FaultCommOverCurrent,
    FaultPowerRelayOpen,
    FaultPowerHealth,
    FaultCommMultiBreaker,
    FaultMotorMultiBreaker,
    WarnSingleBreakerTrip,
    FaultPowerSupplyLoadShare,
    WarnDisplacementSensorRange,
    FaultInclinometerRange,
    FaultMirrorTempSensor,
    WarnMirrorTempSensor,
    WarnCellTemp,
    FaultAxialActuatorEncoderRange,
    FaultTangentActuatorEncoderRange,
    IgnoreWarnMotorRelay,
    IgnoreWarnCommRelay,
    IgnoreFaultHardware,
    IgnoreFaultInterlock,
    FaultTangentLoadCell,
    FaultElevationAngleDiff,
    WarnMonitorIlcRead,
    IgnoreFaultPowerSystemTimeout,
    Spare36,
    Spare37,
    Spare38,
    Spare39,
    Spare40,
    Spare41,
    Spare42,
    Spare43,
    Spare44,
    Spare45,
    Spare46,
    Spare47,
    Spare48,
    Spare49,
    Spare50,
    Spare51,
    Spare52,
    Spare53,
    Spare54,
    FaultParameterFileRead,
    FaultIlcStateTransition,
    FaultCrioComm,
    WarnLossOfTma,
    FaultLossOfTmaCommOnEnable,
    WarnTempDiff,
    FaultCrioTiming,
    WarnCrioTiming,
    FaultUserIdentified,
}

/// Bit of digital output.
#[derive(Debug, PartialEq, Clone, Copy, EnumIter)]
pub enum DigitalOutput {
    MotorPower,
    CommunicationPower,
    InterlockEnable,
    ResetMotorBreakers,
    ResetCommunicationBreakers,
    ClosedLoopControl,
    Spare6,
    Spare7,
}

impl DigitalOutput {
    /// Get the enum from the representation.
    ///
    /// # Arguments
    /// * `discriminant` - Discriminant value.
    ///
    /// # Returns
    /// Enum value.
    pub fn from_repr(discriminant: u8) -> Option<DigitalOutput> {
        let mut bits = Vec::new();
        for digital_output in DigitalOutput::iter() {
            if discriminant & digital_output.bit_value() != 0 {
                bits.push(digital_output);
            }
        }

        if bits.len() == 1 {
            Some(bits[0])
        } else {
            None
        }
    }
}

/// Digital output status to switch the individual bit value of digital output.
#[derive(FromRepr, Debug, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum DigitalOutputStatus {
    BinaryLowLevel = 1,
    BinaryHighLevel = 2,
    ToggleBit = 3,
}

/// Bit of digital input.
#[derive(Debug, Clone, Copy, EnumIter)]
pub enum DigitalInput {
    RedundancyOK,
    LoadDistributionOK,
    PowerSupplyDC2OK,
    PowerSupplyDC1OK,
    PowerSupplyCurrent2OK,
    PowerSupplyCurrent1OK,
    J1W9N1MotorPowerBreaker,
    J1W9N2MotorPowerBreaker,
    J1W9N3MotorPowerBreaker,
    J2W10N1MotorPowerBreaker,
    J2W10N2MotorPowerBreaker,
    J2W10N3MotorPowerBreaker,
    J3W11N1MotorPowerBreaker,
    J3W11N2MotorPowerBreaker,
    J3W11N3MotorPowerBreaker,
    J1W12N1CommunicationPowerBreaker,
    Spare16,
    Spare17,
    Spare18,
    Spare19,
    Spare20,
    Spare21,
    Spare22,
    Spare23,
    J1W12N2CommunicationPowerBreaker,
    J2W13N1CommunicationPowerBreaker,
    J2W13N2CommunicationPowerBreaker,
    J3W14N1CommunicationPowerBreaker,
    J3W14N2CommunicationPowerBreaker,
    Spare29,
    Spare30,
    InterlockPowerRelay,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command_status() {
        assert_eq!(CommandStatus::Success.as_ref().to_lowercase(), "success");
        assert_eq!(CommandStatus::Fail.as_ref().to_lowercase(), "fail");
        assert_eq!(CommandStatus::Ack.as_ref().to_lowercase(), "ack");
        assert_eq!(CommandStatus::NoAck.as_ref().to_lowercase(), "noack");
    }

    #[test]
    fn test_inclination_telemetry_source_value() {
        // Get the enum from the repr.
        assert_eq!(
            InclinationTelemetrySource::from_repr(1).unwrap(),
            InclinationTelemetrySource::OnBoard
        );
        assert_eq!(
            InclinationTelemetrySource::from_repr(2).unwrap(),
            InclinationTelemetrySource::MtMount
        );

        // Get the enum value.
        assert_eq!(InclinationTelemetrySource::OnBoard as u8, 1);
        assert_eq!(InclinationTelemetrySource::MtMount as u8, 2);
    }

    #[test]
    fn test_error_code_value() {
        assert_eq!(ErrorCode::IgnoreWarnStaleData.value(), 0);
        assert_eq!(ErrorCode::IgnoreFaultStaleData.value(), 1);
        assert_eq!(ErrorCode::FaultUserIdentified.value(), 63);
    }

    #[test]
    fn test_error_code_bit_value() {
        assert_eq!(ErrorCode::IgnoreWarnStaleData.bit_value(), 1);
        assert_eq!(ErrorCode::IgnoreFaultStaleData.bit_value(), 2);
        assert_eq!(ErrorCode::FaultUserIdentified.bit_value(), 1 << 63);
    }

    #[test]
    fn test_digital_output_from_repr() {
        assert_eq!(
            DigitalOutput::from_repr(4).unwrap(),
            DigitalOutput::InterlockEnable
        );

        assert!(DigitalOutput::from_repr(0).is_none());
        assert!(DigitalOutput::from_repr(3).is_none());
    }
}
