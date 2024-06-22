pub const NUM_AXIAL_ACTUATOR: usize = 72;
pub const NUM_TANGENT_LINK: usize = 6;
pub const NUM_ACTUATOR: usize = NUM_AXIAL_ACTUATOR + NUM_TANGENT_LINK;

pub const NUM_HARDPOINTS_AXIAL: usize = 3;
pub const NUM_HARDPOINTS_TANGENT: usize = 3;
pub const NUM_HARDPOINTS: usize = NUM_HARDPOINTS_AXIAL + NUM_HARDPOINTS_TANGENT;

pub const NUM_ACTIVE_ACTUATOR_AXIAL: usize = NUM_AXIAL_ACTUATOR - NUM_HARDPOINTS_AXIAL;
pub const NUM_ACTIVE_ACTUATOR_TANGENT: usize = NUM_TANGENT_LINK - NUM_HARDPOINTS_TANGENT;

pub const NUM_ACTIVE_ACTUATOR: usize = NUM_ACTIVE_ACTUATOR_AXIAL + NUM_ACTIVE_ACTUATOR_TANGENT;

pub const NUM_INNER_LOOP_CONTROLLER: usize = 84;

pub const NUM_TEMPERATURE_RING: usize = 12;
pub const NUM_TEMPERATURE_INTAKE: usize = 2;
pub const NUM_TEMPERATURE_EXHAUST: usize = 2;

pub const NUM_LUT_TEMPERATURE: usize = 4;

// Independent measurement system (IMS)
pub const NUM_IMS: usize = 6;
pub const NUM_IMS_READING: usize = 2 * NUM_IMS;

pub const NUM_SPACE_DEGREE_OF_FREEDOM: usize = 6;

// Each column has 5 degree difference from 0 to 360 degree.
// Therefore, we have (360 / 5) + 1 = 73 columns.
pub const NUM_COLUMN_LUT_GRAVITY: usize = 73;

// Outlier threshold from inner-loop controller (ILC) telemetry
pub const OUTLIER_INCLINOMETER_RAW: u32 = 1000000;

pub const LOCAL_HOST: &str = "127.0.0.1";
pub const ALL_HOST: &str = "0.0.0.0";
pub const TERMINATOR: &[u8; 2] = b"\r\n";

pub const BOUND_SYNC_CHANNEL: usize = 100;
