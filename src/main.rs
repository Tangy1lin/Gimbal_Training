#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;

use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Ticker};

// This rig uses GM6020 voltage commands for motor IDs 1 to 4.
const MOTOR_COMMAND_ID: u16 = 0x1FF;
// Ticks without remote input before it counts as lost.
const REMOTE_SILENT_TICKS: u16 = 100;
const TICK: Duration = Duration::from_millis(1);

// ---- 电机 ID（拨码决定；现场确认 yaw/pitch 后可直接对调）----
const YAW_MOTOR_ID: u8 = 1;
const PITCH_MOTOR_ID: u8 = 2;
const YAW_FEEDBACK_ID: u16 = 0x204 + YAW_MOTOR_ID as u16;
const PITCH_FEEDBACK_ID: u16 = 0x204 + PITCH_MOTOR_ID as u16;

// ---- 调试输出上限（协议是 ±25000，先远小于它）----
const BENCH_VOLTAGE_LIMIT: f32 = 3000.0;

// ---- 控制周期（秒）；1 kHz = 0.001 ----
const DT_S: f32 = 0.001;

// ---- yaw PID（kp/ki/kd 从 0 开始，现场从小到大加）----
const YAW_POS_KP: f32 = 0.0;
const YAW_POS_KI: f32 = 0.0;
const YAW_POS_KD: f32 = 0.0;
const YAW_POS_MAXOUT: f32 = 60.0;
const YAW_SPEED_KP: f32 = 0.0;
const YAW_SPEED_KI: f32 = 0.0;
const YAW_SPEED_KD: f32 = 0.0;
const YAW_SPEED_MAXOUT: f32 = 3000.0;

// ---- pitch PID ----
const PITCH_POS_KP: f32 = 0.0;
const PITCH_POS_KI: f32 = 0.0;
const PITCH_POS_KD: f32 = 0.0;
const PITCH_POS_MAXOUT: f32 = 60.0;
const PITCH_SPEED_KP: f32 = 0.0;
const PITCH_SPEED_KI: f32 = 0.0;
const PITCH_SPEED_KD: f32 = 0.0;
const PITCH_SPEED_MAXOUT: f32 = 3000.0;

// ---- 参考姿态（电机轴角度，现场标定后填）----
const YAW_REFERENCE_RAD: f32 = 0.0;
const PITCH_REFERENCE_RAD: f32 = 0.0;

// ---- 摇杆 ----
const MAX_TARGET_RATE_RAD_S: f32 = 1.5;
const STICK_DEADBAND: f32 = 0.06;
const YAW_STICK_SIGN: f32 = 1.0;
const PITCH_STICK_SIGN: f32 = 1.0;
const PITCH_TARGET_MIN_RAD: f32 = -0.8;
const PITCH_TARGET_MAX_RAD: f32 = 0.8;

// ---- 状态机 ----
const FEEDBACK_TIMEOUT_TICKS: u16 = 20;
const SETTLE_ERROR_RAD: f32 = 0.05;
const SETTLE_SPEED_RPM: f32 = 10.0;
const SETTLE_TICKS: u16 = 300;
const PREPARE_TIMEOUT_TICKS: u16 = 3000;

// ---- 故障检测 ----
const STALL_TARGET_RPM: f32 = 20.0;
const STALL_OUTPUT_RAW: f32 = 800.0;
const STALL_SPEED_RPM: f32 = 5.0;
const STALL_TICKS: u16 = 300;
const OSC_AMPLITUDE_RPM: f32 = 20.0;
const OSC_WINDOW_TICKS: u16 = 200;
const OSC_MIN_HZ: f32 = 15.0;
const OSC_HIGH_WINDOWS: u16 = 3;

/// Command frames dropped because the CAN transmit queue was full, for example while no
/// motor acknowledges frames.
pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);

/// Feedback and voltage command for one motor.
#[derive(Default)]
pub struct Motor {
    /// Single-turn motor shaft angle in rad; turns are not counted.
    pub angle_rad: f32,
    pub speed_rpm: f32,
    /// Feedback current as sent on the wire. The motor manual gives no scale to amperes.
    pub current_raw: i16,
    /// Voltage command counts, not volts; protocol range +/-25000.
    pub command_voltage_raw: f32,
}

impl Motor {
    /// Clear the voltage command in RAM. The motor receives zero when
    /// `Control::transmit` next runs, within 1 ms.
    pub fn disable(&mut self) {
        self.command_voltage_raw = 0.0;
    }

    /// decode angle, speed and current as the manual describes
    pub fn read(&mut self, data: &[u8; 8]) {
        let angle_raw = u16::from_be_bytes([data[0], data[1]]);
        let speed_raw = i16::from_be_bytes([data[2], data[3]]);
        let current_raw = i16::from_be_bytes([data[4], data[5]]);
        self.angle_rad = angle_raw as f32 / 8192.0 * 2.0 * core::f32::consts::PI;
        self.speed_rpm = speed_raw as f32;
        self.current_raw = current_raw;
    }
}

/// PID gains, plus the integral and history kept between ticks.
#[derive(Default)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub maxout: f32,
    pub integral: f32,
    pub prev_error: f32,
    pub started: bool,
}

impl Pid {
    // clear the integral and history.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.started = false;
    }

    /// `target` and `measured` share a unit, set by the caller. The position loop takes rad
    /// and returns rpm; the speed loop takes rpm and returns voltage counts. `dt_s` is in seconds.
    /// If a position loop returns rad/s, convert it to rpm before passing it to the speed loop.
    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> f32 {
        // compute the PID with dt_s, and limit the integral and output.
        let error = target - measured;

        let derivative = if self.started && dt_s > 0.0 {
            (error - self.prev_error) / dt_s
        } else {
            0.0
        };
        self.prev_error = error;
        self.started = true;

        if self.ki != 0.0 {
            self.integral += error * dt_s;
            let limit = self.maxout / self.ki.abs();
            self.integral = self.integral.clamp(-limit, limit);
        } else {
            self.integral = 0.0;
        };

        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        output.clamp(-self.maxout, self.maxout)
    }
}

/// The gimbal's stage, which decides the allowed operations and outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Zero output. Waits for valid remote input and fresh feedback from both motors,
    /// then for a new enable request.
    Waiting,
    /// Recenters both axes to the reference pose with limited speed and output.
    /// Enters `Running` once both axes settle.
    Prepare,
    /// The left stick moves each axis's `target_angle_rad`, and its cascade tracks it.
    Running,
    /// Zero output until a manual recovery.
    Error,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    StallYaw,
    StallPitch,
    OscillationYaw,
    OscillationPitch,
}

/// One gimbal axis: its motor, targets and cascaded PIDs.
#[derive(Default)]
pub struct Axis {
    pub motor: Motor,
    /// Position loop target in rad.
    pub target_angle_rad: f32,
    /// Speed loop target in rpm, set from the position loop output.
    pub target_rpm: f32,
    /// Position loop: rad in, rpm out.
    pub position_pid: Pid,
    /// Speed loop: rpm in, voltage counts out.
    pub speed_pid: Pid,
}
#[derive(Default)]
pub struct AxisFaultState {
    pub stall_ticks: u16,
    pub osc_crossings: u16,
    pub osc_window_ticks: u16,
    pub osc_high_windows: u16,
    pub last_speed_sign: i8,
}

/// State shared by every control tick.
// TODO: add feedback ages and a fault record.
pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    pub remote_silent_ticks: u16,
    // Feedback ages in ticks since the last frame was received. If either exceeds a timeout,
    // stop output and the state returns to Waiting unless already in Error.
    pub yaw_feedback_age_ticks: u16,
    pub pitch_feedback_age_ticks: u16,

    pub enable_armed: bool,
    pub recovery_armed: bool,
    pub prepare_ticks: u16,
    pub settle_ticks: u16,

    pub fault: Option<Fault>,
    pub yaw_faults: AxisFaultState,
    pub pitch_faults: AxisFaultState,
}

/// Start in `Waiting` with no remote input.
/// Set the initial value of each field you add here, including the PID gains.
impl Default for Control {
    fn default() -> Self {
        Self {
            state: State::Waiting,
            
            yaw: Axis {
                position_pid: Pid {
                    kp: YAW_POS_KP,
                    ki: YAW_POS_KI,
                    kd: YAW_POS_KD,
                    maxout: YAW_POS_MAXOUT,
                    ..Pid::default()
                },
                speed_pid: Pid {
                    kp: YAW_SPEED_KP,
                    ki: YAW_SPEED_KI,
                    kd: YAW_SPEED_KD,
                    maxout: YAW_SPEED_MAXOUT,
                    ..Pid::default()
                },
                ..Axis::default()
            },
            pitch: Axis {
                position_pid: Pid {
                    kp: PITCH_POS_KP,
                    ki: PITCH_POS_KI,
                    kd: PITCH_POS_KD,
                    maxout: PITCH_POS_MAXOUT,
                    ..Pid::default()
                },
                speed_pid: Pid {
                    kp: PITCH_SPEED_KP,
                    ki: PITCH_SPEED_KI,
                    kd: PITCH_SPEED_KD,
                    maxout: PITCH_SPEED_MAXOUT,
                    ..Pid::default()
                },
                ..Axis::default()
            },

            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
            yaw_feedback_age_ticks: u16::MAX,
            pitch_feedback_age_ticks: u16::MAX,
            enable_armed: false,
            recovery_armed: false,
            prepare_ticks: 0,
            settle_ticks: 0,
            fault: None,
            yaw_faults: AxisFaultState::default(),
            pitch_faults: AxisFaultState::default(),
        }
    }
}

impl Control {
    /// Clear both motors' voltage commands in RAM.
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }

    /// Runs every 1 ms: read the remote and motor feedback, update the state, then
    /// compute and send the outputs. Missing remote input stops output; a latched Error is preserved.
    pub fn tick(&mut self, can: &mut Can1) {
        self.read_remote();
        self.read_feedback(can);
        self.update_mode();
        if self.remote.is_none() && self.state != State::Error {
            self.state = State::Waiting;
        }
        self.update();
        self.transmit(can);
    }

    /// Take the latest input published by the remote task. After an invalid frame, or
    /// `REMOTE_SILENT_TICKS` ticks without one, clear the commands; return to Waiting unless already in Error.
    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(remote) = dm_mc02::REMOTE.try_take() {
            self.remote = remote;
            if self.remote.is_some() {
                self.remote_silent_ticks = 0;
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
        if self.remote.is_none() {
            self.disable_all();
            if self.state != State::Error {
                self.state = State::Waiting;
            }
        }
    }

    /// Read every queued CAN frame without waiting for more.
    pub fn read_feedback(&mut self, can: &mut Can1) {
        // Increment the feedback ages before reading any frames, so a frame received this tick resets the age to 0.
        self.yaw_feedback_age_ticks = self.yaw_feedback_age_ticks.saturating_add(1);
        self.pitch_feedback_age_ticks = self.pitch_feedback_age_ticks.saturating_add(1);

        let receiver = can.reader();
        while let Ok(received) = receiver.try_receive() {
            match received {
                Ok(envelope) => {
                    let frame = envelope.frame;

                    // Only standard frames are used; ignore extended ones.
                    let id = match frame.id() {
                        embedded_can::Id::Standard(id) => id.as_raw(),
                        embedded_can::Id::Extended(_) => continue,
                    };

                    // The manual describes the feedback frame as 8 bytes, so ignore any other length.
                    let data: &[u8; 8] = match frame.data().try_into() {
                        Ok(data) => data,
                        Err(_) => continue,
                    };
                    // The manual describes the feedback frame as 0x204 plus the motor ID.
                    match id {
                        YAW_FEEDBACK_ID => {
                            self.yaw.motor.read(data);
                            self.yaw_feedback_age_ticks = 0;
                        }
                        PITCH_FEEDBACK_ID => {
                            self.pitch.motor.read(data);
                            self.pitch_feedback_age_ticks = 0;
                        }
                        _ => {}
                    }
                }
                Err(error) => dm_mc02::log_can_error(error),
            }
        }
    }

    /// Choose the state from the remote, feedback and fault conditions.
    // TODO: the transitions, and initialising targets and PIDs on entering a state.
    pub fn update_mode(&mut self) {
        use core::f32::consts::{PI, TAU};

        if self.state == State::Prepare || self.state == State::Running {
            if let Some(fault) = self.detect_faults() {
                match fault {
                    Fault::StallYaw => {
                        rtt_target::rprintln!(
                            "Error: yaw suspected stall (rpm={:.1}, out={:.0})",
                            self.yaw.motor.speed_rpm,
                            self.yaw.motor.command_voltage_raw
                        );
                    }
                    Fault::StallPitch => {
                        rtt_target::rprintln!(
                            "Error: pitch suspected stall (rpm={:.1}, out={:.0})",
                            self.pitch.motor.speed_rpm,
                            self.pitch.motor.command_voltage_raw
                        );
                    }
                    Fault::OscillationYaw => {
                        rtt_target::rprintln!(
                            "Error: yaw oscillation (rpm={:.1})",
                            self.yaw.motor.speed_rpm
                        );
                    }
                    Fault::OscillationPitch => {
                        rtt_target::rprintln!(
                            "Error: pitch oscillation (rpm={:.1})",
                            self.pitch.motor.speed_rpm
                        );
                    }
                }
                self.fault = Some(fault);
                self.disable_all();
                self.state = State::Error;
                return;
            }
        }

        let (right_switch, left_switch) = match self.remote {
            Some(remote) => (remote.right_switch, remote.left_switch),
            None => {
                self.enable_armed = false;
                self.recovery_armed = false;
                (0, 0)
            }
        };

        if right_switch == 2 {
            self.enable_armed = true;
        }

        match self.state {
            State::Waiting => {
                // TODO: enter Prepare after valid feedback and a new enable request.
                let feedback_ok = self.yaw_feedback_age_ticks <= FEEDBACK_TIMEOUT_TICKS
                    && self.pitch_feedback_age_ticks <= FEEDBACK_TIMEOUT_TICKS;
                if feedback_ok && self.enable_armed && right_switch == 3 {
                    self.enable_armed = false;
                    self.prepare_ticks = 0;
                    self.settle_ticks = 0;
                    self.fault = None;
                    self.yaw_faults = AxisFaultState::default();
                    self.pitch_faults = AxisFaultState::default();
                    self.yaw.position_pid.reset();
                    self.yaw.speed_pid.reset();
                    self.pitch.position_pid.reset();
                    self.pitch.speed_pid.reset();
                    self.yaw.target_angle_rad = YAW_REFERENCE_RAD;
                    self.pitch.target_angle_rad = PITCH_REFERENCE_RAD;
                    self.yaw.target_rpm = 0.0;
                    self.pitch.target_rpm = 0.0;
                    self.state = State::Prepare;
                }
            }
            State::Prepare => {
                // TODO: enter Running once recentering settles; stop on a timeout or fault.
                let feedback_lost = self.yaw_feedback_age_ticks > FEEDBACK_TIMEOUT_TICKS
                    || self.pitch_feedback_age_ticks > FEEDBACK_TIMEOUT_TICKS;
                if right_switch == 2 || feedback_lost {
                    self.state = State::Waiting;
                } else {
                    self.prepare_ticks = self.prepare_ticks.saturating_add(1);

                    let mut yaw_error = self.yaw.target_angle_rad - self.yaw.motor.angle_rad;
                    yaw_error %= TAU;
                    if yaw_error >= PI {
                        yaw_error -= TAU;
                    }
                    if yaw_error < -PI {
                        yaw_error += TAU;
                    }
                    let yaw_ok = yaw_error.abs() <= SETTLE_ERROR_RAD
                        && self.yaw.motor.speed_rpm.abs() <= SETTLE_SPEED_RPM;
                    let pitch_ok = (self.pitch.target_angle_rad - self.pitch.motor.angle_rad).abs()
                        <= SETTLE_ERROR_RAD
                        && self.pitch.motor.speed_rpm.abs() <= SETTLE_SPEED_RPM;
                    if yaw_ok && pitch_ok {
                        self.settle_ticks = self.settle_ticks.saturating_add(1);
                    } else {
                        self.settle_ticks = 0;
                    }

                    if self.settle_ticks >= SETTLE_TICKS {
                        self.state = State::Running;
                    } else if self.prepare_ticks >= PREPARE_TIMEOUT_TICKS {
                        self.state = State::Waiting;
                    }
                }
            }
            State::Running => {
                // TODO: handle disable/feedback loss; stall or sustained high-frequency oscillation enters Error.
                let feedback_lost = self.yaw_feedback_age_ticks > FEEDBACK_TIMEOUT_TICKS
                    || self.pitch_feedback_age_ticks > FEEDBACK_TIMEOUT_TICKS;
                if right_switch == 2 || feedback_lost {
                    self.state = State::Waiting;
                }
            }
            State::Error => {
                // TODO: keep zero output until a new remote recovery gesture; hold current angles,
                // clear PID history and detection counters, then return to Running.
                if right_switch == 3 {
                    if left_switch == 2 {
                        self.recovery_armed = true;
                    }
                    if left_switch == 1 && self.recovery_armed {
                        self.recovery_armed = false;
                        self.fault = None;
                        self.yaw_faults = AxisFaultState::default();
                        self.pitch_faults = AxisFaultState::default();
                        self.yaw.target_angle_rad = self.yaw.motor.angle_rad;
                        self.pitch.target_angle_rad = self.pitch.motor.angle_rad;
                        self.yaw.target_rpm = 0.0;
                        self.pitch.target_rpm = 0.0;
                        self.yaw.position_pid.reset();
                        self.yaw.speed_pid.reset();
                        self.pitch.position_pid.reset();
                        self.pitch.speed_pid.reset();
                        self.state = State::Running;
                    }
                }
            }
        }
    }

    /// 检测两轴是否有疑似堵转或持续振荡；每 tick 更新计数器。
    fn detect_faults(&mut self) -> Option<Fault> {
        // ---- yaw 堵转 ----
        if self.yaw.target_rpm.abs() >= STALL_TARGET_RPM
            && self.yaw.motor.command_voltage_raw.abs() >= STALL_OUTPUT_RAW
            && self.yaw.motor.speed_rpm.abs() <= STALL_SPEED_RPM
        {
            self.yaw_faults.stall_ticks = self.yaw_faults.stall_ticks.saturating_add(1);
        } else {
            self.yaw_faults.stall_ticks = 0;
        }
        if self.yaw_faults.stall_ticks >= STALL_TICKS {
            return Some(Fault::StallYaw);
        }

        // ---- pitch 堵转 ----
        if self.pitch.target_rpm.abs() >= STALL_TARGET_RPM
            && self.pitch.motor.command_voltage_raw.abs() >= STALL_OUTPUT_RAW
            && self.pitch.motor.speed_rpm.abs() <= STALL_SPEED_RPM
        {
            self.pitch_faults.stall_ticks = self.pitch_faults.stall_ticks.saturating_add(1);
        } else {
            self.pitch_faults.stall_ticks = 0;
        }
        if self.pitch_faults.stall_ticks >= STALL_TICKS {
            return Some(Fault::StallPitch);
        }

        // ---- yaw 振荡 ----
        let yaw_sign = if self.yaw.motor.speed_rpm > OSC_AMPLITUDE_RPM {
            1
        } else if self.yaw.motor.speed_rpm < -OSC_AMPLITUDE_RPM {
            -1
        } else {
            0
        };
        if yaw_sign != 0 {
            if self.yaw_faults.last_speed_sign != 0 && yaw_sign != self.yaw_faults.last_speed_sign {
                self.yaw_faults.osc_crossings = self.yaw_faults.osc_crossings.saturating_add(1);
            }
            self.yaw_faults.last_speed_sign = yaw_sign;
        }
        self.yaw_faults.osc_window_ticks = self.yaw_faults.osc_window_ticks.saturating_add(1);
        if self.yaw_faults.osc_window_ticks >= OSC_WINDOW_TICKS {
            let window_s = OSC_WINDOW_TICKS as f32 * 0.001;
            let freq_hz = self.yaw_faults.osc_crossings as f32 / (2.0 * window_s);
            if freq_hz >= OSC_MIN_HZ {
                self.yaw_faults.osc_high_windows =
                    self.yaw_faults.osc_high_windows.saturating_add(1);
            } else {
                self.yaw_faults.osc_high_windows = 0;
            }
            self.yaw_faults.osc_crossings = 0;
            self.yaw_faults.osc_window_ticks = 0;
        }
        if self.yaw_faults.osc_high_windows >= OSC_HIGH_WINDOWS {
            return Some(Fault::OscillationYaw);
        }

        // ---- pitch 振荡 ----
        let pitch_sign = if self.pitch.motor.speed_rpm > OSC_AMPLITUDE_RPM {
            1
        } else if self.pitch.motor.speed_rpm < -OSC_AMPLITUDE_RPM {
            -1
        } else {
            0
        };
        if pitch_sign != 0 {
            if self.pitch_faults.last_speed_sign != 0
                && pitch_sign != self.pitch_faults.last_speed_sign
            {
                self.pitch_faults.osc_crossings = self.pitch_faults.osc_crossings.saturating_add(1);
            }
            self.pitch_faults.last_speed_sign = pitch_sign;
        }
        self.pitch_faults.osc_window_ticks = self.pitch_faults.osc_window_ticks.saturating_add(1);
        if self.pitch_faults.osc_window_ticks >= OSC_WINDOW_TICKS {
            let window_s = OSC_WINDOW_TICKS as f32 * 0.001;
            let freq_hz = self.pitch_faults.osc_crossings as f32 / (2.0 * window_s);
            if freq_hz >= OSC_MIN_HZ {
                self.pitch_faults.osc_high_windows =
                    self.pitch_faults.osc_high_windows.saturating_add(1);
            } else {
                self.pitch_faults.osc_high_windows = 0;
            }
            self.pitch_faults.osc_crossings = 0;
            self.pitch_faults.osc_window_ticks = 0;
        }
        if self.pitch_faults.osc_high_windows >= OSC_HIGH_WINDOWS {
            return Some(Fault::OscillationPitch);
        }

        None
    }

    /// Set targets for the current state, run each axis's PIDs and write the voltage commands.
    // TODO: limited recentering in Prepare, remote cascade control in Running, and
    // `disable_all` in Waiting and Error. Each tick is 0.001 s.
    pub fn update(&mut self) {
        use core::f32::consts::{PI, TAU};

        match self.state {
            State::Waiting | State::Error => {
                self.disable_all();
                self.yaw.target_rpm = 0.0;
                self.pitch.target_rpm = 0.0;
            }

            State::Prepare | State::Running => {
                if self.state == State::Running {
                    if let Some(remote) = self.remote {
                        let yaw_input = remote.channels[2] as f32 / 660.0;
                        let yaw_input = if yaw_input.abs() < STICK_DEADBAND {
                            0.0
                        } else {
                            yaw_input
                        };
                        let mut yaw_target = self.yaw.target_angle_rad
                            + yaw_input * YAW_STICK_SIGN * MAX_TARGET_RATE_RAD_S * DT_S;
                        yaw_target %= TAU;
                        if yaw_target >= PI {
                            yaw_target -= TAU;
                        }
                        if yaw_target < -PI {
                            yaw_target += TAU;
                        }
                        self.yaw.target_angle_rad = yaw_target;

                        let pitch_input = remote.channels[3] as f32 / 660.0;
                        let pitch_input = if pitch_input.abs() < STICK_DEADBAND {
                            0.0
                        } else {
                            pitch_input
                        };
                        self.pitch.target_angle_rad = (self.pitch.target_angle_rad
                            + pitch_input * PITCH_STICK_SIGN * MAX_TARGET_RATE_RAD_S * DT_S)
                            .clamp(PITCH_TARGET_MIN_RAD, PITCH_TARGET_MAX_RAD);
                    }
                }

                let mut yaw_error = self.yaw.target_angle_rad - self.yaw.motor.angle_rad;
                yaw_error %= TAU;
                if yaw_error >= PI {
                    yaw_error -= TAU;
                }
                if yaw_error < -PI {
                    yaw_error += TAU;
                }
                self.yaw.target_rpm = self.yaw.position_pid.calculate(
                    self.yaw.motor.angle_rad + yaw_error,
                    self.yaw.motor.angle_rad,
                    DT_S,
                );
                self.yaw.motor.command_voltage_raw = self.yaw.speed_pid.calculate(
                    self.yaw.target_rpm,
                    self.yaw.motor.speed_rpm,
                    DT_S,
                );

                self.pitch.target_rpm = self.pitch.position_pid.calculate(
                    self.pitch.target_angle_rad,
                    self.pitch.motor.angle_rad,
                    DT_S,
                );
                self.pitch.motor.command_voltage_raw = self.pitch.speed_pid.calculate(
                    self.pitch.target_rpm,
                    self.pitch.motor.speed_rpm,
                    DT_S,
                );
            }
        }
    }

    /// Queue one group command frame on CAN1 without waiting. This runs every tick, so a
    /// cleared command reaches the motor within 1 ms. A full queue drops the frame and
    /// counts it in `TX_FAILED`.
    pub fn transmit(&mut self, can: &mut Can1) {
        let mut data = [0u8; 8];
        // clamp each voltage count output within the bench limit (protocol +/-25000),
        // encode signed i16 big-endian and write it into the
        // bytes for its motor ID as the manual describes. Other motor IDs stay zero.

        let yaw_counts = self
            .yaw
            .motor
            .command_voltage_raw
            .clamp(-BENCH_VOLTAGE_LIMIT, BENCH_VOLTAGE_LIMIT) as i16;
        let yaw_bytes = yaw_counts.to_be_bytes();
        let yaw_offset = (YAW_MOTOR_ID as usize - 1) * 2;
        data[yaw_offset] = yaw_bytes[0];
        data[yaw_offset + 1] = yaw_bytes[1];

        let pitch_counts = self
            .pitch
            .motor
            .command_voltage_raw
            .clamp(-BENCH_VOLTAGE_LIMIT, BENCH_VOLTAGE_LIMIT) as i16;
        let pitch_bytes = pitch_counts.to_be_bytes();
        let pitch_offset = (PITCH_MOTOR_ID as usize - 1) * 2;
        data[pitch_offset] = pitch_bytes[0];
        data[pitch_offset + 1] = pitch_bytes[1];

        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
    }
}

// Set up RTT and the board, then start the remote and control tasks.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!("Gimbal control started");
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

// Run `tick` at 1 kHz and feed the watchdog after each one. If a tick hangs, the watchdog
// resets the MCU after 100 ms.
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
    loop {
        ticker.next().await;
        // After a stall, such as the 20 ms halt when a debug probe attaches, start the
        // schedule again instead of running every missed tick back to back.
        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
        }
        last = now;
        control.tick(&mut can);
        watchdog.pet();
    }
}
