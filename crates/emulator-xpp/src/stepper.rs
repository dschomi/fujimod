//! A two-channel stepper-motor driver on an X-Processor Pro serial port.
//!
//! The body sends the driver 16-bit words through the port's send register
//! +0x60, one word per transfer. A declared motor owns the words whose top
//! four bits equal its command nibble; in such a word bit 11 starts the motor
//! (clear stops it), bit 10 is the direction and bits 9..0 the step count, 0
//! for a run that lasts until a word without the start bit. Other words are
//! settings the driver keeps but the model does not interpret; they are
//! counted. Every other port register behaves as the [`DiscoveryPort`].
//!
//! A moving motor takes one step per declared step period. Its busy output,
//! a GPIO input pad of the body, reads high while it moves; its position
//! sensor, another input pad, changes level every declared number of steps.
//! Both pads reach external interrupt lines through [`ExternalLines`]. The
//! step period, the sensor spacing and the busy and sensor wiring are
//! declared inputs, not measurements; the command layout is the body's field
//! table of the driver.
use crate::resets::{ExternalLines, LINES};
use crate::serial::DiscoveryPort;
use fujimod_emulator_devices::{Cx, Device};
use fujimod_emulator_ip::socionext::gpio::SharedPins;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

const SEND: u64 = 0x60;
const NIBBLE: u32 = 0xf000;
const START: u32 = 1 << 11;
const REVERSE: u32 = 1 << 10;
const STEPS: u32 = 0x3ff;

/// A pad of the body that the driver drives: GPIO bank, pin and the external
/// interrupt line it reaches.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pad {
    pub bank: usize,
    pub pin: usize,
    pub line: usize,
}

/// One motor channel of the driver.
#[derive(Clone, Debug)]
pub struct Motor {
    /// Top four bits of the words that command this motor.
    pub command: u16,
    /// Pad that reads high while the motor moves.
    pub busy: Option<Pad>,
    /// Position sensor pad and the steps between its level changes.
    pub sensor: Option<(Pad, u64)>,
}

#[derive(Clone, Debug)]
pub struct Wiring {
    /// Base of the serial port the driver is attached to.
    pub base: u64,
    pub motors: Vec<Motor>,
}

#[derive(Clone, Copy, Debug)]
pub struct Timing {
    /// Ticks per motor step.
    pub step: u64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct MotorState {
    /// Tick of the next step while the motor moves.
    next: Option<u64>,
    /// Steps left of a counted move; `None` while moving means a run.
    remaining: Option<u64>,
    reverse: bool,
    /// Signed position in steps since reset.
    position: i64,
    steps: u64,
    starts: u64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct State {
    motors: Vec<MotorState>,
    /// Words that command no declared motor.
    settings: u64,
}

pub struct StepperPort {
    port: DiscoveryPort,
    wiring: Wiring,
    timing: Timing,
    pins: SharedPins,
    lines: ExternalLines,
    state: State,
}

impl StepperPort {
    pub fn new(
        wiring: Wiring,
        timing: Timing,
        pins: SharedPins,
        lines: ExternalLines,
    ) -> Result<Self, String> {
        let pad = |p: &Pad| p.bank < 16 && p.pin < 16 && p.line < LINES;
        let pads: Vec<Pad> = wiring
            .motors
            .iter()
            .flat_map(|m| m.busy.into_iter().chain(m.sensor.map(|(p, _)| p)))
            .collect();
        let mut commands: Vec<u16> = wiring.motors.iter().map(|m| m.command).collect();
        commands.sort_unstable();
        commands.dedup();
        if wiring.motors.is_empty()
            || commands.len() != wiring.motors.len()
            || wiring.motors.iter().any(|m| m.command & 0x0fff != 0)
            || wiring
                .motors
                .iter()
                .any(|m| m.sensor.is_some_and(|(_, n)| n == 0))
            || !pads.iter().all(pad)
            || timing.step == 0
        {
            return Err("Invalid stepper driver wiring or timing".into());
        }
        for p in &pads {
            lines.connect(pins.clone(), p.line, p.bank, p.pin)?;
        }
        Ok(Self {
            port: DiscoveryPort::new(wiring.base)?,
            state: State {
                motors: vec![MotorState::default(); wiring.motors.len()],
                settings: 0,
            },
            wiring,
            timing,
            pins,
            lines,
        })
    }

    fn set_pad(&self, pad: Pad, high: bool) {
        self.pins.lock().unwrap().set_input(pad.bank, pad.pin, high);
    }

    fn command(&mut self, cx: &mut Cx<'_>, word: u32) {
        let Some(index) = self
            .wiring
            .motors
            .iter()
            .position(|m| u32::from(m.command) == word & NIBBLE)
        else {
            self.state.settings += 1;
            return;
        };
        let busy = self.wiring.motors[index].busy;
        let motor = &mut self.state.motors[index];
        if word & START == 0 {
            motor.next = None;
            motor.remaining = None;
        } else {
            let steps = u64::from(word & STEPS);
            motor.reverse = word & REVERSE != 0;
            motor.remaining = (steps != 0).then_some(steps);
            motor.next = cx.now().checked_add(self.timing.step);
            motor.starts += 1;
        }
        let moving = motor.next.is_some();
        if let Some(pad) = busy {
            self.set_pad(pad, moving);
        }
        self.lines.update(cx);
    }

    fn step(&mut self, cx: &mut Cx<'_>, index: usize) {
        let wiring = self.wiring.motors[index].clone();
        let motor = &mut self.state.motors[index];
        motor.steps += 1;
        motor.position += if motor.reverse { -1 } else { 1 };
        let done = match &mut motor.remaining {
            Some(left) => {
                *left -= 1;
                *left == 0
            }
            None => false,
        };
        motor.next = if done {
            motor.remaining = None;
            None
        } else {
            motor.next.and_then(|at| at.checked_add(self.timing.step))
        };
        let (steps, moving) = (motor.steps, motor.next.is_some());
        if let Some((pad, spacing)) = wiring.sensor
            && steps.is_multiple_of(spacing)
        {
            let high = self.pins.lock().unwrap().input(pad.bank, pad.pin);
            self.set_pad(pad, !high);
        }
        if let Some(pad) = wiring.busy
            && !moving
        {
            self.set_pad(pad, false);
        }
        self.lines.update(cx);
    }

    fn check(&self, state: &State) -> Result<(), String> {
        if state.motors.len() != self.wiring.motors.len()
            || state
                .motors
                .iter()
                .any(|m| m.next.is_none() && m.remaining.is_some())
        {
            return Err("Invalid stepper driver checkpoint".into());
        }
        Ok(())
    }
}

impl Device for StepperPort {
    fn name(&self) -> &'static str {
        "xpp-stepper-driver"
    }
    fn read(&mut self, cx: &mut Cx<'_>, address: u64, size: u8) -> u64 {
        self.port.read(cx, address, size)
    }
    fn write(&mut self, cx: &mut Cx<'_>, address: u64, size: u8, value: u64) {
        self.port.write(cx, address, size, value);
        if size == 4 && address.wrapping_sub(self.wiring.base) == SEND {
            self.command(cx, value as u32 & 0xffff);
        }
    }
    fn advance(&mut self, cx: &mut Cx<'_>) {
        let now = cx.now();
        for index in 0..self.state.motors.len() {
            while self.state.motors[index].next.is_some_and(|at| now >= at) {
                self.step(cx, index);
            }
        }
    }
    fn next_deadline(&self, _now: u64) -> Option<u64> {
        self.state.motors.iter().filter_map(|m| m.next).min()
    }
    fn pollable(&self, address: u64, size: u8) -> bool {
        self.port.pollable(address, size)
    }
    fn save(&self) -> Value {
        json!({ "port": self.port.save(), "state": self.state })
    }
    fn restore(&mut self, value: &Value) -> Result<(), String> {
        let state: State =
            serde_json::from_value(value["state"].clone()).map_err(|e| e.to_string())?;
        self.check(&state)?;
        self.port.restore(&value["port"])?;
        self.state = state;
        Ok(())
    }
    /// The port is a child.
    fn checkpoint(&self) -> Result<fujimod_emulator_devices::checkpoint::Checkpoint, String> {
        Ok(
            fujimod_emulator_devices::checkpoint::Checkpoint::encode(self.name(), &self.state)?
                .with_children(vec![self.port.checkpoint()?]),
        )
    }
    fn validate_checkpoint(
        &self,
        saved: &fujimod_emulator_devices::checkpoint::Checkpoint,
    ) -> Result<(), String> {
        if saved.children().len() != 1 || !saved.blobs().is_empty() {
            return Err("Stepper driver checkpoint parts differ".into());
        }
        let state: State = saved.decode(self.name())?;
        self.check(&state)?;
        self.port.validate_checkpoint(&saved.children()[0])
    }
    fn restore_checkpoint(
        &mut self,
        saved: &fujimod_emulator_devices::checkpoint::Checkpoint,
    ) -> Result<(), String> {
        self.validate_checkpoint(saved)?;
        self.state = saved.decode(self.name())?;
        self.port.restore_checkpoint(&saved.children()[0])
    }
    fn fingerprint(&self) -> Result<Value, String> {
        Ok(self.checkpoint()?.state())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fujimod_emulator_devices::test_host::{TestHost, advance, write};
    use fujimod_emulator_ip::socionext::gpio::Pins;

    const BASE: u64 = 0xfff2_c000;
    const BUSY: Pad = Pad {
        bank: 0,
        pin: 7,
        line: 7,
    };
    const SENSOR: Pad = Pad {
        bank: 0,
        pin: 3,
        line: 3,
    };

    fn driver() -> (StepperPort, SharedPins) {
        let pins = Pins::new([0; 16]);
        let driver = StepperPort::new(
            Wiring {
                base: BASE,
                motors: vec![
                    Motor {
                        command: 0x2000,
                        busy: Some(BUSY),
                        sensor: Some((SENSOR, 2)),
                    },
                    Motor {
                        command: 0xa000,
                        busy: None,
                        sensor: None,
                    },
                ],
            },
            Timing { step: 10 },
            pins.clone(),
            ExternalLines::default(),
        )
        .unwrap();
        (driver, pins)
    }
    fn level(pins: &SharedPins, pad: Pad) -> bool {
        pins.lock().unwrap().input(pad.bank, pad.pin)
    }

    #[test]
    fn a_counted_move_holds_busy_and_steps_the_sensor() {
        let (mut d, pins) = driver();
        let mut h = TestHost::new();
        // X100F 2.12 focus word 0x280e: start, forward, 14 steps.
        write(&mut d, &mut h, BASE + SEND, 0x280e);
        assert!(level(&pins, BUSY));
        assert_eq!(d.next_deadline(0), Some(10));
        advance(&mut d, &mut h, 20);
        assert!(level(&pins, SENSOR), "two steps change the sensor once");
        advance(&mut d, &mut h, 139);
        assert!(level(&pins, BUSY));
        advance(&mut d, &mut h, 140);
        assert!(!level(&pins, BUSY));
        assert_eq!(d.state.motors[0].position, 14);
        assert_eq!(d.next_deadline(140), None);
    }

    #[test]
    fn a_run_lasts_until_a_word_without_start() {
        let (mut d, pins) = driver();
        let mut h = TestHost::new();
        // 0x2c00: start, reverse, no count: a run.
        write(&mut d, &mut h, BASE + SEND, 0x2c00);
        advance(&mut d, &mut h, 1000);
        assert!(level(&pins, BUSY));
        assert_eq!(d.state.motors[0].position, -100);
        // 0x2402: start clear, the run stops.
        write(&mut d, &mut h, BASE + SEND, 0x2402);
        assert!(!level(&pins, BUSY));
        advance(&mut d, &mut h, 2000);
        assert_eq!(d.state.motors[0].steps, 100);
    }

    #[test]
    fn settings_and_other_motors_leave_motor_a_alone() {
        let (mut d, pins) = driver();
        let mut h = TestHost::new();
        for word in [0x0461, 0x1081, 0xd554, 0xa800] {
            write(&mut d, &mut h, BASE + SEND, word);
        }
        assert_eq!(d.state.settings, 3);
        assert!(!level(&pins, BUSY));
        assert_eq!(d.state.motors[1].starts, 1);
        advance(&mut d, &mut h, 100);
        assert_eq!(d.state.motors[1].position, 10);
    }

    #[test]
    fn busy_and_sensor_reach_their_external_lines() {
        let pins = Pins::new([0; 16]);
        let lines = ExternalLines::default();
        let mut d = StepperPort::new(
            Wiring {
                base: BASE,
                motors: vec![Motor {
                    command: 0x2000,
                    busy: Some(BUSY),
                    sensor: None,
                }],
            },
            Timing { step: 10 },
            pins,
            lines.clone(),
        )
        .unwrap();
        let mut h = TestHost::new();
        write(&mut d, &mut h, BASE + SEND, 0x2801);
        assert_eq!(lines.levels(), 1 << BUSY.line);
        advance(&mut d, &mut h, 10);
        assert_eq!(lines.levels(), 0);
    }

    #[test]
    fn invalid_wiring_refuses() {
        let pins = Pins::new([0; 16]);
        let motor = |command| Motor {
            command,
            busy: None,
            sensor: None,
        };
        for motors in [
            vec![],
            vec![motor(0x2001)],
            vec![motor(0x2000), motor(0x2000)],
        ] {
            assert!(
                StepperPort::new(
                    Wiring { base: BASE, motors },
                    Timing { step: 1 },
                    pins.clone(),
                    ExternalLines::default()
                )
                .is_err()
            );
        }
    }

    #[test]
    fn checkpoint_resumes_a_moving_motor() {
        use fujimod_emulator_devices::test_host::assert_exact_restore;
        let (mut d, _) = driver();
        let mut h = TestHost::new();
        write(&mut d, &mut h, BASE + SEND, 0x2c00);
        advance(&mut d, &mut h, 35);
        assert_exact_restore(&mut d, |d| {
            let mut h = TestHost::new();
            advance(d, &mut h, 90);
        });
    }
}
