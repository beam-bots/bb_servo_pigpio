<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# BB.Servo.Pigpio Usage Rules

`bb_servo_pigpio` provides `BB.Servo.Pigpio.Actuator`, a `BB.Actuator`
implementation that drives an RC servo from a Raspberry Pi GPIO pin via the
`pigpiod` daemon (over `pigpiox`) for [Beam Bots](https://hexdocs.pm/bb). For BB
framework basics, see `bb`'s rules (`mix usage_rules.sync <file> bb:all`); this
file covers only what's specific to this driver.

## Core principles

1. **It is a per-joint actuator, not a robot-level controller.** There is no bus
   manager to declare — `pigpiod` is the "bus". Attach the actuator directly to
   the joint it drives; the framework wraps it in `BB.Actuator.Server` and
   supervises it for you.
2. **Configuration is derived from the joint, not the actuator options.** The
   servo's travel and speed come from the joint's `limit` (lower, upper,
   velocity) via the injected `motor_profile`. You do not set a rotation range
   or speed on the actuator. The lower/upper limits map linearly onto the pulse
   range; the velocity limit sets the motion timing.
3. **It drives real hardware.** `pigpiod` must be running on the target host
   (`sudo pigpiod`), and `disarm/1` cuts the PWM output (pulse width `0`). Under
   simulation the framework swaps in `BB.Sim.Actuator`, so no Pi is required to
   run the robot in sim.

## Wiring it in

Attach the actuator to a `revolute`/`prismatic` joint, paired with the core
open-loop estimator for position feedback (RC servos have no feedback of their
own):

```elixir
joint :shoulder, type: :revolute do
  limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)

  actuator :servo, {BB.Servo.Pigpio.Actuator, pin: 17}
  sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}
end
```

The joint **must** define lower, upper, and velocity limits — the actuator
refuses to start (`BB.Error.Invalid.JointConfig`) without them.

Once the robot is **armed** (see `bb:safety-and-commands`), command it in
joint-space through `BB.Actuator`; the framework applies the joint transmission
and hands the driver motor-space values:

```elixir
BB.Actuator.set_position(MyRobot.Robot, [:shoulder, :servo], 0.5)
```

## Options

Passed in the `{Module, opts}` tuple.

| Option | Default | Meaning |
|---|---|---|
| `:pin` | required | Broadcom GPIO pin number for the PWM output |
| `:min_pulse` | `500` | Pulse width (µs) mapped to the joint's lower limit |
| `:max_pulse` | `2500` | Pulse width (µs) mapped to the joint's upper limit |
| `:update_speed` | `~u(50 hertz)` | PWM update frequency (a `~u` unit value) |

`:min_pulse`/`:max_pulse` are per-servo hardware calibration — adjust them to
match the datasheet, not to change the joint's range.

## Anti-patterns

- **Don't declare it as a `controller` or invent a bus manager.** It is a
  `BB.Actuator` on a joint. `pigpiod` is a system daemon the driver talks to
  directly; there is no robot-level process to add.
- **Don't put the servo's travel range or speed on the actuator.** Those come
  from the joint `limit`. To reverse direction, use the joint's `transmission`
  (`reversed?`), not an actuator option — there is no `reverse?` option.
- **Don't skip `pigpiod` or arming.** `init/1` opens the pin through the daemon,
  so it must be running first; and a disarmed robot ignores motion commands —
  arm before you expect the servo to move.

## Further reading

- [bb_servo_pigpio docs](https://hexdocs.pm/bb_servo_pigpio)
- `bb`'s actuator and safety rules (`bb:actuators`, `bb:safety-and-commands`)
  and [Writing an Actuator](https://hexdocs.pm/bb/12-writing-an-actuator.html)
