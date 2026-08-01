<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# AGENTS.md

This file provides guidance to AI coding assistants when working with code in this repository.

## Project Overview

BB.Servo.Pigpio is an Elixir library that provides Beam Bots (BB) integration for driving RC servos via pigpio on Raspberry Pi. It implements an actuator module that integrates with the BB robot framework's joint system.

## Build and Development Commands

```bash
# Run all checks (compile, tests, dialyzer, credo, formatting, etc.)
mix check --no-retry

# Run tests
mix test

# Run a single test file
mix test test/bb/servo/pigpio/actuator_test.exs

# Run a specific test by line number
mix test test/bb/servo/pigpio/actuator_test.exs:36

# Format code
mix format

# Generate documentation
mix docs
```

## Architecture

### Core Components

**Actuator** (`lib/bb/servo/pigpio/actuator.ex`)
- GenServer that controls servo position via PWM through pigpiox
- Derives position limits and velocity from BB joint constraints
- Maps joint position range to PWM pulse width range (default 500-2500 microseconds)
- Publishes `BB.Message.Actuator.BeginMotion` messages after each command
- Accepts commands sent via:
  - `BB.Actuator.set_position/4` (pubsub)
  - `BB.Actuator.set_position!/4` (direct)
  - `BB.Actuator.set_position_sync/5` (synchronous)

  All three arrive at `handle_command/2`; `BB.Actuator.Server` checks arm state and applies
  the joint's transmission before the driver sees them.

### Integration Pattern

The actuator is designed to be used within a BB robot joint definition, paired with the
`BB.Sensor.OpenLoopPositionEstimator` from BB core for position feedback:

```elixir
joint :shoulder do
  type :revolute

  limit lower: ~u(-45 degree), upper: ~u(45 degree),
        velocity: ~u(60 degree_per_second), effort: ~u(1 newton_meter)

  actuator :servo, {BB.Servo.Pigpio.Actuator, pin: 17}
  sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}
end
```

### Command Interface

Send commands using the `BB.Actuator` module:

```elixir
# Arm first — a disarmed robot refuses commands before they reach the driver
{:ok, cmd} = MyRobot.arm()
{:ok, :armed, _} = BB.Command.await(cmd)

# Pubsub delivery (for orchestration/logging). Takes a name or a full path.
BB.Actuator.set_position(MyRobot, :servo, 0.5)
BB.Actuator.set_position(MyRobot, [:base, :shoulder, :servo], 0.5)

# Direct delivery (fire-and-forget, lower latency)
BB.Actuator.set_position!(MyRobot, :servo, 0.5)

# Synchronous delivery (with acknowledgement)
{:ok, :accepted} = BB.Actuator.set_position_sync(MyRobot, :servo, 0.5)
```

### Key Dependencies

- `bb` - Beam Bots robot framework (provides `BB.Message`, `BB.Actuator`, `BB.Robot`, unit handling)
- `Pigpiox.Socket` - Communication with pigpio daemon (mocked in tests)
- `Spark.Options` - Option validation with unit type support

### Test Support

Tests use Mimic for mocking:
- `BB` and `BB.Robot` modules for pub/sub
- `Pigpiox.Socket` for hardware communication

Test support modules in `test/support/` provide stubs and fixtures.

### Message Flow

```
BB.Actuator.set_position()
    |
    v
Actuator receives Command.Position
    |
    v
Actuator sends PWM to pigpiod
    |
    v
Actuator publishes BeginMotion
    |
    v
OpenLoopPositionEstimator interpolates position
    |
    v
Sensor publishes JointState
```

## Licensing headers

Every source file must carry an SPDX header — a `#`-style comment for code, an
HTML comment for Markdown, or a `<file>.license` sidecar for files that can't
hold comments (binaries, JSON, lockfiles). `mix check` runs `reuse lint` and
fails the build if one is missing.

When you create a new file, its `SPDX-FileCopyrightText` line must credit **the
user you are working for** — not you (the agent), and not this repo's original
author. Take their name from `git config user.name` (add their `user.email` if
you include one) and use the current year. Match the neighbouring files'
`SPDX-License-Identifier` (usually `Apache-2.0`):

```
SPDX-FileCopyrightText: <current year> <your user's name>

SPDX-License-Identifier: Apache-2.0
```

Never copy an existing file's copyright line onto a new file — that credits the
wrong person. When you only edit an existing file, leave its headers unchanged.
