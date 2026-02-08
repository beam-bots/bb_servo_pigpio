<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

<img src="https://github.com/beam-bots/bb/blob/main/logos/beam_bots_logo.png?raw=true" alt="Beam Bots Logo" width="250" />

# Beam Bots Pigpio servo control

[![CI](https://github.com/beam-bots/bb_servo_pigpio/actions/workflows/ci.yml/badge.svg)](https://github.com/beam-bots/bb_servo_pigpio/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](https://opensource.org/licenses/Apache-2.0)
[![Hex version badge](https://img.shields.io/hexpm/v/bb_servo_pigpio.svg)](https://hex.pm/packages/bb_servo_pigpio)
[![REUSE status](https://api.reuse.software/badge/github.com/beam-bots/bb_servo_pigpio)](https://api.reuse.software/info/github.com/beam-bots/bb_servo_pigpio)

# BB.Servo.Pigpio

BB integration for driving RC servos via pigpio on Raspberry Pi.

This library provides an actuator module for controlling RC servos directly
connected to Raspberry Pi GPIO pins using the pigpio daemon.

## Installation

Add `bb_servo_pigpio` to your list of dependencies in `mix.exs`:

```elixir
def deps do
  [
    {:bb_servo_pigpio, "~> 0.5.0"}
  ]
end
```

## Requirements

- Raspberry Pi with pigpio daemon running (`sudo pigpiod`)
- BB framework (`~> 0.2`)

## Usage

Define a joint with a servo actuator in your robot DSL:

```elixir
defmodule MyRobot do
  use BB

  topology do
    link :base do
      joint :shoulder, type: :revolute do
        limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.Pigpio.Actuator, pin: 17}
        sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}

        link :arm do
          # ...
        end
      end
    end
  end
end
```

The actuator automatically derives its configuration from the joint limits - no
need to specify servo rotation range or speed separately.

## Sending Commands

Use the `BB.Actuator` module to send commands to servos. Three delivery methods
are available:

### Pubsub Delivery (for orchestration)

Commands are published via pubsub, enabling logging, replay, and multi-subscriber
patterns:

```elixir
# Send position command via pubsub
BB.Actuator.set_position(MyRobot, [:base, :shoulder, :servo], 0.5)

# With options
BB.Actuator.set_position(MyRobot, [:base, :shoulder, :servo], 0.5,
  command_id: make_ref()
)
```

### Direct Delivery (for time-critical control)

Commands bypass pubsub for lower latency. Use when responsiveness matters more
than observability:

```elixir
# Fire-and-forget
BB.Actuator.set_position!(MyRobot, :servo, 0.5)
```

### Synchronous Delivery (with acknowledgement)

Wait for the actuator to acknowledge the command:

```elixir
case BB.Actuator.set_position_sync(MyRobot, :servo, 0.5) do
  {:ok, :accepted} -> :ok
  {:error, reason} -> handle_error(reason)
end
```

## Components

### Actuator

`BB.Servo.Pigpio.Actuator` controls servo position via PWM.

**Options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `pin` | integer | required | GPIO pin number |
| `min_pulse` | integer | 500 | Minimum PWM pulse width (microseconds) |
| `max_pulse` | integer | 2500 | Maximum PWM pulse width (microseconds) |
| `reverse?` | boolean | false | Reverse rotation direction |
| `update_speed` | unit | 50 Hz | PWM update frequency |

**Behaviour:**

- Maps joint position limits directly to PWM range
- Clamps commanded positions to joint limits
- Publishes `BB.Message.Actuator.BeginMotion` after each command
- Calculates expected arrival time based on joint velocity limit

### Sensor

Use `BB.Sensor.OpenLoopPositionEstimator` from the BB core library for position
feedback. It subscribes to actuator `BeginMotion` messages and interpolates
position during movement.

```elixir
sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}
```

## How It Works

### Position Mapping

The actuator maps the joint's position limits to the servo's PWM range:

```
Joint lower limit  ->  min_pulse (500 microseconds)
Joint upper limit  ->  max_pulse (2500 microseconds)
Joint centre       ->  mid_pulse (1500 microseconds)
```

For a joint with limits `-45 degrees` to `+45 degrees`:
- `-45 degrees` maps to 500 microseconds
- `0 degrees` maps to 1500 microseconds
- `+45 degrees` maps to 2500 microseconds

### Position Feedback

Since RC servos don't provide position feedback, the open-loop position
estimator estimates position based on commanded targets and expected arrival
times:

1. Actuator sends command and publishes `BeginMotion` with expected arrival time
2. Sensor receives `BeginMotion` and interpolates position during movement
3. After arrival time, sensor reports the target position

This provides realistic position feedback for trajectory planning and monitoring.

### Motion Lifecycle

When a position command is processed:

1. Actuator clamps position to joint limits
2. Converts angle to PWM pulse width
3. Sends PWM command to pigpiod
4. Publishes `BB.Message.Actuator.BeginMotion` with:
   - `initial_position` - where the servo was
   - `target_position` - where it's going
   - `expected_arrival` - when it should arrive (monotonic milliseconds)
   - `command_id` - correlation ID (if provided)
   - `command_type` - `:position`

## Documentation

Full documentation is available at [HexDocs](https://hexdocs.pm/bb_servo_pigpio).
