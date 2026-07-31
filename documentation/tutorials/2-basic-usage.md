<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Basic Usage

This tutorial shows you how to define a servo-controlled joint in your BB robot.

## Prerequisites

- Completed [Getting Started](1-getting-started.md)
- Servo wired to GPIO pin 17

## Defining a Robot with a Servo Joint

Create a robot module with a revolute joint controlled by a servo:

```elixir
defmodule MyRobot do
  use BB

  commands do
    command :arm do
      handler BB.Command.Arm
      allowed_states [:disarmed]
    end

    command :disarm do
      handler BB.Command.Disarm
      allowed_states [:idle]
    end
  end

  topology do
    link :base do
      joint :pan do
        type :revolute

        limit lower: ~u(-90 degree),
              upper: ~u(90 degree),
              effort: ~u(0.2 newton_meter),
              velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.Pigpio.Actuator, pin: 17}

        link :head
      end
    end
  end
end
```

The `commands` section declares the arm and disarm commands. A robot starts
`:disarmed` and won't drive hardware until armed, so every robot needs them —
see [Arming the Robot](#arming-the-robot) below.

## Understanding the Configuration

### Joint Limits

The `limit` block defines the physical constraints of your joint:

- `lower` - Minimum position (maps to servo's minimum pulse)
- `upper` - Maximum position (maps to servo's maximum pulse)
- `effort` - Maximum torque the joint may be commanded to apply
- `velocity` - Maximum rotation speed (used for timing calculations)

`effort` and `velocity` are required by BB; `lower` and `upper` are optional to
BB but required by this actuator, which maps them onto the pulse range.

These values are used by the actuator to:
1. Map positions to PWM pulse widths
2. Clamp commanded positions to safe values
3. Calculate expected movement duration

### Actuator Options

The actuator accepts these options:

```elixir
actuator :servo, {BB.Servo.Pigpio.Actuator,
  pin: 17,           # Required: GPIO pin number
  min_pulse: 500,    # Optional: minimum pulse width in µs (default: 500)
  max_pulse: 2500,   # Optional: maximum pulse width in µs (default: 2500)
  update_speed: ~u(50 hertz)  # Optional: PWM update frequency (default: 50 Hz)
}
```

Most servos work well with the defaults. Adjust `min_pulse` and `max_pulse` if
your servo has different endpoints.

## Starting the Robot

Start your robot in your application supervision tree:

```elixir
defmodule MyApp.Application do
  use Application

  def start(_type, _args) do
    children = [
      MyRobot
    ]

    opts = [strategy: :one_for_one, name: MyApp.Supervisor]
    Supervisor.start_link(children, opts)
  end
end
```

Or start it manually in IEx:

```elixir
iex> MyRobot.start_link()
{:ok, #PID<0.123.0>}
```

## Arming the Robot

The robot boots `:disarmed` with the servo's PWM output cut. Run the `arm`
command before commanding motion:

```elixir
iex> {:ok, cmd} = MyRobot.arm()
iex> {:ok, :armed, _opts} = BB.Command.await(cmd)
```

Each command declared in the `commands` section becomes a function on the robot
module. `MyRobot.disarm/0` reverses it, cutting the servo's pulse again.

## Commanding the Servo

Send position commands to the actuator. `set_position/4` takes the actuator's
full path through the topology — every link and joint name from the root down,
ending in the actuator's own name:

```elixir
# Move to centre (0 degrees)
BB.Actuator.set_position(MyRobot, [:base, :pan, :servo], 0.0)

# Move to -45 degrees (in radians)
BB.Actuator.set_position(MyRobot, [:base, :pan, :servo], -0.785)

# Using the unit sigil for degrees
import BB.Unit
BB.Actuator.set_position(MyRobot, [:base, :pan, :servo], ~u(-45 degree) |> BB.Robot.Units.to_radians())
```

If you'd rather not spell out the path, `set_position!/4` addresses the actuator
by its unique name instead, delivering the command directly rather than over
pubsub:

```elixir
BB.Actuator.set_position!(MyRobot, :servo, -0.785)
```

> **Note:** BB uses radians internally. Convert degrees to radians when sending
> commands, or use the unit conversion functions.

## Position Clamping

The actuator automatically clamps positions to the joint limits:

```elixir
# Joint limits are -90° to +90°
# This command will be clamped to +90° (π/2 radians)
BB.Actuator.set_position(MyRobot, [:base, :pan, :servo], 3.14)  # Requested: 180°, actual: 90°
```

## Reversing Direction

If your servo rotates in the opposite direction to what you expect, reverse the
actuator's joint transmission:

```elixir
actuator :servo, {BB.Servo.Pigpio.Actuator, pin: 17} do
  transmission do
    reversed? true
  end
end
```

BB applies the transmission before passing motor-space positions to the Pigpio
actuator, so direction reversal does not belong in the actuator options.

## Example: Pan-Tilt Head

Here's a complete example with two servos for a pan-tilt mechanism:

```elixir
defmodule PanTiltRobot do
  use BB

  commands do
    command :arm do
      handler BB.Command.Arm
      allowed_states [:disarmed]
    end

    command :disarm do
      handler BB.Command.Disarm
      allowed_states [:idle]
    end
  end

  topology do
    link :base do
      joint :pan do
        type :revolute

        limit lower: ~u(-90 degree),
              upper: ~u(90 degree),
              effort: ~u(0.2 newton_meter),
              velocity: ~u(90 degree_per_second)

        actuator :pan_servo, {BB.Servo.Pigpio.Actuator, pin: 17}

        link :pan_platform do
          joint :tilt do
            type :revolute

            limit lower: ~u(-45 degree),
                  upper: ~u(45 degree),
                  effort: ~u(0.2 newton_meter),
                  velocity: ~u(60 degree_per_second)

            actuator :tilt_servo, {BB.Servo.Pigpio.Actuator, pin: 18}

            link :camera_mount
          end
        end
      end
    end
  end
end
```

Component names must be unique across the whole robot — BB registers every
process under its name — so the two servos are `:pan_servo` and `:tilt_servo`
rather than both being `:servo`.

Command both servos:

```elixir
{:ok, cmd} = PanTiltRobot.arm()
{:ok, :armed, _opts} = BB.Command.await(cmd)

# Look left and up
BB.Actuator.set_position(PanTiltRobot, [:base, :pan, :pan_servo], -0.785)                        # -45°
BB.Actuator.set_position(PanTiltRobot, [:base, :pan, :pan_platform, :tilt, :tilt_servo], 0.524)  # +30°
```

## Next Steps

To get position feedback from your servos, see [Position Feedback](3-position-feedback.md).
