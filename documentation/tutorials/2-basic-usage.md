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

  # A robot starts disarmed and won't move until armed. Arming is a command, so
  # the robot has to declare one.
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

        # Define the joint's motion limits
        limit lower: ~u(-90 degree),
              upper: ~u(90 degree),
              velocity: ~u(60 degree_per_second),
              effort: ~u(1 newton_meter)

        # Attach the servo actuator
        actuator :servo, {BB.Servo.Pigpio.Actuator, pin: 17}

        # The servo reports nothing back, so estimate where it is
        sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}

        link :head do
          # Child links go here
        end
      end
    end
  end
end
```

The `sensor` entry is the one part that isn't about hardware. `BB.Robot.State` is
written from `BB.Message.Sensor.JointState` messages and from nothing else, and an
RC servo has no return path, so without
`BB.Sensor.OpenLoopPositionEstimator` the joint reads as parked at its initial
position however far the servo travels. BB warns at compile time about a joint
nothing reports on.
[Position Feedback](3-position-feedback.md) covers how the estimate is produced.

## Understanding the Configuration

### Joint Limits

The `limit` block defines the physical constraints of your joint:

- `lower` - Minimum position (maps to servo's minimum pulse)
- `upper` - Maximum position (maps to servo's maximum pulse)
- `velocity` - Maximum rotation speed (used for timing calculations)

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

A robot starts `:disarmed` and refuses every command until armed — the
framework drops them before they reach the driver, so a disarmed servo simply
won't move. Arm it through the command declared above:

```elixir
iex> {:ok, cmd} = MyRobot.arm()
iex> BB.Command.await(cmd)
{:ok, :armed, [next_state: :idle]}
```

Arming runs whatever prearm checks the robot defines, which is why it's a
command rather than a flag. Don't reach for `BB.Safety` directly — that skips
them.

## Commanding the Servo

Send position commands to the actuator. An actuator can be addressed by its
unique name, as below, or by its full path through the topology
(`[:base, :pan, :servo]` here):

```elixir
# Move to centre (0 degrees)
:ok = BB.Actuator.set_position(MyRobot, :servo, 0.0)

# Move to -45 degrees (in radians)
:ok = BB.Actuator.set_position(MyRobot, :servo, -0.785)

# Using the unit sigil for degrees
import BB.Unit
:ok = BB.Actuator.set_position(MyRobot, :servo, ~u(-45 degree) |> BB.Robot.Units.to_radians())
```

> **Note:** BB uses radians internally. Convert degrees to radians when sending
> commands, or use the unit conversion functions.

`set_position/4` publishes the command for observers and waits for the actuator
to take it, so it answers `:ok` or `{:error, reason}` — a refusal is something
you find out about rather than assume away:

```elixir
case BB.Actuator.set_position(MyRobot, :servo, -0.785) do
  :ok -> :moving
  {:error, reason} -> Logger.error(Exception.message(reason))
end
```

For a control path that can't afford the round trip, `delivery: :direct` casts to
the actuator and publishes nothing. It always returns `:ok`, so a refusal reaches
only the log and telemetry:

```elixir
BB.Actuator.set_position(MyRobot, :servo, -0.785, delivery: :direct)
```

## Position Clamping

The actuator automatically clamps positions to the joint limits:

```elixir
# Joint limits are -90° to +90°
# This command will be clamped to +90° (π/2 radians)
BB.Actuator.set_position(MyRobot, :servo, 3.14)  # Requested: 180°, actual: 90°
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

        limit lower: ~u(-90 degree), upper: ~u(90 degree),
              velocity: ~u(90 degree_per_second), effort: ~u(1 newton_meter)

        # Component names are unique across the whole robot, so each servo and
        # estimator needs its own name — not `:servo` twice.
        actuator :pan_servo, {BB.Servo.Pigpio.Actuator, pin: 17}
        sensor :pan_feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :pan_servo}

        link :pan_platform do
          joint :tilt do
            type :revolute

            limit lower: ~u(-45 degree), upper: ~u(45 degree),
                  velocity: ~u(60 degree_per_second), effort: ~u(1 newton_meter)

            actuator :tilt_servo, {BB.Servo.Pigpio.Actuator, pin: 18}
            sensor :tilt_feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :tilt_servo}

            link :camera_mount do
              # Camera attached here
            end
          end
        end
      end
    end
  end
end
```

Command both servos:

```elixir
{:ok, cmd} = PanTiltRobot.arm()
{:ok, :armed, _} = BB.Command.await(cmd)

# Look left and up — these name the actuators, not the joints
:ok = BB.Actuator.set_position(PanTiltRobot, :pan_servo, -0.785)    # -45°
:ok = BB.Actuator.set_position(PanTiltRobot, :tilt_servo, 0.524)    # +30°
```

## Next Steps

To get position feedback from your servos, see [Position Feedback](3-position-feedback.md).
