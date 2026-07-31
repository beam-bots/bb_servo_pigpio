<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Position Feedback

RC servos don't provide position feedback, but `BB.Sensor.OpenLoopPositionEstimator`
(from BB core) can estimate position based on commanded targets and timing. This
tutorial shows you how to set up and use position feedback.

## Prerequisites

- Completed [Basic Usage](2-basic-usage.md)
- A working servo joint

## Adding a Feedback Sensor

Add the sensor to your joint definition:

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
        sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}

        link :head
      end
    end
  end
end
```

The sensor requires the `actuator` option to know which actuator to subscribe
to. Unlike the command functions, this option takes the actuator's unique
*name*, not its path.

## How Position Feedback Works

Since RC servos don't report their actual position, the sensor estimates it:

1. **Actuator publishes motion** - When you call `set_position`, the actuator
   publishes a `BB.Message.Actuator.BeginMotion` message with the target position
   and expected arrival time

2. **Sensor subscribes** - The sensor receives these messages and tracks the
   target position and expected arrival time

3. **Position interpolation** - During movement, the sensor interpolates between
   the previous position and target based on elapsed time

4. **JointState publishing** - The sensor publishes `JointState` messages with
   the estimated position

### Timeline Example

```
Time 0ms:    Command sent (target: 45°, arrival: 500ms)
             Sensor receives command, starts interpolating

Time 100ms:  Estimated position: 9° (20% of the way)
Time 250ms:  Estimated position: 22.5° (50% of the way)
Time 500ms:  Estimated position: 45° (arrived)

Time 600ms:  Position stable at 45° (no change published)
Time 5000ms: Sync publish at 45° (max_silence reached)
```

## Sensor Options

```elixir
sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator,
  actuator: :servo,           # Required: actuator to subscribe to
  easing: :linear,            # Optional: interpolation easing function (default: :linear)
  publish_rate: ~u(50 hertz), # Optional: how often to check for changes (default: 50 Hz)
  max_silence: ~u(5 second)   # Optional: max time between publishes (default: 5s)
}
```

### easing

The easing function shapes how the estimated position interpolates from the
previous position to the target during movement. `:linear` (constant velocity)
is the default; sinusoidal, quadratic, cubic, and other curves are available.
See `BB.Sensor.OpenLoopPositionEstimator` for the full list.

### publish_rate

How often the sensor checks for position changes. Higher rates give smoother
feedback during movement but use more resources.

- `~u(50 hertz)` - Default, good for most applications
- `~u(100 hertz)` - Smoother feedback for fast movements
- `~u(10 hertz)` - Lower resource usage for slow-moving joints

### max_silence

Even when the position hasn't changed, the sensor publishes periodically to keep
subscribers in sync. This handles:

- Late subscribers that missed earlier updates
- Recovery from dropped messages
- Monitoring systems that expect regular updates

Set to a higher value if you want less traffic when idle.

## Subscribing to Position Updates

Subscribe to the sensor's JointState messages. Sensor topics are `[:sensor |
path]`, where the path runs from the root link down to the sensor:

```elixir
# Subscribe to the sensor topic
BB.subscribe(MyRobot, [:sensor, :base, :pan, :feedback])

# In your GenServer or process
def handle_info({:bb, _path, %BB.Message{payload: %BB.Message.Sensor.JointState{} = joint_state}}, state) do
  [position] = joint_state.positions
  IO.puts("Pan position: #{position} radians")
  {:noreply, state}
end
```

Subscribing to a shorter path receives everything beneath it, so
`[:sensor, :base]` picks up every sensor in the robot and `[:sensor]` picks up
all of them regardless of topology.

## Reading Current Position

You can also query the robot's current joint positions directly. These are
maintained by the runtime from published `JointState` messages, so they reflect
the estimator's output:

```elixir
iex> BB.Robot.Runtime.positions(MyRobot)
%{pan: 0.785}
```

`BB.Robot.Runtime.velocities/1` returns the matching velocities.

## Example: Position Logger

Here's a complete example that logs position changes:

```elixir
defmodule PositionLogger do
  use GenServer

  def start_link(robot) do
    GenServer.start_link(__MODULE__, robot, name: __MODULE__)
  end

  def init(robot) do
    BB.subscribe(robot, [:sensor, :base, :pan, :feedback])
    {:ok, %{robot: robot}}
  end

  def handle_info({:bb, _path, %BB.Message{payload: %BB.Message.Sensor.JointState{} = js}}, state) do
    [position] = js.positions
    degrees = position * 180 / :math.pi()
    IO.puts("[#{DateTime.utc_now()}] Pan: #{Float.round(degrees, 1)}°")
    {:noreply, state}
  end
end
```

Start the logger:

```elixir
{:ok, _} = MyRobot.start_link()
{:ok, _} = PositionLogger.start_link(MyRobot)

{:ok, cmd} = MyRobot.arm()
{:ok, :armed, _opts} = BB.Command.await(cmd)

# Move the servo and watch the logs
BB.Actuator.set_position(MyRobot, [:base, :pan, :servo], 0.785)
# Output:
# [2025-01-15 10:30:00.000000Z] Pan: 9.0°
# [2025-01-15 10:30:00.020000Z] Pan: 18.0°
# [2025-01-15 10:30:00.040000Z] Pan: 27.0°
# ... (interpolated positions during movement)
# [2025-01-15 10:30:00.500000Z] Pan: 45.0°
```

## Example: Wait for Movement Complete

Wait for the servo to reach its target position:

```elixir
defmodule ServoHelper do
  def move_and_wait(robot, actuator, target, timeout \\ 5000) do
    BB.subscribe(robot, [:sensor, :base, :pan, :feedback])

    BB.Actuator.set_position!(robot, actuator, target)

    wait_for_position(target, timeout)
  end

  defp wait_for_position(target, timeout) do
    receive do
      {:bb, _path, %BB.Message{payload: %BB.Message.Sensor.JointState{positions: [position]}}}
      when abs(position - target) < 0.01 ->
        :ok
    after
      timeout -> {:error, :timeout}
    end
  end
end

# Usage, once the robot is armed
:ok = ServoHelper.move_and_wait(MyRobot, :servo, 0.785)
IO.puts("Servo reached target!")
```

This uses `set_position!/4`, which addresses the actuator by its unique name
rather than its path, so the helper doesn't need to know where in the topology
the servo sits.

## Limitations

Remember that this is **estimated** position, not actual position:

- The servo might not reach the target (blocked, insufficient torque)
- The servo might overshoot or oscillate
- The timing might not match the real servo exactly

For applications requiring precise position feedback, consider adding a physical
sensor (potentiometer, encoder) to your servo or using a servo with built-in
feedback.

## Next Steps

You now have a complete servo setup with position feedback. Explore the BB
framework documentation to learn about:

- Trajectory planning for smooth multi-joint movements
- Inverse kinematics for end-effector positioning
- State machines for complex behaviours
