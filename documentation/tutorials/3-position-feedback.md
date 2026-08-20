<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Position Feedback

RC servos don't provide position feedback, but `BB.Sensor.OpenLoopPositionEstimator`
(from BB core) can estimate position based on commanded targets and timing. This
tutorial shows you how to set up and use position feedback.

The estimator isn't an extra you add once everything else works. `BB.Robot.State`
is written from `BB.Message.Sensor.JointState` messages and from nothing else, so
until something publishes them, a joint reads as parked at its initial position
no matter how far the servo has actually travelled — and forward kinematics, the
URDF visualisers and inverse kinematics all work from that. This driver reads
nothing back from the hardware, so on a servo joint the estimator is the only
thing that can publish those messages. BB warns at compile time about a joint
nothing reports on.

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

        limit lower: ~u(-90 degree), upper: ~u(90 degree),
              velocity: ~u(60 degree_per_second), effort: ~u(1 newton_meter)

        actuator :servo, {BB.Servo.Pigpio.Actuator, pin: 17}
        sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}

        link :head do
        end
      end
    end
  end
end
```

The sensor requires the `actuator` option to know which actuator to subscribe to.
Declaring it is also what silences the compile-time warning for this joint.

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

Subscribe to the sensor's JointState messages:

```elixir
# Subscribe to the sensor topic. The path is the sensor's full path through the
# topology — every link and joint from the root, not just the joint it hangs
# off. Messages are dispatched to the exact published path and its ancestors,
# so a short path matches nothing at all.
{:ok, _} = BB.subscribe(MyRobot, [:sensor, :base, :pan, :feedback])

# In your GenServer or process. Subscribers receive `{:bb, path, message}`.
def handle_info({:bb, _path, %BB.Message{payload: %BB.Message.Sensor.JointState{} = joint_state}}, state) do
  [position] = joint_state.positions
  IO.puts("Pan position: #{position} radians")
  {:noreply, state}
end
```

## Reading Current Position

You can also query the robot's state directly:

```elixir
# Get current joint positions
# Positions are keyed by joint name
%{pan: pan_position} = BB.Robot.Runtime.configurations(MyRobot)
```

## Example: Position Logger

Here's a complete example that logs position changes:

```elixir
defmodule PositionLogger do
  use GenServer

  def start_link(robot) do
    GenServer.start_link(__MODULE__, robot, name: __MODULE__)
  end

  def init(robot) do
    {:ok, _} = BB.subscribe(robot, [:sensor, :base, :pan, :feedback])
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
{:ok, :armed, _} = BB.Command.await(cmd)

# Move the servo and watch the logs
:ok = BB.Actuator.set_position(MyRobot, :servo, 0.785)
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
    # Subscribe to sensor updates
    {:ok, _} = BB.subscribe(robot, [:sensor, :base, :pan, :feedback])

    with :ok <- BB.Actuator.set_position(robot, actuator, target) do
      wait_for_position(target, timeout)
    end
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

# Usage
:ok = ServoHelper.move_and_wait(MyRobot, :servo, 0.785)
IO.puts("Servo reached target!")
```

## Limitations

Remember that this is **estimated** position, not actual position:

- The servo might not reach the target (blocked, insufficient torque)
- The servo might overshoot or oscillate
- The timing might not match the real servo exactly

For applications requiring precise position feedback, consider adding a physical
sensor (potentiometer, encoder) to your servo or using a servo with built-in
feedback. A driver for hardware that really does read position back declares
`c:BB.Actuator.capabilities/1` and publishes `JointState` itself; swapping to one
of those is the one case where you drop the estimator rather than repointing it.

## Next Steps

You now have a complete servo setup with position feedback. Explore the BB
framework documentation to learn about:

- Trajectory planning for smooth multi-joint movements
- Inverse kinematics for end-effector positioning
- State machines for complex behaviours
