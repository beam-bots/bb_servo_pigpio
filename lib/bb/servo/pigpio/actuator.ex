# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Pigpio.Actuator do
  @moduledoc """
  An actuator GenServer that uses Pigpiox to drive a servo.

  This actuator derives its configuration from the joint constraints defined in the robot:
  - Position limits from `joint.limits.lower` and `joint.limits.upper`
  - Velocity limit from `joint.limits.velocity`
  - PWM range maps linearly to the joint's position range

  When a position command is received, the actuator:
  1. Clamps the position to joint limits
  2. Converts to PWM pulse width
  3. Sends PWM command to pigpiox
  4. Publishes a `BB.Message.Actuator.BeginMotion` for sensors to consume
  """
  use GenServer
  import BB.Unit
  import BB.Unit.Option

  alias BB.Cldr.Unit, as: CldrUnit
  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command
  alias BB.Robot.Units

  @options Spark.Options.new!(
             bb: [
               type: :map,
               doc: "Automatically set by the robot supervisor",
               required: true
             ],
             pin: [
               type: :pos_integer,
               doc: "The GPIO pin to use for servo output",
               required: true
             ],
             min_pulse: [
               type: :pos_integer,
               doc: "The minimum PWM pulse that can be sent to the servo",
               default: 500
             ],
             max_pulse: [
               type: :pos_integer,
               doc: "The maximum PWM pulse that can be sent to the servo",
               default: 2500
             ],
             reverse?: [
               type: :boolean,
               doc: "Reverse the servo rotation direction?",
               default: false
             ],
             update_speed: [
               type: unit_type(compatible: :hertz),
               doc: "The servo update frequency",
               default: ~u(50 hertz)
             ]
           )

  def init(opts) do
    with {:ok, opts} <- Spark.Options.validate(opts, @options),
         {:ok, state} <- build_state(opts),
         {:ok, _} <-
           Pigpiox.Socket.command(:set_PWM_frequency, state.pin, round(state.update_speed)),
         {:ok, _} <-
           Pigpiox.Socket.command(:set_servo_pulsewidth, state.pin, round(state.current_pulse)) do
      {:ok, state}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  defp build_state(opts) do
    opts = Map.new(opts)
    [name, joint_name | _] = Enum.reverse(opts.bb.path)
    robot = opts.bb.robot.robot()

    with {:ok, joint} <- fetch_joint(robot, joint_name),
         {:ok, limits} <- validate_joint_limits(joint, joint_name) do
      lower_limit = limits.lower
      upper_limit = limits.upper
      range = upper_limit - lower_limit
      center_angle = (lower_limit + upper_limit) / 2
      velocity_limit = limits.velocity
      pulse_range = opts.max_pulse - opts.min_pulse

      update_speed =
        opts.update_speed
        |> CldrUnit.convert!(:hertz)
        |> Units.extract_float()

      initial_pulse = (opts.max_pulse + opts.min_pulse) / 2

      state = %{
        bb: opts.bb,
        pin: opts.pin,
        min_pulse: opts.min_pulse,
        max_pulse: opts.max_pulse,
        reverse?: opts.reverse?,
        update_speed: update_speed,
        lower_limit: lower_limit,
        upper_limit: upper_limit,
        center_angle: center_angle,
        range: range,
        velocity_limit: velocity_limit,
        pulse_range: pulse_range,
        current_pulse: initial_pulse,
        current_angle: center_angle,
        name: name,
        joint_name: joint_name
      }

      {:ok, state}
    end
  end

  defp fetch_joint(robot, joint_name) do
    case BB.Robot.get_joint(robot, joint_name) do
      nil -> {:error, {:joint_not_found, joint_name}}
      joint -> {:ok, joint}
    end
  end

  defp validate_joint_limits(%{type: :continuous}, joint_name) do
    {:error, {:unsupported_joint_type, :continuous, joint_name}}
  end

  defp validate_joint_limits(%{limits: nil}, joint_name) do
    {:error, {:no_limits_defined, joint_name}}
  end

  defp validate_joint_limits(%{limits: %{lower: nil}}, joint_name) do
    {:error, {:missing_limit, :lower, joint_name}}
  end

  defp validate_joint_limits(%{limits: %{upper: nil}}, joint_name) do
    {:error, {:missing_limit, :upper, joint_name}}
  end

  defp validate_joint_limits(%{limits: limits}, _joint_name) do
    {:ok, limits}
  end

  # Handle position commands via pubsub (from BB.Actuator.set_position/4)
  def handle_info({:bb, _path, %Message{payload: %Command.Position{} = cmd}}, state) do
    {:noreply, state} = do_set_position(cmd.position, cmd.command_id, state)
    {:noreply, state}
  end

  # Handle position commands via direct cast (from BB.Actuator.set_position!/4)
  def handle_cast({:command, %Message{payload: %Command.Position{} = cmd}}, state) do
    do_set_position(cmd.position, cmd.command_id, state)
  end

  # Handle position commands via direct call (from BB.Actuator.set_position_sync/5)
  def handle_call({:command, %Message{payload: %Command.Position{} = cmd}}, _from, state) do
    {:noreply, new_state} = do_set_position(cmd.position, cmd.command_id, state)
    {:reply, {:ok, :accepted}, new_state}
  end

  defp do_set_position(angle, command_id, state) when is_integer(angle),
    do: do_set_position(angle * 1.0, command_id, state)

  defp do_set_position(angle, command_id, state) do
    clamped_angle = clamp_angle(angle, state)
    new_pulse = angle_to_pulse(clamped_angle, state)

    with {:ok, _} <- Pigpiox.Socket.command(:set_servo_pulsewidth, state.pin, new_pulse) do
      travel_distance = abs(state.current_angle - clamped_angle)
      travel_time_ms = round(travel_distance / state.velocity_limit * 1000)
      expected_arrival = System.monotonic_time(:millisecond) + travel_time_ms

      message_opts =
        [
          initial_position: state.current_angle,
          target_position: clamped_angle,
          expected_arrival: expected_arrival,
          command_type: :position
        ]
        |> maybe_add_opt(:command_id, command_id)

      message = Message.new!(BeginMotion, state.joint_name, message_opts)

      BB.publish(state.bb.robot, [:actuator | state.bb.path], message)

      {:noreply, %{state | current_pulse: new_pulse, current_angle: clamped_angle}}
    end
  end

  defp maybe_add_opt(opts, _key, nil), do: opts
  defp maybe_add_opt(opts, key, value), do: Keyword.put(opts, key, value)

  defp clamp_angle(angle, state) do
    angle
    |> max(state.lower_limit)
    |> min(state.upper_limit)
  end

  defp angle_to_pulse(angle, state) do
    normalised = (angle - state.lower_limit) / state.range

    pulse =
      if state.reverse? do
        state.max_pulse - normalised * state.pulse_range
      else
        state.min_pulse + normalised * state.pulse_range
      end

    round(pulse)
  end
end
