# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Pigpio.Actuator do
  @moduledoc """
  An actuator that uses Pigpiox to drive a servo.

  This actuator derives its configuration from the joint constraints defined in the robot:
  - Position limits from `joint.limits.lower` and `joint.limits.upper`
  - Velocity limit from `joint.limits.velocity`
  - PWM range maps linearly to the joint's position range

  When a position command is received, the actuator:
  1. Clamps the position to joint limits
  2. Converts to PWM pulse width
  3. Sends PWM command to pigpiox
  4. Publishes a `BB.Message.Actuator.BeginMotion` for sensors to consume

  ## Safety

  This actuator implements the `BB.Safety` behaviour. When the robot is disarmed
  (either by command or due to supervisor crash), the servo PWM output is disabled
  by setting the pulse width to 0.
  """
  import BB.Unit
  import BB.Unit.Option

  use BB.Actuator,
    options_schema: [
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
      update_speed: [
        type: unit_type(compatible: :hertz),
        doc: "The servo update frequency",
        default: ~u(50 hertz)
      ]
    ]

  alias BB.Actuator, as: ActuatorApi
  alias BB.Error.Invalid.JointConfig, as: JointConfigError
  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command
  alias BB.Robot.Units
  alias BB.Transmission

  @doc """
  Disable the servo by setting pulse width to 0.

  Called by `BB.Safety.Controller` when the robot is disarmed or crashes.
  This function works without GenServer state - it only needs the pin number.
  """
  @impl BB.Actuator
  def disarm(opts) do
    pin = Keyword.fetch!(opts, :pin)

    case Pigpiox.Socket.command(:set_servo_pulsewidth, pin, 0) do
      {:ok, _} -> :ok
      {:error, reason} -> {:error, reason}
    end
  end

  @impl BB.Actuator
  def init(opts) do
    with {:ok, state} <- build_state(opts),
         {:ok, _} <-
           Pigpiox.Socket.command(:set_PWM_frequency, state.pin, round(state.update_speed)),
         {:ok, _} <-
           Pigpiox.Socket.command(:set_servo_pulsewidth, state.pin, round(state.current_pulse)) do
      BB.Safety.register(__MODULE__,
        robot: state.bb.robot,
        path: state.bb.path,
        opts: [pin: state.pin]
      )

      {:ok, state}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  defp build_state(opts) do
    opts = Map.new(opts)
    [name, joint_name | _] = Enum.reverse(opts.bb.path)
    robot = opts.bb.robot.robot()
    transmission = ActuatorApi.current_transmission()

    min_pulse = Map.get(opts, :min_pulse, 500)
    max_pulse = Map.get(opts, :max_pulse, 2500)
    update_speed_unit = Map.get(opts, :update_speed, ~u(50 hertz))

    with {:ok, joint} <- fetch_joint(robot, joint_name),
         {:ok, limits} <- validate_joint_limits(joint, joint_name) do
      {motor_lower, motor_upper} = motor_position_limits(limits, transmission)
      motor_range = motor_upper - motor_lower
      motor_velocity_limit = motor_velocity_limit(limits.velocity, transmission)
      pulse_range = max_pulse - min_pulse

      update_speed =
        update_speed_unit
        |> Localize.Unit.convert!("hertz")
        |> Units.extract_float()

      initial_pulse = (max_pulse + min_pulse) / 2
      current_motor_angle = (motor_lower + motor_upper) / 2

      state = %{
        bb: opts.bb,
        pin: opts.pin,
        min_pulse: min_pulse,
        max_pulse: max_pulse,
        update_speed: update_speed,
        motor_lower: motor_lower,
        motor_upper: motor_upper,
        motor_range: motor_range,
        motor_velocity_limit: motor_velocity_limit,
        pulse_range: pulse_range,
        current_pulse: initial_pulse,
        current_motor_angle: current_motor_angle,
        name: name,
        joint_name: joint_name
      }

      {:ok, state}
    end
  end

  defp motor_position_limits(limits, nil), do: {limits.lower, limits.upper}

  defp motor_position_limits(limits, transmission) do
    a = Transmission.apply_position(limits.lower, transmission)
    b = Transmission.apply_position(limits.upper, transmission)
    {min(a, b), max(a, b)}
  end

  defp motor_velocity_limit(velocity, nil), do: velocity

  defp motor_velocity_limit(velocity, transmission) do
    abs(Transmission.apply_rate(velocity, transmission))
  end

  defp fetch_joint(robot, joint_name) do
    case BB.Robot.get_joint(robot, joint_name) do
      nil ->
        {:error,
         %JointConfigError{joint: joint_name, field: nil, message: "Joint not found in robot"}}

      joint ->
        {:ok, joint}
    end
  end

  defp validate_joint_limits(%{type: :continuous}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :type,
       value: :continuous,
       expected: [:revolute, :prismatic],
       message: "Continuous joints require position limits for servo control"
     }}
  end

  defp validate_joint_limits(%{limits: nil}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :limits,
       value: nil,
       message: "Joint must have limits defined for servo control"
     }}
  end

  defp validate_joint_limits(%{limits: %{lower: nil}}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :lower,
       value: nil,
       message: "Joint must have lower limit defined"
     }}
  end

  defp validate_joint_limits(%{limits: %{upper: nil}}, joint_name) do
    {:error,
     %JointConfigError{
       joint: joint_name,
       field: :upper,
       value: nil,
       message: "Joint must have upper limit defined"
     }}
  end

  defp validate_joint_limits(%{limits: limits}, _joint_name) do
    {:ok, limits}
  end

  @impl BB.Actuator
  def handle_info({:bb, _path, %Message{payload: %Command.Position{} = cmd}}, state) do
    {:noreply, state} = do_set_position(cmd.position, cmd.command_id, state)
    {:noreply, state}
  end

  @impl BB.Actuator
  def handle_cast({:command, %Message{payload: %Command.Position{} = cmd}}, state) do
    do_set_position(cmd.position, cmd.command_id, state)
  end

  @impl BB.Actuator
  def handle_call({:command, %Message{payload: %Command.Position{} = cmd}}, _from, state) do
    {:noreply, new_state} = do_set_position(cmd.position, cmd.command_id, state)
    {:reply, {:ok, :accepted}, new_state}
  end

  defp do_set_position(motor_angle, command_id, state) when is_integer(motor_angle),
    do: do_set_position(motor_angle * 1.0, command_id, state)

  defp do_set_position(motor_angle, command_id, state) do
    clamped_motor_angle = clamp_motor_angle(motor_angle, state)
    new_pulse = motor_angle_to_pulse(clamped_motor_angle, state)

    with {:ok, _} <- Pigpiox.Socket.command(:set_servo_pulsewidth, state.pin, new_pulse) do
      travel_distance = abs(state.current_motor_angle - clamped_motor_angle)
      travel_time_ms = round(travel_distance / state.motor_velocity_limit * 1000)
      expected_arrival = System.monotonic_time(:millisecond) + travel_time_ms

      message_opts =
        [
          initial_position: state.current_motor_angle,
          target_position: clamped_motor_angle,
          expected_arrival: expected_arrival,
          command_type: :position
        ]
        |> maybe_add_opt(:command_id, command_id)

      message = Message.new!(BeginMotion, state.joint_name, message_opts)

      BB.publish(state.bb.robot, [:actuator | state.bb.path], message)

      {:noreply, %{state | current_pulse: new_pulse, current_motor_angle: clamped_motor_angle}}
    end
  end

  defp maybe_add_opt(opts, _key, nil), do: opts
  defp maybe_add_opt(opts, key, value), do: Keyword.put(opts, key, value)

  defp clamp_motor_angle(motor_angle, state) do
    motor_angle
    |> max(state.motor_lower)
    |> min(state.motor_upper)
  end

  defp motor_angle_to_pulse(motor_angle, state) do
    normalised = (motor_angle - state.motor_lower) / state.motor_range
    round(state.min_pulse + normalised * state.pulse_range)
  end
end
