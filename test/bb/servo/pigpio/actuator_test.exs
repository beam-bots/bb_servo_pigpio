# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Pigpio.ActuatorTest do
  use ExUnit.Case, async: true
  use Mimic

  alias BB.Actuator.MotorProfile
  alias BB.Error.Invalid.JointConfig, as: JointConfigError
  alias BB.Message
  alias BB.Message.Actuator.Command
  alias BB.Servo.Pigpio.Actuator

  @joint_name :test_joint
  @actuator_name :test_servo

  defp position_command(position, opts \\ []) do
    message_opts =
      [position: position * 1.0]
      |> maybe_add_opt(:command_id, opts[:command_id])

    {:command, Message.new!(Command.Position, @joint_name, message_opts)}
  end

  defp maybe_add_opt(opts, _key, nil), do: opts
  defp maybe_add_opt(opts, key, value), do: Keyword.put(opts, key, value)

  defp default_bb_context do
    %{robot: TestRobot, path: [@joint_name, @actuator_name]}
  end

  defp motor_profile(overrides \\ []) do
    base = %MotorProfile{
      motor_lower: -0.5,
      motor_upper: 0.5,
      motor_velocity_limit: 1.0,
      motor_initial_position: 0.0
    }

    struct!(base, overrides)
  end

  defp stub_pigpiox_success do
    stub(Pigpiox.Socket, :command, fn _cmd, _pin, _val -> {:ok, 0} end)
    stub(BB.Safety, :register, fn _module, _opts -> :ok end)
    stub(BB.Actuator, :publish_begin_motion, fn _robot, _path, _opts -> :ok end)
  end

  describe "init/1" do
    test "succeeds with a complete motor profile" do
      stub_pigpiox_success()

      opts = [bb: default_bb_context(), pin: 17, motor_profile: motor_profile()]
      assert {:ok, state} = Actuator.init(opts)

      assert state.motor_profile.motor_lower == -0.5
      assert state.motor_profile.motor_upper == 0.5
      assert state.motor_profile.motor_velocity_limit == 1.0
      assert state.motor_range == 1.0
    end

    test "fails when motor profile has no lower limit" do
      opts = [bb: default_bb_context(), pin: 17, motor_profile: motor_profile(motor_lower: nil)]
      assert {:stop, %JointConfigError{joint: @joint_name, field: :lower}} = Actuator.init(opts)
    end

    test "fails when motor profile has no upper limit" do
      opts = [bb: default_bb_context(), pin: 17, motor_profile: motor_profile(motor_upper: nil)]
      assert {:stop, %JointConfigError{joint: @joint_name, field: :upper}} = Actuator.init(opts)
    end

    test "fails when motor profile has no velocity limit" do
      opts = [
        bb: default_bb_context(),
        pin: 17,
        motor_profile: motor_profile(motor_velocity_limit: nil)
      ]

      assert {:stop, %JointConfigError{joint: @joint_name, field: :velocity}} =
               Actuator.init(opts)
    end

    test "initialises servo at center position" do
      stub_pigpiox_success()

      opts = [
        bb: default_bb_context(),
        pin: 17,
        motor_profile: motor_profile(motor_lower: -1.0, motor_upper: 1.0)
      ]

      assert {:ok, state} = Actuator.init(opts)

      assert state.current_motor_angle == 0.0
      assert state.current_pulse == 1500.0
    end

    test "uses the motor profile's initial position" do
      stub_pigpiox_success()

      opts = [
        bb: default_bb_context(),
        pin: 17,
        motor_profile: motor_profile(motor_initial_position: 1.0)
      ]

      assert {:ok, state} = Actuator.init(opts)
      assert state.current_motor_angle == 1.0
    end
  end

  describe "angle_to_pulse conversion" do
    setup do
      stub_pigpiox_success()

      opts = [
        bb: default_bb_context(),
        pin: 17,
        min_pulse: 500,
        max_pulse: 2500,
        motor_profile: motor_profile(motor_lower: -1.0, motor_upper: 1.0)
      ]

      {:ok, state} = Actuator.init(opts)

      {:ok, state: state}
    end

    test "lower limit maps to min_pulse", %{state: state} do
      expect(Pigpiox.Socket, :command, fn :set_servo_pulsewidth, 17, pulse ->
        assert pulse == 500
        {:ok, 0}
      end)

      Actuator.handle_cast(position_command(-1.0), state)
    end

    test "upper limit maps to max_pulse", %{state: state} do
      expect(Pigpiox.Socket, :command, fn :set_servo_pulsewidth, 17, pulse ->
        assert pulse == 2500
        {:ok, 0}
      end)

      Actuator.handle_cast(position_command(1.0), state)
    end

    test "center maps to mid_pulse", %{state: state} do
      expect(Pigpiox.Socket, :command, fn :set_servo_pulsewidth, 17, pulse ->
        assert pulse == 1500
        {:ok, 0}
      end)

      Actuator.handle_cast(position_command(0.0), state)
    end
  end

  describe "position clamping" do
    setup do
      stub_pigpiox_success()

      opts = [
        bb: default_bb_context(),
        pin: 17,
        min_pulse: 500,
        max_pulse: 2500,
        motor_profile: motor_profile(motor_lower: -1.0, motor_upper: 1.0)
      ]

      {:ok, state} = Actuator.init(opts)

      {:ok, state: state}
    end

    test "clamps position below lower limit", %{state: state} do
      expect(Pigpiox.Socket, :command, fn :set_servo_pulsewidth, 17, pulse ->
        assert pulse == 500
        {:ok, 0}
      end)

      Actuator.handle_cast(position_command(-5.0), state)
    end

    test "clamps position above upper limit", %{state: state} do
      expect(Pigpiox.Socket, :command, fn :set_servo_pulsewidth, 17, pulse ->
        assert pulse == 2500
        {:ok, 0}
      end)

      Actuator.handle_cast(position_command(5.0), state)
    end
  end

  describe "begin_motion publishing" do
    setup do
      stub_pigpiox_success()

      opts = [
        bb: default_bb_context(),
        pin: 17,
        motor_profile: motor_profile(motor_lower: -1.0, motor_upper: 1.0)
      ]

      {:ok, state} = Actuator.init(opts)

      {:ok, state: state}
    end

    test "calls publish_begin_motion with motor-space opts", %{state: state} do
      test_pid = self()

      expect(BB.Actuator, :publish_begin_motion, fn robot, path, opts ->
        send(test_pid, {:published, robot, path, opts})
        :ok
      end)

      Actuator.handle_cast(position_command(0.5), state)

      assert_receive {:published, TestRobot, [@joint_name, @actuator_name], opts}

      assert opts[:initial_position] == 0.0
      assert opts[:target_position] == 0.5
      assert is_integer(opts[:expected_arrival])
      assert opts[:expected_arrival] > System.monotonic_time(:millisecond)
    end

    test "calculates expected arrival based on velocity", %{state: state} do
      test_pid = self()

      expect(BB.Actuator, :publish_begin_motion, fn _robot, _path, opts ->
        send(test_pid, {:arrival, opts[:expected_arrival]})
        :ok
      end)

      before = System.monotonic_time(:millisecond)
      Actuator.handle_cast(position_command(1.0), state)

      assert_receive {:arrival, expected_arrival}

      travel_time_ms = round(1.0 / 1.0 * 1000)
      assert_in_delta expected_arrival, before + travel_time_ms, 50
    end
  end

  describe "BB.Safety behaviour" do
    test "disarm/1 sets servo pulse width to 0" do
      expect(Pigpiox.Socket, :command, fn :set_servo_pulsewidth, 17, 0 ->
        {:ok, 0}
      end)

      assert :ok = Actuator.disarm(pin: 17)
    end

    test "disarm/1 returns error on failure" do
      expect(Pigpiox.Socket, :command, fn :set_servo_pulsewidth, 17, 0 ->
        {:error, :connection_failed}
      end)

      assert {:error, :connection_failed} = Actuator.disarm(pin: 17)
    end

    test "init/1 registers with safety controller" do
      stub_pigpiox_success()

      expect(BB.Safety, :register, fn module, opts ->
        assert module == Actuator
        assert opts[:robot] == TestRobot
        assert opts[:path] == [@joint_name, @actuator_name]
        assert opts[:opts] == [pin: 17]
        :ok
      end)

      opts = [bb: default_bb_context(), pin: 17, motor_profile: motor_profile()]
      assert {:ok, _state} = Actuator.init(opts)
    end
  end
end
