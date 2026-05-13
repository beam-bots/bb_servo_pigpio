# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.BbServoPigpio.InstallTest do
  use ExUnit.Case
  import Igniter.Test

  @moduletag :igniter

  test "imports bb_servo_pigpio into .formatter.exs" do
    test_project()
    |> Igniter.compose_task("bb_servo_pigpio.install")
    |> assert_has_patch(".formatter.exs", """
    + |  import_deps: [:bb_servo_pigpio]
    """)
  end

  test "prints a topology snippet for the user to paste" do
    test_project()
    |> Igniter.compose_task("bb_servo_pigpio.install")
    |> assert_has_notice(&String.contains?(&1, "BB.Servo.Pigpio.Actuator"))
  end

  test "running twice produces no further changes" do
    test_project()
    |> Igniter.compose_task("bb_servo_pigpio.install")
    |> apply_igniter!()
    |> Igniter.compose_task("bb_servo_pigpio.install")
    |> assert_unchanged()
  end
end
