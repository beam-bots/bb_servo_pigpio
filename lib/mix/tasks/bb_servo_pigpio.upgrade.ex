# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.BbServoPigpio.Upgrade do
    @shortdoc "Lifts `reverse?` into joint transmissions"
    @moduledoc """
    #{@shortdoc}

    `BB.Servo.Pigpio.Actuator` no longer takes a `reverse?` option. Polarity
    now lives in the joint-level `transmission` block in BB.

    This upgrader rewrites every robot module in the project: for each
    actuator with `BB.Servo.Pigpio.Actuator` as its driver, it removes
    `reverse?:` from the actuator's options and, when it was `true`, inserts
    a `transmission do reversed? true end` block on the parent joint.
    """

    use Igniter.Mix.Task

    alias BB.Igniter.Transmission

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{}
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      Transmission.lift_reverse_question(igniter, BB.Servo.Pigpio.Actuator)
    end
  end
end
