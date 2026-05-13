# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.BbServoPigpio.Install do
    @shortdoc "Installs BB.Servo.Pigpio into a project"
    @moduledoc """
    #{@shortdoc}

    Imports the package's formatter rules. Actuators and sensors belong on
    individual joints in your topology, so no robot-level configuration is
    added — a snippet is printed for you to copy.

    `bb_servo_pigpio` talks to the system-level `pigpiod` daemon directly, so
    there is no controller process to define in the robot.

    ## Example

    ```bash
    mix igniter.install bb_servo_pigpio
    ```
    """

    use Igniter.Mix.Task

    alias Igniter.Project.Formatter

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{
        schema: [robot: :string],
        aliases: [r: :robot]
      }
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      igniter
      |> Formatter.import_dep(:bb_servo_pigpio)
      |> Igniter.add_notice(topology_snippet())
    end

    defp topology_snippet do
      """
      bb_servo_pigpio: add actuators/sensors to your joints. Example:

          joint :shoulder, type: :revolute do
            limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)

            actuator :servo, {BB.Servo.Pigpio.Actuator, pin: 17}
            sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}
          end

      Make sure pigpiod is running on the target host: `sudo pigpiod`.
      """
    end
  end
else
  defmodule Mix.Tasks.BbServoPigpio.Install do
    @shortdoc "Installs BB.Servo.Pigpio into a project"
    @moduledoc false
    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The bb_servo_pigpio.install task requires igniter.

          mix igniter.install bb_servo_pigpio
      """)

      exit({:shutdown, 1})
    end
  end
end
