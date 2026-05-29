# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Servo.Pigpio do
  @moduledoc """
  BB integration for driving RC servos via pigpio on Raspberry Pi.

  This library provides an actuator module for controlling RC servos directly
  connected to Raspberry Pi GPIO pins using the pigpio daemon.

  ## Components

  - `BB.Servo.Pigpio.Actuator` - Controls servo position via PWM

  Position feedback is provided by `BB.Sensor.OpenLoopPositionEstimator` from BB
  core, paired with the actuator in the joint definition.

  ## Requirements

  - Raspberry Pi with pigpio daemon running (`sudo pigpiod`)
  - The `pigpiox` library for communication with pigpiod

  ## Quick Start

  Define a joint with servo actuator in your robot DSL:

      joint :shoulder, type: :revolute do
        limit lower: ~u(-45 degree), upper: ~u(45 degree), velocity: ~u(60 degree_per_second)

        actuator :servo, {BB.Servo.Pigpio.Actuator, pin: 17}
        sensor :feedback, {BB.Sensor.OpenLoopPositionEstimator, actuator: :servo}
      end

  The actuator automatically derives its configuration from the joint limits - no need
  to specify servo rotation range or speed separately.

  ## How It Works

  ### Actuator

  The actuator maps the joint's position limits directly to the servo's PWM range:
  - Joint lower limit → minimum pulse width (default 500µs)
  - Joint upper limit → maximum pulse width (default 2500µs)
  - Centre position calculated as midpoint of limits

  When commanded to a position, the actuator:
  1. Clamps the position to joint limits
  2. Converts to PWM pulse width
  3. Sends command to pigpiod
  4. Publishes `BB.Message.Actuator.BeginMotion`

  ### Position feedback

  Position feedback comes from `BB.Sensor.OpenLoopPositionEstimator` (BB core),
  not this library. It subscribes to the actuator's `BeginMotion` messages,
  interpolates the joint position during movement, and publishes `JointState`.
  """
end
