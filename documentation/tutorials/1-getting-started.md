<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Getting Started

This guide walks you through setting up your Raspberry Pi to control RC servos
with BB.Servo.Pigpio.

## Hardware Requirements

- Raspberry Pi (any model with GPIO pins)
- RC servo (standard hobby servo with 3-wire connector)
- 5V power supply for the servo (Pi's 5V pin can work for small servos)
- Jumper wires

## Wiring

RC servos have three wires:

| Wire Colour | Connection |
|-------------|------------|
| Brown/Black | Ground (GND) |
| Red | Power (5V) |
| Orange/Yellow/White | Signal (GPIO pin) |

### Basic Wiring Diagram

```
Raspberry Pi              Servo
-----------              -----
GPIO 17  ──────────────► Signal (orange)
5V       ──────────────► Power (red)
GND      ──────────────► Ground (brown)
```

> **Warning:** For multiple servos or high-torque servos, use an external 5V
> power supply instead of the Pi's 5V pin. Connect the ground of the external
> supply to the Pi's ground.

## Software Setup

### 1. Install pigpio

The pigpio library provides precise PWM timing for servo control.

```bash
# On Raspberry Pi OS
sudo apt-get update
sudo apt-get install pigpio

# Start the daemon
sudo pigpiod

# Optional: enable pigpiod to start on boot
sudo systemctl enable pigpiod
```

Verify pigpiod is running:

```bash
pigs t  # Should return the current tick count
```

### 2. Add Dependencies

Add `bb_servo_pigpio` and `pigpiox` to your `mix.exs`:

```elixir
def deps do
  [
    {:bb, "~> 0.18"},
    {:bb_servo_pigpio, "~> 0.6.0"},
    {:pigpiox, "~> 0.1"}
  ]
end
```

Then fetch dependencies:

```bash
mix deps.get
```

### 3. Configure Pigpiox (Optional)

By default, pigpiox connects to pigpiod on localhost. If running remotely:

```elixir
# config/config.exs
config :pigpiox,
  host: "192.168.1.100"  # IP of your Raspberry Pi
```

## Verify Your Setup

Create a simple test to verify everything works:

```elixir
# In IEx on your Raspberry Pi
iex> Pigpiox.Socket.command(:set_servo_pulsewidth, 17, 1500)
{:ok, 0}
```

This should move the servo to its centre position. If you see `{:ok, 0}`, your
setup is working correctly.

## Troubleshooting

### "No such file or directory" error

The pigpio daemon isn't running:

```bash
sudo pigpiod
```

### "Permission denied" error

Your user needs access to pigpio:

```bash
sudo usermod -a -G gpio $USER
# Log out and back in
```

### Servo doesn't move

1. Check wiring connections
2. Verify the GPIO pin number matches your wiring
3. Try a different GPIO pin
4. Check servo power supply

### Servo jitters or moves erratically

1. Add a capacitor (100µF) across servo power lines
2. Use a separate power supply for the servo
3. Keep signal wires away from motor/power wires

## Next Steps

Now that your hardware is set up, proceed to [Basic Usage](2-basic-usage.md) to
learn how to integrate servos into your BB robot.
