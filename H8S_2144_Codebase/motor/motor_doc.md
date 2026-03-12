# Motor Control Driver — motor

## Module Summary

Low-level hardware driver for the L200's 3 motor channels (left wheel, right wheel,
blade), each using an ST L6203 full H-bridge driver IC. Provides direction control,
speed setting, encoder capture, differential steering, and blade motor management
including emergency stop with dynamic braking.

| Property | Value |
|----------|-------|
| Motor drivers | 3 x ST L6203 full H-bridge |
| Left wheel | Speed: PWMX ch A (DADRA, 0xFFFFA0), IN1: P2.6 (run), IN2: P9.0 (dir), Enc: FRT ICIA |
| Right wheel | Speed: PWMX ch B (DADRB, 0xFFFFA6), IN1: P2.5 (run), IN2: P9.1 (dir), Enc: FRT ICIB |
| Blade motor | Speed: TMR0 PWM (TCORB duty, TCORA=0xFF period), enable PA.0 (OD active-low), dir P2.2 |
| Global power | P4.0 (wheel motors), P9.5 (blade power) |
| Source doc | `STATES/state_MotorControl.md` |

## Files

| File | Description |
|------|-------------|
| `motor.h` | Types, constants, pin masks, API declarations |
| `motor.c` | Implementation (GPIO, PWMX, TMR0, FRT, steering) |
| `motor_doc.md` | This file |

## How to Include

```c
#include "motor/motor.h"
```

```makefile
SRCS += motor/motor.c
```

### Dependencies

| Dependency | Required For |
|------------|-------------|
| `<stdint.h>` | Fixed-width types |
| `iodefine.h` | P2, P4, P9, PA, TMR0, FRT, STCR, PWMX register access |

## Architecture

### H-Bridge Direction Control

Each L6203 has 3 control inputs: EN (enable/speed), IN1, IN2.

For the wheel motors, the production firmware uses IN1 as a "run enable"
and IN2 as a "direction select" (confirmed via Ghidra reverse engineering):

```
              Wheel Motor Direction (confirmed from production firmware)
 +------+------+------+----------------------------------------------+
 | IN1  | IN2  | EN   | Motor Action                                 |
 +------+------+------+----------------------------------------------+
 |  1   |  0   | PWM  | Forward                                      |
 |  1   |  1   | PWM  | Reverse                                      |
 |  0   |  X   | any  | Disabled (coast)                             |
 +------+------+------+----------------------------------------------+

 Production firmware functions (Ghidra):
   Forward: sub_126F6 sets IN1=1, leftWheel_I2C_DataLow sets IN2=0+speed
   Reverse: sub_126F6 sets IN1=1, SetBit0_P9 sets IN2=1+speed
   Disable: sub_1271A clears IN1=0
```

### Pin Mapping

```
 +---------------------------------------------------------------------+
 |                        H8S/2144 MCU                                  |
 |                                                                      |
 |  +-------------- LEFT WHEEL ----------------------------------------+
 |  |  PWMX ch A (DADRA at 0xFFFFA0) --- EN (speed PWM)               |
 |  |  P2.6 ------------------------------ IN1 (run enable)            |
 |  |  P9.0 ------------------------------ IN2 (direction: 0=fwd 1=rev)|
 |  |  P6.6 <---------------------------- SENSE (1-wire, DS18B20)      |
 |  |  FRT_ICRA <------------------------ Encoder (ICIA IRQ, TIER.5)   |
 |  +----------------------------------------------------------------------+
 |                                                                      |
 |  +-------------- RIGHT WHEEL ---------------------------------------+
 |  |  PWMX ch B (DADRB at 0xFFFFA6) --- EN (speed PWM)               |
 |  |  P2.5 ------------------------------ IN1 (run enable)            |
 |  |  P9.1 ------------------------------ IN2 (direction: 0=fwd 1=rev)|
 |  |  P6.5 <---------------------------- SENSE (1-wire, DS18B20)      |
 |  |  FRT_ICRB <------------------------ Encoder (ICIB IRQ, TIER.7)   |
 |  +----------------------------------------------------------------------+
 |                                                                      |
 |  +-------------- BLADE MOTOR ---------------------------------------+
 |  |  PA.0 (open-drain, active-low) ---- EN                           |
 |  |  P2.2 ------------------------------ IN1 (direction)             |
 |  |  TMR0 PWM (TCORB=duty, TCORA=0xFF)- Speed (via PWM)             |
 |  |  P9.2 <---------------------------- 1-wire bus (DS18B20 temp)    |
 |  |  P9.5 ------------------------------ Power supply enable         |
 |  +----------------------------------------------------------------------+
 |                                                                      |
 |  P4.0 ------------- Global wheel motor power enable                  |
 |  P4.5 ------------- Main PSU enable                                  |
 +----------------------------------------------------------------------+
```

### Speed Control: Wheels (PWMX D/A-PWM Module)

The wheel motor speed uses the PWMX module at 0xFFFFA0, NOT TMR0/TMR1.
This was confirmed by Ghidra RE of SetAccessI2C (writes DADRA) and
SetAccessI2C_Right (writes DADRB).

```
Speed command (0-255)
      |
      v
 MOTOR_PWMX_DUTY(speed) = (speed << 8) | 0x0002
      |
      v
 Write to DADRA (left, 0xFFFFA0) or DADRB (right, 0xFFFFA6)
      |
      v
 PWMX module generates analog/PWM output to L6203 EN pin
      |
      v
 Motor speed proportional to duty cycle

PWMX Register Bank Switching (via DADRB bit 0 = REGS):
  Bank 0 (REGS=0): DADRA = left duty,  DADRB = right duty
  Bank 1 (REGS=1): DADRA = DACR (ctrl), DADRB = DACNT (counter)

Init sequence (from SetupHW_Q at 0x121A4):
  STCR |= 0x10          Enable PWMX bus access
  DADRB = 0x0003         Switch to bank 1 (REGS=1)
  DACR  = 0x3F           OEA=1, OEB=1, OS=1, CKS=1
  DADRB = 0x0002         Switch back to bank 0 (REGS=0)
  DADRA = 0x0002         Zero left speed (CFS=1)
  DADRB = 0x0002         Zero right speed (CFS=1)

Firmware function mapping (Ghidra):
  SetAccessI2C       (0x126DE) -> PWMX_WriteLeft  (STCR|=0x10, writes DADRA)
  SetAccessI2C_Right (0x12852) -> PWMX_WriteRight (STCR|=0x10, writes DADRB)
```

### Speed Control: Blade (TMR0 PWM)

The blade motor speed is controlled by TMR0 hardware PWM output.
Previously assumed to be DAC; corrected via Ghidra RE of 0x129CC.

```
Speed command (0-1000)
      |
      v
 duty = (speed * 255) / 1000
      |
      v
 TMR0.TCORB = duty      (compare match B = duty cycle)
 TMR0.TCORA = 0xFF      (compare match A = period, fixed)
 TMR0.TCR   = 0x09      (phi/8 clock, clear on TCORA)
 TMR0.TCSR  = 0x06      (PWM output enabled) or 0x00 (disabled)
      |
      v
 TMR0 generates PWM waveform
 P4.0 motor power enables the H-bridge
 Motor speed proportional to duty cycle
```

### Encoder Feedback (FRT Input Capture)

```
Wheel encoder magnet passes sensor
      |
      v
FRT_ICRA (left) or FRT_ICRB (right) latches FRT counter
      |
      v
ICIA / ICIB interrupt fires (FRT.TIER bits 5/7)
      |
      v
ISR reads capture value
dt = current - previous capture
RPM = (clock_freq / prescaler) / dt
```

Note: FRT.TIER is used ONLY for encoder capture interrupts.
There is NO TPMR routing for wheel EN pins (wheel speed uses PWMX, not TMR).

### Differential Steering

```
motor_SetSteering(base_speed, angle, turn_right)
      |
      v
 offset = (angle x base_speed) >> 8
      |
      +-- turn_right = 1:
      |     left  = base_speed + offset  (faster)
      |     right = base_speed - offset  (slower)
      |
      +-- turn_right = 0:
            left  = base_speed - offset  (slower)
            right = base_speed + offset  (faster)
```

### Blade Emergency Stop

```
motor_BladeEmergencyStop()
      |
      +-- TMR0.TCORB = 0       Zero PWM duty
      +-- TMR0.TCSR  = 0x00    Disable PWM output
      +-- P2.DR &= 0xF8        Clear IN1 + signal bits
      +-- PA.ODR |= 0x01       EN disabled (OD release)
```

### Blade Dynamic Braking

```
motor_BladeBrake()
      |
      +-- TMR0.TCORB = 0       Zero PWM (no drive signal)
      +-- TMR0.TCSR  = 0x00    Disable PWM output
      +-- P4.0 stays ON        H-bridge shorts for braking
              |
              v
 EN low + motor power ON -> back-EMF creates braking torque
 through L6203 body diodes -> rapid deceleration
```

## API Reference

### Initialization

#### `void motor_Init(void)`

Initialize all motor hardware: direction pins, PWMX, TMR0, FRT, DAC.
All motors disabled, global power off, PSU and blade power in standby.

Firmware: motor-related portion of `SetupHW_Q` (0x121A4)

### Global Power

#### `void motor_PowerOn(void)` / `void motor_PowerOff(void)`

Enable/disable global motor power (P4.0). Must be on before
wheel motors can run. Does not affect blade motor power (P9.5).

### Left Wheel

| Function | Description |
|----------|-------------|
| `motor_LeftSetSpeed(speed)` | Set PWMX DADRA duty (0-255) |
| `motor_LeftForward()` | IN1=H (run), IN2=L (fwd), enable encoder |
| `motor_LeftReverse()` | IN1=H (run), IN2=H (rev), enable encoder |
| `motor_LeftDisable()` | IN1=L, IN2=L (coast), disable encoder, zero PWMX |

### Right Wheel

| Function | Description |
|----------|-------------|
| `motor_RightSetSpeed(speed)` | Set PWMX DADRB duty (0-255) |
| `motor_RightForward()` | IN1=H (run), IN2=L (fwd), enable encoder |
| `motor_RightReverse()` | IN1=H (run), IN2=H (rev), enable encoder |
| `motor_RightDisable()` | IN1=L, IN2=L (coast), disable encoder, zero PWMX |

### Blade Motor

| Function | Description |
|----------|-------------|
| `motor_BladeInit()` | Initialize TMR0 for blade PWM (TCORA=0xFF, TCR=0x09) |
| `motor_BladeStart(speed)` | Full start: direction, TMR0 PWM duty, P4.0 on |
| `motor_BladeSetSpeed(speed)` | Change TMR0.TCORB duty while running |
| `motor_BladeStop()` | Normal stop: disable PWM, power off, clear dir |
| `motor_BladeFullStop()` | Full stop: disable PWM + FRT output compare |
| `motor_BladeBrake()` | Dynamic braking (PWM off, power ON for short-brake) |
| `motor_BladeEmergencyStop()` | Rapid stop: kill PWM + clear direction + disable EN |

### Combined Operations

| Function | Description |
|----------|-------------|
| `motor_StopAll()` | Stop blade + both wheels + power off |
| `motor_SetSteering(base, angle, right)` | Differential steering |

### Diagnostics

| Function | Description |
|----------|-------------|
| `motor_GetState(state)` | Get full motor subsystem state |
| `motor_ReadEncoder(left, right)` | Read FRT input capture values |
| `motor_BladeDevicePresent()` | 1-wire presence detect on P9.2 (DS18B20) |
| `motor_BladeCurrentOK()` | Legacy alias for BladeDevicePresent |

## GPIO / Register Map

| Register | Address | Bits Used | Function |
|----------|---------|-----------|----------|
| P2.DR | 0xFFFFAE | bit 2 | Blade IN1 |
| P2.DR | 0xFFFFAE | bit 5 | Right wheel IN1 (run enable) |
| P2.DR | 0xFFFFAE | bit 6 | Left wheel IN1 (run enable) |
| P4.DR | 0xFFFFB7 | bit 0 | Global motor power |
| P4.DR | 0xFFFFB7 | bit 5 | Main PSU enable |
| P6.DR | 0xFFFFBB | bit 5 | Right wheel 1-wire (DS18B20) |
| P6.DR | 0xFFFFBB | bit 6 | Left wheel 1-wire (DS18B20) |
| P9.DR | 0xFFFFC2 | bit 0 | Left wheel IN2 (direction) |
| P9.DR | 0xFFFFC2 | bit 1 | Right wheel IN2 (direction) |
| P9.DR | 0xFFFFC2 | bit 2 | Blade 1-wire bus (DS18B20 temp) |
| P9.DR | 0xFFFFC2 | bit 5 | Blade power enable |
| PA.ODR | 0xFFFFAA | bit 0 | Blade EN (OD, active-low) |
| STCR | 0xFFFFC3 | bit 4 | PWMX bus access enable |
| PWMX DADRA | 0xFFFFA0 | 16-bit | Left wheel speed (PWMX ch A) |
| PWMX DADRB | 0xFFFFA6 | 16-bit | Right wheel speed (PWMX ch B) |
| PWMX DACR | 0xFFFFA0 | 8-bit | PWMX control (bank 1 mode) |
| TMR0.TCORA | 0xFFFFCE | 8-bit | Blade PWM period (0xFF fixed) |
| TMR0.TCORB | 0xFFFFCC | 8-bit | Blade PWM duty (0-255) |
| TMR0.TCR | 0xFFFFC8 | 8-bit | Blade TMR0 control (0x09) |
| TMR0.TCSR | 0xFFFFC9 | 8-bit | Blade TMR0 status (0x06=PWM on) |
| FRT.TIER | 0xFFFF90 | bit 5 | Left encoder capture IRQ (ICICE) |
| FRT.TIER | 0xFFFF90 | bit 7 | Right encoder capture IRQ (ICIAE) |
| FRT.ICRA | 0xFFFF98 | 16-bit | Left encoder capture value |
| FRT.ICRB | 0xFFFF9A | 16-bit | Right encoder capture value |
| DA.DADR0 | 0xFFFFF8 | 8-bit | DAC ch0 (legacy, zeroed at init) |
| DA.DACR | 0xFFFFFA | 8-bit | DAC control (0xC0 = enable) |

## Firmware Traceability

| Our Function | Firmware Name | Address | Notes |
|-------------|---------------|---------|-------|
| `motor_Init` | `SetupHW_Q` (motor portion) | 0x121A4 | PWMX init + GPIO |
| `motor_LeftForward` | `sub_126F6` | 0x126F6 | P2.6=1, TIER\|=0x20 |
| `motor_LeftDisable` | `sub_1271A` | 0x1271A | P2.6=0, TIER&=0xDF |
| `motor_LeftReverse` | `SetBit0_P9` | 0x12762 | speed+P9.0=1 (IN2=rev) |
| `motor_LeftSetSpeed` (fwd) | `leftWheel_I2C_DataLow` | 0x12792 | speed+P9.0=0 (IN2=fwd) |
| `motor_LeftSetSpeed` (PWMX) | `SetAccessI2C` | 0x126DE | STCR\|=0x10, DADRA write |
| `motor_RightForward` | `sub_1286A` | 0x1286A | P2.5=1, TIER\|=0x80 |
| `motor_RightDisable` | `sub_1288E` | 0x1288E | P2.5=0, TIER&=0x7F |
| `motor_RightReverse` | `SetBit1_P9` | 0x1290C | speed+P9.1=1 (IN2=rev) |
| `motor_RightSetSpeed` (fwd) | `rightWheel_I2C_DataLow` | 0x128D6 | speed+P9.1=0 (IN2=fwd) |
| `motor_RightSetSpeed` (PWMX) | `SetAccessI2C_Right` | 0x12852 | STCR\|=0x10, DADRB write |
| `motor_BladeInit` | `timerPWM_Init` | 0x129CC | TMR0 PWM setup |
| `motor_BladeStart` | `setSpeed` | 0x12A22 | TCORB=duty, P4.0 on |
| `motor_BladeStop` | `pwmOff` | 0x12B3A | PWM off + power off |
| `motor_BladeFullStop` | `fullStop` | 0x12B04 | PWM off + power off + FRT off |
| `motor_BladeBrake` | `brake` | 0x12ADA | PWM off, P4.0 ON (short-brake) |
| `motor_BladeEmergencyStop` | `fDoPorts1` | 0x12BA8 | Kill all blade outputs |
| `motor_BladeDevicePresent` | `blade_CurrentSense_Enable` | 0x15420 | 1-wire detect on P9.2 |
| `motor_StopAll` | `stopAndWait` (hw portion) | 0x05830 | |
| `motor_SetSteering` | `ao_post_motor_set_steering` | 0x0CFF2 | |

## Key Corrections from Ghidra RE

The following items were corrected from the original assumptions based on
production firmware reverse engineering:

1. **Wheel speed is PWMX, not TMR0/TMR1**: The PWMX D/A-PWM module at
   0xFFFFA0 generates wheel motor speed, not the 8-bit timers. TMR0 is
   blade-only. TMR1 is not used for motor control.

2. **Left wheel IN2 is P9.0, not P2.7**: Confirmed by Ghidra function
   SetBit0_P9 (0x12762) which sets P9.0=1 for reverse direction.

3. **No TPMR routing**: FRT.TIER (0xFFFF90) is used ONLY for encoder
   capture interrupts (bits 5 and 7). There is no TPMR routing of timer
   outputs to wheel EN pins.

4. **Direction truth table**: IN1 is "run enable" (always 1 when running),
   IN2 is "direction select" (0=forward, 1=reverse). IN1=0 disables the
   motor. This differs from the classic L6203 truth table where IN1/IN2
   independently control each half-bridge.

5. **Blade uses TMR0 PWM, not DAC**: The blade motor speed is set via
   TMR0.TCORB (PWM duty), not DA.DADR0. The DAC is initialized but
   zeroed and not used for blade speed.

6. **P9.2 is 1-wire, not current sense**: The blade "current sense" pin
   is actually a Dallas/Maxim 1-wire bus connected to a DS18B20 temperature
   sensor on the blade motor assembly.

## Not Included

| Feature | Reason | Location |
|---------|--------|----------|
| Wheel encoder ISRs | Interrupt handlers -- separate module | ICIA/ICIB vectors |
| Motor AO state machine | Active Object framework -- application level | ao_mower_ops |
| I2C encoder communication | SCI2/I2C bus -- separate from motor GPIO | 0x12872/0x12852 |
| Speed scaling (cmd->duty) | Application logic: `duty = (cmd * cal) / 1000` | Motor AO |
| Blocked detection | Application logic: timeout + retry counter | Main mower loop |
| Motor temperature read | 1-wire DS18B20 protocol on P9.2/P6.5/P6.6 | 0x15420+ |
| Charge relay control | P1.5 -- separate power management module | P1.DR bit 5 |

## Usage Example

```c
#include "motor/motor.h"

/* At startup */
motor_Init();

/* Start mowing */
motor_PowerOn();
motor_LeftForward();
motor_RightForward();
motor_LeftSetSpeed(180);      /* ~70% duty via PWMX DADRA */
motor_RightSetSpeed(180);     /* ~70% duty via PWMX DADRB */
motor_BladeStart(200);        /* Blade at ~20% TMR0 PWM */

/* Steering: gentle right turn */
motor_SetSteering(180, 45, 1);  /* base=180, angle=45, turn right */

/* Bump recovery: reverse at moderate speed */
motor_LeftReverse();          /* IN1=H (run), IN2=H (reverse) */
motor_RightReverse();
motor_LeftSetSpeed(120);
motor_RightSetSpeed(120);

/* Emergency stop */
motor_BladeEmergencyStop();
motor_StopAll();

/* Check blade motor 1-wire device */
if (!motor_BladeDevicePresent()) {
    /* No DS18B20 temperature sensor detected */
}

/* Read encoder values for speed feedback */
uint16_t enc_l, enc_r;
motor_ReadEncoder(&enc_l, &enc_r);
```
