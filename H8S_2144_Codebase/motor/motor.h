/*
 * motor.h - Motor Control Driver (Wheel + Blade)
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_MotorControl.md
 *
 * The L200 has 3 motor channels, each using an ST L6203 H-bridge driver:
 *
 *   Left wheel  — PWM via PWMX ch A (DADRA at 0xFFFFA0), dir P2.6/P9.0
 *   Right wheel — PWM via PWMX ch B (DADRB at 0xFFFFA6), dir P2.5/P9.1
 *   Blade       — PWM via TMR0 ch0 TCORB, power via P4.0, direction P2.2
 *
 * Wheel motors use the PWMX D/A-PWM module on the L6203 EN pins for
 * speed control, with direction set by the IN1/IN2 logic inputs.
 * Encoder feedback is captured via FRT input capture interrupts.
 *
 * The blade motor uses TMR0 compare-match B output for PWM speed
 * control (TCORB duty, TCORA=0xFF period). P4.0 controls motor power.
 * Direction is single-direction only via P2.2 (IN1).
 *
 * NOTE: Original test code incorrectly used DAC (DA_DADR0) for blade
 * speed. Ghidra RE of production firmware (vtable at 0x1A324) shows
 * the blade motor actually uses TMR0 PWM:
 *   bladeMotor_TimerPWM_Init (0x129CC): TCORA=0xFF, TCR=9, TCSR=0
 *   bladeMotor_SetSpeed_PWM  (0x12A22): TCORB=(speed*255)/1000
 *   bladeMotor_Brake         (0x12ADA): TCORB=0, TCSR=0, P4.0 ON
 *   bladeMotor_FullStop      (0x12B04): TCORB=0, P4.0 OFF, FRT off
 *
 * Wheel motor speed uses the PWMX D/A-PWM module (NOT TMR0/TMR1):
 *   PWMX_SetLeftSpeed  (0x12792): DADRA=(speed*0xFFF/1000)<<4|2, P9.0=0
 *   PWMX_SetRightSpeed (0x128D6): DADRB=(speed*0xFFF/1000)<<4|2, P9.1=0
 *   PWMX_ZeroLeft      (0x127C2): DADRA=2 (zero speed)
 *   PWMX_ZeroRight     (0x128BC): DADRB=2 (zero speed)
 *
 * Global motor power is controlled by P4.0.
 *
 * Original firmware functions:
 *   SetupHW_Q              (0x121A4) -> motor_Init() (port init + PWMX)
 *   sub_126F6              (0x126F6) -> motor_LeftFwd()
 *   sub_1271A              (0x1271A) -> motor_LeftDisable()
 *   sub_1286A              (0x1286A) -> motor_RightFwd()
 *   sub_1288E              (0x1288E) -> motor_RightDisable()
 *   SetBit0_P9             (0x12762) -> motor_LeftReverse() (P9.0=1)
 *   leftWheel_I2C_DataLow  (0x12792) -> PWMX_SetLeftSpeed (P9.0=0)
 *   rightWheel_I2C_DataLow (0x128D6) -> PWMX_SetRightSpeed (P9.1=0)
 *   fDoPorts1              (0x12BA8) -> motor_BladeEmergencyStop()
 *   stopAndWait            (0x05830) -> (application-level)
 *   motorSCI_StopBoth      (0x0A00E) -> (AO-level stop)
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* ====================================================================
 * Motor Channel Identifiers
 * ==================================================================== */

#define MOTOR_LEFT      0       /* Left wheel motor */
#define MOTOR_RIGHT     1       /* Right wheel motor */
#define MOTOR_BLADE     2       /* Blade (cutting) motor */

/* ====================================================================
 * Motor Direction
 *
 * L6203 truth table:
 *   IN1=H, IN2=L, EN=PWM → Forward
 *   IN1=L, IN2=H, EN=PWM → Reverse
 *   IN1=L, IN2=L, EN=H   → Brake (low-side on)
 *   X,     X,     EN=L   → Disabled (high impedance)
 *
 * Left wheel:  IN1=P2.6, IN2=P9.0  (confirmed by Ghidra SetBit0_P9)
 * Right wheel: IN1=P2.5, IN2=P9.1
 * Blade:       IN1=P2.2  (single direction only)
 * ==================================================================== */

#define MOTOR_DIR_STOP      0   /* Both IN pins low — coast/brake */
#define MOTOR_DIR_FORWARD   1   /* IN1=H, IN2=L — forward */
#define MOTOR_DIR_REVERSE   2   /* IN1=L, IN2=H — reverse */

/* ====================================================================
 * Speed Constants
 *
 * Wheel motors: 8-bit speed value written via PWMX D/A-PWM module.
 *   0x00 = stopped, 0xFF = max speed
 *   Firmware scales: duty = (speed_command * 0xFFF) / 1000
 *
 * Blade motor: TMR0 PWM via TCORB register.
 *   AO commands use 0-1000 range, scaled: duty = (speed * 255) / 1000
 *   0 = stopped, 1000 = max speed
 *
 * Reference from state_MotorControl.md + Ghidra RE:
 * ==================================================================== */

#define MOTOR_SPEED_STOP    0x00    /* Motor stopped */
#define MOTOR_SPEED_MAX     0xFF    /* Maximum speed (8-bit) */

/* Application-level speed values (from AO events, 0-4095 range)
 * These are scaled to 8-bit before writing to hardware.
 * Firmware: state_MotorControl.md speed reference table */
#define MOTOR_CMD_NORMAL    700     /* Standard mowing forward speed */
#define MOTOR_CMD_ACCEL     200     /* Standard acceleration parameter */
#define MOTOR_CMD_STEER     0x5A    /* Standard steering angle (90) */
#define MOTOR_CMD_RETRY     0x177   /* Blocked retry forward (375) */

/* ====================================================================
 * Wheel Motor Speed — PWMX D/A-PWM Module
 *
 * Production firmware uses the PWMX module (NOT TMR0/TMR1) for wheel
 * motor speed control. PWMX is at base address 0xFFFFA0:
 *
 *   DADRA (0xFFFFA0, 16-bit) — Left wheel duty cycle
 *   DADRB (0xFFFFA6, 16-bit) — Right wheel duty cycle
 *   DACR  (0xFFFFA0, 8-bit)  — Control register (bank 1 mode)
 *
 * Register bank switching: DADRB bit 0 (REGS) selects the bank:
 *   REGS=0: 0xFFFFA0 → DADRA, 0xFFFFA6 → DADRB  (normal operation)
 *   REGS=1: 0xFFFFA0 → DACR,  0xFFFFA6 → DACNT   (config mode)
 *
 * Speed value format: ((12bit_duty << 4) | 0x0002)
 *   Upper 12 bits = duty cycle, bit 1 (CFS) always set.
 *   For 8-bit API: DADRA/DADRB = (speed << 8) | 0x0002
 *
 * DACR value from SetupHW_Q: 0x3F (OEA=1, OEB=1, OS=1, CKS=1)
 * STCR bit 4 must be set for PWMX access.
 *
 * Firmware: SetupHW_Q (0x121A4) — PWMX init sequence
 *           PWMX_SetLeftSpeed  (0x12792, Ghidra: leftWheel_I2C_DataLow)
 *           PWMX_SetRightSpeed (0x128D6, Ghidra: rightWheel_I2C_DataLow)
 *           PWMX_ZeroLeft      (0x127C2, Ghidra: doAccessI2C)
 *           PWMX_ZeroRight     (0x128BC)
 * ==================================================================== */

/* Direct PWMX register access (avoids iodefine.h union complexity) */
#define MOTOR_PWMX_DADRA    (*(volatile uint16_t *)0xFFFFA0)  /* Left duty */
#define MOTOR_PWMX_DADRB    (*(volatile uint16_t *)0xFFFFA6)  /* Right duty */
#define MOTOR_PWMX_DACR     (*(volatile uint8_t  *)0xFFFFA0)  /* Control (bank 1) */

#define MOTOR_PWMX_CFS      0x0002  /* CFS bit — always set in duty writes */
#define MOTOR_PWMX_REGS     0x0001  /* REGS bit — bank select (in DADRB) */
#define MOTOR_PWMX_DACR_VAL 0x3F    /* DACR: OEA=1, OEB=1, OS=1, CKS=1 */
#define MOTOR_PWMX_ZERO     0x0002  /* Zero speed (CFS=1 only) */

/* Convert 8-bit speed to 16-bit PWMX register value */
#define MOTOR_PWMX_DUTY(speed8)  (((uint16_t)(speed8) << 8) | MOTOR_PWMX_CFS)

/* STCR bit for PWMX access (Ghidra: SetAccessI2C uses STCR |= 0x10) */
#define MOTOR_STCR_PWMX     0x10    /* STCR bit 4: enable PWMX bus access */

/* ====================================================================
 * Encoder Capture (FRT Input Capture)
 *
 * FRT_TIER interrupt enable bits for wheel encoder capture:
 *   bit 5 (0x20) = ICICE — Left wheel encoder
 *   bit 7 (0x80) = ICIAE — Right wheel encoder
 *
 * These are the ONLY bits the original firmware sets in FRT.TIER for
 * wheel motor control. There is NO TPMR routing — speed is via PWMX.
 *
 * Firmware: sub_126F6 (left: TIER|=0x20), sub_1286A (right: TIER|=0x80)
 * ==================================================================== */

#define MOTOR_ENC_LEFT      0x20    /* FRT_TIER bit 5: left encoder IRQ */
#define MOTOR_ENC_RIGHT     0x80    /* FRT_TIER bit 7: right encoder IRQ */

/* ====================================================================
 * GPIO Pin Masks
 *
 * Port 2 — Motor direction control (P2_DR at 0xFFFF6B)
 *   bit 2 = Blade IN1 (also wire signal detector control)
 *   bit 5 = Right wheel IN1 (forward)
 *   bit 6 = Left wheel IN1 (forward)
 *
 * Port 4 — Power control (P4_DR at 0xFFFF6F)
 *   bit 0 = Global motor power enable
 *   bit 5 = Main PSU enable
 *
 * Port 9 — Motor direction + aux (P9_DR at 0xFFFF79)
 *   bit 0 = Left wheel IN2 (reverse) — Ghidra: SetBit0_P9 (0x12762)
 *   bit 1 = Right wheel IN2 (reverse)
 *   bit 2 = Blade 1-wire bus (DS18B20 temp sensor, motor hot protection)
 *   bit 5 = Blade power enable
 *
 * Port A — Open drain (PA_ODR at 0xFFFFAA)
 *   bit 0 = Blade EN (active-low, open-drain)
 *
 * Firmware: SetupHW_Q (0x121A4), state_MotorControl.md §GPIO Pin Map
 * ==================================================================== */

/* Port 2 direction pins */
#define MOTOR_P2_LEFT_IN1   0x40    /* P2.6 — Left wheel IN1 */
#define MOTOR_P2_RIGHT_IN1  0x20    /* P2.5 — Right wheel IN1 */
#define MOTOR_P2_BLADE_IN1  0x04    /* P2.2 — Blade IN1 */

/* Port 4 power control */
#define MOTOR_P4_POWER      0x01    /* P4.0 — Global motor power enable */
#define MOTOR_P4_PSU        0x20    /* P4.5 — Main PSU enable */

/* Port 9 motor direction + aux */
#define MOTOR_P9_LEFT_IN2   0x01    /* P9.0 — Left wheel IN2 (Ghidra: SetBit0_P9) */
#define MOTOR_P9_RIGHT_IN2  0x02    /* P9.1 — Right wheel IN2 */
#define MOTOR_P9_BLADE_1WIRE 0x04   /* P9.2 — Blade 1-wire bus (DS18B20 temp sensor) */
#define MOTOR_P9_BLADE_SENSE 0x04   /* Legacy alias for MOTOR_P9_BLADE_1WIRE */
#define MOTOR_P9_BLADE_PWR  0x20    /* P9.5 — Blade power enable */

/* Port A open drain */
#define MOTOR_PA_BLADE_EN   0x01    /* PA.0 — Blade EN (active-low OD) */

/* Emergency stop: P2.DR mask to clear bits 0-2
 * Firmware: fDoPorts1 (0x12BA8) — P2_DR &= 0xF8 */
#define MOTOR_P2_ESTOP_MASK 0xF8    /* Clear P2 bits 0-2 */

/* ====================================================================
 * DAC Configuration (legacy — blade no longer uses DAC)
 *
 * DA_DACR control register value to enable both DAC channels.
 * Firmware: SetupHW_Q writes 0xC0 to DA_DACR.
 *   bit 7 (DAOE1) = 1: enable DAC ch1 output
 *   bit 6 (DAOE0) = 1: enable DAC ch0 output
 *   bit 5 (DAE)   = 0: D/A conversion on register write
 * ==================================================================== */

#define MOTOR_DACR_ENABLE   0xC0    /* Enable DAC ch0 + ch1 output */

/* ====================================================================
 * Blade Motor TMR0 PWM Configuration
 *
 * Reversed from production firmware vtable at 0x1A324:
 *   bladeMotor_TimerPWM_Init (0x129CC)
 *   bladeMotor_SetSpeed_PWM  (0x12A22)
 *
 * TMR0 register layout (st_tmr0 at 0xFFFFC8):
 *   TCR  (+0) = 0x09: CKS=001 (phi/8), CCLR=01 (clear on TCORA match)
 *   TCSR (+2) = 0x06 when running: OS=0110 (PWM output on compare B)
 *              = 0x00 when stopped: OS=0000 (output disabled)
 *   TCORA(+4) = 0xFF: period (counter counts 0→255 then resets)
 *   TCORB(+6) = duty: blade PWM duty cycle (0=off, 0xFF=max)
 *
 * Speed conversion: duty = (speed_cmd * 255) / 1000
 *   where speed_cmd is 0-1000 from the AO motor state config
 *
 * STCR bit 0 (ICKS) is set during init — required for timer
 * clock routing on H8S/2144.
 * ==================================================================== */

#define BLADE_TMR_PERIOD    0xFF    /* TMR0.TCORA: full 8-bit period */
#define BLADE_TCR_INIT      0x09    /* phi/8 clock, clear on TCORA match */
#define BLADE_TCSR_PWM      0x06    /* OS=0110: PWM output active */
#define BLADE_TCSR_OFF      0x00    /* OS=0000: output disabled */
#define BLADE_TCSR_MASK     0x1F    /* Bits 4:0 writable; 7:5 are flags (CMFB,CMFA,OVF) */
#define BLADE_SPEED_SCALE   1000    /* AO speed range: 0-1000 */

/* ====================================================================
 * FRT Configuration
 *
 * FRT_TCR init value from SetupHW_Q:
 *   0xE2 = internal clock /8, rising edge capture, clear on compare match
 *
 * Firmware: SetupHW_Q (0x121A4)
 * ==================================================================== */

#define MOTOR_FRT_TCR_INIT  0xE2    /* FRT: internal /8, rising, clear-on-match */

/* ====================================================================
 * Types
 * ==================================================================== */

/* Per-channel motor state */
typedef struct {
    uint8_t  direction;     /* Current direction: STOP/FORWARD/REVERSE */
    uint8_t  speed;         /* Current speed (8-bit: PWM duty or DAC value) */
    uint8_t  enabled;       /* 1 = motor enabled (EN active), 0 = disabled */
    uint8_t  encoder_on;    /* 1 = encoder capture enabled (wheels only) */
} motor_channel_t;

/* Complete motor subsystem state */
typedef struct {
    motor_channel_t left;       /* Left wheel motor state */
    motor_channel_t right;      /* Right wheel motor state */
    motor_channel_t blade;      /* Blade motor state */
    uint8_t  power_on;          /* 1 = global motor power (P4.0) on */
    uint8_t  blade_power;       /* 1 = blade power (P9.5) on */
    uint16_t left_encoder;      /* Last left encoder capture value (FRT_ICRA) */
    uint16_t right_encoder;     /* Last right encoder capture value (FRT_ICRB) */
} motor_state_t;

/* ====================================================================
 * Public API — Initialization
 * ==================================================================== */

/*
 * motor_Init - Initialize motor hardware and state
 *
 * Configures all motor-related GPIO pins, timers, DAC, and FRT
 * to their power-on default state. All motors disabled, power off.
 *
 * This covers the motor-related portion of SetupHW_Q (0x121A4):
 *   - P2_DDR = 0xE7 (direction pins as output)
 *   - P4_DDR = 0x23, P4_DR = 0x20 (PSU on, motors off)
 *   - P9_DDR = 0x23, P9_DR = 0x20 (blade power on standby, IN2 pins low)
 *   - PA_ODR = 0xC0, PA_DDR = 0x03 (blade EN open-drain)
 *   - DA_DADR0 = 0, DA_DACR = 0xC0 (DAC enabled, blade stopped)
 *   - FRT_TCR = 0xE2 (encoder timer config)
 *   - PWMX DACR = 0x3F (enable both wheel PWM channels)
 *   - PWMX DADRA/DADRB = 0x0002 (wheel speeds zeroed)
 *
 * Call once at startup before any motor operations.
 */
void motor_Init(void);

/* ====================================================================
 * Public API — Global Power Control
 * ==================================================================== */

/*
 * motor_PowerOn - Enable global motor power supply
 *
 * Sets P4.0 high to enable the motor power bus.
 * Must be called before any motor can run.
 *
 * Firmware: bset #0, P4_DR in forward/reverse drive sequences
 */
void motor_PowerOn(void);

/*
 * motor_PowerOff - Disable global motor power supply
 *
 * Clears P4.0 to cut power to all wheel motors.
 * Does NOT affect blade motor power (P9.5) or DAC.
 *
 * Firmware: bclr #0, P4_DR in stop sequences
 */
void motor_PowerOff(void);

/* ====================================================================
 * Public API — Left Wheel Motor
 * ==================================================================== */

/*
 * motor_LeftSetSpeed - Set left wheel PWM duty cycle
 *
 * Writes the 8-bit speed value to PWMX DADRA register at 0xFFFFA0.
 * Higher value = higher duty cycle = faster motor.
 *
 * @param speed  PWM duty cycle (0x00=stop, 0xFF=max)
 *
 * Firmware: PWMX_SetLeftSpeed (0x12792, Ghidra: leftWheel_I2C_DataLow)
 */
void motor_LeftSetSpeed(uint8_t speed);

/*
 * motor_LeftForward - Set left wheel to forward direction
 *
 * Sets P2.6 (IN1) high, clears P9.0 (IN2) low.
 * Enables encoder capture interrupt (FRT_TIER bit 5).
 *
 * Firmware: sub_126F6 (0x126F6): P2.6=1, FRT_TIER|=0x20
 *           leftWheel_I2C_DataLow (0x12792): speed + P9.0=0
 */
void motor_LeftForward(void);

/*
 * motor_LeftReverse - Set left wheel to reverse direction
 *
 * Clears P2.6 (IN1) low, sets P9.0 (IN2) high.
 * Enables encoder capture interrupt (FRT_TIER bit 5).
 *
 * Firmware: SetBit0_P9 (0x12762): speed + P9.0=1 (IN2 high)
 */
void motor_LeftReverse(void);

/*
 * motor_LeftDisable - Disable left wheel motor
 *
 * Clears P2.6 (IN1) and P9.0 (IN2) low. Disables encoder capture.
 * Zeroes PWMX DADRA (left speed).
 *
 * Firmware: sub_1271A (0x1271A): P2.6=0, FRT_TIER&=0xDF
 *           PWMX_ZeroLeft (0x127C2): DADRA=2
 */
void motor_LeftDisable(void);

/* ====================================================================
 * Public API — Right Wheel Motor
 * ==================================================================== */

/*
 * motor_RightSetSpeed - Set right wheel PWM duty cycle
 *
 * Writes the 8-bit speed value to PWMX DADRB register at 0xFFFFA6.
 *
 * @param speed  PWM duty cycle (0x00=stop, 0xFF=max)
 *
 * Firmware: PWMX_SetRightSpeed (0x128D6, Ghidra: rightWheel_I2C_DataLow)
 */
void motor_RightSetSpeed(uint8_t speed);

/*
 * motor_RightForward - Set right wheel to forward direction
 *
 * Sets P2.5 (IN1) high, clears P9.1 (IN2) low.
 * Enables encoder capture interrupt (FRT_TIER bit 7).
 *
 * Firmware: sub_1286A (0x1286A): P2.5=1, FRT_TIER|=0x80
 */
void motor_RightForward(void);

/*
 * motor_RightReverse - Set right wheel to reverse direction
 *
 * Clears P2.5 (IN1) low, sets P9.1 (IN2) high.
 * Enables encoder capture interrupt (FRT_TIER bit 7).
 *
 * Firmware: rightWheel_I2C_DataLow (0x128D6): speed + P9.1=0
 *           (reverse uses P9.1=1 via complementary vtable entry)
 */
void motor_RightReverse(void);

/*
 * motor_RightDisable - Disable right wheel motor
 *
 * Clears P2.5 (IN1) and P9.1 (IN2) low. Disables encoder capture.
 * Zeroes PWMX DADRB (right speed).
 *
 * Firmware: sub_1288E (0x1288E): P2.5=0, FRT_TIER&=0x7F
 *           PWMX_ZeroRight (0x128BC): DADRB=2
 */
void motor_RightDisable(void);

/* ====================================================================
 * Public API — Blade Motor
 *
 * CORRECTED: Production firmware uses TMR0 PWM, not DAC.
 * Reversed from blade motor vtable at 0x1A324 (8 function pointers).
 * ==================================================================== */

/*
 * motor_BladeInit - Initialize blade motor TMR0 PWM hardware
 *
 * Reversed from bladeMotor_TimerPWM_Init (0x129CC):
 *   STCR |= 1           (ICKS clock routing)
 *   TMR0.TCORA = 0xFF   (period = full 8-bit count)
 *   TMR0.TCORB = 0      (duty = 0, stopped)
 *   TMR0.TCR = 0x09     (phi/8 clock, clear on TCORA match)
 *   TMR0.TCSR = 0x00    (output disabled initially)
 *
 * Call from motor_Init() or separately before blade operations.
 */
void motor_BladeInit(void);

/*
 * motor_BladeStart - Start blade motor at specified speed
 *
 * Reversed from bladeMotor_SetSpeed_PWM (0x12A22):
 *   1. P2.2 = 1 (IN1 high — set direction)
 *   2. TMR0.TCORB = (speed * 255) / 1000  (PWM duty cycle)
 *   3. TMR0.TCSR = 0x06  (enable PWM output)
 *   4. P4.0 = 1 (motor power on)
 *
 * @param speed  Motor speed command (0-1000)
 */
void motor_BladeStart(uint16_t speed);

/*
 * motor_BladeSetSpeed - Change blade motor speed (while running)
 *
 * Reversed from bladeMotor_SetSpeed_PWM (0x12A22):
 *   TMR0.TCORB = (speed * 255) / 1000
 *   TMR0.TCSR = 0x06 if duty > 0, else 0x00
 *
 * @param speed  Motor speed command (0-1000)
 */
void motor_BladeSetSpeed(uint16_t speed);

/*
 * motor_BladeStop - Normal blade motor stop (power off)
 *
 * Reversed from bladeMotor_PWM_Off (0x12B3A):
 *   1. TMR0.TCORB = 0    (zero PWM duty)
 *   2. TMR0.TCSR = 0x00  (disable output)
 *   3. P4.0 = 0           (motor power off)
 *   4. P2.2 = 0           (IN1 low — direction cleared)
 */
void motor_BladeStop(void);

/*
 * motor_BladeFullStop - Full stop with FRT disable
 *
 * Reversed from bladeMotor_FullStop (0x12B04):
 *   1. TMR0.TCORB = 0    (zero PWM)
 *   2. TMR0.TCSR = 0x00  (disable output)
 *   3. P4.0 = 0           (motor power off)
 *   4. FRT.TIER &= ~0x40  (disable FRT interrupt)
 */
void motor_BladeFullStop(void);

/*
 * motor_BladeBrake - Dynamic braking (short-brake)
 *
 * Reversed from bladeMotor_Brake (0x12ADA):
 *   1. TMR0.TCORB = 0    (no PWM drive)
 *   2. TMR0.TCSR = 0x00  (output disabled)
 *   3. P4.0 = 1           (power STAYS ON — short-brake via H-bridge)
 *
 * With PWM off but power on, the H-bridge low-side FETs create a
 * short-circuit path for back-EMF braking current.
 */
void motor_BladeBrake(void);

/*
 * motor_BladeEmergencyStop - Emergency blade shutdown
 *
 * Firmware: fDoPorts1 (0x12BA8) — clears direction + disables EN.
 * Now also kills TMR0 PWM output.
 */
void motor_BladeEmergencyStop(void);

/* ====================================================================
 * Reversed Structures from Ghidra (blade motor AO config)
 *
 * These mirror the production firmware's Active Object configuration
 * blocks used by initBladeMotorAO (0x13BA) and the blade motor
 * state machine.
 * ==================================================================== */

/* Blade motor state config — at 0x1A150 in firmware ROM
 * Copied by tm_sub_146C and sent via MainMotorState event.
 * Values: {0,0,0,0, 100, 250, 1000, 0} */
typedef struct {
    uint16_t reserved0;     /* +0x00: 0 */
    uint16_t reserved1;     /* +0x02: 0 */
    uint16_t reserved2;     /* +0x04: 0 */
    uint16_t reserved3;     /* +0x06: 0 */
    uint16_t defaultSpeed;  /* +0x08: 100 (0x64) — default blade speed */
    uint16_t rampRate;      /* +0x0A: 250 (0xFA) — speed ramp rate */
    uint16_t maxSpeed;      /* +0x0C: 1000 (0x3E8) — max blade speed */
    uint16_t reserved4;     /* +0x0E: 0 */
} blade_motor_state_cfg_t;

/* Blade motor vtable — at 0x1A324 in firmware ROM
 * 8 function pointers for hardware control, called by AO dispatch. */
typedef struct {
    void (*timerPWM_Init)(void);              /* +0x00: 0x129CC */
    void (*waitReady)(void);                  /* +0x04: 0x12AB2 */
    void (*enableFRT_IRQ)(void);              /* +0x08: 0x12A06 */
    void (*fullStop)(void);                   /* +0x0C: 0x12B04 */
    void (*pwmOff)(void);                     /* +0x10: 0x12B3A */
    void (*setSpeed2)(uint16_t speed);        /* +0x14: 0x12B64 */
    void (*setSpeed)(uint16_t speed);         /* +0x18: 0x12A22 */
    void (*brake)(void);                      /* +0x1C: 0x12ADA */
} blade_motor_vtable_t;

/* ====================================================================
 * Public API — Combined Operations
 * ==================================================================== */

/*
 * motor_StopAll - Disable all motors and cut power
 *
 * Disables both wheel motors and blade motor, then cuts
 * global motor power. Safe shutdown sequence.
 */
void motor_StopAll(void);

/*
 * motor_SetSteering - Set differential steering
 *
 * Computes per-wheel speeds for differential steering.
 * The offset is applied to the base speed:
 *   Right turn: left faster, right slower
 *   Left turn:  left slower, right faster
 *
 * Offset = (angle * base_speed) >> 8
 *
 * @param base_speed  Base wheel speed (8-bit PWM duty)
 * @param angle       Steering angle (0=straight, 0xFF=max turn)
 * @param turn_right  1 = turn right, 0 = turn left
 *
 * Firmware: ao_post_motor_set_steering, differential steering diagram
 */
void motor_SetSteering(uint8_t base_speed, uint8_t angle, uint8_t turn_right);

/* ====================================================================
 * Public API — Status / Diagnostics
 * ==================================================================== */

/*
 * motor_GetState - Get full motor subsystem state
 *
 * @param state  Pointer to motor_state_t to populate
 */
void motor_GetState(motor_state_t *state);

/*
 * motor_ReadEncoder - Read wheel encoder capture values
 *
 * Reads the FRT input capture registers for wheel speed feedback.
 *
 * @param left   Pointer to store left encoder value (FRT_ICRA), or NULL
 * @param right  Pointer to store right encoder value (FRT_ICRB), or NULL
 */
void motor_ReadEncoder(uint16_t *left, uint16_t *right);

/*
 * motor_BladeDevicePresent - 1-wire presence detect on blade bus (P9.2)
 *
 * P9.2 is a Dallas/Maxim 1-wire bus connected to a DS18B20 temperature sensor on the blade motor
 * (originally misidentified as current sense). Firmware uses reading
 * to trigger 'motor hot' protection.
 *
 * Performs a 1-wire reset + presence detect:
 *   1. Drive P9.2 LOW for ~100µs (reset pulse)
 *   2. Release to input (external pull-up brings HIGH)
 *   3. Wait ~70µs for device presence response
 *   4. Read pin: LOW = device pulling (present), HIGH = no response
 *
 * Firmware: MotorCurrentSense_Dispatch (0x153B6, param=2) +
 *           DoMotorBits_CurrentSense    (0x15594)
 *
 * The firmware's full 1-wire stack also includes:
 *   sub_15650 (0x15650) — Write Byte (cmd 0x33=ReadROM, 0xCC=SkipROM)
 *   sub_15686 (0x15686) — Read Byte (LSB first)
 *   sub_14500 (0x14500) — Read ROM + CRC validate
 *   sub_1455A (0x1455A) — Skip ROM + parasitic power
 *
 * @return  1 = device present, 0 = no device
 */
uint8_t motor_BladeDevicePresent(void);

/* Legacy alias — original Ghidra name was blade_CurrentSense_Enable */
uint8_t motor_BladeCurrentOK(void);

/* ====================================================================
 * Public API — Bus Recovery (EMI protection)
 *
 * Motor EMI can corrupt shared control registers via read-modify-write
 * hazards.  motor.c maintains shadow copies of ALL port registers and
 * STCR; all internal functions write from shadows (never read hardware
 * just to modify one bit).
 *
 * motor_RecoverBus() re-writes STCR and P4.DR from their shadows,
 * ensuring PWMX access (IICE), TMR clock (ICKS0), and PSU (P4.5)
 * are maintained even if EMI corrupted the hardware registers.
 * Call periodically during motor operations.
 *
 * motor_KillEncoderIRQs() disables both wheel encoder FRT capture
 * interrupts via the shadow.  Use instead of direct FRT.TIER writes.
 * ==================================================================== */

/*
 * motor_RecoverBus - Restore STCR + P4.DR from shadow after EMI
 *
 * Writes the known-good STCR (0x11: IICE + ICKS0) and P4.DR
 * (preserving PSU bit) to hardware.  Safe to call at any time.
 */
void motor_RecoverBus(void);

/*
 * motor_KillEncoderIRQs - Disable both wheel encoder capture IRQs
 *
 * Clears MOTOR_ENC_LEFT and MOTOR_ENC_RIGHT in the FRT.TIER shadow
 * and writes to hardware.  Use this instead of direct FRT.TIER = 0.
 */
void motor_KillEncoderIRQs(void);

/* ====================================================================
 * Shadow Registers — Port I/O EMI Protection
 *
 * H8S read-modify-write hazard: during motor EMI, a bus read can
 * return corrupted data.  An OR or AND operation then writes garbage
 * back — potentially killing PSU (P4.5), corrupting motor direction
 * (P2/P9), or enabling unwanted interrupts (FRT.TIER).
 *
 * H8S DDR registers are WRITE-ONLY — reads return undefined values.
 * Any DDR modification MUST use the shadow.
 *
 * The original Ambrogio firmware maintains:
 *   bShadow_P6_DR, bShadow_P7_PIN, bShadow_P8_DDR, bShadow_P9_DDR
 *
 * We shadow all motor-relevant port registers.  Shadows are global
 * so other modules (I2C, etc.) can safely share port registers.
 *
 * All shadows are initialized by motor_Init() to match hwinit.c.
 * motor_RecoverBus() periodically refreshes critical hardware regs.
 * ==================================================================== */

/* Port Data Register shadows */
extern uint8_t shadow_p2_dr;        /* P2.DR: motor direction IN1 pins    */
extern uint8_t shadow_p4_dr;        /* P4.DR: P4.0=motor pwr, P4.5=PSU   */
extern uint8_t shadow_p8_dr;        /* P8.DR: I2C/bumper/misc outputs     */
extern uint8_t shadow_p9_dr;        /* P9.DR: motor IN2 + blade power     */

/* Port DDR shadows — DDR is WRITE-ONLY on H8S */
extern uint8_t shadow_p2_ddr;       /* P2.DDR = 0xE7 after hwinit        */
extern uint8_t shadow_p4_ddr;       /* P4.DDR = 0x23 after hwinit        */
extern uint8_t shadow_p8_ddr;       /* P8.DDR = 0x0F after hwinit        */
extern uint8_t shadow_p9_ddr;       /* P9.DDR = 0x23 after hwinit        */

/* FRT.TIER shadow — encoder capture interrupt enables */
extern uint8_t shadow_frt_tier;     /* FRT.TIER: bits 5,7 = encoder IRQs */

#endif /* MOTOR_H */
