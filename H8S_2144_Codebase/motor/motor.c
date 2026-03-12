/*
 * motor.c - Motor Control Driver (Wheel + Blade)
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_MotorControl.md
 *
 * Hardware architecture:
 *   3 × ST L6203 full H-bridge drivers, each with EN, IN1, IN2, SENSE.
 *
 *   Left wheel:  EN = PWMX ch A (DADRA, 0xFFFFA0), IN1 = P2.6, IN2 = P9.0
 *   Right wheel: EN = PWMX ch B (DADRB, 0xFFFFA6), IN1 = P2.5, IN2 = P9.1
 *   Blade:       EN = PA.0 (OD, active-low), IN1 = P2.2, speed = TMR0 TCORB
 *
 * Original firmware functions:
 *   SetupHW_Q              (0x121A4) -> motor_Init()
 *   sub_126F6              (0x126F6) -> motor_LeftForward()
 *   sub_1271A              (0x1271A) -> motor_LeftDisable()
 *   SetBit0_P9             (0x12762) -> motor_LeftReverse() [P9.0=1]
 *   leftWheel_I2C_DataLow  (0x12792) -> PWMX_SetLeftSpeed [P9.0=0]
 *   sub_1286A              (0x1286A) -> motor_RightForward()
 *   sub_1288E              (0x1288E) -> motor_RightDisable()
 *   rightWheel_I2C_DataLow (0x128D6) -> PWMX_SetRightSpeed [P9.1=0]
 *   fDoPorts1              (0x12BA8) -> motor_BladeEmergencyStop()
 *   blade_CurrentSense_Enable (0x15420) -> motor_BladeCurrentOK()
 */

#include "motor.h"
#include "../iodefine.h"

/* ---- Wheel motor speed: PWMX D/A-PWM module ----
 *
 * Production firmware uses the PWMX module at 0xFFFFA0 for wheel motor
 * speed (NOT TMR0/TMR1 and NOT "TPMR" routing as previously assumed).
 *
 * PWMX registers (with bank switching via DADRB bit 0 = REGS):
 *   Bank 0 (REGS=0): 0xFFFFA0 = DADRA (left duty), 0xFFFFA6 = DADRB (right duty)
 *   Bank 1 (REGS=1): 0xFFFFA0 = DACR (control),    0xFFFFA6 = DACNT (counter)
 *
 * Speed value format: (speed_8bit << 8) | 0x0002
 *   Upper 8 bits = duty, bit 1 (CFS) always set.
 *
 * FRT.TIER is used ONLY for encoder capture interrupts:
 *   bit 5 (0x20) = left encoder  (firmware: sub_126F6 sets this)
 *   bit 7 (0x80) = right encoder (firmware: sub_1286A sets this)
 *
 * Ghidra firmware function name mapping:
 *   SetAccessI2C           -> PWMX_WriteLeft  (writes DADRA at 0xFFFFA0)
 *   SetAccessI2C_Right     -> PWMX_WriteRight (writes DADRB at 0xFFFFA6)
 *   leftWheel_I2C_DataLow  -> PWMX_SetLeftSpeed  (speed + P9.0=0)
 *   rightWheel_I2C_DataLow -> PWMX_SetRightSpeed (speed + P9.1=0)
 *   SetBit0_P9             -> PWMX_SetLeftReverse (speed + P9.0=1)
 *   doAccessI2C            -> PWMX_ZeroLeft  (DADRA=2)
 *
 * Firmware: SetupHW_Q (0x121A4) — PWMX init sequence
 */

/* ---- Module state ---- */

/* Track motor state internally for diagnostics */
static motor_channel_t ch_left  = { MOTOR_DIR_STOP, 0, 0, 0 };
static motor_channel_t ch_right = { MOTOR_DIR_STOP, 0, 0, 0 };
static motor_channel_t ch_blade = { MOTOR_DIR_STOP, 0, 0, 0 };
static uint8_t power_on = 0;
static uint8_t blade_power = 0;

/* ---- Shadow registers ----
 *
 * H8S read-modify-write hazard: a `REG |= bit` instruction reads the
 * hardware register, ORs in the new bit, then writes back.  If motor
 * EMI corrupts the bus during the READ phase, the write-back carries
 * garbage — potentially setting FLSHE (flash remap → crash),
 * clearing ICKS0 (TMR clock lost), or clearing P4.5 (PSU power loss).
 *
 * The original Ambrogio firmware keeps shadow copies of:
 *   bShadow_P6_DR, bShadow_P7_PIN, bShadow_P8_DDR, bShadow_P9_DDR
 * (and possibly others) so it NEVER reads a hardware register just
 * to modify one bit.  We follow the same pattern here.
 *
 * Additionally, H8S DDR registers are WRITE-ONLY.  Reading them
 * returns undefined values.  Any DDR RMW (e.g. DDR |= bit) MUST
 * use a shadow.
 *
 * Shadows are global (not static) so other modules (I2C, etc.) can
 * safely modify shared port registers via motor.h externs.
 *
 * STCR is the only private shadow — only motor.c writes STCR.
 */

/* STCR (0xFFFFC3) — Serial Timer Control Register (private)
 *   Bit 4 IICE : PWMX bus access (set by motor_Init)
 *   Bit 3 FLSHE: flash enable — MUST NEVER be set accidentally!
 *   Bit 0 ICKS0: TMR internal clock source (set by motor_BladeInit)
 *   Steady-state value after init: 0x11 (IICE + ICKS0)
 */
static uint8_t stcr_shadow = 0x00;

/* Port data register shadows — initialized in motor_Init()
 * Values match hwinit.c HardwareSetup() initial writes.  */
uint8_t shadow_p2_dr  = 0x00;   /* P2.DR: all direction pins off      */
uint8_t shadow_p4_dr  = 0x20;   /* P4.DR: P4.5=PSU on, P4.0=motor off */
uint8_t shadow_p8_dr  = 0x00;   /* P8.DR: all outputs low              */
uint8_t shadow_p9_dr  = 0x20;   /* P9.DR: P9.5=blade power standby     */

/* Port DDR shadows — DDR is WRITE-ONLY on H8S, reads are undefined.
 * Values match hwinit.c HardwareSetup() initial writes.  */
uint8_t shadow_p2_ddr = 0xE7;   /* P2.DDR: bits 7,6,5,2,1,0 output    */
uint8_t shadow_p4_ddr = 0x23;   /* P4.DDR: bits 5,1,0 output           */
uint8_t shadow_p8_ddr = 0x0F;   /* P8.DDR: bits 3-0 output             */
uint8_t shadow_p9_ddr = 0x23;   /* P9.DDR: bits 5,1,0 output           */

/* FRT.TIER shadow — encoder capture interrupt enables */
uint8_t shadow_frt_tier = 0x00;

/* ====================================================================
 * Initialization
 * ==================================================================== */

/*
 * motor_Init - Initialize motor hardware
 *
 * Firmware: SetupHW_Q (0x121A4) — motor-related portion
 *
 * Ghidra decompilation (motor-relevant excerpts):
 *   P2_DR_BYTE = 0;            // All direction pins low
 *   P2_DDR = 0xE7;             // P2.7,6,5,2,1,0 output
 *   P4_DR_BYTE = 0x20;         // P4.5 high (PSU on), P4.0 low (motors off)
 *   P4_DDR = 0x23;             // P4.5,1,0 output
 *   P9_DR_BYTE = 0x20;         // P9.5 high (blade power standby)
 *   P9_DDR = 0x23;             // P9.5,1,0 output
 *   PA_ODR_BYTE = 0xC0;        // PA.6,7 open-drain (blade EN not OD yet)
 *   PA_DDR = 0x03;             // PA.0,1 output
 *   FRT_TCR_BYTE = 0xE2;       // FRT: internal /8, rising edge
 *   DA_DADR0 = 0;              // Blade DAC off
 *   DA_DADR1 = 0;              // Aux DAC off
 *   DA_DACR_BYTE = 0xC0;       // Enable DAC ch0 + ch1
 */
void motor_Init(void)
{
    /* Initialize shadow registers to known-good state.
     * Values match hwinit.c HardwareSetup() so the shadows are
     * correct even if motor_Init is called as a recovery re-init. */
    shadow_p2_dr  = 0x00;              /* All direction pins off */
    shadow_p4_dr  = MOTOR_P4_PSU;      /* PSU on, motor power off */
    shadow_p8_dr  = 0x00;              /* Preserve — no motor bits */
    shadow_p9_dr  = MOTOR_P9_BLADE_PWR;/* Blade power standby, IN2 off */
    shadow_p2_ddr = 0xE7;              /* Match hwinit.c */
    shadow_p4_ddr = 0x23;
    shadow_p8_ddr = 0x0F;
    shadow_p9_ddr = 0x23;
    shadow_frt_tier = 0x00;            /* No encoder interrupts */

    /* Write all port shadows to hardware (write-only, no reads) */
    P2.DR.BYTE = shadow_p2_dr;
    P4.DR.BYTE = shadow_p4_dr;
    P9.DR.BYTE = shadow_p9_dr;

    /* Blade EN disabled: PA.0 = 1 (open-drain release = high-Z = EN off) */
    PA.ODR.BYTE |= MOTOR_PA_BLADE_EN;

    /* Configure FRT for encoder capture */
    FRT.TCR.BYTE = MOTOR_FRT_TCR_INIT;

    /* Disable encoder capture interrupts — shadow write */
    FRT.TIER.BYTE = shadow_frt_tier;

    /* ---- PWMX init (wheel motor speed) ----
     * Reversed from SetupHW_Q (0x121A4):
     *   STCR &= 0xFC; STCR |= 0x10;
     *   DADRB=3 (REGS=1), DACR=0x3F, DADRB=2 (REGS=0)
     *   DADRA=2, DADRB=2 (zero speed)
     *
     * Shadow register: write-only, never read STCR during RMW.
     * Clears ICKS bits (1:0) and sets IICE (bit 4) for PWMX access.
     * ICKS0 will be re-added by motor_BladeInit below.
     */
    stcr_shadow = MOTOR_STCR_PWMX;       /* 0x10 — IICE only */
    STCR.BYTE = stcr_shadow;

    /* Switch to bank 1 (REGS=1) to access DACR */
    MOTOR_PWMX_DADRB = MOTOR_PWMX_CFS | MOTOR_PWMX_REGS;  /* 0x0003 */
    /* Write DACR: OEA=1, OEB=1, OS=1, CKS=1 */
    MOTOR_PWMX_DACR  = MOTOR_PWMX_DACR_VAL;                /* 0x3F */
    /* Switch back to bank 0 (REGS=0) */
    MOTOR_PWMX_DADRB = MOTOR_PWMX_CFS;                     /* 0x0002 */
    /* Zero both wheel speeds */
    MOTOR_PWMX_DADRA = MOTOR_PWMX_ZERO;                    /* 0x0002 */
    MOTOR_PWMX_DADRB = MOTOR_PWMX_ZERO;                    /* 0x0002 */

    /* Initialize DAC (legacy — kept for aux use) */
    DA.DADR0 = MOTOR_SPEED_STOP;
    DA.DADR1 = 0;
    DA.DACR.BYTE = MOTOR_DACR_ENABLE;

    /* Initialize blade motor TMR0 PWM (reversed from 0x129CC) */
    motor_BladeInit();

    /* Reset internal state */
    ch_left.direction  = MOTOR_DIR_STOP;
    ch_left.speed      = 0;
    ch_left.enabled    = 0;
    ch_left.encoder_on = 0;

    ch_right.direction  = MOTOR_DIR_STOP;
    ch_right.speed      = 0;
    ch_right.enabled    = 0;
    ch_right.encoder_on = 0;

    ch_blade.direction  = MOTOR_DIR_STOP;
    ch_blade.speed      = 0;
    ch_blade.enabled    = 0;
    ch_blade.encoder_on = 0;

    power_on    = 0;
    blade_power = 1;    /* P9.5 is set high at init (standby) */
}

/* ====================================================================
 * Global Power Control
 * ==================================================================== */

void motor_PowerOn(void)
{
    shadow_p4_dr |= MOTOR_P4_POWER;
    P4.DR.BYTE = shadow_p4_dr;
    power_on = 1;
}

void motor_PowerOff(void)
{
    shadow_p4_dr &= (uint8_t)~MOTOR_P4_POWER;
    P4.DR.BYTE = shadow_p4_dr;
    power_on = 0;
}

/* ====================================================================
 * Left Wheel Motor
 * ==================================================================== */

void motor_LeftSetSpeed(uint8_t speed)
{
    STCR.BYTE = stcr_shadow;              /* shadow: no read-modify-write */
    MOTOR_PWMX_DADRA = MOTOR_PWMX_DUTY(speed);
    ch_left.speed = speed;
}

/*
 * motor_LeftForward - Enable left wheel in forward direction
 *
 * Firmware: sub_126F6 (0x126F6) + leftWheel_I2C_DataLow (0x12792)
 *   P2.6 = 1 (IN1 = run enable), P9.0 = 0 (IN2 = forward)
 *   FRT_TIER |= 0x20 (encoder capture)
 *
 * H-bridge direction: IN1=1, IN2=0 → Forward
 */
void motor_LeftForward(void)
{
    /* Set direction: IN1=H (P2.6), IN2=L (P9.0) — shadow writes */
    shadow_p2_dr |= MOTOR_P2_LEFT_IN1;
    P2.DR.BYTE = shadow_p2_dr;
    shadow_p9_dr &= (uint8_t)~MOTOR_P9_LEFT_IN2;
    P9.DR.BYTE = shadow_p9_dr;

    /* Enable encoder capture (matching firmware: TIER |= 0x20) */
    shadow_frt_tier |= MOTOR_ENC_LEFT;
    FRT.TIER.BYTE = shadow_frt_tier;

    ch_left.direction  = MOTOR_DIR_FORWARD;
    ch_left.enabled    = 1;
    ch_left.encoder_on = 1;
}

/*
 * motor_LeftReverse - Enable left wheel in reverse direction
 *
 * Firmware: SetBit0_P9 (0x12762):
 *   speed → PWMX DADRA, P9.0 = 1 (IN2 high = reverse)
 *   IN1 stays HIGH (run enable); IN2 selects direction.
 *
 * H-bridge direction truth table (confirmed from production firmware):
 *   IN1=1, IN2=0 → Forward
 *   IN1=1, IN2=1 → Reverse
 *   IN1=0         → Disabled (coast)
 */
void motor_LeftReverse(void)
{
    /* Set direction: IN1=H (P2.6 = run), IN2=H (P9.0 = reverse) — shadow */
    shadow_p2_dr |= MOTOR_P2_LEFT_IN1;
    P2.DR.BYTE = shadow_p2_dr;
    shadow_p9_dr |= MOTOR_P9_LEFT_IN2;
    P9.DR.BYTE = shadow_p9_dr;

    /* Enable encoder capture (matching firmware: TIER |= 0x20) */
    shadow_frt_tier |= MOTOR_ENC_LEFT;
    FRT.TIER.BYTE = shadow_frt_tier;

    ch_left.direction  = MOTOR_DIR_REVERSE;
    ch_left.enabled    = 1;
    ch_left.encoder_on = 1;
}

/*
 * motor_LeftDisable - Disable left wheel motor
 *
 * Firmware: sub_1271A (0x1271A)
 *   P2_DR_BYTE &= 0xBF;        // P2.6 = 0 (IN1 low)
 *   FRT_TIER_BYTE &= 0xDF;     // Disable encoder interrupt
 *
 * Also zeros PWMX left speed (doAccessI2C at 0x127C2: DADRA=2).
 */
void motor_LeftDisable(void)
{
    /* Clear direction: IN1=L (P2.6), IN2=L (P9.0) → coast — shadow */
    shadow_p2_dr &= (uint8_t)~MOTOR_P2_LEFT_IN1;
    P2.DR.BYTE = shadow_p2_dr;
    shadow_p9_dr &= (uint8_t)~MOTOR_P9_LEFT_IN2;
    P9.DR.BYTE = shadow_p9_dr;

    /* Disable encoder capture (matching firmware: TIER &= 0xDF) */
    shadow_frt_tier &= (uint8_t)~MOTOR_ENC_LEFT;
    FRT.TIER.BYTE = shadow_frt_tier;

    /* Zero PWMX left speed (shadow: no read-modify-write on STCR) */
    STCR.BYTE = stcr_shadow;
    MOTOR_PWMX_DADRA = MOTOR_PWMX_ZERO;

    ch_left.direction  = MOTOR_DIR_STOP;
    ch_left.speed      = 0;
    ch_left.enabled    = 0;
    ch_left.encoder_on = 0;
}

/* ====================================================================
 * Right Wheel Motor
 * ==================================================================== */

void motor_RightSetSpeed(uint8_t speed)
{
    STCR.BYTE = stcr_shadow;              /* shadow: no read-modify-write */
    MOTOR_PWMX_DADRB = MOTOR_PWMX_DUTY(speed);
    ch_right.speed = speed;
}

/*
 * motor_RightForward - Enable right wheel in forward direction
 *
 * Firmware: sub_1286A (0x1286A)
 *   P2_DR_BYTE |= 0x20;        // P2.5 = 1 (IN1 high)
 *   FRT_TIER_BYTE |= 0x80;     // Enable encoder capture interrupt
 *
 * rightWheel_I2C_DataLow (0x128D6):
 *   speed → PWMX DADRB, P9.1 = 0 (IN2 low = forward)
 */
void motor_RightForward(void)
{
    /* Set direction: IN1=H (P2.5), IN2=L (P9.1) — shadow */
    shadow_p2_dr |= MOTOR_P2_RIGHT_IN1;
    P2.DR.BYTE = shadow_p2_dr;
    shadow_p9_dr &= (uint8_t)~MOTOR_P9_RIGHT_IN2;
    P9.DR.BYTE = shadow_p9_dr;

    /* Enable encoder capture (matching firmware: TIER |= 0x80) */
    shadow_frt_tier |= MOTOR_ENC_RIGHT;
    FRT.TIER.BYTE = shadow_frt_tier;

    ch_right.direction  = MOTOR_DIR_FORWARD;
    ch_right.enabled    = 1;
    ch_right.encoder_on = 1;
}

/*
 * motor_RightReverse - Enable right wheel in reverse direction
 *
 * Firmware: SetBit1_P9 (0x1290C):
 *   speed → PWMX DADRB, P9.1 = 1 (IN2 high = reverse)
 *   IN1 stays HIGH (run enable); IN2 selects direction.
 */
void motor_RightReverse(void)
{
    /* Set direction: IN1=H (P2.5 = run), IN2=H (P9.1 = reverse) — shadow */
    shadow_p2_dr |= MOTOR_P2_RIGHT_IN1;
    P2.DR.BYTE = shadow_p2_dr;
    shadow_p9_dr |= MOTOR_P9_RIGHT_IN2;
    P9.DR.BYTE = shadow_p9_dr;

    /* Enable encoder capture (matching firmware: TIER |= 0x80) */
    shadow_frt_tier |= MOTOR_ENC_RIGHT;
    FRT.TIER.BYTE = shadow_frt_tier;

    ch_right.direction  = MOTOR_DIR_REVERSE;
    ch_right.enabled    = 1;
    ch_right.encoder_on = 1;
}

/*
 * motor_RightDisable - Disable right wheel motor
 *
 * Firmware: sub_1288E (0x1288E)
 *   P2_DR_BYTE &= 0xDF;        // P2.5 = 0 (IN1 low)
 *   FRT_TIER_BYTE &= 0x7F;     // Disable encoder interrupt
 *
 * Also zeros PWMX right speed (0x128BC: DADRB=2).
 */
void motor_RightDisable(void)
{
    /* Clear direction: IN1=L (P2.5), IN2=L (P9.1) → coast — shadow */
    shadow_p2_dr &= (uint8_t)~MOTOR_P2_RIGHT_IN1;
    P2.DR.BYTE = shadow_p2_dr;
    shadow_p9_dr &= (uint8_t)~MOTOR_P9_RIGHT_IN2;
    P9.DR.BYTE = shadow_p9_dr;

    /* Disable encoder capture (matching firmware: TIER &= 0x7F) */
    shadow_frt_tier &= (uint8_t)~MOTOR_ENC_RIGHT;
    FRT.TIER.BYTE = shadow_frt_tier;

    /* Zero PWMX right speed (shadow: no read-modify-write on STCR) */
    STCR.BYTE = stcr_shadow;
    MOTOR_PWMX_DADRB = MOTOR_PWMX_ZERO;

    ch_right.direction  = MOTOR_DIR_STOP;
    ch_right.speed      = 0;
    ch_right.enabled    = 0;
    ch_right.encoder_on = 0;
}

/* ====================================================================
 * Blade Motor
 *
 * CORRECTED: Production firmware uses TMR0 PWM, not DAC.
 * Reversed from blade motor vtable at 0x1A324:
 *
 *   - Speed: TMR0 PWM via TCORB (NOT DAC as previously assumed)
 *   - Power: P4.0 motor power enable
 *   - Direction: single direction only (IN1 = P2.2)
 *   - No encoder feedback
 *   - TMR0.TCORA = 0xFF (period), TMR0.TCORB = duty (0-255)
 *   - TMR0.TCR = 0x09 (phi/8, clear on TCORA)
 *   - TMR0.TCSR = 0x06 (PWM output) or 0x00 (disabled)
 *
 * Firmware vtable handlers reversed from 0x1A324:
 *   [0] timerPWM_Init  (0x129CC) — TMR0 init
 *   [1] waitReady      (0x12AB2) — 500ms settling delay
 *   [2] enableFRT_IRQ  (0x12A06) — FRT TIER |= 0x40
 *   [3] fullStop       (0x12B04) — PWM off + power off + FRT off
 *   [4] pwmOff         (0x12B3A) — PWM off + power off
 *   [5] setSpeed2      (0x12B64) — set speed (duplicate)
 *   [6] setSpeed       (0x12A22) — TCORB=(speed*255)/1000, P4.0 on
 *   [7] brake          (0x12ADA) — PWM off, P4.0 ON (short-brake)
 * ==================================================================== */

/*
 * motor_BladeInit - Initialize blade TMR0 PWM hardware
 *
 * Reversed from bladeMotor_TimerPWM_Init (0x129CC)
 */
void motor_BladeInit(void)
{
    /* Enable ICKS clock routing (STCR bit 0) — required for TMR
     * Shadow register: no read-modify-write on STCR */
    stcr_shadow |= 0x01;
    STCR.BYTE = stcr_shadow;

    /* Configure TMR0 for blade PWM:
     *   TCORA = 0xFF (period = full count)
     *   TCORB = 0x00 (duty = 0, stopped)
     *   TCR   = 0x09 (phi/8 clock, clear on TCORA match)
     *   TCSR  = 0x00 (output disabled initially) */
    TMR0.TCORA = BLADE_TMR_PERIOD;
    TMR0.TCORB = MOTOR_SPEED_STOP;
    TMR0.TCR.BYTE = BLADE_TCR_INIT;
    TMR0.TCSR.BYTE = BLADE_TCSR_OFF;
}

/*
 * motor_BladeStart - Start blade motor at specified speed
 *
 * Reversed from bladeMotor_SetSpeed_PWM (0x12A22)
 */
void motor_BladeStart(uint16_t speed)
{
    uint8_t duty;

    /* Set blade direction (IN1 high) — shadow */
    shadow_p2_dr |= MOTOR_P2_BLADE_IN1;
    P2.DR.BYTE = shadow_p2_dr;

    /* Convert speed (0-1000) to 8-bit PWM duty (0-255) */
    duty = (uint8_t)(((uint32_t)speed * 0xFF) / BLADE_SPEED_SCALE);
    TMR0.TCORB = duty;

    /* Enable/disable PWM output based on duty */
    if (duty == 0) {
        TMR0.TCSR.BYTE = BLADE_TCSR_OFF;
    } else {
        TMR0.TCSR.BYTE = BLADE_TCSR_PWM;
    }

    /* Motor power on (P4.0) — shadow */
    shadow_p4_dr |= MOTOR_P4_POWER;
    P4.DR.BYTE = shadow_p4_dr;

    ch_blade.direction = MOTOR_DIR_FORWARD;
    ch_blade.speed     = duty;
    ch_blade.enabled   = 1;
    power_on = 1;
}

/*
 * motor_BladeSetSpeed - Change blade motor speed (while running)
 *
 * Reversed from bladeMotor_SetSpeed_PWM (0x12A22)
 */
void motor_BladeSetSpeed(uint16_t speed)
{
    uint8_t duty;

    duty = (uint8_t)(((uint32_t)speed * 0xFF) / BLADE_SPEED_SCALE);
    TMR0.TCORB = duty;

    if (duty == 0) {
        TMR0.TCSR.BYTE = BLADE_TCSR_OFF;
    } else {
        TMR0.TCSR.BYTE = BLADE_TCSR_PWM;
    }

    ch_blade.speed = duty;
}

/*
 * motor_BladeStop - Normal blade motor stop (power off)
 *
 * Reversed from bladeMotor_PWM_Off (0x12B3A)
 */
void motor_BladeStop(void)
{
    /* Zero PWM duty and disable output */
    TMR0.TCORB = MOTOR_SPEED_STOP;
    TMR0.TCSR.BYTE = BLADE_TCSR_OFF;

    /* Motor power off (P4.0) — shadow */
    shadow_p4_dr &= (uint8_t)~MOTOR_P4_POWER;
    P4.DR.BYTE = shadow_p4_dr;

    /* Clear direction — shadow */
    shadow_p2_dr &= (uint8_t)~MOTOR_P2_BLADE_IN1;
    P2.DR.BYTE = shadow_p2_dr;

    ch_blade.direction = MOTOR_DIR_STOP;
    ch_blade.speed     = 0;
    ch_blade.enabled   = 0;
    power_on = 0;
}

/*
 * motor_BladeFullStop - Full stop with FRT disable
 *
 * Reversed from bladeMotor_FullStop (0x12B04)
 */
void motor_BladeFullStop(void)
{
    /* Zero PWM duty and disable output */
    TMR0.TCORB = MOTOR_SPEED_STOP;
    TMR0.TCSR.BYTE = BLADE_TCSR_OFF;

    /* Motor power off (P4.0) — shadow */
    shadow_p4_dr &= (uint8_t)~MOTOR_P4_POWER;
    P4.DR.BYTE = shadow_p4_dr;

    /* Disable FRT output compare interrupt — shadow */
    shadow_frt_tier &= (uint8_t)~0x40;
    FRT.TIER.BYTE = shadow_frt_tier;

    ch_blade.direction = MOTOR_DIR_STOP;
    ch_blade.speed     = 0;
    ch_blade.enabled   = 0;
    power_on = 0;
}

/*
 * motor_BladeBrake - Dynamic braking (short-brake)
 *
 * Reversed from bladeMotor_Brake (0x12ADA):
 * PWM output disabled but motor power stays ON. The H-bridge
 * low-side FETs provide a short-circuit path for back-EMF,
 * creating braking torque.
 */
void motor_BladeBrake(void)
{
    /* Zero PWM — no drive signal */
    TMR0.TCORB = MOTOR_SPEED_STOP;
    TMR0.TCSR.BYTE = BLADE_TCSR_OFF;

    /* Power stays ON — H-bridge shorts for braking — shadow */
    shadow_p4_dr |= MOTOR_P4_POWER;
    P4.DR.BYTE = shadow_p4_dr;

    ch_blade.direction = MOTOR_DIR_STOP;
    ch_blade.enabled   = 0;
    power_on = 1;
}

/*
 * motor_BladeEmergencyStop - Emergency blade shutdown
 *
 * Firmware: fDoPorts1 (0x12BA8) — clears direction + kills output.
 * Also disables TMR0 PWM to ensure motor stops.
 */
void motor_BladeEmergencyStop(void)
{
    /* Kill TMR0 PWM output immediately */
    TMR0.TCORB = MOTOR_SPEED_STOP;
    TMR0.TCSR.BYTE = BLADE_TCSR_OFF;

    /* Clear P2 bits 0-2: blade IN1 + other low bits — shadow */
    shadow_p2_dr &= MOTOR_P2_ESTOP_MASK;
    P2.DR.BYTE = shadow_p2_dr;

    /* Disable blade EN (PA.0 high = OD release) */
    PA.ODR.BYTE |= MOTOR_PA_BLADE_EN;

    ch_blade.direction = MOTOR_DIR_STOP;
    ch_blade.enabled   = 0;
}

/* ====================================================================
 * Combined Operations
 * ==================================================================== */

void motor_StopAll(void)
{
    /* Disable blade first (safety priority) */
    motor_BladeStop();

    /* Disable both wheel motors */
    motor_LeftDisable();
    motor_RightDisable();

    /* Cut global motor power */
    motor_PowerOff();
}

/*
 * motor_SetSteering - Differential steering
 *
 * Firmware: ao_post_motor_set_steering, state_MotorControl.md
 *
 * The steering model applies an offset to each wheel's speed:
 *   offset = (angle * base_speed) >> 8
 *
 * For a right turn: left wheel runs faster, right wheel slower.
 * For a left turn:  right wheel runs faster, left wheel slower.
 *
 * Speed is clamped to 0-255 (8-bit PWM range).
 */
void motor_SetSteering(uint8_t base_speed, uint8_t angle, uint8_t turn_right)
{
    uint16_t offset;
    uint16_t speed_fast, speed_slow;

    /* Compute steering offset: proportional to angle and base speed */
    offset = ((uint16_t)angle * (uint16_t)base_speed) >> 8;

    /* Compute per-wheel speeds with clamping */
    speed_fast = (uint16_t)base_speed + offset;
    speed_slow = (uint16_t)base_speed - offset;

    /* Clamp to 8-bit range */
    if (speed_fast > 0xFF)
        speed_fast = 0xFF;
    /* speed_slow can underflow if offset > base_speed */
    if (offset > base_speed)
        speed_slow = 0;

    if (turn_right) {
        /* Right turn: left fast, right slow */
        motor_LeftSetSpeed((uint8_t)speed_fast);
        motor_RightSetSpeed((uint8_t)speed_slow);
    } else {
        /* Left turn: left slow, right fast */
        motor_LeftSetSpeed((uint8_t)speed_slow);
        motor_RightSetSpeed((uint8_t)speed_fast);
    }
}

/* ====================================================================
 * Status / Diagnostics
 * ==================================================================== */

void motor_GetState(motor_state_t *state)
{
    state->left          = ch_left;
    state->right         = ch_right;
    state->blade         = ch_blade;
    state->power_on      = power_on;
    state->blade_power   = blade_power;
    state->left_encoder  = FRT.ICRA;
    state->right_encoder = FRT.ICRB;
}

void motor_ReadEncoder(uint16_t *left, uint16_t *right)
{
    if (left != (void *)0)
        *left = FRT.ICRA;
    if (right != (void *)0)
        *right = FRT.ICRB;
}

/*
 * motor_BladeDevicePresent - 1-wire presence detect on blade bus (P9.2)
 *
 * Firmware: MotorCurrentSense_Dispatch (0x153B6, param_1=2) +
 *           DoMotorBits_CurrentSense    (0x15594)
 *
 * P9.2 is a Dallas/Maxim 1-wire bus connected to a DS18B20 temperature sensor on the blade motor
 * (NOT a simple current sense pin). Firmware reads temperature
 * to trigger 'motor hot' protection.
 *
 * Ghidra reveals the full protocol stack:
 *   sub_15650 (0x15650) — 1-wire Write Byte (LSB first)
 *   sub_15686 (0x15686) — 1-wire Read Byte (LSB first)
 *   sub_1562C (0x1562C) — Write-0 slot (drive LOW ~60µs)
 *   sub_15608 (0x15608) — Write-1 slot (drive LOW ~1µs)
 *   sub_14500 (0x14500) — Read ROM (cmd 0x33) + CRC validate
 *   sub_1455A (0x1455A) — Skip ROM (cmd 0xCC) + parasitic power
 *
 * 1-wire reset + presence detect sequence:
 *   1. Master drives P9.2 LOW (output) for ≥100µs (reset pulse)
 *   2. Master releases P9.2 to input (pull-up brings HIGH)
 *   3. Device responds by pulling LOW within 15-60µs (presence pulse)
 *   4. Device releases after 60-240µs
 *
 * Detection:
 *   After release, pin LOW = device responded (present) = SUCCESS
 *   After release, pin HIGH = no response (absent) = FAIL
 *
 * The three 1-wire channels in the original firmware:
 *   Channel 0: P6.6 (left wheel module)
 *   Channel 1: P6.5 (right wheel module)
 *   Channel 2: P9.2 (blade module)
 *
 * NOTE: H8S P9.DDR is write-only. Uses shadow register.
 * motor_Init() sets P9_DDR = 0x23 (bits 5,1,0 output).
 *
 * @return  1 = device present, 0 = no device
 */

uint8_t motor_BladeDevicePresent(void)
{
    volatile uint16_t i;

    /* Step 1: Reset pulse — drive P9.2 LOW for ~100µs.
     * Firmware: MotorCurrentSense_Dispatch(2, 500) ≈ 100-200µs */
    shadow_p9_dr &= (uint8_t)~MOTOR_P9_BLADE_SENSE;   /* P9.2 = 0 */
    P9.DR.BYTE = shadow_p9_dr;
    shadow_p9_ddr |= MOTOR_P9_BLADE_SENSE;             /* DDR bit 2 = output */
    P9.DDR = shadow_p9_ddr;

    for (i = 0; i < 500; i++)    /* ~100µs reset pulse */
        ;

    /* Step 2: Release P9.2 to input — pull-up brings line HIGH */
    shadow_p9_ddr &= (uint8_t)~MOTOR_P9_BLADE_SENSE;   /* DDR bit 2 = input */
    P9.DDR = shadow_p9_ddr;

    /* Step 3: Wait ~70µs for device presence response.
     * 1-wire device pulls LOW within 15-60µs of release.
     * Sample during the presence pulse window. */
    for (i = 0; i < 200; i++)    /* ~70µs presence window */
        ;

    /* Step 4: Read pin.
     * Pin LOW = device pulling (presence pulse) = FOUND
     * Pin HIGH = no response = NOT FOUND */
    if (P9.DR.BYTE & MOTOR_P9_BLADE_SENSE)
        return 0;   /* No device — pin stayed HIGH */
    else
        return 1;   /* Device present — pulling LOW */
}

/* Legacy alias — original Ghidra name was blade_CurrentSense_Enable
 * but the pin is actually a 1-wire bus, not current sense */
uint8_t motor_BladeCurrentOK(void)
{
    return motor_BladeDevicePresent();
}

/* ====================================================================
 * Bus Recovery
 * ==================================================================== */

void motor_RecoverBus(void)
{
    STCR.BYTE = stcr_shadow;
    P4.DR.BYTE = shadow_p4_dr;     /* Protect PSU (P4.5) from EMI loss */
}

void motor_KillEncoderIRQs(void)
{
    shadow_frt_tier &= (uint8_t)~(MOTOR_ENC_LEFT | MOTOR_ENC_RIGHT);
    FRT.TIER.BYTE = shadow_frt_tier;
}
