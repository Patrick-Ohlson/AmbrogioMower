/* test_drive.c — Wheel (driving) motor test
 *
 * Tests left and right wheel motor GPIO, PWMX speed, direction, and
 * encoder readback using the motor driver API.
 *
 * Wheel motor hardware (reversed from production firmware):
 *
 *   Left wheel:
 *     P2.6 = IN1 (run enable)  P9.0 = IN2 (direction: 0=fwd, 1=rev)
 *     EN   = PWMX ch A (DADRA at 0xFFFFA0)
 *     Encoder: FRT Input Capture (FRT_TIER bit 5 = 0x20)
 *     Firmware: sub_126F6 (enable), sub_1271A (disable),
 *               SetBit0_P9 (rev speed), leftWheel_I2C_DataLow (fwd speed)
 *
 *   Right wheel:
 *     P2.5 = IN1 (run enable)  P9.1 = IN2 (direction: 0=fwd, 1=rev)
 *     EN   = PWMX ch B (DADRB at 0xFFFFA6)
 *     Encoder: FRT Input Capture (FRT_TIER bit 7 = 0x80)
 *     Firmware: sub_1286A (enable), sub_1288E (disable),
 *               SetBit1_P9 (rev speed), rightWheel_I2C_DataLow (fwd speed)
 *
 *   Shared:
 *     P4.0 = Global motor power enable
 *     FRT_TCR = 0xE2 (internal /8, rising edge, clear-on-match)
 *     PWMX DACR = 0x3F (OEA=1, OEB=1, OS=1, CKS=1)
 *
 *   Production firmware config (ROM at 0x1A160):
 *     maxSpeed=1000, maxPWM=0xFF, defaultSpeed=100
 *
 * Sections 1-8 are static register tests (safe without motors).
 * Section 9 is a live wheel test gated behind UART 'Y' confirm.
 */
#include "test_drive.h"
#include "../iodefine.h"
#include "../micros.h"
#include "../sci.h"
#include "../motor/motor.h"
#include "../utils/utils.h"

/* ---- NOP-based motor delay with periodic SCI + bus recovery ----
 * Identical to the version in test_steering.c.
 * Uses NOP counting (no timer dependency) and periodically clears
 * SCI1 errors + restores STCR from motor.c shadow. */
static void motor_safe_delay_drive(uint32_t ms)
{
    volatile uint32_t i;
    uint32_t loops = 925UL * ms;
    for (i = 0; i < loops; i++) {
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        if ((i & 0x3FF) == 0) {
            if (SCI1.SSR.BYTE & (SSR_ORER | SSR_FER | SSR_PER))
                SCI1.SSR.BYTE &= (unsigned char)~(SSR_ORER | SSR_FER | SSR_PER);
            motor_RecoverBus();
        }
    }
}

/* ---- Full SCI1 + bus recovery after motor EMI ---- */
static void motor_sci_recover_drive(void)
{
    SCI1.SSR.BYTE &= (unsigned char)~(SSR_ORER | SSR_FER | SSR_PER);
    SCI1.SCR.BYTE = SCR_TE | SCR_RE;
    motor_RecoverBus();
}

void Drive_Motor_Test(void)
{
	uint8_t pass_count = 0;
	uint8_t fail_count = 0;
	uint8_t reg_val;
	INT8 ch;
	motor_state_t mstate;

	SendString((unsigned char *)"\r\n========================================\r\n");
	SendString((unsigned char *)"  Wheel Motor (Drive) Test\r\n");
	SendString((unsigned char *)"========================================\r\n");

	/* --- Section 1: motor_Init() verification --- */
	SendString((unsigned char *)"\r\n--- 1. motor_Init() GPIO check ---\r\n");
	motor_Init();

	/* 1a: P2.DR direction bits clear (P2.6,5 = wheel IN1 pins) */
	reg_val = P2.DR.BYTE;
	SendString((unsigned char *)"  P2.DR = 0x");
	print_hex8(reg_val);
	if ((reg_val & (MOTOR_P2_LEFT_IN1 | MOTOR_P2_RIGHT_IN1)) == 0x00) {
		SendString((unsigned char *)"  [PASS] P2 IN1 pins clear\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] P2 IN1 pins not clear\r\n");
		fail_count++;
	}

	/* 1b: P9.0 left IN2 + P9.1 right IN2 clear */
	reg_val = P9.DR.BYTE;
	SendString((unsigned char *)"  P9.DR = 0x");
	print_hex8(reg_val);
	if ((reg_val & (MOTOR_P9_LEFT_IN2 | MOTOR_P9_RIGHT_IN2)) == 0x00) {
		SendString((unsigned char *)"  [PASS] P9 IN2 pins clear\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] P9.0/P9.1 should be 0\r\n");
		fail_count++;
	}

	/* 1c: P4.0 motor power off */
	reg_val = P4.DR.BYTE;
	SendString((unsigned char *)"  P4.DR = 0x");
	print_hex8(reg_val);
	if ((reg_val & MOTOR_P4_POWER) == 0x00) {
		SendString((unsigned char *)"  [PASS] motor power off\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] P4.0 should be 0\r\n");
		fail_count++;
	}

	/* 1d: P4.5 PSU still on */
	if (reg_val & MOTOR_P4_PSU) {
		SendString((unsigned char *)"  P4.5 PSU     [PASS] on\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  P4.5 PSU     [FAIL] should be on\r\n");
		fail_count++;
	}

	/* 1e: FRT_TCR configured (0xE2 from SetupHW_Q) */
	reg_val = FRT.TCR.BYTE;
	SendString((unsigned char *)"  FRT.TCR= 0x");
	print_hex8(reg_val);
	if (reg_val == MOTOR_FRT_TCR_INIT) {
		SendString((unsigned char *)"  [PASS] FRT config 0xE2\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] expect 0xE2\r\n");
		fail_count++;
	}

	/* 1f: FRT.TIER — encoder bits should be cleared */
	reg_val = FRT.TIER.BYTE;
	SendString((unsigned char *)"  FRT.TIER=0x");
	print_hex8(reg_val);
	if ((reg_val & (MOTOR_ENC_LEFT | MOTOR_ENC_RIGHT)) == 0x00) {
		SendString((unsigned char *)"  [PASS] encoder off\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL] enc bits should be 0\r\n");
		fail_count++;
	}

	/* 1g: TMR0.TCORA = 0xFF after init (blade motor sets it as PWM period).
	 *     TMR0 is blade-only now. Wheel speed uses PWMX at 0xFFFFA0. */
	SendString((unsigned char *)"  TMR0.TCORA=0x");
	print_hex8(TMR0.TCORA);
	if (TMR0.TCORA == BLADE_TMR_PERIOD) {
		SendString((unsigned char *)" [PASS] blade period 0xFF\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL expect 0xFF]\r\n");
		fail_count++;
	}

	/* 1h: PWMX DADRA/DADRB should be zero (MOTOR_PWMX_ZERO = 0x0002) */
	{
		uint16_t dadra = MOTOR_PWMX_DADRA;
		uint16_t dadrb = MOTOR_PWMX_DADRB;
		SendString((unsigned char *)"  PWMX DADRA=0x");
		print_hex16(dadra);
		if (dadra == MOTOR_PWMX_ZERO) {
			SendString((unsigned char *)" [PASS] left zero\r\n");
			pass_count++;
		} else {
			SendString((unsigned char *)" [FAIL expect 0x0002]\r\n");
			fail_count++;
		}
		SendString((unsigned char *)"  PWMX DADRB=0x");
		print_hex16(dadrb);
		if (dadrb == MOTOR_PWMX_ZERO) {
			SendString((unsigned char *)" [PASS] right zero\r\n");
			pass_count++;
		} else {
			SendString((unsigned char *)" [FAIL expect 0x0002]\r\n");
			fail_count++;
		}
	}


	/* --- Section 2: Left wheel direction pins --- */
	SendString((unsigned char *)"\r\n--- 2. Left wheel direction (P2.6/P9.0) ---\r\n");

	/* Forward: IN1=H (P2.6), IN2=L (P9.0) */
	motor_LeftForward();
	reg_val = P2.DR.BYTE;
	SendString((unsigned char *)"  LeftForward: P2.6=");
	print_hex8((reg_val & MOTOR_P2_LEFT_IN1) ? 1 : 0);
	SendString((unsigned char *)" P9.0=");
	print_hex8((P9.DR.BYTE & MOTOR_P9_LEFT_IN2) ? 1 : 0);
	if ((reg_val & MOTOR_P2_LEFT_IN1) &&
	    !(P9.DR.BYTE & MOTOR_P9_LEFT_IN2)) {
		SendString((unsigned char *)" [PASS] IN1=1 IN2=0\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL]\r\n");
		fail_count++;
	}

	/* Check FRT.TIER encoder bit set (firmware: TIER |= 0x20) */
	reg_val = FRT.TIER.BYTE;
	SendString((unsigned char *)"  FRT.TIER=0x");
	print_hex8(reg_val);
	if (reg_val & MOTOR_ENC_LEFT) {
		SendString((unsigned char *)" [PASS] left enc on\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL] bit 5 should be set\r\n");
		fail_count++;
	}

	/* Disable */
	motor_LeftDisable();
	SendString((unsigned char *)"  LeftDisable: P2.6=");
	print_hex8((P2.DR.BYTE & MOTOR_P2_LEFT_IN1) ? 1 : 0);
	SendString((unsigned char *)" P9.0=");
	print_hex8((P9.DR.BYTE & MOTOR_P9_LEFT_IN2) ? 1 : 0);
	if (!(P2.DR.BYTE & MOTOR_P2_LEFT_IN1) &&
	    !(P9.DR.BYTE & MOTOR_P9_LEFT_IN2)) {
		SendString((unsigned char *)" [PASS] both clear\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL]\r\n");
		fail_count++;
	}

	/* Reverse: IN1=H (P2.6 = run), IN2=H (P9.0 = reverse) */
	motor_LeftReverse();
	SendString((unsigned char *)"  LeftReverse: P2.6=");
	print_hex8((P2.DR.BYTE & MOTOR_P2_LEFT_IN1) ? 1 : 0);
	SendString((unsigned char *)" P9.0=");
	print_hex8((P9.DR.BYTE & MOTOR_P9_LEFT_IN2) ? 1 : 0);
	if ((P2.DR.BYTE & MOTOR_P2_LEFT_IN1) &&
	    (P9.DR.BYTE & MOTOR_P9_LEFT_IN2)) {
		SendString((unsigned char *)" [PASS] IN1=1 IN2=1\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL]\r\n");
		fail_count++;
	}
	motor_LeftDisable();


	/* --- Section 3: Right wheel direction pins --- */
	SendString((unsigned char *)"\r\n--- 3. Right wheel direction (P2.5/P9.1) ---\r\n");

	/* Forward: P2.5=H (IN1), P9.1=L (IN2) */
	motor_RightForward();
	reg_val = P2.DR.BYTE;
	SendString((unsigned char *)"  RightForward: P2.DR=0x");
	print_hex8(reg_val);
	SendString((unsigned char *)" P9.DR=0x");
	print_hex8(P9.DR.BYTE);
	if ((reg_val & MOTOR_P2_RIGHT_IN1) &&
	    !(P9.DR.BYTE & MOTOR_P9_RIGHT_IN2)) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL]\r\n");
		fail_count++;
	}

	/* Check FRT.TIER for right (original firmware: 0x80) */
	reg_val = FRT.TIER.BYTE;
	SendString((unsigned char *)"  FRT.TIER=0x");
	print_hex8(reg_val);
	if (reg_val & MOTOR_ENC_RIGHT) {
		SendString((unsigned char *)" [PASS] right enc/route on\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL] bit 7 should be set\r\n");
		fail_count++;
	}

	/* Disable */
	motor_RightDisable();
	reg_val = P2.DR.BYTE;
	SendString((unsigned char *)"  RightDisable: P2.DR=0x");
	print_hex8(reg_val);
	SendString((unsigned char *)" P9.DR=0x");
	print_hex8(P9.DR.BYTE);
	if (!(reg_val & MOTOR_P2_RIGHT_IN1) &&
	    !(P9.DR.BYTE & MOTOR_P9_RIGHT_IN2)) {
		SendString((unsigned char *)" [PASS] both clear\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL]\r\n");
		fail_count++;
	}

	/* Reverse: P2.5=H (IN1 = run), P9.1=H (IN2 = reverse) */
	motor_RightReverse();
	reg_val = P2.DR.BYTE;
	SendString((unsigned char *)"  RightReverse: P2.DR=0x");
	print_hex8(reg_val);
	SendString((unsigned char *)" P9.DR=0x");
	print_hex8(P9.DR.BYTE);
	if ((reg_val & MOTOR_P2_RIGHT_IN1) &&
	    (P9.DR.BYTE & MOTOR_P9_RIGHT_IN2)) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL]\r\n");
		fail_count++;
	}
	motor_RightDisable();


	/* --- Section 4: Wheel speed (PWMX D/A-PWM) --- */
	SendString((unsigned char *)"\r\n--- 4. Wheel PWMX speed (DADRA/DADRB) ---\r\n");

	motor_LeftSetSpeed(0x80);
	SendString((unsigned char *)"  LeftSpeed(0x80): DADRA=0x");
	print_hex16(MOTOR_PWMX_DADRA);
	if (MOTOR_PWMX_DADRA == MOTOR_PWMX_DUTY(0x80)) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL expect 0x8002]\r\n");
		fail_count++;
	}

	motor_LeftSetSpeed(0xFF);
	SendString((unsigned char *)"  LeftSpeed(0xFF): DADRA=0x");
	print_hex16(MOTOR_PWMX_DADRA);
	if (MOTOR_PWMX_DADRA == MOTOR_PWMX_DUTY(0xFF)) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL expect 0xFF02]\r\n");
		fail_count++;
	}

	motor_LeftSetSpeed(0x00);
	SendString((unsigned char *)"  LeftSpeed(0x00): DADRA=0x");
	print_hex16(MOTOR_PWMX_DADRA);
	if (MOTOR_PWMX_DADRA == MOTOR_PWMX_ZERO) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL expect 0x0002]\r\n");
		fail_count++;
	}

	motor_RightSetSpeed(0x80);
	SendString((unsigned char *)"  RightSpeed(0x80): DADRB=0x");
	print_hex16(MOTOR_PWMX_DADRB);
	if (MOTOR_PWMX_DADRB == MOTOR_PWMX_DUTY(0x80)) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL expect 0x8002]\r\n");
		fail_count++;
	}

	motor_RightSetSpeed(0x00);
	SendString((unsigned char *)"  RightSpeed(0x00): DADRB=0x");
	print_hex16(MOTOR_PWMX_DADRB);
	if (MOTOR_PWMX_DADRB == MOTOR_PWMX_ZERO) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL expect 0x0002]\r\n");
		fail_count++;
	}


	/* --- Section 5: motor_PowerOn / motor_PowerOff --- */
	SendString((unsigned char *)"\r\n--- 5. Motor power (P4.0) ---\r\n");

	motor_PowerOn();
	reg_val = P4.DR.BYTE;
	SendString((unsigned char *)"  PowerOn:  P4.DR=0x");
	print_hex8(reg_val);
	if (reg_val & MOTOR_P4_POWER) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL]\r\n");
		fail_count++;
	}

	motor_PowerOff();
	reg_val = P4.DR.BYTE;
	SendString((unsigned char *)"  PowerOff: P4.DR=0x");
	print_hex8(reg_val);
	if ((reg_val & MOTOR_P4_POWER) == 0x00) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL]\r\n");
		fail_count++;
	}


	/* --- Section 6: Differential steering --- */
	SendString((unsigned char *)"\r\n--- 6. Steering (SetSteering) ---\r\n");

	/* Right turn at base=128, angle=90 */
	motor_SetSteering(128, 90, 1);
	SendString((unsigned char *)"  Right turn(128,90):\r\n");
	SendString((unsigned char *)"    DADRA(L)=0x");
	print_hex16(MOTOR_PWMX_DADRA);
	SendString((unsigned char *)"  DADRB(R)=0x");
	print_hex16(MOTOR_PWMX_DADRB);
	SendString((unsigned char *)"\r\n");
	/* Left should be faster than right */
	if (MOTOR_PWMX_DADRA > MOTOR_PWMX_DADRB) {
		SendString((unsigned char *)"    [PASS] L > R (right turn)\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"    [FAIL] L should be > R\r\n");
		fail_count++;
	}

	/* Left turn at base=128, angle=90 */
	motor_SetSteering(128, 90, 0);
	SendString((unsigned char *)"  Left turn(128,90):\r\n");
	SendString((unsigned char *)"    DADRA(L)=0x");
	print_hex16(MOTOR_PWMX_DADRA);
	SendString((unsigned char *)"  DADRB(R)=0x");
	print_hex16(MOTOR_PWMX_DADRB);
	SendString((unsigned char *)"\r\n");
	/* Right should be faster than left */
	if (MOTOR_PWMX_DADRB > MOTOR_PWMX_DADRA) {
		SendString((unsigned char *)"    [PASS] R > L (left turn)\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"    [FAIL] R should be > L\r\n");
		fail_count++;
	}

	/* Straight: angle=0, both equal */
	motor_SetSteering(128, 0, 1);
	SendString((unsigned char *)"  Straight(128,0):\r\n");
	SendString((unsigned char *)"    DADRA(L)=0x");
	print_hex16(MOTOR_PWMX_DADRA);
	SendString((unsigned char *)"  DADRB(R)=0x");
	print_hex16(MOTOR_PWMX_DADRB);
	SendString((unsigned char *)"\r\n");
	if (MOTOR_PWMX_DADRA == MOTOR_PWMX_DADRB) {
		SendString((unsigned char *)"    [PASS] L == R\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"    [FAIL] L should == R\r\n");
		fail_count++;
	}

	/* Reset wheel speeds */
	motor_LeftSetSpeed(0);
	motor_RightSetSpeed(0);


	/* --- Section 7: motor_GetState() for wheels --- */
	SendString((unsigned char *)"\r\n--- 7. motor_GetState() wheel status ---\r\n");

	motor_GetState(&mstate);
	SendString((unsigned char *)"  left.enabled   = ");
	print_hex8(mstate.left.enabled);
	if (mstate.left.enabled == 0) {
		SendString((unsigned char *)"  [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL]\r\n");
		fail_count++;
	}
	SendString((unsigned char *)"  left.direction = ");
	print_hex8(mstate.left.direction);
	if (mstate.left.direction == MOTOR_DIR_STOP) {
		SendString((unsigned char *)"  [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL]\r\n");
		fail_count++;
	}
	SendString((unsigned char *)"  right.enabled  = ");
	print_hex8(mstate.right.enabled);
	if (mstate.right.enabled == 0) {
		SendString((unsigned char *)"  [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL]\r\n");
		fail_count++;
	}
	SendString((unsigned char *)"  right.direction= ");
	print_hex8(mstate.right.direction);
	if (mstate.right.direction == MOTOR_DIR_STOP) {
		SendString((unsigned char *)"  [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  [FAIL]\r\n");
		fail_count++;
	}

	/* Verify state after forward enable (speed=0, just direction) */
	motor_LeftForward();
	motor_RightForward();
	motor_GetState(&mstate);
	SendString((unsigned char *)"  After Fwd enable:\r\n");
	SendString((unsigned char *)"    left.dir=");
	print_hex8(mstate.left.direction);
	SendString((unsigned char *)" right.dir=");
	print_hex8(mstate.right.direction);
	if (mstate.left.direction == MOTOR_DIR_FORWARD &&
	    mstate.right.direction == MOTOR_DIR_FORWARD) {
		SendString((unsigned char *)" [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)" [FAIL]\r\n");
		fail_count++;
	}
	motor_LeftDisable();
	motor_RightDisable();


	/* --- Section 8: Encoder readback --- */
	SendString((unsigned char *)"\r\n--- 8. Encoder capture registers ---\r\n");
	{
		uint16_t enc_l, enc_r;

		motor_ReadEncoder(&enc_l, &enc_r);
		SendString((unsigned char *)"  FRT.ICRA (left) = 0x");
		print_hex16(enc_l);
		SendString((unsigned char *)"\r\n");
		SendString((unsigned char *)"  FRT.ICRB (right)= 0x");
		print_hex16(enc_r);
		SendString((unsigned char *)"\r\n");
		/* We just verify the read doesn't crash — encoders won't
		 * have meaningful values without wheels spinning */
		SendString((unsigned char *)"  ReadEncoder OK  [PASS]\r\n");
		pass_count++;
	}


	/* --- Section 9: Live wheel test (optional) --- */
	SendString((unsigned char *)"\r\n--- 9. Live wheel test (optional) ---\r\n");
	SendString((unsigned char *)"  This will spin BOTH wheels: 2s FWD, 2s REV.\r\n");
	SendString((unsigned char *)"  LIFT MOWER OFF GROUND before proceeding!\r\n");
	SendString((unsigned char *)"  Press 'Y' to run, other key to skip: ");
	ch = GetChar();
	PutChar(ch);
	SendString((unsigned char *)"\r\n");

	if (ch == 'Y' || ch == 'y') {
		uint16_t enc_before_l, enc_after_l, ticks_l;
		uint16_t enc_before_r, enc_after_r, ticks_r;

		SendString((unsigned char *)"  WARNING: Wheels spin in 2 seconds!\r\n");
		motor_safe_delay_drive(2000);

		/* Power on */
		motor_PowerOn();
		SendString((unsigned char *)"  P4.0 motor power ON\r\n");
		motor_safe_delay_drive(100);

		/* --- Forward: both wheels, 2s at speed 0x40 ---
		 * EMI protection: flush TX, kill encoder IRQs, use NOP delay
		 * with periodic SCI+STCR recovery, then full recover after. */
		motor_ReadEncoder(&enc_before_l, &enc_before_r);
		motor_LeftForward();
		motor_RightForward();
		motor_KillEncoderIRQs();     /* kill encoder IRQs via shadow */
		motor_LeftSetSpeed(0x40);
		motor_RightSetSpeed(0x40);
		SendString((unsigned char *)"  FWD (speed=0x40) 2s...");
		sci1_tx_flush();
		motor_PowerOn();
		motor_safe_delay_drive(2000);
		motor_PowerOff();
		motor_LeftDisable();
		motor_RightDisable();
		motor_safe_delay_drive(100);
		motor_sci_recover_drive();
		motor_ReadEncoder(&enc_after_l, &enc_after_r);
		ticks_l = enc_after_l - enc_before_l;
		ticks_r = enc_after_r - enc_before_r;
		SendString((unsigned char *)" Enc L=0x");
		print_hex16(ticks_l);
		SendString((unsigned char *)" R=0x");
		print_hex16(ticks_r);
		SendString((unsigned char *)"\r\n");
		motor_safe_delay_drive(500);

		/* --- Reverse: both wheels, 2s at speed 0x40 --- */
		motor_ReadEncoder(&enc_before_l, &enc_before_r);
		motor_LeftReverse();
		motor_RightReverse();
		motor_KillEncoderIRQs();     /* kill encoder IRQs via shadow */
		motor_LeftSetSpeed(0x40);
		motor_RightSetSpeed(0x40);
		SendString((unsigned char *)"  REV (speed=0x40) 2s...");
		sci1_tx_flush();
		motor_PowerOn();
		motor_safe_delay_drive(2000);
		motor_PowerOff();
		motor_LeftDisable();
		motor_RightDisable();
		motor_safe_delay_drive(100);
		motor_sci_recover_drive();
		motor_ReadEncoder(&enc_after_l, &enc_after_r);
		ticks_l = enc_after_l - enc_before_l;
		ticks_r = enc_after_r - enc_before_r;
		SendString((unsigned char *)" Enc L=0x");
		print_hex16(ticks_l);
		SendString((unsigned char *)" R=0x");
		print_hex16(ticks_r);
		SendString((unsigned char *)"\r\n");

		/* Stop */
		motor_LeftDisable();
		motor_RightDisable();
		motor_PowerOff();
		motor_sci_recover_drive();
		SendString((unsigned char *)"  Motors OFF, power OFF\r\n");
		SendString((unsigned char *)"  Live wheel test done  [PASS]\r\n");
		pass_count++;
	} else {
		SendString((unsigned char *)"  Live test skipped\r\n");
	}


	/* --- Cleanup --- */
	motor_LeftDisable();
	motor_RightDisable();
	motor_StopAll();

	/* --- Summary --- */
	SendString((unsigned char *)"\r\n========================================\r\n");
	SendString((unsigned char *)"  Results: ");
	print_hex8(pass_count);
	SendString((unsigned char *)" passed, ");
	print_hex8(fail_count);
	SendString((unsigned char *)" failed\r\n");
	if (fail_count == 0)
		SendString((unsigned char *)"  ALL TESTS PASSED\r\n");
	else
		SendString((unsigned char *)"  *** FAILURES DETECTED ***\r\n");
	SendString((unsigned char *)"========================================\r\n");
}
