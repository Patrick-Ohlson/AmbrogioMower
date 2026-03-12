/*
 * demo.c — Demo mode runner
 *
 * Sequentially executes the command table from demo_config.h,
 * printing each step on the 16x2 LCD as it runs.
 *
 * LCD layout per step:
 *   Row 0: command name     (e.g. "Blade ON", "Forward")
 *   Row 1: parameters       (e.g. "Spd:500 5s", "90 deg L")
 *
 * All movement commands block until completion.
 * motor_RecoverBus() is called periodically during waits
 * to protect against EMI register corruption.
 */

#include <stdint.h>
#include <stdio.h>
#include "demo.h"
#include "demo_config.h"
#include "../lcd/lcd.h"
#include "../motor/motor.h"
#include "../steering/steering.h"
#include "../misc/misc.h"
#include "../utils/utils.h"
#include "../sci.h"

/* ---- Ticks-per-second (1 tick = ~10ms) ---- */
#define TICKS_PER_SEC   100

/* ---- Internal: busy-wait while steering is active ----
 *
 * Polls steer_IsMoving() with periodic motor_RecoverBus()
 * to keep STCR/P4 safe during motor EMI.
 * Also blinks LED so the user sees the system is alive.
 */
static void demo_wait_steering(void)
{
    uint16_t blink = 0;

    while (steer_IsMoving()) {
        motor_RecoverBus();
        delay_ms(50);
        if (++blink >= 10) {        /* toggle LED every ~500ms */
            misc_BlinkLed();
            blink = 0;
        }
    }
}

/* ---- Internal: timed delay with recovery + LED blink ----
 *
 * Delays for the given number of seconds while keeping
 * the bus healthy and blinking the LED.
 */
static void demo_safe_delay(uint8_t seconds)
{
    uint16_t i;
    uint16_t loops = (uint16_t)seconds * 20;    /* 20 × 50ms = 1s */

    for (i = 0; i < loops; i++) {
        motor_RecoverBus();
        delay_ms(50);
        if ((i & 0x09) == 0) {     /* blink ~every 500ms */
            misc_BlinkLed();
        }
    }
}

/* ---- Internal: print step number on UART ---- */
static void demo_uart_step(uint8_t step_num, const char *name)
{
    SendString((unsigned char *)"\r\nDEMO [");
    print_hex8(step_num);
    SendString((unsigned char *)"]: ");
    SendString((unsigned char *)name);
}

/* ================================================================
 * Demo_Run — Execute the full demo sequence
 * ================================================================ */

void Demo_Run(void)
{
    const demo_step_t *step;
    uint8_t  step_num;
    uint16_t ticks;

    /* ---- Banner ---- */
    lcd_Clear();
    lcd_PrintCenter(0, "=== DEMO ===");
    lcd_PrintCenter(1, "Starting...");
    SendString((unsigned char *)"\r\n\r\n*** DEMO MODE ***\r\n");
    demo_safe_delay(2);

    /* ---- Init motor + steering (safe to re-init) ---- */
    motor_Init();
    steer_Init((const steer_motor_ops_t *)0,
               (const steer_config_t *)0,
               (steer_done_cb_t)0);

    /* ---- Walk the command sequence ---- */
    for (step_num = 0, step = demo_sequence;
         step->cmd != DEMO_CMD_END;
         step++, step_num++)
    {
        switch (step->cmd) {

        /* ---- Blade ON ---- */
        case DEMO_CMD_BLADE_ON:
            lcd_Print(0, "Blade ON");
            lcd_Print(1, "Spd:%u  %us", step->param, step->seconds);
            demo_uart_step(step_num, "Blade ON");
            motor_BladeStart(step->param);
            demo_safe_delay(step->seconds);
            break;

        /* ---- Blade OFF ---- */
        case DEMO_CMD_BLADE_OFF:
            lcd_Print(0, "Blade OFF");
            lcd_Print(1, "                ");
            demo_uart_step(step_num, "Blade OFF");
            motor_BladeStop();
            if (step->seconds > 0)
                demo_safe_delay(step->seconds);
            break;

        /* ---- Forward ---- */
        case DEMO_CMD_FORWARD:
            ticks = (uint16_t)step->seconds * TICKS_PER_SEC;
            lcd_Print(0, "Forward");
            lcd_Print(1, "Spd:%u  %us",
                      step->param ? step->param : 0x80, step->seconds);
            demo_uart_step(step_num, "Forward");
            steer_Forward((uint8_t)step->param, ticks);
            demo_wait_steering();
            break;

        /* ---- Reverse ---- */
        case DEMO_CMD_REVERSE:
            ticks = (uint16_t)step->seconds * TICKS_PER_SEC;
            lcd_Print(0, "Reverse");
            lcd_Print(1, "Spd:%u  %us",
                      step->param ? step->param : 0x60, step->seconds);
            demo_uart_step(step_num, "Reverse");
            steer_Reverse((uint8_t)step->param, ticks);
            demo_wait_steering();
            break;

        /* ---- Turn Left ---- */
        case DEMO_CMD_TURN_LEFT:
            if (step->seconds > 0)
                ticks = (uint16_t)step->seconds * TICKS_PER_SEC;
            else
                ticks = steer_EstimateTurnTicks(step->param);
            lcd_Print(0, "Turn Left");
            lcd_Print(1, "%u deg", step->param);
            demo_uart_step(step_num, "Turn Left");
            steer_TurnLeft(0, ticks);
            demo_wait_steering();
            break;

        /* ---- Turn Right ---- */
        case DEMO_CMD_TURN_RIGHT:
            if (step->seconds > 0)
                ticks = (uint16_t)step->seconds * TICKS_PER_SEC;
            else
                ticks = steer_EstimateTurnTicks(step->param);
            lcd_Print(0, "Turn Right");
            lcd_Print(1, "%u deg", step->param);
            demo_uart_step(step_num, "Turn Right");
            steer_TurnRight(0, ticks);
            demo_wait_steering();
            break;

        /* ---- Stop (with optional pause) ---- */
        case DEMO_CMD_STOP:
            lcd_Print(0, "Stop");
            lcd_Print(1, step->seconds > 0 ? "Pause %us" : "                ",
                      step->seconds);
            demo_uart_step(step_num, "Stop");
            steer_Stop();
            motor_PowerOff();
            if (step->seconds > 0)
                demo_safe_delay(step->seconds);
            break;

        /* ---- Delay (no motor change) ---- */
        case DEMO_CMD_DELAY:
            lcd_Print(0, "Wait");
            lcd_Print(1, "%us", step->seconds);
            demo_uart_step(step_num, "Delay");
            if (step->seconds > 0)
                demo_safe_delay(step->seconds);
            break;

        /* ---- Shutoff ---- */
        case DEMO_CMD_SHUTOFF:
            lcd_Print(0, "Shutoff");
            lcd_Print(1, "Demo complete");
            demo_uart_step(step_num, "Shutoff");
            steer_Stop();
            motor_BladeStop();
            motor_StopAll();
            demo_safe_delay(2);
            misc_BacklightOff();
            SendString((unsigned char *)"\r\n*** DEMO COMPLETE ***\r\n");
            return;     /* Exit demo — return to caller */

        default:
            /* Unknown command — skip */
            break;
        }
    }

    /* Reached DEMO_CMD_END without explicit shutoff */
    lcd_Print(0, "Demo complete");
    lcd_Print(1, "                ");
    steer_Stop();
    motor_StopAll();
    SendString((unsigned char *)"\r\n*** DEMO COMPLETE ***\r\n");
}
