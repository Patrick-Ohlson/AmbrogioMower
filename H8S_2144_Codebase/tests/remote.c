/*
 * remote.c — UART Remote Control mode
 *
 * Single-byte ASCII command protocol for real-time mower control
 * from a host PC (with PS5 controller or terminal).
 *
 * Two safety layers:
 *   1. Watchdog (3s): no valid command → emergency stop everything
 *   2. Movement idle (300ms): no movement cmd → pause wheels (blade stays)
 *
 * Host→Mower: H P F B L R S K X Q  (single ASCII bytes)
 * Mower→Host: h p k ? !            (single ASCII byte replies)
 */

#include <stdint.h>
#include "remote.h"
#include "../lcd/lcd.h"
#include "../motor/motor.h"
#include "../steering/steering.h"
#include "../misc/misc.h"
#include "../utils/utils.h"
#include "../sci.h"

/* ---- Timing constants (1 tick = ~10ms) ---- */
#define TICKS_PER_SEC       100
#define WATCHDOG_TICKS      (3 * TICKS_PER_SEC)     /* 3 seconds */
#define IDLE_TICKS          30                       /* 300 ms    */
#define POLL_MS             20                       /* main loop period */
#define BLINK_DIVISOR       25                       /* blink every ~500ms (25 × 20ms) */

/* ---- Default blade speed (0-1000 scale) ---- */
#define BLADE_DEFAULT_SPEED 500

/* ---- Forward declarations (from main.c) ---- */
extern unsigned char getch2(void);
extern unsigned char keyhit(void);

/* ---- Forward declarations ---- */
static void demo_safe_wait(uint8_t seconds);

/* ---- Internal: send single response byte ---- */
static void reply(unsigned char c)
{
    PutChar(c);
}

/* ---- Internal: emergency stop everything ---- */
static void emergency_stop(void)
{
    steer_Stop();
    motor_BladeStop();
    motor_StopAll();
}

/* ================================================================
 * Remote_Run — UART remote control entry point
 * ================================================================ */

void Remote_Run(void)
{
    uint32_t now;
    uint32_t last_ping;
    uint32_t last_move;
    uint8_t  moving;
    uint8_t  blade_on;
    uint8_t  blink_cnt;
    uint8_t  cmd;

    /* ---- Init hardware ---- */
    motor_Init();
    steer_Init((const steer_motor_ops_t *)0,
               (const steer_config_t *)0,
               (steer_done_cb_t)0);

    SendString((unsigned char *)"\r\n*** REMOTE CONTROL MODE ***\r\n");
    SendString((unsigned char *)"Waiting for host 'H'...\r\n");

    /* ============================================================
     * Phase 1: Wait for Hello handshake
     * ============================================================ */
wait_hello:
    lcd_Clear();
    lcd_Print(0, "Remote Mode");
    lcd_Print(1, "Waiting...");

    blink_cnt = 0;
    while (1) {
        if (keyhit()) {
            cmd = getch2();
            if (cmd == 'H' || cmd == 'h') {
                reply('h');
                break;      /* → Phase 2 */
            }
            if (cmd == 'Q' || cmd == 'q') {
                reply('k');
                SendString((unsigned char *)"REMOTE: Quit\r\n");
                lcd_Print(0, "Remote Exit");
                lcd_Print(1, "                ");
                return;     /* → back to menu */
            }
            reply('?');
        }

        motor_RecoverBus();
        delay_ms(50);
        if (++blink_cnt >= 10) {
            misc_BlinkLed();
            blink_cnt = 0;
        }
    }

    /* ============================================================
     * Phase 2: Command loop
     * ============================================================ */
    lcd_Clear();
    lcd_Print(0, "Remote: Ready");
    lcd_Print(1, "Connected");
    SendString((unsigned char *)"REMOTE: Connected\r\n");

    last_ping = GetSystemCounter();
    last_move = GetSystemCounter();
    moving    = 0;
    blade_on  = 0;
    blink_cnt = 0;

    while (1) {
        now = GetSystemCounter();

        /* ---- Watchdog: 3 seconds no valid command ---- */
        if ((now - last_ping) > WATCHDOG_TICKS) {
            emergency_stop();
            reply('!');
            lcd_Print(0, "!! WATCHDOG !!");
            lcd_Print(1, "Stopped");
            SendString((unsigned char *)"REMOTE: Watchdog timeout!\r\n");
            moving   = 0;
            blade_on = 0;
            demo_safe_wait(2);
            goto wait_hello;    /* require fresh handshake */
        }

        /* ---- Movement idle: 300ms no movement command ---- */
        if (moving && (now - last_move) > IDLE_TICKS) {
            steer_Stop();
            moving = 0;
            lcd_Print(1, "Paused");
        }

        /* ---- Poll UART for commands ---- */
        if (keyhit()) {
            cmd = getch2();
            last_ping = now;    /* any valid byte resets watchdog */

            switch (cmd) {

            /* Ping keepalive */
            case 'P':
            case 'p':
                reply('p');
                break;

            /* Movement commands — continuous (ticks=0) */
            case 'F':
            case 'f':
                steer_Forward(0, 0);
                moving = 1;
                last_move = now;
                lcd_Print(1, "Forward");
                reply('k');
                break;

            case 'B':
            case 'b':
                steer_Reverse(0, 0);
                moving = 1;
                last_move = now;
                lcd_Print(1, "Reverse");
                reply('k');
                break;

            case 'L':
            case 'l':
                steer_TurnLeft(0, 0);
                moving = 1;
                last_move = now;
                lcd_Print(1, "Turn Left");
                reply('k');
                break;

            case 'R':
            case 'r':
                steer_TurnRight(0, 0);
                moving = 1;
                last_move = now;
                lcd_Print(1, "Turn Right");
                reply('k');
                break;

            /* Stop wheels (blade unchanged) */
            case 'S':
            case 's':
                steer_Stop();
                moving = 0;
                lcd_Print(1, "Stop");
                reply('k');
                break;

            /* Blade on */
            case 'K':
            case 'k':
                motor_BladeStart(BLADE_DEFAULT_SPEED);
                blade_on = 1;
                lcd_Print(0, "Remote: Blade");
                reply('k');
                break;

            /* Blade off */
            case 'X':
            case 'x':
                motor_BladeStop();
                blade_on = 0;
                lcd_Print(0, "Remote: Ready");
                reply('k');
                break;

            /* Quit — clean shutdown */
            case 'Q':
            case 'q':
                emergency_stop();
                reply('k');
                lcd_Print(0, "Remote Exit");
                lcd_Print(1, "                ");
                SendString((unsigned char *)"REMOTE: Quit\r\n");
                return;

            default:
                reply('?');
                break;
            }
        }

        /* ---- Housekeeping ---- */
        motor_RecoverBus();
        delay_ms(POLL_MS);
        if (++blink_cnt >= BLINK_DIVISOR) {
            misc_BlinkLed();
            blink_cnt = 0;
        }
    }
}

/* ---- Internal: safe delay with bus recovery ---- */
static void demo_safe_wait(uint8_t seconds)
{
    uint16_t i;
    uint16_t loops = (uint16_t)seconds * (1000 / 50);

    for (i = 0; i < loops; i++) {
        motor_RecoverBus();
        delay_ms(50);
    }
}
