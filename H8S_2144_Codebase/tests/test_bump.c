/* test_bump.c -- Bump / lift sensor test
 *
 * Tests: P6 GPIO, debounce, trigger, motor dir, detect,
 *        speed map, post-state, state snapshot, live LCD (20s)
 *
 * Hardware: P6.0 = bump/lift sensor (active high)
 */
#include "test_bump.h"
#include "../iodefine.h"
#include "../sci.h"
#include "../bump/bump.h"
#include "../lcd/lcd.h"
#include "../timer/timer.h"
#include "../utils/utils.h"

static void pstr(const char *s) { SendString((unsigned char *)s); }
static void pnl(void) { pstr("\r\n"); }

void Bump_Test(void)
{
    uint8_t pc = 0, fc = 0;

    pstr("\r\n=== Bump Sensor Test ===\r\n");
    bump_Init();

    /* 1. P6 GPIO */
    pstr("\r\n- 1. P6 GPIO -\r\n  DR=0x");
    print_hex8(P6.DR.BYTE);
    pstr(" bit0="); PutChar('0'+(P6.DR.BYTE & 0x01));
    pnl(); pc++;

    /* 2. Debounce */
    pstr("\r\n- 2. Debounce -\r\n");
    {
        bump_state_t s; uint8_t i; uint32_t t=0;
        bump_Init();
        for (i=0;i<10;i++) { t+=0x11; bump_Poll(t); }
        bump_GetState(&s);
        pstr("  cnt="); print_hex32(s.debounce_count); pnl();
        pc++;
    }

    /* 3. Trigger */
    pstr("\r\n- 3. Trigger -\r\n");
    {
        uint8_t tr=bump_IsTriggered();
        bump_state_t s; bump_GetState(&s);
        pstr("  trig="); PutChar('0'+tr);
        pstr(" cnt="); print_hex32(s.debounce_count); pnl();
        if ((s.debounce_count>=4)==(tr!=0)) { pstr("  [PASS]\r\n"); pc++; }
        else { pstr("  [FAIL]\r\n"); fc++; }
    }

    /* 4. Motor dir */
    pstr("\r\n- 4. Motor dir -\r\n");
    {
        bump_state_t s; uint8_t ok=1;
        bump_SetMotorDir(1); bump_GetState(&s);
        pstr("  L:"); print_hex16(s.motor_dir);
        if (s.motor_dir!=1) ok=0;
        bump_SetMotorDir(2); bump_GetState(&s);
        pstr(" R:"); print_hex16(s.motor_dir);
        if (s.motor_dir!=2) ok=0;
        bump_SetMotorDir(0); bump_GetState(&s);
        pstr(" N:"); print_hex16(s.motor_dir); pnl();
        if (s.motor_dir!=0) ok=0;
        if (ok) { pstr("  [PASS]\r\n"); pc++; }
        else { pstr("  [FAIL]\r\n"); fc++; }
    }

    /* 5. Detect */
    pstr("\r\n- 5. Detect -\r\n");
    {
        uint8_t r; bump_state_t s;
        bump_Init(); r=bump_Detect(); bump_GetState(&s);
        pstr("  code=0x"); print_hex8(r);
        pstr(" act="); print_hex16(s.active); pnl();
        if (r==0||r==1) { pstr("  [PASS]\r\n"); pc++; }
        else { pstr("  [FAIL]\r\n"); fc++; }
    }

    /* 6. Speed map */
    pstr("\r\n- 6. Speed -\r\n");
    {
        uint16_t sf,sr,st,sn; uint8_t ok=1;
        sf=bump_GetReverseSpeed(0x0A);
        sr=bump_GetReverseSpeed(0x0B);
        st=bump_GetReverseSpeed(0x01);
        sn=bump_GetReverseSpeed(0x00);
        pstr("  F=0x"); print_hex16(sf);
        pstr(" R=0x"); print_hex16(sr);
        pstr(" T=0x"); print_hex16(st);
        pstr(" N=0x"); print_hex16(sn); pnl();
        if (sf!=0x01D5||sr!=0x02C0||st!=0x00EA||sn!=0) ok=0;
        if (ok) { pstr("  [PASS]\r\n"); pc++; }
        else { pstr("  [FAIL]\r\n"); fc++; }
    }

    /* 7. Post-state */
    pstr("\r\n- 7. Post -\r\n");
    {
        bump_state_t s; uint8_t ok=1;
        bump_SetPostState(1); bump_GetState(&s);
        pstr("  fwd:"); PutChar('0'+s.post_state);
        if (s.post_state!=2) ok=0;
        bump_SetPostState(0); bump_GetState(&s);
        pstr(" rev:"); PutChar('0'+s.post_state); pnl();
        if (s.post_state!=3) ok=0;
        if (ok) { pstr("  [PASS]\r\n"); pc++; }
        else { pstr("  [FAIL]\r\n"); fc++; }
    }

    /* 8. State snapshot */
    pstr("\r\n- 8. State -\r\n");
    {
        bump_state_t s; bump_GetState(&s);
        pstr("  deb="); print_hex32(s.debounce_count);
        pstr(" dir="); print_hex16(s.motor_dir);
        pstr(" act="); print_hex16(s.active); pnl();
        pstr("  con="); PutChar('0'+s.contact);
        pstr(" pst="); PutChar('0'+s.post_state); pnl();
        pc++;
    }

    /* 9. Live LCD (20s) */
    pstr("\r\n- 9. Live LCD (20s) -\r\n");
    {
        uint32_t st=GetSystemCounter(), lu=0;
        lcd_Clear();
        while (1) {
            uint32_t n=GetSystemCounter();
            if ((n-st)>=2000) break;
            bump_Poll(n);
            if ((n-lu)>=25) {
                lu=n;
                {
                    uint8_t raw=P6.DR.BYTE&0x01;
                    uint8_t tr=bump_IsTriggered();
                    uint8_t det=bump_Detect();
                    bump_state_t s; bump_GetState(&s);
                    lcd_Print(0,"P6:%d D:%3lu T:%d",
                        raw,(unsigned long)s.debounce_count,tr);
                    lcd_Print(1,"Det:0x%02X %2us",
                        det,(uint16_t)((2000-(n-st))/100));
                }
            }
        }
        lcd_Clear();
        pstr("  Done.\r\n");
    }

    /* Summary */
    pstr("\r\n=== Results: ");
    print_hex8(pc); pstr(" pass, ");
    print_hex8(fc); pstr(" fail");
    if (fc==0) pstr(" ALL PASSED"); else pstr(" *** FAIL ***");
    pstr(" ===\r\n");
}
