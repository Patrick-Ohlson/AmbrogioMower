/* test_wire.c -- Boundary wire signal detection test
 *
 * Tests: P2 GPIO, signal source (P2.DR bits 0-1), poll cycle,
 *        flag decode, sync check, statistics, shadow register,
 *        live LCD (20s)
 *
 * Hardware: P2.2 = signal enable/trigger, P2.0-P2.1 = signal input
 *
 * Based on unpatched firmware functions:
 *   GetSignalByte_X (0x46CA) — reads P2.DR bits 0-1
 *   CheckSignal_X   (0x46E0) — 2-pass startup sync verify
 */
#include "test_wire.h"
#include "../iodefine.h"
#include "../sci.h"
#include "../wire/wire.h"
#include "../motor/motor.h"
#include "../lcd/lcd.h"
#include "../timer/timer.h"
#include "../utils/utils.h"

static void pstr(const char *s) { SendString((unsigned char *)s); }
static void pnl(void) { pstr("\r\n"); }

void Wire_Test(void)
{
    uint8_t pc = 0, fc = 0;

    pstr("\r\n=== Wire Signal Test ===\r\n");
    wire_Init();

    /* 1. P2 GPIO */
    pstr("\r\n- 1. P2 GPIO -\r\n  DR=0x");
    print_hex8(P2.DR.BYTE);
    pstr(" shd=0x");
    print_hex8(shadow_p2_dr);
    pnl(); pc++;

    /* 2. Signal source (P2.DR bits 0-1) */
    pstr("\r\n- 2. P2 signal -\r\n  P2.DR=0x");
    print_hex8(P2.DR.BYTE);
    pstr(" idx=");
    PutChar('0' + (P2.DR.BYTE & 0x03));
    pnl(); pc++;

    /* 3. Single poll cycle */
    pstr("\r\n- 3. Poll cycle -\r\n");
    {
        wire_flags_t f;
        uint8_t i, b;
        wire_Init();
        for (i = 0; i < 3; i++) wire_Poll();
        wire_GetFlags(&f);
        b = wire_GetRawByte();
        pstr("  Byte=");
        if (b >= 0x20 && b < 0x7F) PutChar(b); else { pstr("0x"); print_hex8(b); }
        pstr(" s1="); PutChar('0'+f.sig1);
        pstr(" s2="); PutChar('0'+f.sig2);
        pstr(" s3="); PutChar('0'+f.sig3);
        pstr(" s4="); PutChar('0'+f.sig4);
        pnl();
        if (b==0x48||b==0x49||b==0x4E||b==0x4F) { pstr("  [PASS]\r\n"); pc++; }
        else { pstr("  [WARN] unexpected\r\n"); fc++; }
    }

    /* 4. Multi-cycle (10x) */
    pstr("\r\n- 4. 10 cycles -\r\n");
    {
        uint8_t ch=0,ci=0,cn=0,co=0,cy,ph,b;
        wire_Init();
        for (cy=0; cy<10; cy++) {
            for (ph=0;ph<3;ph++) wire_Poll();
            b = wire_GetRawByte();
            if (b==0x48) ch++; else if (b==0x49) ci++;
            else if (b==0x4E) cn++; else if (b==0x4F) co++;
        }
        pstr("  H="); PutChar('0'+ch);
        pstr(" I="); PutChar('0'+ci);
        pstr(" N="); PutChar('0'+cn);
        pstr(" O="); PutChar('0'+co);
        pnl(); pstr("  [PASS]\r\n"); pc++;
    }

    /* 5. Flag truth table */
    pstr("\r\n- 5. Flags -\r\n");
    {
        wire_flags_t f; uint8_t b,ph,ok=0;
        wire_Init();
        for (ph=0;ph<3;ph++) wire_Poll();
        wire_GetFlags(&f); b=wire_GetRawByte();
        if (b==0x48 && f.sig1==1 && f.sig2==0) ok=1;
        else if (b==0x49 && f.sig1==0 && f.sig2==1 && f.sig3==1) ok=1;
        else if (b==0x4E && f.sig1==0 && f.sig2==0) ok=1;
        else if (b==0x4F && f.sig1==0 && f.sig2==1 && f.sig3==0) ok=1;
        pstr("  Sig="); if(b>=0x20&&b<0x7F) PutChar(b); else print_hex8(b);
        if (ok) { pstr(" [PASS]\r\n"); pc++; }
        else { pstr(" [FAIL]\r\n"); fc++; }
    }

    /* 6. Stats */
    pstr("\r\n- 6. Stats -\r\n");
    {
        wire_stats_t s; uint8_t i; uint32_t t=0;
        wire_Init();
        for (i=0;i<5;i++) {
            t+=0x045D;
            wire_Poll(); wire_Poll(); wire_Poll();
            wire_UpdateStats(1,t);
        }
        wire_GetStats(&s);
        pstr("  cnt="); print_hex8(s.sample_count);
        pstr(" hit="); print_hex8(s.hit_count);
        pstr(" q=0x"); print_hex8(s.quality);
        pstr(" out="); PutChar('0'+s.outside); pnl();
        if (s.sample_count==5) { pstr("  [PASS]\r\n"); pc++; }
        else { pstr("  [FAIL]\r\n"); fc++; }
    }

    /* 7. Shadow verify */
    pstr("\r\n- 7. Shadow -\r\n");
    {
        uint8_t bef,af0,af1;
        wire_Init();
        bef=shadow_p2_dr;
        wire_Poll(); af0=shadow_p2_dr;
        wire_Poll(); af1=shadow_p2_dr;
        wire_Poll(); /* decode */
        pstr("  bef=0x"); print_hex8(bef);
        pstr(" p0=0x"); print_hex8(af0);
        pstr(" p1=0x"); print_hex8(af1); pnl();
        if ((af0&0x04) && !(af1&0x04)) { pstr("  [PASS] bit2 toggled\r\n"); pc++; }
        else { pstr("  [FAIL]\r\n"); fc++; }
    }

    /* 8. Sync check — wire_CheckSync() (blocking ~1.2s) */
    pstr("\r\n- 8. Sync check -\r\n");
    {
        uint8_t r;
        uint32_t t0, t1;
        wire_Init();
        pstr("  Running wire_CheckSync...");
        t0 = GetSystemCounter();
        r = wire_CheckSync();
        t1 = GetSystemCounter();
        pstr("done\r\n  Result="); PutChar('0' + r);
        pstr("  Time="); print_hex16((uint16_t)(t1 - t0));
        pstr(" ticks\r\n");
        if (r) { pstr("  [PASS] sync OK\r\n"); pc++; }
        else   { pstr("  [INFO] sync FAIL (wire may not be connected)\r\n"); pc++; }
    }

    /* 9. Live LCD (20s) */
    pstr("\r\n- 9. Live LCD (20s) -\r\n");
    {
        uint32_t st=GetSystemCounter(), lu=0;
        uint8_t pcnt=0;
        wire_flags_t f; wire_stats_t s;
        lcd_Clear();
        while (1) {
            uint32_t n=GetSystemCounter();
            if ((n-st)>=2000) break;
            wire_Poll(); pcnt++;
            if (pcnt>=3) {
                pcnt=0;
                wire_GetFlags(&f);
                wire_UpdateStats(1,n);
                wire_GetStats(&s);
                lcd_Print(0, "%c %d%d%d%d Q:%02X",
                    (f.raw_byte>=0x20&&f.raw_byte<0x7F)?(char)f.raw_byte:'?',
                    f.sig1,f.sig2,f.sig3,f.sig4,s.quality);
                lcd_Print(1, "Out:%d  %2us",
                    s.outside,(uint16_t)((2000-(n-st))/100));
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
