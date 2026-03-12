/*
 * accel.c - Accelerometer/tilt sensor driver (LIS302DL / LIS3DH)
 *
 * Clean-room reimplementation for Ambrogio/Husqvarna L200 (H8S/2144)
 * Derived from Ghidra decompilation documented in STATES/state_i2c.md
 *
 * Both sensors connect via software I2C (bit-banged through function
 * pointer table ptrBladeJumpTable2 in the original firmware).
 *
 * Sensor A: LIS302DL at I2C 0x1D (SDO=1).
 *   Original firmware mislabels this as "MMA7455L" but register map
 *   and WHO_AM_I (0x3B) confirm LIS302DL. 8-bit axis outputs.
 *
 * Sensor B: LIS3DH at I2C 0x19 (SA0=1).
 *   16-bit axis outputs with auto-increment reads.
 *
 * IIR filter: 15/16 old + 1/16 new (fixed-point, accumulators scaled x16)
 */

#include "accel.h"
#include "../i2c/i2c.h"

/* Accelerometer uses I2C Bus 2 (firmware: ptrBladeJumpTable2),
 * separate from the RTC/EEPROM bus. */
#define ACCEL_I2C_BUS  I2C_BUS2

/* ---- Module state ---- */
static uint8_t sensor_type = ACCEL_NONE;

/* IIR filter accumulators (scaled by 16 for fixed-point)
 * Firmware: dwAccelFilterX/Y/Z at 0xFFE9E0-0xFFE9E8 */
static int32_t filter_x = 0;
static int32_t filter_y = 0;
static int32_t filter_z = 0;

/* ---- LIS302DL low-level I2C functions ---- */
/* (Firmware labels these as "mma7455_*" but the chip is an LIS302DL) */

/*
 * Write a single LIS302DL register.
 *
 * Protocol: START -> 0x3A -> reg -> value -> STOP
 *
 * Firmware: mma7455_WriteReg (0x14AFA) -- hardcodes reg 0x20 (CTRL_REG1),
 *           but our version accepts any register address.
 */
static int lis302dl_write_reg(uint8_t reg, uint8_t val)
{
    i2c_Start(ACCEL_I2C_BUS);
    if (!i2c_WriteByte(ACCEL_I2C_BUS, LIS302DL_ADDR_W)) { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    if (!i2c_WriteByte(ACCEL_I2C_BUS, reg))               { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    if (!i2c_WriteByte(ACCEL_I2C_BUS, val))               { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    i2c_Stop(ACCEL_I2C_BUS);
    return 0;
}

/*
 * Read a single LIS302DL register.
 *
 * Protocol: START -> 0x3A -> reg -> rSTART -> 0x3B -> read -> STOP
 *
 * Firmware: mma7455_ReadReg (0x14A1E)
 */
static int lis302dl_read_reg(uint8_t reg, uint8_t *val)
{
    i2c_Start(ACCEL_I2C_BUS);
    if (!i2c_WriteByte(ACCEL_I2C_BUS, LIS302DL_ADDR_W)) { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    if (!i2c_WriteByte(ACCEL_I2C_BUS, reg))               { i2c_Stop(ACCEL_I2C_BUS); return -1; }

    i2c_Start(ACCEL_I2C_BUS);
    if (!i2c_WriteByte(ACCEL_I2C_BUS, LIS302DL_ADDR_R)) { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    *val = i2c_ReadByte(ACCEL_I2C_BUS, 1);    /* NACK (single byte) */
    i2c_Stop(ACCEL_I2C_BUS);
    return 0;
}

/*
 * Read X, Y, Z axes (8-bit signed each) from LIS302DL.
 *
 * Registers: OUT_X=0x29, OUT_Y=0x2B, OUT_Z=0x2D (non-contiguous).
 * Each is a single signed byte (-128..+127), representing ~18mg/LSB
 * at +/-2g full scale.
 *
 * Firmware: mma7455_ReadAxes (0x14A80) reads 0x29, 0x2B, 0x2D
 */
static int lis302dl_read_axes(int8_t *x, int8_t *y, int8_t *z)
{
    uint8_t val;

    if (lis302dl_read_reg(LIS302DL_REG_OUT_X, &val) != 0) return -1;
    *x = (int8_t)val;

    if (lis302dl_read_reg(LIS302DL_REG_OUT_Y, &val) != 0) return -1;
    *y = (int8_t)val;

    if (lis302dl_read_reg(LIS302DL_REG_OUT_Z, &val) != 0) return -1;
    *z = (int8_t)val;

    return 0;
}

/*
 * Probe and initialize LIS302DL.
 *
 * Firmware: mma7455_Probe (0x14B3E) calls mma7455_WriteReg(ctx, 0x47)
 *   which writes 0x47 to register 0x20 (CTRL_REG1).
 *   0x47 = PD active | DR 100Hz | +/-2g | Zen,Yen,Xen
 *
 * This both detects the chip (via ACK) and configures it.
 * Returns 0 if sensor responds with ACK.
 */
static int lis302dl_probe(void)
{
    return lis302dl_write_reg(LIS302DL_REG_CTRL_REG1, LIS302DL_CTRL1_ACTIVE);
}

/* ---- LIS3DH low-level I2C functions ---- */

/* LIS3DH I2C address -- SA0=1 on the L200 board.
 * Firmware: lis3dh_Init(ctx, 0x32) called from Startup.
 * Verified: Startup loads Bus 2 context from ROM 0x1A288 (=0xFFE09C). */

/*
 * Write a single LIS3DH register.
 *
 * Protocol: START -> addr_w -> reg -> value -> STOP
 *
 * Firmware: lis3dh_WriteReg (0x14DE2)
 *           Called 16 times by lis3dh_Init for non-contiguous registers.
 */
static int lis3dh_write_reg(uint8_t reg, uint8_t val)
{
    i2c_Start(ACCEL_I2C_BUS);
    if (!i2c_WriteByte(ACCEL_I2C_BUS, LIS3DH_ADDR_W))  { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    if (!i2c_WriteByte(ACCEL_I2C_BUS, reg))              { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    if (!i2c_WriteByte(ACCEL_I2C_BUS, val))              { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    i2c_Stop(ACCEL_I2C_BUS);
    return 0;
}

/*
 * Read N registers from LIS3DH with auto-increment (OR 0x80 on reg addr).
 *
 * Protocol: START -> addr_w -> (reg|0x80) -> rSTART -> addr_r -> read N -> STOP
 *
 * Firmware: lis3dh_ReadRegs (0x14CAE)
 */
static int lis3dh_read_regs(uint8_t reg, uint8_t *buf, uint8_t count)
{
    uint8_t i;

    i2c_Start(ACCEL_I2C_BUS);
    if (!i2c_WriteByte(ACCEL_I2C_BUS, LIS3DH_ADDR_W))           { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    if (!i2c_WriteByte(ACCEL_I2C_BUS, reg | LIS3DH_AUTO_INCREMENT)) { i2c_Stop(ACCEL_I2C_BUS); return -1; }

    i2c_Start(ACCEL_I2C_BUS);
    if (!i2c_WriteByte(ACCEL_I2C_BUS, LIS3DH_ADDR_R))           { i2c_Stop(ACCEL_I2C_BUS); return -1; }

    for (i = 0; i < count; i++)
        buf[i] = i2c_ReadByte(ACCEL_I2C_BUS, i == count - 1);
    i2c_Stop(ACCEL_I2C_BUS);
    return 0;
}

/*
 * Read raw XYZ from LIS3DH (6 bytes: XL,XH,YL,YH,ZL,ZH -> 3x int16).
 *
 * Firmware: lis3dh_ReadXYZ_Raw (0x14D1A)
 */
static int lis3dh_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];

    if (lis3dh_read_regs(LIS3DH_REG_OUT_X_L, buf, 6) != 0)
        return -1;

    *x = (int16_t)(buf[1] << 8 | buf[0]);
    *y = (int16_t)(buf[3] << 8 | buf[2]);
    *z = (int16_t)(buf[5] << 8 | buf[4]);
    return 0;
}

/*
 * Initialize LIS3DH -- write control and interrupt registers.
 *
 * Firmware: lis3dh_Init (0x14E32) writes 16 registers individually
 * using lis3dh_WriteReg. We use individual writes to match firmware
 * behaviour (required for non-contiguous register addresses).
 *
 * Register values extracted from firmware ROM via disassembly:
 *   Phase 1: CTRL_REG1=0x57, CTRL_REG4=0x80
 *   Phase 2: INT1 config (tilt detection)
 *   Phase 3: Reconfigure CTRL registers + INT2 + CLICK
 */
static int lis3dh_init(void)
{
    /* Phase 1: Core configuration */
    if (lis3dh_write_reg(LIS3DH_REG_CTRL_REG1, 0x57) != 0) return -1;
    /* 0x57 = ODR 100Hz | Normal mode | Z,Y,X enabled */

    if (lis3dh_write_reg(LIS3DH_REG_CTRL_REG4, 0x80) != 0) return -1;
    /* 0x80 = BDU=1 (block data update) | FS=+/-2g | Normal mode */

    /* Phase 2: Interrupt 1 -- tilt/lift detection
     * Values from firmware disassembly (0x14E6E..0x14EA6) */
    if (lis3dh_write_reg(LIS3DH_REG_INT1_CFG,      0x95) != 0) return -1;
    /* 0x95 = AOI=1 | 6D=0 | ZH=1 | ZL=0 | YH=1 | YL=0 | XH=0 | XL=1 */

    if (lis3dh_write_reg(LIS3DH_REG_INT1_THS,      0x64) != 0) return -1;
    /* 0x64 = threshold 100 * 16mg = 1600mg */

    if (lis3dh_write_reg(LIS3DH_REG_INT1_DURATION,  0x7F) != 0) return -1;
    /* 0x7F = duration 127 * 1/ODR = 1.27s at 100Hz */

    /* Phase 3: Reconfigure with interrupt latch */
    if (lis3dh_write_reg(LIS3DH_REG_CTRL_REG5, 0x04) != 0) return -1;
    /* 0x04 = LIR_INT1 latch interrupt on INT1 */

    return 0;
}

/* ---- Common IIR filter ---- */

/*
 * Apply 15/16 + 1/16 IIR low-pass filter to axes.
 *
 * Formula: acc = (acc * 15 + new * 16) / 16
 * This gives ~0.94 weight to old value, ~0.06 to new.
 *
 * Firmware: accel_FilterXYZ (0xBA44)
 */
static void filter_update(int16_t x, int16_t y, int16_t z)
{
    filter_x = (filter_x * 15 + (int32_t)x * 16) >> 4;
    filter_y = (filter_y * 15 + (int32_t)y * 16) >> 4;
    filter_z = (filter_z * 15 + (int32_t)z * 16) >> 4;
}

/* ---- Simple fixed-point atan2 approximation ---- */

/*
 * Approximate atan2(y, x) in degrees (signed, -180 to +180).
 * Uses a simple polynomial approximation for embedded use.
 *
 * Firmware: accel_CalcTiltAngle (0x157D4) uses lookup table at sub_1614C
 * with scaling constant 0x5A82.
 */
static int16_t approx_atan2_deg(int32_t y, int32_t x)
{
    int32_t abs_x, abs_y, angle;
    int16_t result;

    if (x == 0 && y == 0)
        return 0;

    abs_x = x < 0 ? -x : x;
    abs_y = y < 0 ? -y : y;

    /* First-order atan approximation: atan(a) ~ 45*a for |a|<=1 */
    if (abs_x >= abs_y)
        angle = (abs_y * 45) / abs_x;
    else
        angle = 90 - (abs_x * 45) / abs_y;

    /* Quadrant adjustment */
    if (x < 0)  angle = 180 - angle;
    if (y < 0)  angle = -angle;

    result = (int16_t)angle;
    return result;
}

/* ==================================================================
 * Public API
 * ================================================================== */

uint8_t accel_Init(void)
{
    /* Try LIS302DL first -- firmware probes by writing CTRL_REG1.
     * If ACK received, sensor is present and now configured.
     * Firmware: accel_Init (0xBC7A) tries this path first ("mma7455"). */
    if (lis302dl_probe() == 0) {
        sensor_type = ACCEL_LIS302DL;
        return ACCEL_LIS302DL;
    }

    /* Try LIS3DH -- write config registers, check for ACK.
     * Firmware: lis3dh_Init(ctx, 0x32) with SA0=1 address. */
    if (lis3dh_init() == 0) {
        sensor_type = ACCEL_LIS3DH;
        return ACCEL_LIS3DH;
    }

    sensor_type = ACCEL_NONE;
    return ACCEL_NONE;
}

/* ---- Low-level access for tests ---- */

int accel_WriteReg(uint8_t dev_addr_w, uint8_t reg, uint8_t val)
{
    i2c_Start(ACCEL_I2C_BUS);
    if (!i2c_WriteByte(ACCEL_I2C_BUS, dev_addr_w))  { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    if (!i2c_WriteByte(ACCEL_I2C_BUS, reg))           { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    if (!i2c_WriteByte(ACCEL_I2C_BUS, val))           { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    i2c_Stop(ACCEL_I2C_BUS);
    return 0;
}

int accel_ReadReg(uint8_t dev_addr_w, uint8_t dev_addr_r,
                  uint8_t reg, uint8_t *val)
{
    i2c_Start(ACCEL_I2C_BUS);
    if (!i2c_WriteByte(ACCEL_I2C_BUS, dev_addr_w))  { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    if (!i2c_WriteByte(ACCEL_I2C_BUS, reg))           { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    i2c_Start(ACCEL_I2C_BUS);
    if (!i2c_WriteByte(ACCEL_I2C_BUS, dev_addr_r))  { i2c_Stop(ACCEL_I2C_BUS); return -1; }
    *val = i2c_ReadByte(ACCEL_I2C_BUS, 1);
    i2c_Stop(ACCEL_I2C_BUS);
    return 0;
}

int accel_Read(accel_raw_t *raw)
{
    int16_t x, y, z;

    if (sensor_type == ACCEL_LIS302DL) {
        int8_t rx, ry, rz;
        if (lis302dl_read_axes(&rx, &ry, &rz) != 0)
            return -1;
        /* LIS302DL: negate X and Y for mounting orientation
         * Firmware: accel_ReadMMA7455 negates both X and Y */
        x = -(int16_t)rx;
        y = -(int16_t)ry;
        z = (int16_t)rz;
    }
    else if (sensor_type == ACCEL_LIS3DH) {
        int16_t rx, ry, rz;
        if (lis3dh_read_xyz(&rx, &ry, &rz) != 0)
            return -1;
        /* LIS3DH: swap and negate Y,X for mounting */
        x = -ry;
        y = -rx;
        z = rz;
    }
    else {
        return -1;
    }

    if (raw) {
        raw->x = x;
        raw->y = y;
        raw->z = z;
    }

    filter_update(x, y, z);
    return 0;
}

void accel_GetFiltered(accel_filtered_t *f)
{
    f->x = filter_x;
    f->y = filter_y;
    f->z = filter_z;
}

void accel_GetTilt(accel_tilt_t *tilt)
{
    /* Compute tilt angles from filtered data.
     * Standard accelerometer tilt uses Z (gravity) as reference:
     *   tilt1 (pitch) = atan2(X, Z) — tilt around Y-axis
     *   tilt2 (roll)  = atan2(Y, Z) — tilt around X-axis
     * When flat: X≈0, Y≈0, Z≈1g → both ≈ 0°.
     *
     * Firmware: accel_PackVector (0x157C8) + accel_CalcTiltAngle (0x157D4)
     * uses a lookup-table atan2; our polynomial approximation is adequate
     * for tilt detection (not navigation-grade). */
    tilt->tilt1 = approx_atan2_deg(filter_x, filter_z);
    tilt->tilt2 = approx_atan2_deg(filter_y, filter_z);
}

uint8_t accel_GetType(void)
{
    return sensor_type;
}
