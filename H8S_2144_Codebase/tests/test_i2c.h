/* test_i2c.h — I2C bus diagnostic test */
#ifndef TEST_I2C_H
#define TEST_I2C_H

void I2C_Test(void);

/* I2C bus init helpers (also called from main for startup) */
void i2c_bus1_init(void);
void i2c_bus2_init(void);

#endif /* TEST_I2C_H */
