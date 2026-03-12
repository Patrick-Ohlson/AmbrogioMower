/*
 * demo.h — Demo mode runner
 *
 * Executes the command sequence defined in demo_config.h,
 * printing each step on the LCD as it runs.
 */

#ifndef DEMO_H
#define DEMO_H

/*
 * Demo_Run — Execute the full demo sequence.
 *
 * Iterates through demo_sequence[] from demo_config.h:
 *   - Prints the current command on LCD row 0
 *   - Prints parameters/progress on LCD row 1
 *   - Executes the command (blade, steering, delay, shutoff)
 *   - Blocks until each step completes before advancing
 *
 * Assumes motor_Init(), steer_Init(), and timer_HW_Init()
 * have already been called.
 *
 * Returns when the sequence reaches DEMO_CMD_END or DEMO_CMD_SHUTOFF.
 */
void Demo_Run(void);

#endif /* DEMO_H */
