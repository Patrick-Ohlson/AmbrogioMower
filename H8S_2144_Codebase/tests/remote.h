/*
 * remote.h — UART Remote Control mode
 *
 * Receives single-byte ASCII commands from a host (PC + PS5 controller)
 * over UART and drives the mower in real-time.
 *
 * Protocol:
 *   Host sends: H=Hello, P=Ping, F=Forward, B=Back,
 *               L=Left, R=Right, S=Stop, K=Blade on,
 *               X=Blade off, Q=Quit
 *   Mower replies: h=hello ack, p=ping ack, k=ok, ?=unknown, !=watchdog
 *
 * Safety:
 *   - Watchdog (3s): no ping → emergency stop all
 *   - Idle (300ms): no movement cmd → pause wheels (blade stays)
 */

#ifndef REMOTE_H
#define REMOTE_H

/*
 * Remote_Run — Enter UART remote control mode.
 *
 * Waits for 'H' handshake from host, then enters command loop.
 * Returns to caller (menu) on 'Q' command.
 *
 * Assumes timer_HW_Init() has been called (for GetSystemCounter).
 * Calls motor_Init() and steer_Init() internally.
 */
void Remote_Run(void);

#endif /* REMOTE_H */
