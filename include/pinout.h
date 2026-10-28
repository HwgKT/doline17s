#ifndef PINOUT_H
#define PINOUT_H

// Line sensor CD74HC4067 and buttons (current firmware wiring).
#define PIN_LINE_SIG       36
#define PIN_MUX_S0         17
#define PIN_MUX_S1         18
#define PIN_MUX_S2         19
#define PIN_MUX_S3         23
#define PIN_CALIB_BUTTON   34
#define PIN_START_BUTTON   35  // Reserved; the current firmware does not use it yet.

// Wheel encoder outputs: L1/L2 and R1/R2.
#define PIN_ENCODER_L1     21
#define PIN_ENCODER_L2     22
#define PIN_ENCODER_R1     32
#define PIN_ENCODER_R2     33

// Motor driver inputs.
#define PIN_MOTOR_L1       25
#define PIN_MOTOR_L2       26
#define PIN_MOTOR_R1       27
#define PIN_MOTOR_R2       13

#define PIN_DEBUG_LED 2
#define DEBUG_LED_ACTIVE_HIGH 1

#endif // PINOUT_H
