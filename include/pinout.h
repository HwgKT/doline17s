#ifndef PINOUT_H
#define PINOUT_H

// Pins used by the current firmware. Keep the encoder pins clear of the MUX pins.
#define PIN_LINE_SIG       36
#define PIN_MUX_S0         17
#define PIN_MUX_S1         18
#define PIN_MUX_S2         19
#define PIN_MUX_S3         23
#define PIN_CALIB_BUTTON   34
#define PIN_START_BUTTON   35

// Quadrature encoder channels: left A/B, then right A/B.
#define PIN_ENCODER_L1     21
#define PIN_ENCODER_L2     22
#define PIN_ENCODER_R1     32
#define PIN_ENCODER_R2     33

// Motor driver inputs.
#define PIN_MOTOR_L1       25
#define PIN_MOTOR_L2       26
#define PIN_MOTOR_R1       27
#define PIN_MOTOR_R2       13

#endif // PINOUT_H
