#ifndef SENSOR_H
#define SENSOR_H
#include <stdint.h>
#include <stdbool.h>
#include "pinout.h"
#ifdef __cplusplus
extern "C" {
#endif
#define N_SENSORS 16
#define SENSOR_START_IDX 0
#define SENSOR_END_IDX 15
#define ADC_SAMPLES 4
#define MIN_CALIBRATION_DELTA 150
#define BUTTON_DEBOUNCE_MS 50
#define CAL_FRAMES 128
#define CAL_MAX_STDDEV 90
extern uint16_t sensorRaw[N_SENSORS];
extern uint16_t minValues[N_SENSORS]; // WHITE mean ADC
extern uint16_t maxValues[N_SENSORS]; // BLACK mean ADC
extern uint16_t thresholds[N_SENSORS];
extern bool calibrationReady;
extern bool isCalibrating;
typedef enum { CAL_EMPTY, CAL_WHITE_RUNNING, CAL_WHITE_READY, CAL_BLACK_RUNNING, CAL_READY, CAL_FAILED } CalState;
typedef enum { CAL_NONE, CAL_WHITE_STARTED, CAL_WHITE_DONE, CAL_BLACK_STARTED, CAL_SAVED, CAL_ERROR } CalEvent;
void sensor_init(void);
void sensor_mux_select(uint8_t channel);
uint16_t sensor_read_mux(uint8_t channel);
void sensor_read_all(void);
bool sensor_capture_white(void);
bool sensor_capture_black(void);
void sensor_update_calibration_loop(void);
bool sensor_is_calibrating(void);
bool sensor_get_last_calibration_result(void);
CalState sensor_calibration_state(void);
CalEvent sensor_take_calibration_event(void);
void sensor_print_calibration(void);
bool sensor_is_button_pressed(void);
void sensor_handle_calibration_step(void);
uint16_t sensor_get_calibrated(uint8_t index);
bool sensor_is_black(uint8_t index);
uint16_t sensor_get_line_bits(void);
void sensor_print_states(void);
#ifdef __cplusplus
}
#endif
#endif
