#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configure both quadrature encoders and start counting their edges.
void encoder_init(void);

// Read signed quadrature counts. Each valid A/B edge transition counts as one tick.
// Either output pointer may be NULL.
void encoder_get_counts(int32_t* left, int32_t* right);

// Clear both counters and synchronize the decoder to the current pin levels.
void encoder_reset_counts(void);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H
