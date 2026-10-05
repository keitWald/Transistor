// LQR Gain Matrix Coefficients for Embedded Systems
// Source: MATLAB polynomial fit coefficients (40x6).

#ifndef LQR_COEFFS_H
#define LQR_COEFFS_H

#include <stdint.h>

typedef enum { GAINS_NORMAL, GAINS_OFF_GROUND } GainType_e;

#define K_ROWS 40
#define K_COEFFS 6

extern const float K_coeffs_normal[K_ROWS][K_COEFFS];
extern const float K_coeffs_off_ground[K_ROWS][K_COEFFS];

#endif // LQR_COEFFS_H
