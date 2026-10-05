#ifndef LESO_COEFFS_H
#define LESO_COEFFS_H

#define LESO_STATE_DIM 10U
#define LESO_INPUT_DIM 4U
#define LESO_EXTENDED_DIM 14U
#define LESO_POLY_TERMS 6U

extern const float leso_ad_coeffs[LESO_STATE_DIM * LESO_STATE_DIM][LESO_POLY_TERMS];
extern const float leso_bd_coeffs[LESO_STATE_DIM * LESO_INPUT_DIM][LESO_POLY_TERMS];
extern const float leso_gain[LESO_EXTENDED_DIM][LESO_STATE_DIM];
extern const float leso_fit_center;
extern const float leso_fit_half_range;

#endif // LESO_COEFFS_H
