#ifndef LESO_H
#define LESO_H

#include "leso_coeffs.h"
#include <stdint.h>

#ifdef __cplusplus

class Leso {
public:
  Leso();

  void Reset();
  void Reset(const float measurement[LESO_STATE_DIM]);
  bool Step(const float measurement[LESO_STATE_DIM],
            const float applied_input[LESO_INPUT_DIM], float left_leg_length,
            float right_leg_length);

  bool IsInitialized() const { return initialized_; }
  float GetWheelCommonDisturbance() const;
  float GetHipCommonDisturbance() const;
  uint32_t GetResetCount() const { return reset_count_; }

private:
  static float EvaluatePolynomial(
      const float coefficients[LESO_POLY_TERMS], const float features[6]);
  static float Clamp(float value, float limit);
  static bool VectorIsFinite(const float *values, uint32_t count);

  float state_hat_[LESO_EXTENDED_DIM];
  bool initialized_;
  uint32_t reset_count_;
};

#endif // __cplusplus
#endif // LESO_H
