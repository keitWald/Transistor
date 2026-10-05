#include "leso.h"

#include <math.h>

namespace {

const float kMinimumLegLength = 0.23f;
const float kMaximumLegLength = 0.43f;
const float kWheelDisturbanceStateLimit = 20.0f;
const float kHipDisturbanceStateLimit = 60.0f;

float ClampRange(float value, float minimum, float maximum) {
  if (value > maximum) {
    return maximum;
  }
  if (value < minimum) {
    return minimum;
  }
  return value;
}

} // namespace

Leso::Leso() : initialized_(false), reset_count_(0U) {
  for (uint32_t i = 0U; i < LESO_EXTENDED_DIM; ++i) {
    state_hat_[i] = 0.0f;
  }
}

void Leso::Reset() {
  for (uint32_t i = 0U; i < LESO_EXTENDED_DIM; ++i) {
    state_hat_[i] = 0.0f;
  }
  initialized_ = false;
  ++reset_count_;
}

void Leso::Reset(const float measurement[LESO_STATE_DIM]) {
  if (measurement == 0 || !VectorIsFinite(measurement, LESO_STATE_DIM)) {
    Reset();
    return;
  }
  for (uint32_t i = 0U; i < LESO_STATE_DIM; ++i) {
    state_hat_[i] = measurement[i];
  }
  for (uint32_t i = LESO_STATE_DIM; i < LESO_EXTENDED_DIM; ++i) {
    state_hat_[i] = 0.0f;
  }
  initialized_ = true;
  ++reset_count_;
}

bool Leso::Step(const float measurement[LESO_STATE_DIM],
                const float applied_input[LESO_INPUT_DIM],
                float left_leg_length, float right_leg_length) {
  if (measurement == 0 || applied_input == 0 ||
      !VectorIsFinite(measurement, LESO_STATE_DIM) ||
      !VectorIsFinite(applied_input, LESO_INPUT_DIM) ||
      !isfinite(left_leg_length) || !isfinite(right_leg_length)) {
    Reset();
    return false;
  }

  if (!initialized_) {
    Reset(measurement);
    return initialized_;
  }

  const float left_length =
      ClampRange(left_leg_length, kMinimumLegLength, kMaximumLegLength);
  const float right_length =
      ClampRange(right_leg_length, kMinimumLegLength, kMaximumLegLength);
  const float normalized_left =
      (left_length - leso_fit_center) / leso_fit_half_range;
  const float normalized_right =
      (right_length - leso_fit_center) / leso_fit_half_range;
  const float features[6] = {
      1.0f,
      normalized_left,
      normalized_right,
      normalized_left * normalized_left,
      normalized_right * normalized_right,
      normalized_left * normalized_right,
  };

  float innovation[LESO_STATE_DIM];
  for (uint32_t i = 0U; i < LESO_STATE_DIM; ++i) {
    innovation[i] = measurement[i] - state_hat_[i];
  }

  float next_state[LESO_EXTENDED_DIM];
  for (uint32_t row = 0U; row < LESO_STATE_DIM; ++row) {
    float value = 0.0f;
    for (uint32_t col = 0U; col < LESO_STATE_DIM; ++col) {
      const uint32_t coefficient_row = row * LESO_STATE_DIM + col;
      value += EvaluatePolynomial(leso_ad_coeffs[coefficient_row], features) *
               state_hat_[col];
    }
    for (uint32_t input = 0U; input < LESO_INPUT_DIM; ++input) {
      const uint32_t coefficient_row = row * LESO_INPUT_DIM + input;
      const float input_gain =
          EvaluatePolynomial(leso_bd_coeffs[coefficient_row], features);
      value += input_gain *
               (applied_input[input] + state_hat_[LESO_STATE_DIM + input]);
    }
    for (uint32_t output = 0U; output < LESO_STATE_DIM; ++output) {
      value += leso_gain[row][output] * innovation[output];
    }
    next_state[row] = value;
  }

  for (uint32_t disturbance = 0U; disturbance < LESO_INPUT_DIM;
       ++disturbance) {
    const uint32_t row = LESO_STATE_DIM + disturbance;
    float value = state_hat_[row];
    for (uint32_t output = 0U; output < LESO_STATE_DIM; ++output) {
      value += leso_gain[row][output] * innovation[output];
    }
    const float limit = disturbance < 2U ? kWheelDisturbanceStateLimit
                                         : kHipDisturbanceStateLimit;
    next_state[row] = Clamp(value, limit);
  }

  if (!VectorIsFinite(next_state, LESO_EXTENDED_DIM)) {
    Reset(measurement);
    return false;
  }

  for (uint32_t i = 0U; i < LESO_EXTENDED_DIM; ++i) {
    state_hat_[i] = next_state[i];
  }
  return true;
}

float Leso::GetWheelCommonDisturbance() const {
  if (!initialized_) {
    return 0.0f;
  }
  return 0.5f * (state_hat_[LESO_STATE_DIM] +
                 state_hat_[LESO_STATE_DIM + 1U]);
}

float Leso::GetHipCommonDisturbance() const {
  if (!initialized_) {
    return 0.0f;
  }
  return 0.5f * (state_hat_[LESO_STATE_DIM + 2U] +
                 state_hat_[LESO_STATE_DIM + 3U]);
}

float Leso::EvaluatePolynomial(const float coefficients[LESO_POLY_TERMS],
                               const float features[6]) {
  float value = 0.0f;
  for (uint32_t i = 0U; i < LESO_POLY_TERMS; ++i) {
    value += coefficients[i] * features[i];
  }
  return value;
}

float Leso::Clamp(float value, float limit) {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

bool Leso::VectorIsFinite(const float *values, uint32_t count) {
  if (values == 0) {
    return false;
  }
  for (uint32_t i = 0U; i < count; ++i) {
    if (!isfinite(values[i])) {
      return false;
    }
  }
  return true;
}
