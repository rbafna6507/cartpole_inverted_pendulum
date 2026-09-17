#pragma once
#include <cmath>
#include <cstdint>

namespace encoder_reference {
// User-recorded 180-degree pose; absolute AS5600 count, independent of boot zero.
// Source: encoder_calibration/encoder_20260916_135926_031977.json
constexpr int counts_per_turn = 4096;
constexpr int upright_raw = 3416;
constexpr float radians_per_count = 6.2831853071795864769f / counts_per_turn;

// Uniform count scale only. The other three poses are diagnostic reference points,
// not a validated nonlinear correction. Retain ENC_INVERT for direction convention.
inline float angle_from_upright(int32_t raw, bool invert) {
  int32_t delta = (raw-upright_raw) % counts_per_turn;
  if (delta > counts_per_turn/2) delta -= counts_per_turn;
  if (delta < -counts_per_turn/2) delta += counts_per_turn;
  return (invert ? -delta : delta) * radians_per_count;
}
}
