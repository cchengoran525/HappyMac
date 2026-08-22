#pragma once

#include <stdint.h>

// T1b aligned v0, sklearn DecisionTreeClassifier(max_depth=4).
// Feature order:
// |slope_X|, slope_Y, std_X, std_Y, mean_Es, std_Es,
// mean_abs_V, max_abs_V, Es_edge
// Labels: 0 ABSENT, 1 STILL, 2 LATERAL, 3 APPROACH, 4 RETREAT.
// Generated from training/models/tinyml_v0_tree_d4.pkl.
static inline uint8_t tinymlV0Predict(const float f[9]) {
  const float abs_slope_x = f[0];
  const float slope_y = f[1];
  const float mean_es = f[4];
  const float std_es = f[5];
  const float mean_abs_v = f[6];
  const float max_abs_v = f[7];

  if (mean_es <= 9.50f) {
    if (mean_es <= 7.44444442f) return 0;
    if (std_es <= 1.33473241f) {
      return abs_slope_x <= 11.11721635f ? 1 : 3;
    }
    return slope_y <= 18.86263752f ? 0 : 3;
  }

  if (mean_abs_v <= 2.51851857f) {
    if (mean_es <= 54.12962914f) return slope_y <= 0.94871792f ? 1 : 4;
    return abs_slope_x <= 3.08638585f ? 1 : 2;
  }

  if (max_abs_v <= 35.50f) return 2;
  return slope_y <= -2.62210011f ? 3 : 4;
}

static inline const char* tinymlV0LabelName(uint8_t label) {
  switch (label) {
    case 0: return "ABSENT";
    case 1: return "STILL";
    case 2: return "LATERAL";
    case 3: return "APPROACH";
    case 4: return "RETREAT";
  }
  return "UNKNOWN";
}
