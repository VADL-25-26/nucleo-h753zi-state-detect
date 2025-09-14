/*
 * kalman.c
 *
 *  Created on: Sep 12, 2025
 *      Author: Owner
 */


#include "kalman.h"
static float prev_alt = 0.0f;
void KalmanFilter_Update(float z, float* zf, float* vf) {
  static int first = 1;
  if (first) { prev_alt = z; first = 0; }
  *zf = 0.8f*prev_alt + 0.2f*z;      // light LPF
  *vf = (z - prev_alt) * 6.5617f;     // fake “ft/s” from delta(ft)/0.05s (≈ *20), scaled a bit calmer
  prev_alt = z;
}
