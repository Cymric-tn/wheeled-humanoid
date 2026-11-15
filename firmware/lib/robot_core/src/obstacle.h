// Obstacle distance from the four forward range sensors.
// Each sensor goes through a median of its last three readings (rejects single spikes);
// the obstacle distance is the minimum over the sensors that reported recently.
// ToF is precise but narrow and misses dark or glossy surfaces; ultrasound is wide but coarse.
#pragma once
#include <stdint.h>
#include "config.h"

class Obstacle {
 public:
  enum { TOF_L = 0, TOF_R, SONAR_L, SONAR_R, N };

  void reset() {
    for (uint8_t i = 0; i < N; i++) {
      n_[i] = 0;
      age_[i] = 0xFFFF;
    }
  }

  // mm <= 0 means "no echo / out of range": stored as the sensor's maximum range.
  void push(uint8_t i, int16_t mm) {
    int16_t maxmm = (i < SONAR_L) ? cfg::TOF_MAX_MM : cfg::SONAR_MAX_MM;
    if (mm < cfg::RANGE_MIN_MM || mm > maxmm) mm = maxmm;
    h_[i][2] = h_[i][1];
    h_[i][1] = h_[i][0];
    h_[i][0] = mm;
    if (n_[i] < 3) n_[i]++;
    age_[i] = 0;
  }

  void tick(uint16_t ms) {
    for (uint8_t i = 0; i < N; i++)
      if (age_[i] < 0xFFFF - ms) age_[i] += ms; else age_[i] = 0xFFFF;
  }

  bool fresh(uint8_t i) const { return n_[i] == 3 && age_[i] < cfg::RANGE_STALE_MS; }

  int16_t filtered(uint8_t i) const {
    int16_t a = h_[i][0], b = h_[i][1], c = h_[i][2];
    if (a > b) { int16_t t = a; a = b; b = t; }
    if (b > c) { b = c; }
    return a > b ? a : b;  // median of three
  }

  // Nearest obstacle in mm, or -1 if no sensor is fresh (the caller refuses forward motion).
  int16_t nearest() const {
    int16_t best = -1;
    for (uint8_t i = 0; i < N; i++) {
      if (!fresh(i)) continue;
      int16_t f = filtered(i);
      if (best < 0 || f < best) best = f;
    }
    return best;
  }

  uint8_t freshMask() const {
    uint8_t m = 0;
    for (uint8_t i = 0; i < N; i++) if (fresh(i)) m |= 1 << i;
    return m;
  }

 private:
  int16_t h_[N][3];
  uint8_t n_[N];
  uint16_t age_[N];
};
