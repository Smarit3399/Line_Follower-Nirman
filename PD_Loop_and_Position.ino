// How dark sensor i is: 0 = white, 4095 = black.
int darkness(int i) {
  return 4095 - norm[i];
}

// Finds the line position (negative = line on the left) and counts sensors on the line.
// linePos is only updated when the line is visible, so it keeps the last known position.
void computePosition() {
  float peak = 0;
  leftCount = 0;
  rightCount = 0;
  for (int i = 0; i < 16; i++) {
    int d = darkness(i);
    if (d > peak) peak = d;
    if (d > ON_LEVEL) {
      if (i < 8) leftCount++;
      else       rightCount++;
    }
  }

  lineFound = (peak > 1200.0f);
  if (!lineFound) return;

  float num = 0, den = 0;
  float cutoff = peak * 0.35f;             // ignore sensors much lighter than the line
  for (int i = 0; i < 16; i++) {
    float d = darkness(i);
    if (d < cutoff) continue;
    num += d * sensorWeight[i];
    den += d;
  }
  linePos = num / den;                     // den > 0 because the darkest sensor passed the cutoff
}

// Turns linePos into the steering value "turn".
void PDcompute() {
  static float prevErr = 0, dFiltered = 0;
  static uint32_t lastUs = 0;

  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;

  if (pdReset || dt > 0.05f) {             // start, line came back, or a pause: no derivative kick
    pdReset = false;
    prevErr = linePos;
    dFiltered = 0;
  } else {
    if (dt < 0.0002f) dt = 0.0002f;
    float d = (linePos - prevErr) / (dt * 1000.0f);   // error change per millisecond
    dFiltered += 0.3f * (d - dFiltered);              // smooth the noisy derivative
    prevErr = linePos;
  }

  turn = constrain(kp * linePos + kd * dFiltered, -baseSpeed, baseSpeed);
}
