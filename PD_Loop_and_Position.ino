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

  float cutoff = peak * 0.4f;             // ignore sensors much lighter than the line

  // Follow ONE line: start at the dark sensor closest to where the line was last time,
  // so a second line elsewhere under the array cannot drag the position toward the gap between them.
  int start = -1;
  float best = 1e9f;
  for (int i = 0; i < 16; i++) {
    if (darkness(i) < cutoff) continue;
    float gap = fabs(sensorWeight[i] - linePos);
    if (gap < best) { best = gap; start = i; }
  }
  int lo = start, hi = start;              // grow the group left and right while sensors stay dark
  while (lo > 0  && darkness(lo - 1) >= cutoff) lo--;
  while (hi < 15 && darkness(hi + 1) >= cutoff) hi++;

  float num = 0, den = 0;
  for (int i = lo; i <= hi; i++) {
    float d = darkness(i);
    num += d * sensorWeight[i];
    den += d;
  }
  linePos = num / den;                     // den > 0 because the start sensor passed the cutoff
}

// Turns linePos into the steering value "turn".
// P  = kp * error.   D = kd * (how much the error changed over the last 10 ms), smoothed.
// The D term is only recomputed every 10 ms, so the 30-unit steps of the position don't turn into noise spikes.
void PDcompute() {
  static float prevErr = 0, dTerm = 0;
  static uint32_t lastUs = 0;

  uint32_t nowUs = micros();
  uint32_t elapsed = nowUs - lastUs;

  if (pdReset || elapsed > 50000) {        // start, line came back, or a pause: no derivative kick
    pdReset = false;
    prevErr = linePos;
    dTerm = 0;
    lastUs = nowUs;
  } else if (elapsed >= 10000) {           // every 10 ms
    float change = (linePos - prevErr) * (10000.0f / elapsed);   // error change per 10 ms
    dTerm += 0.5f * (change - dTerm);                            // smooth it
    prevErr = linePos;
    lastUs = nowUs;
  }

  turn = constrain(kp * linePos + kd * dTerm, -baseSpeed, baseSpeed);
}
