// ---------- helpers ----------

// Drives straight using the gyro (for dotted gaps, crossings and the step before a pivot).
void holdHeading() {
  float t = (yaw - headingRef) * GYRO_GAIN;   // if it steers the wrong way, flip this sign
  turn = constrain(t, -baseSpeed, baseSpeed);
  MotorDrive();
}

// True when the middle sensors are on a thin line again (end of a pivot).
bool centered() {
  bool middleOn = (darkness(7) > ON_LEVEL) || (darkness(8) > ON_LEVEL);
  return middleOn && (leftCount + rightCount <= 4);
}

// Normal case: the line is visible, run the PD.
void followLine() {
  if (lostSince != 0) pdReset = true;         // line just came back
  lostSince = 0;
  headingRef = yaw;                           // remember the heading while we can see the line
  PDcompute();
  MotorDrive();
}

// The line is not visible.
void lostLine(uint32_t now) {
  if (lostSince == 0) lostSince = now;

  if (fabs(linePos) > EDGE_POS) {             // it left past the edge: sharp or acute turn
    turnDir = (linePos < 0) ? -1 : 1;
    raceMode = M_PIVOT;
    modeStart = now;
  } else if (now - lostSince < GAP_MS) {      // it was near the middle: dotted gap, go straight
    holdHeading();
  } else {
    raceEnd();                                // lost for good: stop
  }
}

// ---------- race start / stop ----------

void raceBegin() {
  raceRunning = false;
  if (!calibrated) return;                    // the screen already says NOT CALIBRATED
  for (int i = 0; i < 30; i++) {              // 3 s countdown, Select cancels
    if (CurrentState != Race) return;
    delay(100);
  }
  OLEDFLAG = false;                           // the IMU needs the I2C bus alone
  delay(80);
  yaw = 0;
  headingRef = 0;
  IMUFLAG = true;
  delay(100);                                 // let the IMU task wake up
  raceMode = M_FOLLOW;
  lostSince = 0;
  boxSince = 0;
  longestBar = 0;
  crossCnt = 0; cornerCnt = 0; centerCnt = 0;
  linePos = 0;
  turn = 0;
  pdReset = true;
  raceRunning = true;
}

void raceEnd() {
  raceRunning = false;
  stopMotors();
  IMUFLAG = false;
  OLEDFLAG = true;
}

// ---------- one pass of the race, called over and over from loop() ----------

void raceStep() {
  scan();
  computePosition();
  uint32_t now = millis();

  // stop box: (almost) every sensor black for longer than any crossing bar seen so far
  if (leftCount + rightCount >= BOX_MIN) {
    if (boxSince == 0) boxSince = now;
    uint32_t limit = 2 * longestBar;           // earlier bars teach us how long "all black" lasts on a crossing
    if (limit < BOX_MS) limit = BOX_MS;
    if (now - boxSince >= limit) { raceEnd(); return; }
  } else if (boxSince != 0) {
    uint32_t spell = now - boxSince;           // the black spell ended, so it was a crossing bar
    if (spell > longestBar) longestBar = spell;
    boxSince = 0;
  }

  bool crossing = (leftCount >= 4 && rightCount >= 4 && leftCount + rightCount >= CROSS_MIN);
  bool cornerL  = (leftCount  >= SIDE_MIN && rightCount <= 1);
  bool cornerR  = (rightCount >= SIDE_MIN && leftCount  <= 1);

  crossCnt  = crossing ? crossCnt + 1 : 0;                 // count scans in a row, so one noisy scan changes nothing
  cornerCnt = (cornerL || cornerR) ? cornerCnt + 1 : 0;

  switch (raceMode) {

    case M_FOLLOW:
      if (crossCnt >= CONFIRM) {              // intersection or loop crossing: go straight through
        raceMode = M_CROSS;
        modeStart = now;
      } else if (cornerCnt >= CONFIRM) {      // 90 degree corner OR a side stub: drive forward, then decide
        turnDir = cornerL ? -1 : 1;
        raceMode = M_STEP;
        modeStart = now;
      } else if (lineFound) {
        followLine();
      } else {
        lostLine(now);
      }
      break;

    case M_CROSS:
      holdHeading();
      if (leftCount + rightCount <= 4) {      // the bar is behind us
        pdReset = true;
        lostSince = 0;
        raceMode = M_FOLLOW;
      }
      break;

    case M_STEP:
      holdHeading();
      if (now - modeStart >= STEP_MS) {
        if (lineFound && leftCount + rightCount <= 4) {   // line still ahead: it was a side stub, ignore it
          pdReset = true;
          lostSince = 0;
          raceMode = M_FOLLOW;
        } else {                                          // nothing ahead: real corner, pivot
          raceMode = M_PIVOT;
          modeStart = now;
        }
      }
      break;

    case M_PIVOT:
      MotorSpin(turnDir);
      centerCnt = centered() ? centerCnt + 1 : 0;
      if (centerCnt >= CONFIRM && now - modeStart >= PIVOT_MIN_MS) {   // centered for a few scans, and not too early
        centerCnt = 0;
        pdReset = true;
        lostSince = 0;
        raceMode = M_FOLLOW;
      } else if (now - modeStart > PIVOT_MS) {
        raceEnd();                            // could not find the line: stop
      }
      break;
  }
}
