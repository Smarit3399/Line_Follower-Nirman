// Drives both wheels forward. turn > 0 steers right (left wheel faster).
// deadL / deadR = the lowest duty at which that wheel still turns smoothly.
void MotorDrive() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);

  int speed = baseSpeed - (int)(fabs(linePos) * CURVE_SLOW * baseSpeed / 234.0f);  
  int left  = speed + (int)turn;       
  int right = speed - (int)turn;        

  left  = constrain(max(left,  (int)deadL), 0, 1023); 
  right = constrain(max(right, (int)deadR), 0, 1023); 

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, left);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, right);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// Line lost: arc toward the side it was last seen on. The outer wheel runs, the inner wheel stops.
// dir = -1: line was on the left, +1: on the right.
void MotorSearch(int dir) {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  int left  = (dir > 0) ? constrain(max((int)baseSpeed, (int)deadL), 0, 1023) : 0;   // turning right: left wheel runs
  int right = (dir < 0) ? constrain(max((int)baseSpeed, (int)deadR), 0, 1023) : 0;   // turning left: right wheel runs
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, left);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, right);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// Spins on the spot. dir = +1 spins right, -1 spins left.
void MotorSpin(int dir) {
  digitalWrite(AIN1, dir > 0 ? HIGH : LOW);  digitalWrite(AIN2, dir > 0 ? LOW : HIGH);   // left wheel
  digitalWrite(BIN1, dir > 0 ? LOW : HIGH);  digitalWrite(BIN2, dir > 0 ? HIGH : LOW);   // right wheel

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, constrain(max(PIVOT_SPEED, (int)deadL), 0, 1023));
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, constrain(max(PIVOT_SPEED, (int)deadR), 0, 1023));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void stopMotors() {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}
