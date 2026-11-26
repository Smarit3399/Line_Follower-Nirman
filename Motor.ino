// Drives both wheels forward. turn > 0 steers right (left wheel faster).
void MotorDrive() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);

  int left  = baseSpeed + (int)turn;       // channel 0
  int right = baseSpeed - (int)turn;       // channel 1

  left  = (left  > 0) ? min(left  + deadL, 1023) : 0;   // deadzone only if it should move
  right = (right > 0) ? min(right + deadR, 1023) : 0;

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, left);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, right);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// Spins on the spot. dir = +1 spins right, -1 spins left.
void MotorSpin(int dir) {
  digitalWrite(AIN1, dir > 0 ? HIGH : LOW);  digitalWrite(AIN2, dir > 0 ? LOW : HIGH);   // left wheel
  digitalWrite(BIN1, dir > 0 ? LOW : HIGH);  digitalWrite(BIN2, dir > 0 ? HIGH : LOW);   // right wheel

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, min(PIVOT_SPEED + deadL, 1023));
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, min(PIVOT_SPEED + deadR, 1023));
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
