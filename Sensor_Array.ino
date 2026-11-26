// Reads all 16 sensors. Fills norm[]: 4095 = white, 0 = black (after calibration).
void scan()
{
  uint32_t wait = 25 * getCpuFrequencyMhz();       // minimum time per channel, in CPU ticks
  for (int i = 0; i < 16; i++)
  {
    for (int b = 0; b < 4; b++)                    // select mux channel i
    {
      if (MUX_TRUTH[i][b]) GPIO.out_w1ts = (1UL << S_pin[b]);
      else                 GPIO.out_w1tc = (1UL << S_pin[b]);
    }
    delayMicroseconds(2);
    uint32_t t0 = ESP.getCycleCount();

    int raw = 0;
    if (adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &raw) == ESP_OK)
    {
      if (calibrating)
      {
        if (raw < minV[i]) minV[i] = raw;
        if (raw > maxV[i]) maxV[i] = raw;
      }
      int span = maxV[i] - minV[i];
      if (span > 0)
        norm[i] = constrain(((raw - minV[i]) * 4095) / span, 0, 4095);
      else
        norm[i] = 4095;                            // not calibrated yet: report "white"
    }
    while ((uint32_t)(ESP.getCycleCount() - t0) < wait) {}
  }
}

// Spins the bot on the spot while scanning, so every sensor sees both white and black.
void SensorCalibration()
{
  for (int i = 0; i < 16; i++) { minV[i] = 4095; maxV[i] = 0; }
  calibrated = false;
  calibrating = true;
  OLEDFLAG = false;                                // the IMU needs the I2C bus alone
  delay(80);                                       // let the OLED finish its last draw
  yaw = 0;
  IMUFLAG = true;

  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);     // left forward
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);    // right backward
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 180 + deadL);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 180 + deadR);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

  uint32_t start = millis();                       // stops after 720 degrees, 10 s, or Select
  while (fabs(yaw) < 720.0f && CurrentState == Calibration && millis() - start < 10000)
    scan();

  stopMotors();
  IMUFLAG = false;
  calibrating = false;

  bool ok = (fabs(yaw) >= 720.0f);                 // finished the full spin
  for (int i = 0; i < 16; i++)
    if (maxV[i] - minV[i] < 200) ok = false;       // every sensor must have seen white and black
  calibrated = ok;

  yaw = 0;
  OLEDFLAG = true;
}
