void IMU_READ(void *param)
{ 
  unsigned long now;
  float dt;
  bool wasActive=false;
  float y=0.0;
 while(1)
 {
  if(IMUFLAG==false)
  {
    vTaskDelay(pdMS_TO_TICKS(50));
    wasActive=false;
  }
  else
  {  if(wasActive==false)
    {
      lastT=micros();
      y=0.0;
      dt=0.0;
      wasActive=true;
    }
     gz=mpu.getRotationZ();
     if(gyroDone==false)
      gyroSamples++;
     now=micros();
     dt=(now-lastT)/1000000.0f;
     lastT=now;
     y+=((gz-bias)/131.0f)*dt;
     yaw=y;
     vTaskDelay(pdMS_TO_TICKS(5));
  }
 }
}

void gyroCalibrate()
{
  gyroDone = false;
  delay(250);
  OLEDFLAG = false;                      // OLED off so only the IMU uses I2C
  vTaskDelay(pdMS_TO_TICKS(50));         // let any draw in progress finish
  unsigned long last = gyroSamples;
  IMUFLAG = true;
  long sum = 0;
  int count = 0;
  unsigned long start = millis();
  while (millis() - start < 2000)
  {
    if (gyroSamples != last)             // a new reading arrived
    {
      last = gyroSamples;
      sum += gz;
      count++;
    }
  }
  IMUFLAG = false;
  vTaskDelay(pdMS_TO_TICKS(20));         // let the IMU task park
  if (count > 0) bias = (float)sum / count;
  gyroDone = true;
  OLEDFLAG = true;                       // OLED back on
}