void Oled_Menu(void *param)
{  const char* menuItems[5]={"Calibration","PID_Values","Gyro_Calibration","Motor_Deadzone","Race"};
   int y;
  while(true)
  {
    if(OLEDFLAG==false)
      vTaskDelay(pdMS_TO_TICKS(500));
    else
    {   u8g2.setFont(u8g2_font_6x13B_tr);
        u8g2.clearBuffer();
        for(int i=0;i<5;i++)    
        {
          y=11+i*13;
          if(CurrentState==Menu)
          {
          if(i==menuIndex)
          {
            u8g2.drawBox(0, y-10, 128, 13);   
            u8g2.setDrawColor(0);                
          }
          u8g2.drawStr(2,y,menuItems[i]);
          u8g2.setDrawColor(1);
         
        }
        }
        if(CurrentState==Menu) u8g2.sendBuffer();
        menuState st =CurrentState;
        if(st!=Menu)
        {
          u8g2.clearBuffer();
          switch (st) {
            case PID_Values:       drawPID(); break;
            case Motor_Deadzone:   drawDeadzone(); break;
            case Calibration:      drawCalibration(); break;
            case Gyro_Calibration: drawGyro(); break;
            case Race:             drawRace(); break;
            default: break;
            }
            u8g2.sendBuffer();
        }
        vTaskDelay(pdMS_TO_TICKS(80));
    }
  }
}
void drawCalibration()
{
  u8g2.drawStr(0, 11, "CALIBRATION");
  u8g2.drawStr(0, 24, calibrated ? "Done" : "Sweep the array");
}
void drawGyro()
{
  if(gyroDone)
  {
    char buf[24];
    u8g2.drawStr(0, 11, "GYRO DONE");
    snprintf(buf, sizeof(buf), "Bias: %.1f", (float)bias);
    u8g2.drawStr(0, 24, buf);
  }
  else
  {
    u8g2.drawStr(0, 11, "GYRO CALIB");
    u8g2.drawStr(0, 24, "Keep bot still");
  }
}
void drawRace() {
  u8g2.drawStr(0, 11, "RACE");
  u8g2.drawStr(0, 24, calibrated ? "Ready" : "NOT CALIBRATED");
}
void drawPID()
{
  char lines[4][24];
  snprintf(lines[0], 24, "Kp: %.1f", kp);
  snprintf(lines[1], 24, "Kd: %.1f", kd);
  snprintf(lines[2], 24, "Speed: %d", baseSpeed);
  snprintf(lines[3], 24, "Back");
  for(int i=0;i<4;i++)
  {
    int y=11+i*13;
    if(i==fieldIndex)
    {
      u8g2.drawBox(0, y-10, 128, 13);
      u8g2.setDrawColor(0);
    }
    u8g2.drawStr(2,y,lines[i]);
    if(editing && i==fieldIndex)
      u8g2.drawStr(112,y,"<>");
    u8g2.setDrawColor(1);
  }
}

void drawDeadzone()
{
  char lines[3][24];
  snprintf(lines[0], 24, "Left: %d", deadL);
  snprintf(lines[1], 24, "Right: %d", deadR);
  snprintf(lines[2], 24, "Back");
  for(int i=0;i<3;i++)
  {
    int y=11+i*13;
    if(i==fieldIndex)
    {
      u8g2.drawBox(0, y-10, 128, 13);
      u8g2.setDrawColor(0);
    }
    u8g2.drawStr(2,y,lines[i]);
    if(editing && i==fieldIndex)
      u8g2.drawStr(112,y,"<>");
    u8g2.setDrawColor(1);
  }
}