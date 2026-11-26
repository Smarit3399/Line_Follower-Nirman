void BTN_Read(void *param)
{ bool lastState[3]={false,false,false};
  bool btnUpPressed=false;
  bool btnDownPressed=false;
  bool btnSelectPressed=false;
  while(true)
  { if(BTNFLAG==false)
    {
      vTaskDelay(pdMS_TO_TICKS(150));
    }
    else
    {
        btnUpPressed = (digitalRead(BTN_UP) == LOW);
        btnDownPressed = (digitalRead(BTN_DWN) == LOW);
        btnSelectPressed = (digitalRead(BTN_ACT) == LOW);
        menuState st = CurrentState; 
        if(lastState[0]!=btnUpPressed && btnUpPressed==true && CurrentState==Menu)
        {
          menuIndex-=1;
          if(menuIndex<0)
              menuIndex=4;
        }
        else if(lastState[1]!=btnDownPressed && btnDownPressed==true && CurrentState==Menu)
        {
          menuIndex+=1;
          if(menuIndex>4)
              menuIndex=0;
        }
        if(btnSelectPressed!=lastState[2] && btnSelectPressed==true && CurrentState==Menu)
        {
          if(menuIndex==0)
             CurrentState =Calibration;
          else if(menuIndex==1)
           { CurrentState =PID_Values;
             fieldIndex=0;
             editing=false;
           }
          else if(menuIndex==2)
             CurrentState =Gyro_Calibration;
          else if(menuIndex==3)
            {CurrentState =Motor_Deadzone;
             fieldIndex=0;
             editing=false;
            }
          else if(menuIndex==4)
             CurrentState =Race;
        }
        else if(btnSelectPressed!=lastState[2] && btnSelectPressed==true && CurrentState!=Menu && CurrentState!=PID_Values && CurrentState!=Motor_Deadzone)
                CurrentState=Menu;
        if(st==PID_Values)                            
        {
          if(editing==false)                              
          {
            if(lastState[0]!=btnUpPressed && btnUpPressed==true)
            {
              fieldIndex-=1;
              if(fieldIndex<0)
                  fieldIndex=3;
            }
            else if(lastState[1]!=btnDownPressed && btnDownPressed==true)
            {
              fieldIndex+=1;
              if(fieldIndex>3)
                 fieldIndex=0;
            }
            if(btnSelectPressed!=lastState[2] && btnSelectPressed==true)
            {
              if(fieldIndex==3)
                  CurrentState=Menu;                     
              else
                  editing=true;                           
            }
          }
          else                                           
            {
              if(fieldIndex==0)
              {
                if(lastState[0]!=btnUpPressed && btnUpPressed==true)
                {
                  kp+=0.2;
                }
                else if(lastState[1]!=btnDownPressed && btnDownPressed==true)
                {
                  kp-=0.2;
                }
              }
              if(fieldIndex==1)
              {
                if(lastState[0]!=btnUpPressed && btnUpPressed==true)
                {
                 kd+=0.2;
                }
                else if(lastState[1]!=btnDownPressed && btnDownPressed==true)
                {
                 kd-=0.2;
                }
              }
              if(fieldIndex==2)
              {
                if(lastState[0]!=btnUpPressed && btnUpPressed==true)
                {
                  baseSpeed+=50;
                  if(baseSpeed>1023) baseSpeed=1023;
                }
                else if(lastState[1]!=btnDownPressed && btnDownPressed==true)
                {
                  baseSpeed-=50;
                } 
              }
                if(btnSelectPressed!=lastState[2] && btnSelectPressed==true)
                    editing=false; 
            }
        } 
        if(st==Motor_Deadzone)                            
        {
          if(editing==false)                              
          {
            if(lastState[0]!=btnUpPressed && btnUpPressed==true)
            {
              fieldIndex-=1;
              if(fieldIndex<0)
                  fieldIndex=2;
            }
            else if(lastState[1]!=btnDownPressed && btnDownPressed==true)
            {
              fieldIndex+=1;
              if(fieldIndex>2)
                 fieldIndex=0;
            }
            if(btnSelectPressed!=lastState[2] && btnSelectPressed==true)
            {
              if(fieldIndex==2)
                  CurrentState=Menu;                     
              else
                  editing=true;                           
            }
          }
          else
          {
            if(fieldIndex==0)
            {
              if(lastState[0]!=btnUpPressed && btnUpPressed==true)
                  deadL+=3;
              else if(lastState[1]!=btnDownPressed && btnDownPressed==true)
              {
                deadL-=3;
                if(deadL<0) deadL=0;              }
            }
            if(fieldIndex==1)
            {
              if(lastState[0]!=btnUpPressed && btnUpPressed==true)
                  deadR+=3;
              else if(lastState[1]!=btnDownPressed && btnDownPressed==true)
              {
                deadR-=3;
              }
            }
            if(btnSelectPressed!=lastState[2] && btnSelectPressed==true)
                editing=false;
          }
        }
        }

    
        lastState[0] =btnUpPressed;
        lastState[1] =btnDownPressed;
        lastState[2] =btnSelectPressed;
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}