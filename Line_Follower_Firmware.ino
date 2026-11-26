#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <MPU6050.h>
#include <U8g2lib.h>
#include <driver/ledc.h>
#include "esp_bt.h"
#include "esp_wifi.h"
#include "soc/soc.h"
#include "soc/gpio_struct.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_adc/adc_oneshot.h"
#include <driver/gpio.h>
#include <math.h>
//PIN DEFINATION:-
//Motor Driver Pins
  const int PWMA =32;
  const int PWMB =13;
  const int AIN1 =25;
  const int AIN2 =33;
  const int BIN1 =26;
  const int BIN2 =27;
//Mux Pins
  const int SIG =34;
  const int S_pin[4] ={4,18,19,23};
  const uint8_t MUX_TRUTH[16][4] = {
  {0, 0, 0, 0}, // ch 0
  {1, 0, 0, 0}, // ch 1
  {0, 1, 0, 0}, // ch 2
  {1, 1, 0, 0}, // ch 3
  {0, 0, 1, 0}, // ch 4
  {1, 0, 1, 0}, // ch 5
  {0, 1, 1, 0}, // ch 6
  {1, 1, 1, 0}, // ch 7
  {0, 0, 0, 1}, // ch 8
  {1, 0, 0, 1}, // ch 9
  {0, 1, 0, 1}, // ch 10
  {1, 1, 0, 1}, // ch 11
  {0, 0, 1, 1}, // ch 12
  {1, 0, 1, 1}, // ch 13
  {0, 1, 1, 1}, // ch 14
  {1, 1, 1, 1}  // ch 15
};
//Button Pins
   const int BTN_UP =16;
   const int BTN_DWN =5;
   const int BTN_ACT =17;
//ADC-Reader Handler
  adc_oneshot_unit_handle_t adc1_handle;
// Sensor & Display Objects
  MPU6050 mpu;
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);
//Global Variables and Core 0 StateFlags
volatile bool BTNFLAG =true;
volatile bool OLEDFLAG =true;
volatile bool IMUFLAG =false;
enum menuState {Menu,Calibration,PID_Values,Gyro_Calibration,Motor_Deadzone,Race};
volatile menuState CurrentState = Menu;
volatile int menuIndex=0;
volatile int fieldIndex;
volatile bool calibrated=false;
volatile float yaw=0;
volatile uint32_t lastT;
volatile float bias =0;
volatile bool gyroDone = false; 
volatile bool editing = false;
volatile float kp = 1.0f;
volatile float kd = 0.0f;
volatile int baseSpeed = 400;
volatile int deadL = 0;
volatile int deadR = 0;
volatile unsigned long gyroSamples = 0;
volatile int16_t gz;
float prevError;
int norm[16];                
int minV[16]={4096,4096,4096,4096,4096,4096,4096,4096,4096,4096,4096,4096,4096,4096,4096,4096};
int maxV[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};                  
const float sensorWeight[16] = {
  -234, -226, -180, -135, -105, -75, -45, -15,
  15,   45,   75,  105,  135, 180, 226, 234
};
float linePos=0;
volatile bool lineFound=false;
volatile float turn = 0;
volatile bool invertedMode = false;
volatile bool raceRunning = false;
volatile bool pdReset = false;
volatile bool calibrating=false;
volatile bool firstRun = true;

//Race logic: modes, shared variables and tunables (all numbers are guesses, tune them on the track)
enum RaceMode {M_FOLLOW, M_CROSS, M_STEP, M_PIVOT};
RaceMode raceMode = M_FOLLOW;
uint32_t modeStart = 0;           // when the current mode began (ms)
uint32_t lostSince = 0;           // when the line was lost (ms), 0 = line is visible
int   turnDir = 0;                // -1 = pivot left, +1 = pivot right
float headingRef = 0;             // heading to hold in dotted gaps (degrees)
int   leftCount = 0;              // sensors on the line, left half (0-7)
int   rightCount = 0;             // sensors on the line, right half (8-15)
uint32_t boxSince = 0;            // when the array first went (almost) all black, 0 = not now
uint32_t longestBar = 0;          // longest all-black spell that ended so far (a crossing bar), ms
int   crossCnt = 0;                // consecutive scans that looked like a crossing
int   cornerCnt = 0;               // consecutive scans that looked like a corner or stub
int   centerCnt = 0;               // consecutive scans the pivot looked finished

const int      ON_LEVEL     = 2000;  // darkness (0-4095) above this = sensor sees the line
const int      CROSS_MIN    = 10;    // sensors on, both halves together = crossing
const int      SIDE_MIN     = 5;     // sensors on in one half and almost none in the other = 90 degree corner
const int      EDGE_POS     = 150;   // line lost after being out past this = sharp or acute turn
const int      PIVOT_SPEED  = 350;   // wheel duty while spinning on the spot
const uint32_t STEP_MS      = 150;   // drive forward before a 90 degree pivot (sensor-to-axle distance / speed)
const uint32_t PIVOT_MS     = 700;   // stop if a pivot takes longer than this
const uint32_t GAP_MS       = 300;   // dotted gap: hold straight this long
const int      CONFIRM      = 3;     // scans in a row needed before a mode change (filters noise)
const int      BOX_MIN      = 14;    // this many sensors on (almost all) = inside the stop box
const uint32_t BOX_MS       = 120;   // ...for at least this long, and 2x longer than any crossing bar seen so far = stop
const uint32_t PIVOT_MIN_MS = 80;    // a pivot cannot end before this
const float    GYRO_GAIN    = 3.0f;  // turn added per degree of heading error
void setup() {
  setCpuFrequencyMhz(240);
  //This section is to disable the wifi and bt so we can avoid any adc noice and free up cpu(this part is optional u can remove this and the line 7 and 8)
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_mem_release(ESP_BT_MODE_BTDM);
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_wifi_set_mode(WIFI_MODE_NULL);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  //This part is used to intialize all the pin modes
  for(int i=0;i<4;i++)
   {
    pinMode(S_pin[i],OUTPUT);     
   }
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(BTN_UP,INPUT_PULLUP);
  pinMode(BTN_DWN,INPUT_PULLUP);
  pinMode(BTN_ACT,INPUT_PULLUP);
  //In this part we start and initialize imp things
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  mpu.initialize();
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  u8g2.begin();
  lastT=micros();
  adc_oneshot_unit_init_cfg_t init_config ={.unit_id =ADC_UNIT_1, .ulp_mode =ADC_ULP_MODE_DISABLE,};
  ESP_ERROR_CHECK(adc_oneshot_new_unit( &init_config, &adc1_handle));//this is for error handeling if we get any error intializing it will print directly to the serial monitor
  adc_oneshot_chan_cfg_t chnl_config ={.atten =ADC_ATTEN_DB_12,.bitwidth =ADC_BITWIDTH_DEFAULT};
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &chnl_config));//this is also checks for errors and this one print in serial monitor if theres some problem when setting up the channel
  ledc_timer_config_t ledc_timer = {
    .speed_mode      = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num       = LEDC_TIMER_0,
    .freq_hz         = 20000,
    .clk_cfg         = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t channel_a = {
    .gpio_num       = PWMA,
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = LEDC_CHANNEL_0,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 0,
    .hpoint         = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&channel_a));

  ledc_channel_config_t channel_b = {
    .gpio_num       = PWMB,
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = LEDC_CHANNEL_1,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 0,
    .hpoint         = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&channel_b));
  //In this part we make the task to run on core 0
  xTaskCreatePinnedToCore(
    BTN_Read,
    "button",
    2048,//stack
    NULL,
    2,//priority
    NULL,
    0
  );
  xTaskCreatePinnedToCore(
    Oled_Menu,
    "Oled",
    6000,//stack
    NULL,
    1,//priority
    NULL,
    0
  );
  xTaskCreatePinnedToCore(
    IMU_READ,
    "imu",
    4096,//stack
    NULL,
    3,//priority
    NULL,
    0
  );
}


void loop() {
  static menuState prev = Menu;
  menuState st = CurrentState;
  if (st != prev) {                          // the menu state just changed
    if (prev == Race) raceEnd();
    prev = st;
    if (st == Calibration)           SensorCalibration();
    else if (st == Gyro_Calibration) gyroCalibrate();
    else if (st == Race)             raceBegin();
  }
  if (raceRunning && CurrentState == Race) raceStep();
  else delay(5);
}
