#include <CrsfSerial.h>
#include <IWatchdog.h>

//Pins
#define SIG11     PC7
#define SIG12     PC6
#define SIG13     PA9
#define SIG14     PA8
#define SIG15     PB15
#define SIG16     PB14

#define SIG21     PB1
#define SIG22     PB2
#define SIG23     PB10
#define SIG24     PB11
#define SIG25     PB12
#define SIG26     PB13

#define LED_PWR   PB3
#define LED_SIG   PB4
#define LED_ARM   PB5
#define LED_CAM   PB8
#define LED_DROP  PB9

#define LED_M_1   PD1
#define LED_M_2   PD2
#define LED_M_3   PD3

#define SW_M_1    PC15
#define SW_M_2    PC14
#define SW_M_3    PC13

#define VBAT      PA4
#define CAM_EN    PA7

#define DE        PA12

#define THROT1    PA10
#define THROT2    PA11

#define DIR_L     SIG11
#define DIR_R     SIG21
#define BRK_L     SIG12
#define BRK_R     SIG22
#define FOOT_L    SIG13
#define FOOT_R    SIG23
#define GEAR_HL   SIG14
#define GEAR_HR   SIG24
#define GEAR_LL   SIG15
#define GEAR_LR   SIG25

#define BOARD_USART1_TX_PIN   PB6
#define BOARD_USART1_RX_PIN   PB7
#define BOARD_USART2_TX_PIN   PA2
#define BOARD_USART2_RX_PIN   PA3
#define BOARD_USART3_TX_PIN   PA5
#define BOARD_USART3_RX_PIN   PB0
#define BOARD_USART4_TX_PIN   PA0
#define BOARD_USART4_RX_PIN   PA1

//Channels
#define SPEED_CH      2
#define TURN_CH       4
#define ARM_CH        6
#define CAM_CH        7
#define DROP1_CH      8
#define DROP2_CH      9
#define DROP_ARM_CH   10
#define GEAR_CH       11

//Params
#define CAR_ID         6
#define VCOEF          22
#define MEASURES       16
#define TELEM_PERIOD   320
#define SERVO_PERIOD   2000
#define SCHECK_PERIOD  5000
#define IDLE_TIME      120000

#define BAT_MAX        546
#define BAT_MIN        403

#define FAST           100
#define NORM           70
#define SLOW           40

//Arrays

char *servo_id[] = 
{
  "A1_S1_", "A1_S2_", "A1_S3_", "A1_S4_", "A2_S1_", "A2_S2_", "A2_S3_", "A2_S4_", "A3_S1_", "A3_S2_", "A3_S3_", 
  "A4_S1_", "A4_S2_", "A4_S3_", "A4_S4_", "A5_S1_", "A5_S2_", "A5_S3_", "A5_S4_", "A6_S1_", "A6_S2_", "A6_S3_"
};

uint16_t servo_open[22] = 
{
  1100, 1100,  980, 1150, 1130, 1150, 1100, 1100, 1050, 1100, 1100,
  1050, 1080, 1100, 1095, 1050, 1090, 1100, 1095, 1160, 1080, 1120
};

uint16_t servo_close[22] = 
{
  1430, 1430, 1480, 1480, 1480, 1480, 1400, 1420, 1510, 1410, 1440,
  1380, 1600, 1430, 1580, 1580, 1570, 1430, 1570, 1475, 1410, 1580
};

uint8_t servo_states[22] = 
{
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

//Vars
uint16_t adc_vals[MEASURES];
uint8_t  adc_idx = 0;
uint32_t last_vbat_read = 0;
uint32_t last_telemetry = 0;

uint32_t last_servoset   = 0;
uint32_t last_servoCheck = 0;

uint8_t  arm = 0, gear = 0, cam_en = 0;
int16_t  Speed = 0, Turn = 0;
int16_t SpeedL = 0, SpeedR = 0;
int8_t   dir = 0, tdir = 0;
uint8_t  dirL = 0, dirR = 0;

uint8_t drop1 = 0, drop2 = 0, drop_arm = 0;
uint8_t d_tmp1 = 0, d_tmp2 = 0;
uint8_t cur_servo = 0;

uint8_t  failsafe = 1;
uint32_t idle_timer = 0;

HardwareSerial Serial1(BOARD_USART1_RX_PIN, BOARD_USART1_TX_PIN);
HardwareSerial Serial2(BOARD_USART2_RX_PIN, BOARD_USART2_TX_PIN);
HardwareSerial Serial3(BOARD_USART3_RX_PIN, BOARD_USART3_TX_PIN);
//HardwareSerial Serial4(BOARD_USART4_RX_PIN, BOARD_USART4_TX_PIN);
CrsfSerial crsf(Serial2);


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void SendTelemetry()
{
  if(millis() - last_telemetry > TELEM_PERIOD)
  {
    uint16_t flags = 0;
    crsf_sensor_gps_t crsfgps = {0};

    flags |= (arm       & (0b11)) << 0;
    flags |= (gear      & (0b11)) << 2;
    flags |= (cam_en    & (0b01)) << 4; 
    flags |= (0         & (0b01)) << 5;
    flags |= (drop_arm  & (0b01)) << 6;

    flags += 1000;

    crsfgps.satellites = CAR_ID;
    crsfgps.altitude = htobe16(flags);
    crsfgps.heading = htobe16(cur_servo*1000);

    crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_GPS, &crsfgps, sizeof(crsfgps));
    last_telemetry = millis();
  }
}

void CheckVbat()
{
  if(millis() - last_vbat_read > TELEM_PERIOD/MEASURES*10)
  {
    adc_vals[adc_idx] = analogRead(VBAT);
    adc_idx++;
    last_vbat_read = millis();
  }

  if(adc_idx >= MEASURES)
  {
    adc_idx = 0;

    uint16_t adc_max = 0;
    uint16_t adc_min = (1<<12);
    uint16_t adc_sum = 0;
    uint16_t adc_avg = 0;
    uint16_t scaled_Voltage = 0;
    uint8_t  percent;
    crsf_sensor_battery_t crsfbatt = { 0 };

    for(uint8_t i = 0; i < MEASURES; i++)
    {
      adc_sum += adc_vals[i];
      if(adc_vals[i] > adc_max) adc_max = adc_vals[i];
      if(adc_vals[i] < adc_min) adc_min = adc_vals[i];
    }
    adc_sum -= adc_max + adc_min;
    adc_avg = adc_sum/(MEASURES - 2);

    scaled_Voltage = 33 * adc_avg * VCOEF / ((1<<12)-1);
    if(scaled_Voltage < BAT_MIN) percent = 0;
    else if(scaled_Voltage > BAT_MAX) percent = 100;
    else percent = 100*(scaled_Voltage - BAT_MIN)/(BAT_MAX - BAT_MIN);

    crsfbatt.voltage   = htobe16(scaled_Voltage);
    crsfbatt.remaining = percent;
    crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfbatt, sizeof(crsfbatt));
  }
}

void PacketChannels()
{
  arm        = (crsf.getChannel(ARM_CH)  > 1300) ? 1 : 0;
  arm        = (crsf.getChannel(ARM_CH)  > 1700) ? 2 : arm;

  gear       = (crsf.getChannel(GEAR_CH) > 1300) ? 1 : 0;
  gear       = (crsf.getChannel(GEAR_CH) > 1700) ? 2 : gear;

  cam_en     = (crsf.getChannel(CAM_CH)  > 1500) ? 1 : 0;

  drop_arm   = (crsf.getChannel(DROP_ARM_CH) > 1300) ? 1 : 0;
  drop1      = (crsf.getChannel(DROP1_CH) > 1500) ? 1 : 0;
  drop2      = (crsf.getChannel(DROP2_CH) > 1500) ? 1 : 0;

  Speed = crsf.getChannel(SPEED_CH);
  if(Speed < 1500)
  {
    Speed = 1500-Speed;
    dir = -1;
  }
  else
  {
    Speed = Speed-1500;
    dir = 1;
  }
  if(Speed < 20) Speed = 0;
  if(Speed > 500) Speed = 500;

  Turn = crsf.getChannel(TURN_CH);
  if(Turn < 1500)
  {
    Turn = 1500-Turn;
    tdir = -1;
  }
  else
  {
    Turn = Turn-1500;
    tdir = 1;
  }
  if(Turn < 20) Turn = 0;
  if(Turn > 500) Turn = 500;

}

void LinkUp()
{
  digitalWrite(LED_SIG, 1);
  failsafe = 0;
}

void LinkDown()
{
  digitalWrite(LED_SIG, 0);
  failsafe = 1;
  idle_timer = millis();
}

uint16_t pwm_map_speed(int16_t val)
{
  uint32_t pwm_val = val * ((1 << 15)-1) / 500;
  return pwm_val;
}

void set_servo(uint8_t id, uint16_t val)
{
  digitalWrite(DE, 1);
  Serial1.print("!"); Serial1.print(servo_id[id]); Serial1.println(val);
  Serial1.flush();
  digitalWrite(DE, 0);
}

void set_all_servos()
{
  if(millis() - last_servoset > SERVO_PERIOD)
  {
    last_servoset = millis();
    for(uint8_t i = 0; i < 22; i++)
    {
      if(servo_states[i] == 0) set_servo(i, servo_close[i]);
      if(servo_states[i] == 1) set_servo(i, servo_open[i]);
    }
  }
}

/*
uint8_t check_servos()
{
  if(millis() - last_servoCheck > SERVOCHECK_PERIOD)
  {
    last_servoCheck = millis();
    if(Serial1.available())
    {
      Serial1.readString();
    }
    else 
    {
      digitalWrite(DE, 1);
      Serial1.println("!CALL");d
      Serial1.flush();
      digitalWrite(DE, 0);
      last_servoset = millis();
    }

  }
}
*/

void setup()
{
  //Pins
  pinMode(LED_PWR,   OUTPUT);
  pinMode(LED_SIG,   OUTPUT);
  pinMode(LED_ARM,   OUTPUT);
  pinMode(LED_CAM,   OUTPUT);
  pinMode(LED_DROP,  OUTPUT);
  digitalWrite(LED_PWR, 1);

  pinMode(LED_M_1,   OUTPUT);
  pinMode(LED_M_2,   OUTPUT);
  pinMode(LED_M_3,   OUTPUT);

  pinMode(SIG11,   OUTPUT);
  pinMode(SIG12,   OUTPUT);
  pinMode(SIG13,   OUTPUT);
  pinMode(SIG14,   OUTPUT);
  pinMode(SIG15,   OUTPUT);
  pinMode(SIG16,   OUTPUT);

  pinMode(SIG21,   OUTPUT);
  pinMode(SIG22,   OUTPUT);
  pinMode(SIG23,   OUTPUT);
  pinMode(SIG24,   OUTPUT);
  pinMode(SIG25,   OUTPUT);
  pinMode(SIG26,   OUTPUT);

  pinMode(THROT1,  OUTPUT);
  pinMode(THROT2,  OUTPUT);

  pinMode(CAM_EN, OUTPUT);
  digitalWrite(CAM_EN, 1);

  pinMode(SW_M_1,    INPUT_PULLUP);
  pinMode(SW_M_2,    INPUT_PULLUP);
  pinMode(SW_M_3,    INPUT_PULLUP);
  pinMode(VBAT, INPUT);

  pinMode(DE,  OUTPUT);


  //Pins params
  analogReadResolution(12);
  analogWriteFrequency(10000);
  analogWriteResolution(15);

  //Interfaces
  Serial1.setRx(BOARD_USART1_RX_PIN);
  Serial1.setTx(BOARD_USART1_TX_PIN);
  Serial1.begin(38400);
  Serial2.setRx(BOARD_USART2_RX_PIN);
  Serial2.setTx(BOARD_USART2_TX_PIN);
  crsf.begin(420000);
  /*
  Serial3.setRx(BOARD_USART3_RX_PIN);
  Serial3.setTx(BOARD_USART3_TX_PIN);
  Serial3.begin(19200);
  Serial4.setRx(BOARD_USART4_RX_PIN);
  Serial4.setTx(BOARD_USART4_TX_PIN);
  Serial4.begin(19200);
  */
  

  //Callbacks
  crsf.onPacketChannels = &PacketChannels;
  crsf.onLinkUp = &LinkUp;
  crsf.onLinkDown = &LinkDown;

}

void loop()
{
  d_tmp1 = drop1;
  d_tmp2 = drop2;

  crsf.loop();
  CheckVbat();
  SendTelemetry();

  digitalWrite(LED_M_1, !digitalRead(SW_M_1));
  digitalWrite(LED_M_2, !digitalRead(SW_M_2));
  digitalWrite(LED_M_3, !digitalRead(SW_M_3));

  SpeedL = dir*Speed - tdir*Turn;
  dirL   = (SpeedL > 0) ? 0 : 1;
  SpeedL = constrain(abs(SpeedL), 0, 500);

  SpeedR = dir*Speed + tdir*Turn;
  dirR   = (SpeedR > 0) ? 0 : 1;
  SpeedR = constrain(abs(SpeedR), 0, 500);


  if(arm == 2 && failsafe == 0)
  {
    digitalWrite(LED_ARM, 1);
    digitalWrite(LED_CAM, cam_en);
    digitalWrite(CAM_EN,  !cam_en);

    if(gear == 0)
    {
      digitalWrite(GEAR_HL, 0);
      digitalWrite(GEAR_HR, 0);
      digitalWrite(GEAR_LL, 1);
      digitalWrite(GEAR_LR, 1);
    }
    if(gear == 1)
    {
      digitalWrite(GEAR_HL, 0);
      digitalWrite(GEAR_HR, 0);
      digitalWrite(GEAR_LL, 0);
      digitalWrite(GEAR_LR, 0);
    }
    if(gear == 2)
    {
      digitalWrite(GEAR_HL, 1);
      digitalWrite(GEAR_HR, 1);
      digitalWrite(GEAR_LL, 0);
      digitalWrite(GEAR_LR, 0);
    }
    
    if(SpeedL > 30)
    {
      analogWrite(THROT1, pwm_map_speed(SpeedL));
      digitalWrite(DIR_L, dirL);
      digitalWrite(FOOT_L, 1);
      digitalWrite(BRK_L,  0);
    }
    else
    {
      analogWrite(THROT1, 0);
      digitalWrite(DIR_L, dirL);
      digitalWrite(FOOT_L, 1);
      digitalWrite(BRK_L,  1);
    }
    
    if(SpeedR > 30)
    {
      analogWrite(THROT2, pwm_map_speed(SpeedR));
      digitalWrite(DIR_R, dirR);
      digitalWrite(FOOT_R, 1);
      digitalWrite(BRK_R,  0);
    }
    else
    {
      analogWrite(THROT2, 0);
      digitalWrite(DIR_R, dirR);
      digitalWrite(FOOT_R, 1);
      digitalWrite(BRK_R,  1);
    }
    

    if(drop_arm)
    {
      if(drop1 && !d_tmp1 && cur_servo < 22)
      {
        servo_states[cur_servo++] = 1;
        last_servoset += SERVO_PERIOD;
      }
      if(drop2 && !d_tmp2 && cur_servo > 0) 
      {
        servo_states[--cur_servo] = 0;
        last_servoset += SERVO_PERIOD;
      }
      set_all_servos();
    }    
  }

  if(arm == 1 || failsafe == 1)
  {
    digitalWrite(LED_ARM, 0);
    digitalWrite(LED_CAM, cam_en);
    digitalWrite(CAM_EN,  !cam_en);
    analogWrite(THROT1, pwm_map_speed(0));
    analogWrite(THROT2, pwm_map_speed(0));
    digitalWrite(FOOT_L, 0);
    digitalWrite(BRK_L,  1);
    digitalWrite(FOOT_R, 0);
    digitalWrite(BRK_R,  1);
  }

  if(arm == 0)
  {
    digitalWrite(LED_ARM, 0);
    digitalWrite(LED_CAM, 0);
    digitalWrite(CAM_EN,  1);
    analogWrite(THROT1, pwm_map_speed(0));
    analogWrite(THROT2, pwm_map_speed(0));
    digitalWrite(FOOT_L, 0);
    digitalWrite(BRK_L,  1);
    digitalWrite(FOOT_R, 0);
    digitalWrite(BRK_R,  1);
  }

  if(failsafe == 1 && millis() - idle_timer > IDLE_TIME) arm = 0;
  
}