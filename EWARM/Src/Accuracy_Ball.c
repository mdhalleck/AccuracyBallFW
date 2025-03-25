/******************************************************************************
*@file:         Accuracy_Ball.h
*@author:       Robert Banks
*@version:      0.9.1
*@date:         10/30/2015
*@breif:        
*******************************************************************************/

/* Include Files--------------------------------------------------------------*/

#include "Accuracy_Ball.h"

/* Private Variables----------------------------------------------------------*/
BLEMsg message;
AccelMag peakMag;
uint8_t peakFlag = 0;
uint32_t debounceTimmer = 0;
ADC_HandleTypeDef ADC_HandleStructure;
AccelMag previousMag;
uint32_t shutDownTimer = TIME_OUT;

/* Global Variables-----------------------------------------------------------*/
uint8_t bleConnection;
uint8_t batteryLevel;

/* Battery Functions----------------------------------------------------------*/

/*******************************************************************************
* @Name: ADC_Init
*
* @Type:        void
* @Arguments:   void
* @Returns:     void
* @Description: 
*******************************************************************************/
void ADC_Init()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_ChannelConfTypeDef adcChannel;
  
  __ADC1_CLK_ENABLE();
  
  GPIO_InitStructure.Pin = ADC_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  ADC_HandleStructure.Instance = ADC1;
  ADC_HandleStructure.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  ADC_HandleStructure.Init.Resolution = ADC_RESOLUTION_12B;
  ADC_HandleStructure.Init.ScanConvMode = DISABLE;
  ADC_HandleStructure.Init.ContinuousConvMode = ENABLE;
  ADC_HandleStructure.Init.DiscontinuousConvMode = DISABLE;
  ADC_HandleStructure.Init.NbrOfDiscConversion = 1;
  ADC_HandleStructure.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  ADC_HandleStructure.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  ADC_HandleStructure.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  ADC_HandleStructure.Init.NbrOfConversion = 1;
  ADC_HandleStructure.Init.DMAContinuousRequests = ENABLE;
  ADC_HandleStructure.Init.EOCSelection = DISABLE;
  
  HAL_ADC_Init(&ADC_HandleStructure);
  
  adcChannel.Channel = ADC_CHANNEL_2;
  adcChannel.Rank = 1;
  adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  adcChannel.Offset = 0;

  HAL_ADC_ConfigChannel(&ADC_HandleStructure, &adcChannel);
  HAL_ADC_Start(&ADC_HandleStructure);
  HAL_Delay(1);
  
  GPIO_InitStructure.Pin = GPIO_PIN_3;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Speed = LEDS_SPEED;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/* Accelerometer Functions----------------------------------------------------*/

/*******************************************************************************
* @Name: Read_ACC
*
* @Type:        void
* @Arguments:   void
* @Returns:     void
* @Description: Checks to make sure the acceleromet is ready to be read and
*               then reads the accelerometr data storing it to message.
*******************************************************************************/
void Read_Acc()
{
  uint8_t status;
  H3LIS331DL_getStatusReg(&status);
  if(((status & 0x08) >> 3) == 1)       // Makes sure the Accelerometer is ready
  {
    // Disable Data from being updated
    H3LIS331DL_setBDU(MEMS_ENABLE);
    Imu_6axes_Sensor_Handler(&message);
    // Enable Data to be updated
    H3LIS331DL_setBDU(MEMS_DISABLE);
    // Clear Accelerometer Interrupt
    H3LIS331DL_getInt1SrcBit(0x01, &status);
  }
}

/*******************************************************************************
* @Name: IMU_6axes_Sensor_Handler
*
* @Type:        void
* @Arguments:   BLEMsg *Msg
* @Returns:     void
* @Description: Reads the x, y, & Z data of the accelerometer then calls
*               Check_Peak
*******************************************************************************/
void Imu_6axes_Sensor_Handler(BLEMsg *Msg)
{
  int16_t x;
  int16_t y;
  int16_t z;
  
  H3LIS331DL_readXYZ(&x, &y, &z);
  Check_Peak(Msg, x, y, z);
}

/* Main Functions-------------------------------------------------------------*/

void Init_Globals()
{
  previousMag.ACC_Value.AXIS_X = 0;
  previousMag.ACC_Value.AXIS_Y = 0;
  previousMag.ACC_Value.AXIS_Z = 0;
  previousMag.magnitude = 0;
  
  peakMag.ACC_Value.AXIS_X = 0;
  peakMag.ACC_Value.AXIS_Y = 0;
  peakMag.ACC_Value.AXIS_Z = 0;
  peakMag.magnitude = 0;
  
  memset(&message.ACC_Value[0], 0, message.len * 3);
}

/*******************************************************************************
* @Name: Init_Accuracy_Ball
*
* @Type:        void
* @Arguments:   void
* @Returns:     void
* @Description: 
*******************************************************************************/
void Init_Accuracy_Ball()
{
  /* Configure the system clock */
  SystemClock_Config();
  
  Power_Control_Init();
  
  Init_Globals();
  
  
  /* STM32Cube HAL library initialization:
   *  - Configure the Flash prefetch, Flash preread and Buffer caches
   *  - Systick timer is configured by default as source of time base, but user 
   *    can eventually implement his proper time base source (a general purpose 
   *    timer for example or other time source), keeping in mind that Time base 
   *    duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
   *    handled in milliseconds basis.
   *  - Low Level Initialization
   */
  HAL_Init();
  
  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();
  
  /* Initialize the BlueNRG HCI */
  HCI_Init();
  
  /* Reset BlueNRG hardware */
  BlueNRG_RST();
  
  /* Initialize the ADC */
  ADC_Init();
  
  /* Initialize the BLE Connection */
  Init_BLE();
  
  /* Initializes the Accelerometer */
  H3LIS331DL_ODR_t odr = H3LIS331DL_ODR_1000Hz;
  H3LIS331DL_Mode_t mode = H3LIS331DL_NORMAL;
  H3LIS331DL_Fullscale_t fullscale = H3LIS331DL_FULLSCALE_2;
  H3LIS331DL_init(odr, mode, fullscale);
  
  Init_LEDs();
}

/*******************************************************************************
* @Name: Init_LEDs
*
* @Type:        void
* @Arguments:   void
* @Returns:     void
* @Description: 
*******************************************************************************/
void Init_LEDs()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  uint32_t start_tick = ms_counter + 1000;
  uint8_t led_toggle = 0;
  
  LEDS_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  
  /* Pull IRQ high */
  GPIO_InitStructure.Pin = CONN_STATUS_PIN | EVENT_PIN | DATA_SENT_PIN | GREEN_LED_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = LEDS_SPEED;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LEDS_PORT, &GPIO_InitStructure);
  HAL_GPIO_WritePin(LEDS_PORT, CONN_STATUS_PIN | EVENT_PIN | DATA_SENT_PIN | GREEN_LED_PIN, GPIO_PIN_RESET);
  
  GPIO_InitStructure.Pin = RED_LED_PIN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  HAL_GPIO_WritePin(GPIOB, RED_LED_PIN, GPIO_PIN_SET);
  
  while(start_tick > ms_counter)
  {
    led_toggle = (~led_toggle) & 0x01;
    HAL_GPIO_WritePin(GPIOC, GREEN_LED_PIN, (GPIO_PinState)led_toggle);
    HAL_Delay(50);
  }
  HAL_GPIO_WritePin(GPIOC, GREEN_LED_PIN, GPIO_PIN_SET);
}

/*******************************************************************************
* @Name: Check_Peak
*
* @Type:        void
* @Arguments:   BLEMsg *Msg, int16_t x, int16_t y, int16_t z
* @Returns:     void
* @Description: Calculates the magnitude of the accelerometer data. Stores the
*               maximum value in peakMag. If the magnitude goes above the
*               magThreshold and then bellow the threshold a valid peak is
*               detected and the hit is stored into Msg. The BLE accelerometer
*               service update (Acc_Update()) is then called. All peaks ocurring
*               less than 50ms after a peak are ignored.
*******************************************************************************/
void Check_Peak(BLEMsg *Msg, int16_t x, int16_t y, int16_t z)
{
  uint16_t scale = 100;
  float xValue, yValue, zValue, magnitude;
  
  xValue = (x / (65536.0 / 2.0)) * scale;
  yValue = (y / (65536.0 / 2.0)) * scale;
  zValue = (z / (65536.0 / 2.0)) * scale;
  
  magnitude = sqrt(pow(xValue, 2) + pow(yValue, 2) + pow(zValue, 2));
  
  if((magnitude > peakMag.magnitude && magnitude > MINIMUM_G) || Check_for_Forced_Send())
  {
    peakMag.magnitude =  magnitude;
    peakMag.ACC_Value.AXIS_X = x;
    peakMag.ACC_Value.AXIS_Y = y;
    peakMag.ACC_Value.AXIS_Z = z;
    peakFlag = 1;
  }
  if((peakFlag == 1 && magnitude <= MINIMUM_G) || Check_for_Forced_Send())
  {
    if(debounceTimmer < ms_counter)
    {
      Msg->ACC_Value[Msg->len].AXIS_X = peakMag.ACC_Value.AXIS_X;
      Msg->ACC_Value[Msg->len].AXIS_Y = peakMag.ACC_Value.AXIS_Y;
      Msg->ACC_Value[Msg->len].AXIS_Z = peakMag.ACC_Value.AXIS_Z;
      Msg->len++;
      
      peakMag.ACC_Value.AXIS_X = 0;
      peakMag.ACC_Value.AXIS_Y = 0;
      peakMag.ACC_Value.AXIS_Z = 0;
      peakMag.magnitude = 0;
      
      peakFlag = 0;
      Acc_Update(Msg);
      debounceTimmer = ms_counter + HIT_DEBOUNCE;
      if(bleConnection == CONNECTED)
      {
        shutDownTimer = ms_counter + TIME_OUT;
      }
    }
    else
    {
      peakFlag = 0;
    }
  }
  else if(magnitude <= MINIMUM_G  && debounceTimmer < ms_counter)
  { 
    peakMag.magnitude =  0;
    peakMag.ACC_Value.AXIS_X = 0;
    peakMag.ACC_Value.AXIS_Y = 0;
    peakMag.ACC_Value.AXIS_Z = 0;
  }
}

/*******************************************************************************
* @Name: Heart_Beat
*
* @Type:        void
* @Arguments:   void
* @Returns:     void
* @Description: 
*******************************************************************************/
void Heart_Beat(uint32_t* heartBeatTimer, uint8_t* heartBeatToggle, uint32_t on_ms, uint32_t off_ms)
{
  if(*heartBeatTimer < ms_counter)
  {
    if(((*heartBeatToggle) & 0x01) == 0)
    {
      Update_Battery();
      *heartBeatTimer = ms_counter + off_ms;
    }
    else
    {
      *heartBeatTimer = ms_counter + on_ms;
      if(bleConnection == DISCONNECTED)
      {
        setConnectable();
      }
    }
    if(bleConnection == CONNECTED)
    {
      HAL_GPIO_WritePin(GPIOC, GREEN_LED_PIN, (GPIO_PinState)((*heartBeatToggle) & 0x01));
      HAL_GPIO_WritePin(GPIOB, RED_LED_PIN, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, RED_LED_PIN, (GPIO_PinState)((*heartBeatToggle) & 0x01));
      HAL_GPIO_WritePin(GPIOC, GREEN_LED_PIN, GPIO_PIN_SET);
    }
    (*heartBeatToggle)++;
  }
}

/*******************************************************************************
* @Name: Update_Battery
*
* @Type:        void
* @Arguments:   void
* @Returns:     void
* @Description: 
*******************************************************************************/
void Update_Battery()
{
  uint32_t adcValue;
  double batteryVoltage = 100;
  double templevel = 0;
  
  adcValue = HAL_ADC_GetValue(&ADC_HandleStructure);


  for(uint8_t i = 0; i < 200; i++)
  {
    adcValue = adcValue + HAL_ADC_GetValue(&ADC_HandleStructure);
  }

  batteryVoltage = adcValue / 200.0;

  batteryVoltage = (double)(3.3 * (batteryVoltage));
  batteryVoltage = batteryVoltage / 4096.0;
  
  if(batteryVoltage <= 3.375 / 2.0)
  {
    Shutdown();
  }
  
  if(batteryVoltage >= 4.05 / 2)
  {
    templevel = 100;
  }
  else
  {
    templevel = (batteryVoltage - 3.375 / 2) * 275.1;
  }

  if(templevel <= batteryLevel)
  {
    batteryLevel = batteryLevel - 1;
  }
  else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 1 && templevel >= batteryLevel && batteryLevel!= 0)
  {
    batteryLevel = batteryLevel + 1;
  }
  else if(batteryLevel ==0)
  {
    batteryLevel = (uint8_t)templevel;
  }
  if(bleConnection == CONNECTED)
  {
    Batt_Update((uint8_t)batteryLevel);
  }
}

/* BLE Functions--------------------------------------------------------------*/
/*******************************************************************************
* @Name: Init_BLE
*
* @Type:        void
* @Arguments:   void
* @Returns:     void
* @Description: 
*******************************************************************************/
void Init_BLE()
{
  uint32_t bleAddress;
  int ret;
  char randStr[2];
  
  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 1)
  {
      srand(ms_counter);
      if(ms_counter >= 250)
      {
        break;
      }
  }
  bleAddress = rand();
  
  sprintf(&randStr[0], "%d", (uint8_t)bleAddress - 1);

  char name[] = {'A','C','C',randStr[0], randStr[1]};
  
  //This avoid to clear the cache on some Android device
#ifdef BLUENRG_MS
  uint8_t SERVER_BDADDR[] = {0x12, 0x34, (uint8_t)(bleAddress >> 24), (uint8_t)(bleAddress >> 16), (uint8_t)(bleAddress >> 8), (uint8_t)(bleAddress >> 0)};
#else
  uint8_t SERVER_BDADDR[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x03};
#endif
  
  uint8_t bdaddr[BDADDR_SIZE];
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  
  /* The Nucleo board must be configured as SERVER */
  Osal_MemCpy(bdaddr, SERVER_BDADDR, sizeof(SERVER_BDADDR));
  
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if(ret){
    PRINTF("Setting BD_ADDR failed.\n");
  }
  
  ret = aci_gatt_init();    
  if(ret){
    PRINTF("GATT_Init failed.\n");
  }

#ifdef BLUENRG_MS
        ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
#else
        ret = aci_gap_init(GAP_PERIPHERAL_ROLE, &service_handle, &dev_name_char_handle, &appearance_char_handle);
#endif

  if(ret != BLE_STATUS_SUCCESS){
    PRINTF("GAP_Init failed.\n");
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   strlen(name), (uint8_t *)name);

  if(ret)
  {
    PRINTF("aci_gatt_update_char_value failed.\n");            
    while(1);
  }
  
  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     BONDING);
  if (ret == BLE_STATUS_SUCCESS) {
    PRINTF("BLE Stack Initialized.\n");
  }
  
  PRINTF("SERVER: BLE Stack Initialized\n");
  
  
  ret = Add_Batt_Service();
  
  if(ret == BLE_STATUS_SUCCESS)
    PRINTF("Batt service added successfully.\n");
  else
    PRINTF("Error while adding Batt service.\n");
  
  
  ret = Add_Acc_Service();
  
  if(ret == BLE_STATUS_SUCCESS)
    PRINTF("Acc service added successfully.\n");
  else
    PRINTF("Error while adding Acc service.\n");

  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);
}

/* Power Functions------------------------------------------------------------*/

/*******************************************************************************
* @Name: Power_Control_Init
*
* @Type:        void
* @Arguments:   void
* @Returns:     void
* @Description: 
*******************************************************************************/
void Power_Control_Init()
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  LEDS_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /* Pull IRQ high */
  GPIO_InitStructure.Pin = GPIO_PIN_10;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = GPIO_PIN_12;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = GPIO_PIN_10;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

/*******************************************************************************
* @Name: Shutdown
*
* @Type:        void
* @Arguments:   void
* @Returns:     void
* @Description: Blinks the LED button red 10 times in 1 second and then shuts
*               off the device
*******************************************************************************/
void Shutdown()
{
  uint8_t led_toggle = 0;
  uint32_t stop_tick = ms_counter + 1000;
  while(stop_tick > ms_counter)
  {
    led_toggle = (~led_toggle) & 0x01;
    HAL_GPIO_WritePin(GPIOB, RED_LED_PIN, (GPIO_PinState)led_toggle);
    HAL_Delay(50);
  }
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
}

/*******************************************************************************
* @Name: Check_for_Shutdown
*
* @Type:        void
* @Arguments:   uint32* shutDownTimer
* @Returns:     void
* @Description: Updated MdH for press/hold for power off.
*******************************************************************************/
void Check_for_Shutdown()
{
  int delay = 100;
  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 0 /*&& HAL_GPIO_ReadPin(GPIOC, CHARGE_PIN) == 1*/)
  {
    // Press and hold
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 0)
    {
      HAL_Delay(10);
      if (--delay <= 0)
      {
        Shutdown();
      }
    }
  }
  
  if(shutDownTimer < ms_counter && HAL_GPIO_ReadPin(GPIOC, CHARGE_PIN) == 1)
  {
    Shutdown();
  }
}

/*******************************************************************************
* @Name: Check_for_Shutdown
*
* @Type:        void
* @Arguments:   uint32* shutDownTimer
* @Returns:     void
* @Description: 
*******************************************************************************/
uint8_t Check_for_Forced_Send()
{
  uint8_t ret = 0;
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0)
  {
    ret = 1;
  }
  return ret;
}

uint8_t Check_for_Charge()
{
  uint8_t ret = 0;
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 1)
  {
    ret = 1;
  }
  return ret;
}















