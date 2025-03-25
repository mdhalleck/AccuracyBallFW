/******************************************************************************
*@file:         Accuracy_Ball.h
*@author:       Robert Banks
*@version:      0.9.1
*@date:         10/30/2015
*@breif:        
*******************************************************************************/

/**Includes********************************************************************/
#include "math.h"
#include "stm32xx_it.h"
#include "stm32f4xx_hal.h"
#include "H3LIS331DL.h"
#include "stm32_bluenrg_ble.h"
#include "osal.h"
#include <stdio.h>
#include <stdlib.h>

/**Defines*********************************************************************/

// Version Number
#define VERSION_NUMBER          "0.9.1"

// LEDs: PA.2,3,10
#define CONN_STATUS_PIN         GPIO_PIN_0
#define EVENT_PIN               GPIO_PIN_1
#define DATA_SENT_PIN           GPIO_PIN_2
#define GREEN_LED_PIN           GPIO_PIN_8
#define RED_LED_PIN             GPIO_PIN_13
#define LEDS_MODE               GPIO_MODE_IT_RISING
#define LEDS_PULL               GPIO_NOPULL
#define LEDS_SPEED              GPIO_SPEED_HIGH
#define LEDS_PORT               GPIOC
#define LEDS_CLK_ENABLE()       __GPIOC_CLK_ENABLE()

#define CHARGE_PIN              GPIO_PIN_7

// ADC: PA2
#define ADC_PIN         GPIO_PIN_2

// System Shutdown Time
#define TIME_OUT                (15 * 60 * 1000) // Time in milli-seconds

// Minumum Hit Magnutud in gs
#define MINIMUM_G               6

// Hit Debounce Time in milli-seconds
#define HIT_DEBOUNCE            50

//BLE
#define CONNECTED       1
#define DISCONNECTED    0
#define BLEMsg_MaxLen   512
#define BDADDR_SIZE     6
#define BLE_NAME        "TRIAD"
// Accelerometer
#define ACCELEROMETER_SENSOR                    0x00000010


/* Typedefs ------------------------------------------------------------------*/
#ifndef Accuracy_Ball_H
#define Accuracy_Ball_H
typedef struct
{
  uint16_t len;
  AxesRaw_TypeDef ACC_Value[BLEMsg_MaxLen / (2 * 8)];         /*!< Acceleration Value */
} BLEMsg;

  
typedef struct
{
  double magnitude;
  AxesRaw_TypeDef ACC_Value;
} AccelMag;
#endif

/**Public Variables************************************************************/
extern uint8_t bleConnection;
extern uint8_t batteryLevel;
extern BLEMsg message;

/* Prototypes-----------------------------------------------------------------*/
void ADC_Init();
void Read_Acc();
void Imu_6axes_Sensor_Handler(BLEMsg *Msg);
void Init_Accuracy_Ball();
void Init_Globals();
void Init_LEDs();
void Check_Peak(BLEMsg *Msg, int16_t x, int16_t y, int16_t z);
void Heart_Beat(uint32_t* heartBeatTimer, uint8_t* heartBeatToggle, uint32_t on_ms, uint32_t off_ms);
void Update_Battery();
void Init_BLE();
void Power_Control_Init();
void Shutdown();
void Check_for_Shutdown();
void SetRedLED(uint8_t state);
void SetGreenLED(uint8_t state);
uint8_t Check_for_Forced_Send();
uint8_t Check_for_Charge();



