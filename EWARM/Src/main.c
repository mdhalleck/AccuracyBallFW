/**Include Files***************************************************************/
#include "main.h"

/**Global Variabes**************************************************************/


/*******************************************************************************
* @Name: main
*
* @Type:        int
* @Arguments:   void
* @Returns:     int
* @Description: 
*******************************************************************************/
int main(void)
{
  uint32_t heartBeatTimer = 0;
  uint8_t heartBeatToggle = 0;
  
  Init_Accuracy_Ball();

  while(1)
  {
    if(!Check_for_Charge())
    {
      Update_Battery();
      if(set_connectable)
      {
        Update_Battery();
        setConnectable();
        set_connectable = FALSE;
      } 
      
      Heart_Beat(&heartBeatTimer, &heartBeatToggle, 4500, 500);
      
      Check_for_Shutdown();

      HCI_Process();
      HAL_Delay(1);
      Read_Acc();
    }
    else
    {
      Update_Battery();
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, (GPIO_PinState)0x00);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (GPIO_PinState)0x00);
      if(batteryLevel >= 95)
      {
        HAL_GPIO_WritePin(GPIOC, GREEN_LED_PIN, (GPIO_PinState)0x00);
        HAL_GPIO_WritePin(GPIOB, RED_LED_PIN, (GPIO_PinState)0x01);
      }
      else
      {
        HAL_GPIO_WritePin(GPIOC, GREEN_LED_PIN, (GPIO_PinState)0x00);
        HAL_GPIO_WritePin(GPIOB, RED_LED_PIN, (GPIO_PinState)0x00);
      }
    }
  }
}
