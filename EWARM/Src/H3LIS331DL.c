/*
 * H3LIS331DL.cpp
 * A library for 3-Axis Digital Accelerometer(±400g)
 *  
 * Copyright (c) 2014 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : lawliet zou
 * Create Time: April 2014
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
#include "H3LIS331DL.h"
#include "stdint.h"
#include "x_nucleo_iks01a1.h"

int16_t _adjVal[3];

void H3LIS331DL_init(H3LIS331DL_ODR_t  odr, H3LIS331DL_Mode_t mode, H3LIS331DL_Fullscale_t fullScale)
{
  IMU_6AXES_IO_Init();
  // Configure interrupts
  H3LIS331DL_IO_ITConfig();
  
  /* Configure Control Register 1 */
  //set output data rate
  H3LIS331DL_setODR(odr);
  //set PowerMode 
  H3LIS331DL_setMode( mode);
  //set axis Enable
  H3LIS331DL_setAxis( H3LIS331DL_X_ENABLE | H3LIS331DL_Y_ENABLE |  H3LIS331DL_Z_ENABLE);
  
  /* Configure Control Register 4 */
  //set Fullscale
  H3LIS331DL_setFullScale( fullScale);
  
  /* Configure Control Register 5 */
  // Disables Low Power Sleep Mode
  H3LIS331DL_turnONEnable(H3LIS331DL_SLEEP_TO_WAKE_DIS);
}

void H3LIS331DL_importPara(int16_t val_x, int16_t val_y, int16_t val_z)
{
    _adjVal[0] = val_x;
    _adjVal[1] = val_y;
    _adjVal[2] = val_z;
}
void H3LIS331DL_readXYZ(int16_t* x, int16_t* y, int16_t* z)
{
    //get Acceleration Raw data  
    AxesRaw_TypeDef data;
    status_t response = H3LIS331DL_getAccAxesRaw(&data);
    if(MEMS_SUCCESS == response){
        *x = (data.AXIS_X - _adjVal[0]);
        *y = (data.AXIS_Y - _adjVal[1]);
        *z = (data.AXIS_Z - _adjVal[2]);
    }
}

void H3LIS331DL_getAcceleration(double* xyz)
{
    AxesRaw_TypeDef data;
    double gains = 0.003;
    H3LIS331DL_getAccAxesRaw(&data);
    
    xyz[0] = (data.AXIS_X - _adjVal[0]) * gains;
    xyz[1] = (data.AXIS_Y - _adjVal[1]) * gains;
    xyz[2] = (data.AXIS_Z - _adjVal[2]) * gains;
}


/*******************************************************************************
* Function Name  : getWHO_AM_I
* Description    : Read identification code from H3LIS331DL_WHO_AM_I register
* Input          : char to be filled with the Device identification Value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t H3LIS331DL_getWHO_AM_I(uint8_t* val){
  
    if(IMU_6AXES_IO_Read(val, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_WHO_AM_I, 1))
        return MEMS_ERROR;
  
    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setODR
* Description    : Sets H3LIS331DL Accelerometer Output Data Rate 
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setODR(H3LIS331DL_ODR_t dr){
    uint8_t value;
    //IMU_6AXES_StatusTypeDef status = IMU_6AXES_IO_Read( &value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, 1);
    
    if(IMU_6AXES_IO_Read( &value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, 1))
    {
      return MEMS_ERROR;
    }
  
    value &= 0xE7;
    value |= dr<<H3LIS331DL_DR;
  
    if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, 1))
    {
      return MEMS_ERROR;
    }
  
    return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setMode
* Description    : Sets H3LIS331DLH Accelerometer Operating Mode
* Input          : Modality (H3LIS331DL_LOW_POWER, H3LIS331DL_NORMAL, H3LIS331DL_POWER_DOWN...)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setMode(H3LIS331DL_Mode_t pm) {
    uint8_t value;
  
    if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, 1) )
        return MEMS_ERROR;
  
    value &= 0x1F;
    value |= (pm<<H3LIS331DL_PM);   
  
    if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, 1) )
        return MEMS_ERROR;
  
    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : setAxis
* Description    : Enable/Disable LIS331DLH Axis
* Input          : H3LIS331DL_X_ENABLE/H3LIS331DL_X_DISABLE | H3LIS331DL_Y_ENABLE/H3LIS331DL_Y_DISABLE
                   | H3LIS331DL_Z_ENABLE/H3LIS331DL_Z_DISABLE
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setAxis(H3LIS331DL_Axis_t axis) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, 1) )
    return MEMS_ERROR;
  
  value &= 0xF8;
  value |= (0x07 & axis);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG1, 1) )
    return MEMS_ERROR;   
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setFullScale
* Description    : Sets the LIS331DLH FullScale
* Input          : H3LIS331DL_FULLSCALE_2/H3LIS331DL_FULLSCALE_4/H3LIS331DL_FULLSCALE_8
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setFullScale(H3LIS331DL_Fullscale_t fs) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, 1) )
    return MEMS_ERROR;
  
  value &= 0xCF;    
  value |= (fs<<H3LIS331DL_FS);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setBDU(State_t bdu) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, 1) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  value |= (bdu<<H3LIS331DL_BDU);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setBLE
* Description    : Set Endianess (MSB/LSB)
* Input          : H3LIS331DL_BLE_LSB / H3LIS331DL_BLE_MSB
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setBLE(H3LIS331DL_Endianess_t ble) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, 1) )
    return MEMS_ERROR;
  
  value &= 0xBF;    
  value |= (ble<<H3LIS331DL_BLE);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setFDS
* Description    : Set Filter Data Selection
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setFDS(State_t fds) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, 1) )
    return MEMS_ERROR;
  
  value &= 0xEF;    
  value |= (fds<<H3LIS331DL_FDS);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setBOOT
* Description    : Rebot memory content
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setBOOT(State_t boot) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, 1) )
    return MEMS_ERROR;
  
  value &= 0x7F;    
  value |= (boot<<H3LIS331DL_BOOT);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setSelfTest
* Description    : Set Self Test Modality
* Input          : MEMS_DISABLE/MEMS_ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setSelfTest(State_t st) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, 1) )
    return MEMS_ERROR;
  
  value &= 0xFD;
  value |= (st<<H3LIS331DL_ST);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setSelfTestSign
* Description    : Set Self Test Sign (Disable = st_plus, Enable = st_minus)
* Input          : MEMS_DISABLE/MEMS_ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setSelfTestSign(State_t st_sign) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, 1) )
    return MEMS_ERROR;
  
  value &= 0xF7;
  value |= (st_sign<<H3LIS331DL_ST_SIGN);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setIntHighLow
* Description    : Set Interrupt active state (Disable = active high, Enable = active low)
* Input          : MEMS_DISABLE/MEMS_ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setIntHighLow(State_t ihl) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, 1) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  value |= (ihl<<H3LIS331DL_IHL);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setIntPPOD
* Description    : Set Interrupt Push-Pull/OpenDrain Pad (Disable = Push-Pull, Enable = OpenDrain)
* Input          : MEMS_DISABLE/MEMS_ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setIntPPOD(State_t pp_od) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, 1) )
    return MEMS_ERROR;
  
  value &= 0xBF;
  value |= (pp_od<<H3LIS331DL_PP_OD);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setInt1DataSign
* Description    : Set Data signal Interrupt 1 pad
* Input          : Modality by H3LIS331DL_INT_Conf_t Typedef 
                  (H3LIS331DL_INT_SOURCE, H3LIS331DL_INT_1OR2_SOURCE, H3LIS331DL_DATA_READY, H3LIS331DL_BOOT_RUNNING)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setInt1DataSign(H3LIS331DL_INT_Conf_t i_cfg) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, 1) )
    return MEMS_ERROR;
  
  value &= 0xFC;
  value |= (i_cfg<<H3LIS331DL_I1_CFG);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setInt2DataSign
* Description    : Set Data signal Interrupt 2 pad
* Input          : Modality by H3LIS331DL_INT_Conf_t Typedef 
                  (H3LIS331DL_INT_SOURCE, H3LIS331DL_INT_1OR2_SOURCE, H3LIS331DL_DATA_READY, H3LIS331DL_BOOT_RUNNING)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setInt2DataSign(H3LIS331DL_INT_Conf_t i_cfg) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, 1) )
    return MEMS_ERROR;
  
  value &= 0xE7;
  value |= (i_cfg<<H3LIS331DL_I2_CFG);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setSPI34Wire
* Description    : Set SPI mode 
* Input          : Modality by H3LIS331DL_SPIMode_t Typedef (H3LIS331DL_SPI_4_WIRE, H3LIS331DL_SPI_3_WIRE)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setSPI34Wire(H3LIS331DL_SPIMode_t sim) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, 1) )
    return MEMS_ERROR;
  
  value &= 0xFE;
  value |= (sim<<H3LIS331DL_SIM);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG4, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : turnONEnable
* Description    : TurnON Mode selection for sleep to wake function
* Input          : H3LIS331DL_SLEEP_TO_WAKE_DIS/H3LIS331DL_SLEEP_TO_WAKE_ENA
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_turnONEnable(H3LIS331DL_Sleep_To_Wake_Conf_t stw) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG5, 1) )
    return MEMS_ERROR;
  
  value &= 0x00;
  value |= (stw<<H3LIS331DL_TURN_ON);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG5, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

#if 0
/*******************************************************************************
* Function Name  : HPFilterReset
* Description    : Reading register for reset the content of internal HP filter
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_HPFilterReset(void) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_HP_FILTER_RESET, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}
#endif

/*******************************************************************************
* Function Name  : setReference
* Description    : Sets Reference register acceleration value as a reference for HP filter
* Input          : Value of reference acceleration value (0-255)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setReference(uint8_t ref) {
  
  if(IMU_6AXES_IO_Write(&ref, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_REFERENCE_REG, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setHPFMode
* Description    : Set High Pass Filter Modality
* Input          : H3LIS331DL_HPM_NORMAL_MODE_RES/H3LIS331DL_HPM_REF_SIGNAL/H3LIS331DL_HPM_NORMAL_MODE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setHPFMode(H3LIS331DL_HPFMode_t hpm) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, 1) )
    return MEMS_ERROR;
  
  value &= 0x9F;
  value |= (hpm<<H3LIS331DL_HPM);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setHPFCutOFF
* Description    : Set High Pass CUT OFF Freq
* Input          : H3LIS331DL_HPFCF [0,3]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setHPFCutOFF(H3LIS331DL_HPFCutOffFreq_t hpf) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, 1) )
    return MEMS_ERROR;
  
  value &= 0xFC;
  value |= (hpf<<H3LIS331DL_HPCF);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : setInt2HPEnable
* Description    : Set Interrupt2 hp filter enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* example        : H3LIS331DL_SetInt2HPEnable(MEMS_ENABLE)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setInt2HPEnable(State_t stat) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, 1) )
    return MEMS_ERROR;
  
  value &= 0xF7;
  value |= stat<<H3LIS331DL_HPEN2 ;
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}     


/*******************************************************************************
* Function Name  : setInt1HPEnable
* Description    : Set Interrupt1 hp filter enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* example        : H3LIS331DL_SetInt1HPEnable(MEMS_ENABLE)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setInt1HPEnable(State_t stat) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, 1) )
    return MEMS_ERROR;
  
  value &= 0xFB;
  value |= stat<<H3LIS331DL_HPEN1 ;
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG2, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}  


/*******************************************************************************
* Function Name  : int1LatchEnable
* Description    : Enable Interrupt 1 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_int1LatchEnable(State_t latch) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, 1) )
    return MEMS_ERROR;
  
  value &= 0xFB;
  value |= latch<<H3LIS331DL_LIR1;
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : int2LatchEnable
* Description    : Enable Interrupt 2 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_int2LatchEnable(State_t latch) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, 1) )
    return MEMS_ERROR;
  
  value &= 0xDF;
  value |= latch<<H3LIS331DL_LIR2;
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_CTRL_REG3, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : resetInt1Latch
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_resetInt1Latch(void) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_SRC, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : resetInt2Latch
* Description    : Reset Interrupt 2 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_resetInt2Latch(void) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_SRC, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setInt1Configuration
* Description    : Interrupt 1 Configuration (without 6D_INT)
* Input          : H3LIS331DL_INT_AND/OR | H3LIS331DL_INT_ZHIE_ENABLE/DISABLE | H3LIS331DL_INT_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use ALL input variable in the argument, as in example above
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setInt1Configuration(H3LIS331DL_IntConf_t ic) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_CFG, 1) )
    return MEMS_ERROR;
  
  value &= 0x40; 
  value |= ic;
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_CFG, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setInt2Configuration
* Description    : Interrupt 2 Configuration (without 6D_INT)
* Input          : H3LIS331DL_INT_AND/OR | H3LIS331DL_INT_ZHIE_ENABLE/DISABLE | H3LIS331DL_INT_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setInt2Configuration(H3LIS331DL_IntConf_t ic) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_CFG, 1) )
    return MEMS_ERROR;
  
  value &= 0x40; 
  value |= ic;
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_CFG, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setInt1Mode
* Description    : Interrupt 1 Configuration mode (OR, 6D Movement, AND, 6D Position)
* Input          : H3LIS331DL_INT_MODE_OR, H3LIS331DL_INT_MODE_6D_MOVEMENT, H3LIS331DL_INT_MODE_AND, H3LIS331DL_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setInt1Mode(H3LIS331DL_IntMode_t int_mode) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_CFG, 1) )
    return MEMS_ERROR;
  
  value &= 0x3F; 
  value |= (int_mode<<H3LIS331DL_INT_6D);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_CFG, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setInt2Mode
* Description    : Interrupt 2 Configuration mode (OR, 6D Movement, AND, 6D Position)
* Input          : H3LIS331DL_INT_MODE_OR, H3LIS331DL_INT_MODE_6D_MOVEMENT, H3LIS331DL_INT_MODE_AND, H3LIS331DL_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setInt2Mode(H3LIS331DL_IntMode_t int_mode) {
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_CFG, 1) )
    return MEMS_ERROR;
  
  value &= 0x3F; 
  value |= (int_mode<<H3LIS331DL_INT_6D);
  
  if(IMU_6AXES_IO_Write(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_CFG, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : get6DPositionInt1
* Description    : 6D Interrupt 1 Position Detect
* Input          : uint8_t to be filled with H3LIS331DL_POSITION_6D_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_get6DPositionInt1(uint8_t* val){
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_SRC, 1) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  
  switch (value){
  case H3LIS331DL_UP_SX:   
    *val = H3LIS331DL_UP_SX;    
    break;
  case H3LIS331DL_UP_DX:   
    *val = H3LIS331DL_UP_DX;    
    break;
  case H3LIS331DL_DW_SX:   
    *val = H3LIS331DL_DW_SX;    
    break;
  case H3LIS331DL_DW_DX:   
    *val = H3LIS331DL_DW_DX;    
    break;
  case H3LIS331DL_TOP:     
    *val = H3LIS331DL_TOP;      
    break;
  case H3LIS331DL_BOTTOM:  
    *val = H3LIS331DL_BOTTOM;  
    break;
  }
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : get6DPositionInt2
* Description    : 6D Interrupt 2 Position Detect
* Input          : uint8_t to be filled with H3LIS331DL_POSITION_6D_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_get6DPositionInt2(uint8_t* val){
  uint8_t value;
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_SRC, 1) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  
  switch (value){
  case H3LIS331DL_UP_SX:   
    *val = H3LIS331DL_UP_SX;    
    break;
  case H3LIS331DL_UP_DX:   
    *val = H3LIS331DL_UP_DX;    
    break;
  case H3LIS331DL_DW_SX:   
    *val = H3LIS331DL_DW_SX;    
    break;
  case H3LIS331DL_DW_DX:   
    *val = H3LIS331DL_DW_DX;    
    break;
  case H3LIS331DL_TOP:     
    *val = H3LIS331DL_TOP;      
    break;
  case H3LIS331DL_BOTTOM:  
    *val = H3LIS331DL_BOTTOM;   
    break;
  }
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : setInt1Threshold
* Description    : Sets Interrupt 1 Threshold
* Input          : Threshold = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setInt1Threshold(uint8_t ths) {
  if (ths > 127)
    return MEMS_ERROR;
  
  if(IMU_6AXES_IO_Write(&ths, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_THS, 1) )
    return MEMS_ERROR;    
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setInt1Duration
* Description    : Sets Interrupt 1 Duration
* Input          : Duration = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setInt1Duration(uint8_t id) {  
  if (id > 127)
    return MEMS_ERROR;
  
  if(IMU_6AXES_IO_Write(&id, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_DURATION, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setInt2Threshold
* Description    : Sets Interrupt 2 Threshold
* Input          : Threshold = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setInt2Threshold(uint8_t ths) {
  if (ths > 127)
    return MEMS_ERROR;
  
  if(IMU_6AXES_IO_Write(&ths, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_THS, 1) )
    return MEMS_ERROR;    
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : setInt2Duration
* Description    : Sets Interrupt 2 Duration
* Input          : Duration = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_setInt2Duration(uint8_t id) {  
  if (id > 127)
    return MEMS_ERROR;
  
  if(IMU_6AXES_IO_Write(&id, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_DURATION, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : getStatusReg
* Description    : Read the status register
* Input          : char to empty by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_getStatusReg(uint8_t* val) {
  
  if(IMU_6AXES_IO_Read(val, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_STATUS_REG, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : getStatusBIT
* Description    : Read the status register BIT
* Input          : H3LIS331DL_STATUS_REG_ZYXOR, H3LIS331DL_STATUS_REG_ZOR, H3LIS331DL_STATUS_REG_YOR, H3LIS331DL_STATUS_REG_XOR,
                   H3LIS331DL_STATUS_REG_ZYXDA, H3LIS331DL_STATUS_REG_ZDA, H3LIS331DL_STATUS_REG_YDA, H3LIS331DL_STATUS_REG_XDA, 
                   H3LIS331DL_DATAREADY_BIT
* Output         : status register BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_getStatusBit(uint8_t statusBIT, uint8_t *val) {
  uint8_t value;  
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_STATUS_REG, 1) )
    return MEMS_ERROR;
  
  switch (statusBIT){
  case H3LIS331DL_STATUS_REG_ZYXOR:     
    if(value &= H3LIS331DL_STATUS_REG_ZYXOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  case H3LIS331DL_STATUS_REG_ZOR:       
    if(value &= H3LIS331DL_STATUS_REG_ZOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  case H3LIS331DL_STATUS_REG_YOR:       
    if(value &= H3LIS331DL_STATUS_REG_YOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }                                 
  case H3LIS331DL_STATUS_REG_XOR:       
    if(value &= H3LIS331DL_STATUS_REG_XOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  case H3LIS331DL_STATUS_REG_ZYXDA:     
    if(value &= H3LIS331DL_STATUS_REG_ZYXDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case H3LIS331DL_STATUS_REG_ZDA:       
    if(value &= H3LIS331DL_STATUS_REG_ZDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case H3LIS331DL_STATUS_REG_YDA:       
    if(value &= H3LIS331DL_STATUS_REG_YDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case H3LIS331DL_STATUS_REG_XDA:       
    if(value &= H3LIS331DL_STATUS_REG_XDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }                                      
  }
  
  return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : getAccAxesRaw
* Description    : Read the Acceleration Values Output Registers
* Input          : buffer to empty by AccAxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_getAccAxesRaw(AxesRaw_TypeDef* buff) {
    uint8_t valueL = 0,valueH = 0;
    IMU_6AXES_IO_Read(&valueL, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_OUT_X_L, 1);
    IMU_6AXES_IO_Read(&valueH, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_OUT_X_H, 1);
    buff->AXIS_X = (valueH<<8)|valueL;
    IMU_6AXES_IO_Read(&valueL, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_OUT_Y_L, 1);
    IMU_6AXES_IO_Read(&valueH, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_OUT_Y_H, 1);
    buff->AXIS_Y = (valueH<<8)|valueL;
    IMU_6AXES_IO_Read(&valueL, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_OUT_Z_L, 1);
    IMU_6AXES_IO_Read(&valueH, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_OUT_Z_H, 1);
    buff->AXIS_Z = (valueH<<8)|valueL;
    return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : getInt1Src
* Description    : Reset Interrupt 1 Latching function
* Input          : buffer to empty by Int1 Source Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_getInt1Src(uint8_t* val) {  
  if(IMU_6AXES_IO_Read(val, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_SRC, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : getInt2Src
* Description    : Reset Interrupt 2 Latching function
* Input          : buffer to empty by Int2 Source Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_getInt2Src(uint8_t* val) {  
  if(IMU_6AXES_IO_Read(val, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_SRC, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : getInt1SrcBit
* Description    : Reset Interrupt 1 Latching function
* Input          : H3LIS331DL_INT1_SRC_IA, H3LIS331DL_INT1_SRC_ZH, H3LIS331DL_INT1_SRC_ZL .....
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_getInt1SrcBit(uint8_t statusBIT, uint8_t *val) {
  uint8_t value;  
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT1_SRC, 1) )
    return MEMS_ERROR;
  
  if(statusBIT == H3LIS331DL_INT_SRC_IA){
    if(value &= H3LIS331DL_INT_SRC_IA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }    
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_ZH){
    if(value &= H3LIS331DL_INT_SRC_ZH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_ZL){
    if(value &= H3LIS331DL_INT_SRC_ZL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_YH){
    if(value &= H3LIS331DL_INT_SRC_YH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_YL){
    if(value &= H3LIS331DL_INT_SRC_YL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_XH){
    if(value &= H3LIS331DL_INT_SRC_XH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_XL){
    if(value &= H3LIS331DL_INT_SRC_XL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }    
  } 
  return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : getInt2SrcBit
* Description    : Reset Interrupt 2 Latching function
* Input          : H3LIS331DL_INT_SRC_IA, H3LIS331DL_INT_SRC_ZH, H3LIS331DL_INT_SRC_ZL .....
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_getInt2SrcBit(uint8_t statusBIT, uint8_t *val) {
  uint8_t value;  
  
  if(IMU_6AXES_IO_Read(&value, H3LIS331DL_MEMS_I2C_ADDRESS, H3LIS331DL_INT2_SRC, 1) )
    return MEMS_ERROR;
  
  if(statusBIT == H3LIS331DL_INT_SRC_IA){
    if(value &= H3LIS331DL_INT_SRC_IA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_ZH){
    if(value &= H3LIS331DL_INT_SRC_ZH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_ZL){
    if(value &= H3LIS331DL_INT_SRC_ZL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_YH){
    if(value &= H3LIS331DL_INT_SRC_YH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }    
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_YL){
    if(value &= H3LIS331DL_INT_SRC_YL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_XH){
    if(value &= H3LIS331DL_INT_SRC_XH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_XL){
    if(value &= H3LIS331DL_INT_SRC_XL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }    
  } 
  return MEMS_ERROR;
}
        
        
        