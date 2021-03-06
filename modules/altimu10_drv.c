/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright 
 *
 *    File name   : altimu_drv.c
 *    Description : Gyro, Accelerometer, Compass, and Altimeter (LSM6DS33, LIS3MDL, and LPS25H Carrier driver source file
 *    Link1 : http://www.st.com/content/ccc/resource/technical/document/datasheet/ed/aa/b8/e5/ab/3a/41/df/DM00157718.pdf/files/DM00157718.pdf/jcr:content/translations/en.DM00157718.pdf
 *    Link2 : http://www.st.com/content/ccc/resource/technical/document/datasheet/54/2a/85/76/e3/97/42/18/DM00075867.pdf/files/DM00075867.pdf/jcr:content/translations/en.DM00075867.pdf
 *    Link3 : http://www.st.com/content/ccc/resource/technical/document/datasheet/58/d2/33/a4/42/89/42/0b/DM00066332.pdf/files/DM00066332.pdf/jcr:content/translations/en.DM00066332.pdf
 *
 *    History :
 *    1. Date        : January 12, 2017
 *       Author      : Vakkas Celik
 *       Description : Create
 *
 **************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "altimu10_drv.h"
#include "i2c1_drv.h"


/* Private typedef -----------------------------------------------------------*/
typedef enum _Gyro_Accl_regs_t
{
  WHO_AM_I_GA = 0xF,
  CTRL1_XL,CTRL2_G,CTRL3_C,CTRL4_C,CTRL5_C,CTRL6_C,CTRL7_G,CTRL8_XL,CTRL9_XL,CTRL10_C,
  STATUS_GA_REG = 0x1E,
  OUT_TEMP_L = 0x20,OUT_TEMP_H,
  OUTX_L_G,OUTX_H_G,OUTY_L_G,OUTY_H_G,OUTZ_L_G,OUTZ_H_G,
  OUTX_L_XL,OUTX_H_XL,OUTY_L_XL,OUTY_H_XL,OUTZ_L_XL,OUTZ_H_XL
} Gyro_Accl_regs_t;

typedef enum _Magn_Get_regs_t
{
  WHO_AM_I_M = 0xF,
  CTRL_REG1 = 0x20, CTRL_REG2, CTRL_REG3, CTRL_REG4, CTRL_REG5,
  STATUS_M_REG = 0x27, OUTX_L, OUTX_H, OUTY_L, OUTY_H, OUTZ_L, OUTZ_H,  TEMP_OUT_L, TEMP_OUT_H,
  FF_WU_CFG = 0x30, FF_WU_SRC, FF_WU_THS_L, FF_WU_THS_H
} Magn_Get_regs_t;

typedef enum _Press_regs_t
{
  WHO_AM_I_P = 0xF,
  CTRL_P_REG1 = 0x20, CTRL_P_REG2, CTRL_P_REG3, CTRL_P_REG4, INT_CFG, INT_SOURCE,
  STATUS_P_REG = 0x27, 
  PRESS_OUT_XL,PRESS_OUT_L,PRESS_OUT_H,TEMP_OUT_P_L,TEMP_OUT_P_H
} Press_regs_t;



/*************************************************************************
 * Function Name: Altimu10_Init
 * Parameters: none
 *
 * Return: Boolean
 *
 * Description: Init Accelerometer sensor
 *
 *************************************************************************/
Boolean Altimu10_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
u8 Data[2];

  // Enable GPIOF port
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
  RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOF, DISABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

  if(FALSE == I2C1_Open())
  {
    return(FALSE);
  }
   
  
  ////////////////////////////// Gyro + Accelerometer Sensor Initialization ////////////////////
  // Get Accl sensor ID
  Data[0] = WHO_AM_I_GA;
  if(FALSE == I2C1_DataTransfer(GYRO_ACC_SENSOR_ADDR, Data, 1))
  {
    I2C1_Close();
    return(FALSE);
  }
  if(FALSE == I2C1_DataTransfer(GYRO_ACC_SENSOR_ADDR | 1, Data, 1))
  {
    I2C1_Close();
    return(FALSE);
  }
  if(0x69 != Data[0])
  {
    I2C1_Close();
    return(FALSE);
  }

  // Sensor configure
  Data[0] = CTRL1_XL;
  Data[1] = 0x50; // acceleration sensor: 208 Hz (normal mode) ,  �2 g,  400 Hz Anti-aliasing filter bandwidth
  if(FALSE == I2C1_DataTransfer(GYRO_ACC_SENSOR_ADDR, Data, 2))
  {
    I2C1_Close();
    return(FALSE);
  }

  Data[0] = CTRL2_G;
  Data[1] = 0x50; // Angular rate sensor: 208 Hz (normal mode) , 00: 245 dps
  if(FALSE == I2C1_DataTransfer(GYRO_ACC_SENSOR_ADDR, Data, 2))
  {
    I2C1_Close();
    return(FALSE);
  }
  
  Data[0] = CTRL9_XL;
  Data[1] = 0x38; // Accelerometer X-axis, Y-axis and Z-axis output enable
  if(FALSE == I2C1_DataTransfer(GYRO_ACC_SENSOR_ADDR, Data, 2))
  {
    I2C1_Close();
    return(FALSE);
  }
  
  Data[0] = CTRL10_C;
  Data[1] = 0x38; // Gyroscope yaw axis (Z),  roll axis (Y) ,  pitch axis (X) output enable, Enable embedded functionalities, Reset pedometer step counter, Enable significant motion function
  if(FALSE == I2C1_DataTransfer(GYRO_ACC_SENSOR_ADDR, Data, 2))
  {
    I2C1_Close();
    return(FALSE);
  }
  
  ////////////////////////////// Magnetometer Sensor Initialization /////////////////////////////////////////////////////////////////////
   Data[0] = WHO_AM_I_M;
  if(FALSE == I2C1_DataTransfer(MAGN_SENSOR_ADDR, Data, 1))
  {
    I2C1_Close();
    return(FALSE);
  }
  if(FALSE == I2C1_DataTransfer(MAGN_SENSOR_ADDR | 1, Data, 1))
  {
    I2C1_Close();
    return(FALSE);
  }
  if(0x3D != Data[0])
  {
    I2C1_Close();
    return(FALSE);
  }

  // Sensor configure
  Data[0] = CTRL_REG1;
  Data[1] = 0xD0; // Temperature sensor enable, X and Y axes operative  High-performance mode, Output data rate 10 Hz
  if(FALSE == I2C1_DataTransfer(MAGN_SENSOR_ADDR, Data, 2))
  {
    I2C1_Close();
    return(FALSE);
  }

  Data[0] = CTRL_REG3;
  Data[1] = 0x00; // Continuous-conversion mode
  if(FALSE == I2C1_DataTransfer(MAGN_SENSOR_ADDR, Data, 2))
  {
    I2C1_Close();
    return(FALSE);
  }
  
  Data[0] = CTRL_REG4;
  Data[1] = 0x04; // Z-axis Medium-performance mode
  if(FALSE == I2C1_DataTransfer(MAGN_SENSOR_ADDR, Data, 2))
  {
    I2C1_Close();
    return(FALSE);
  }
  
  
  ////////////////////////////// Pressure Sensor Initialization ////////////////////
   Data[0] = WHO_AM_I_P;
  if(FALSE == I2C1_DataTransfer(PRESS_SENSOR_ADDR, Data, 1))
  {
    I2C1_Close();
    return(FALSE);
  }
  if(FALSE == I2C1_DataTransfer(PRESS_SENSOR_ADDR | 1, Data, 1))
  {
    I2C1_Close();
    return(FALSE);
  }
  if(0xBD != Data[0])
  {
    I2C1_Close();
    return(FALSE);
  }

  // Sensor configure
  Data[0] = CTRL_P_REG1;
  Data[1] = 0xA4; //  The device is actived, output data rate 7 Hz, block data update enabled
  if(FALSE == I2C1_DataTransfer(PRESS_SENSOR_ADDR, Data, 2))
  {
    I2C1_Close();
    return(FALSE);
  }
  
  /////// Close I2C
  
  I2C1_Close();
  return(TRUE);
}

/*************************************************************************
 * Function Name: Gyro_Accl_Get
 * Parameters: pInt16S gX, pInt16S gY, pInt16S gZ, pInt16S aX, pInt16S aY, pInt16S aZ
 *
 * Return: Boolean
 *
 * Description: Read Gyro X,Y,Z data - Read Acclerometer X,Y,Z data
 *
 *************************************************************************/
Boolean Gyro_Accl_Get (pInt16S gX, pInt16S gY, pInt16S gZ, pInt16S aX, pInt16S aY, pInt16S aZ)
{
#pragma pack(1)
union
{
  u8 Data[12];
  struct
  {
    s16 gyX;
    s16 gyY;
    s16 gyZ;
    s16 acX;
    s16 acY;
    s16 acZ;
  };
} OutData;
#pragma pack()

 /* if(Bit_RESET == GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_10))
  {
    return(FALSE);
  }
 */ 
  if(FALSE == I2C1_Open())
  {
    return(FALSE);
  }

  // Get Gyro-Accl sensor ID
  OutData.Data[0] = OUTX_L_G; // read multiple bytes
  if(FALSE == I2C1_DataTransfer(GYRO_ACC_SENSOR_ADDR, OutData.Data, 1))
  {
    I2C1_Close();
    return(FALSE);
  }
  if(FALSE == I2C1_DataTransfer(GYRO_ACC_SENSOR_ADDR | 1, OutData.Data, 12))
  {
    I2C1_Close();
    return(FALSE);
  }
  I2C1_Close();

  *gX = OutData.gyX;
  *gY = OutData.gyY;
  *gZ = OutData.gyZ;
  *aX = OutData.acX;
  *aY = OutData.acY;
  *aZ = OutData.acZ;
  return(TRUE);
}


/*************************************************************************
 * Function Name: Magn_Get
 * Parameters: pInt16S mX, pInt16S mY, pInt16S mZ
 *
 * Return: Boolean
 *
 * Description: Read Magnetometer X,Y,Z data 
 *
 *************************************************************************/
Boolean Magn_Get (pInt16S mX, pInt16S mY, pInt16S mZ)
{
#pragma pack(1)
union
{
  u8 Data[6];
  struct
  {
    s16 maX;
    s16 maY;
    s16 maZ;
  };
} OutData;
#pragma pack()

/*  if(Bit_RESET == GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_10))
  {
    return(FALSE);
  }
 */ 
  if(FALSE == I2C1_Open())
  {
    return(FALSE);
  }

  // Get Accl sensor ID
  OutData.Data[0] = OUTX_L | 0x80;  // read multiple bytes
  if(FALSE == I2C1_DataTransfer(MAGN_SENSOR_ADDR, OutData.Data, 1))
  {
    I2C1_Close();
    return(FALSE);
  }
  if(FALSE == I2C1_DataTransfer(MAGN_SENSOR_ADDR | 1, OutData.Data, 6))
  {
    I2C1_Close();
    return(FALSE);
  }
  I2C1_Close();

  *mX = OutData.maX;
  *mY = OutData.maY;
  *mZ = OutData.maZ;
  return(TRUE); 
}



/*************************************************************************
 * Function Name: Press_Get
 * Parameters: pInt16S press
 *
 * Return: Boolean
 *
 * Description: Read Press data 
 *
 *************************************************************************/
Boolean Press_Get (pInt32S press)
{
#pragma pack(1)
union
{
  u8 Data[3];
  struct
  {
    s32 press;
  };
} OutData;
#pragma pack()

/*  if(Bit_RESET == GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_10))
  {
    return(FALSE);
  }
 */ 
  if(FALSE == I2C1_Open())
  {
    return(FALSE);
  }

  // Get Accl sensor ID
  OutData.Data[0] = PRESS_OUT_XL | 0x80;  // read multiple bytes
  if(FALSE == I2C1_DataTransfer(PRESS_SENSOR_ADDR, OutData.Data, 1))
  {
    I2C1_Close();
    return(FALSE);
  }
  if(FALSE == I2C1_DataTransfer(PRESS_SENSOR_ADDR | 1, OutData.Data, 3))
  {
    I2C1_Close();
    return(FALSE);
  }
  I2C1_Close();

  *press = OutData.press;
  return(TRUE);
}
