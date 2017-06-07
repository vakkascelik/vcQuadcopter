/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright 
 *
 *    File name   : accl_drv.h
 *    Description : Gyro, Accelerometer, Compass, and Altimeter (LSM6DS33, LIS3MDL, and LPS25H Carrier driver include file
 *
 *    History :
 *    1. Date        : January 12, 2017
 *       Author      : Vakkas Celik
 *       Description : Create
 *
 **************************************************************************/
#include "includes.h"

#ifndef __ALTIMU10_DRV_H
#define __ALTIMU10_DRV_H



#define GYRO_ACC_SENSOR_ADDR     0xD6
#define MAGN_SENSOR_ADDR         0x3C
#define PRESS_SENSOR_ADDR        0xBA

/*************************************************************************
 * Function Name: Altimu10_Init
 * Parameters: none
 *
 * Return: Boolean
 *
 * Description: Init Accelerometer sensor
 *
 *************************************************************************/
Boolean Altimu10_Init (void);

/*************************************************************************
 * Function Name: Gyro_Accl_Get
 * Parameters: pInt16S gX, pInt16S gY, pInt16S gZ, pInt16S aX, pInt16S aY, pInt16S aZ
 *
 * Return: Boolean
 *
 * Description: Read Gyro X,Y,Z data - Read Acclerometer X,Y,Z data
 *
 *************************************************************************/
Boolean Gyro_Accl_Get (pInt16S gX, pInt16S gY, pInt16S gZ, pInt16S aX, pInt16S aY, pInt16S aZ);


/*************************************************************************
 * Function Name: Magn_Get
 * Parameters: pInt16S mX, pInt16S mY, pInt16S mZ
 *
 * Return: Boolean
 *
 * Description: Read Magnetometer X,Y,Z data 
 *
 *************************************************************************/
Boolean Magn_Get (pInt16S mX, pInt16S mY, pInt16S mZ);



/*************************************************************************
 * Function Name: Press_Get
 * Parameters: pInt16S press
 *
 * Return: Boolean
 *
 * Description: Read Press data 
 *
 *************************************************************************/
Boolean Press_Get (pInt32S press);

#endif // __ALTIMU10_DRV_H