/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2011
 *
 *    File name   : main.c
 *    Description : Main module
 *
 *    History :
 *    1. Date        : June 2011
 *       Author      : Stoyan Choynev
 *       Description :
 *
 *   This example project shows how to use the IAR Embedded Workbench for ARM
 *  to develop code for the IAR-STM32F407ZG-SK board. It shows basic use of the I/O,
 *  the timer, the interrupt controllers and the LDC module and the on board
 *  accelerometer
 *
 *    The IAR logo and a cursor are displayed on the LCD. The cursor moves as
 *  the board position is changed.
 *
 *  SRAM Debug - The Progam is loaded to internal RAM.
 *  Flash Debug - The Progam is loaded to internal Flash.
 *
 *  Make sure that the following jumpers are correctly configured on the
 *  IAR-STM32F407ZG-SK evaluation board:
 *
 *  Jumpers:
 *   PWR_SEL - depending of power source
 *   B0_1/B0_0 - B0_0
 *   B1_1/B1_0 - B1_0
 *
 *
 *    $Revision: 1466 $
 **************************************************************************/
#include "includes.h"

#define TICK_PER_SEC          25

/* AHRS/IMU structure */
TM_AHRSIMU_t AHRSIMU;

static volatile Int32U TimingDelay;

extern uint16_t Throttle;       // DutyCycle1
extern uint16_t Yaw;            // DutyCycle2
extern uint16_t Pitch;          // DutyCycle3
extern uint16_t Roll;           // DutyCycle4

extern void initReceiverPWM();

extern void initMotorPWM();
extern void setFrontLeftPwmValue (uint16_t value);
extern void setFrontRightPwmValue (uint16_t value);
extern void setRearLeftPwmValue (uint16_t value);
extern void setRearRightPwmValue (uint16_t value);

extern void InitUSART2(u32 bauld);
extern void TxUSART(char byte);
extern void USART_puts(USART_TypeDef* USARTx, volatile char *s);

/* variable for critical section entry control */
Int32U CriticalSecCntr;

/*************************************************************************
 * Function Name: DelayResolution100us
 * Parameters: Int32U Dly
 *
 * Return: none
 *
 * Description: Delay ~ (arg * 100us)
 *
 *************************************************************************/
void DelayResolution100us(Int32U Dly)
{
  TimingDelay = Dly;

  while(TimingDelay != 0);
}

/*************************************************************************
 * Function Name: TimingDelay_Decrement
 * Parameters: none
 *
 * Return: none
 *
 * Description: Decrements the TimingDelay variable.
 *
 *************************************************************************/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

/*************************************************************************
 * Function Name: main
 * Parameters: none
 *
 * Return: none
 *
 * Description: main
 *
 *************************************************************************/
int main(void)
{

  int cursor_x;
    
  Int16S X, Y, Z;
  Int16S gX, gY, gZ, aX, aY, aZ;
  Int16S mX, mY, mZ;
  Int32S press; 
  
  float gx, gy, gz, ax, ay, az, mx, my, mz;
  
  uint16_t usart_receive;
  
  int firstDelay;

  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */

  ENTR_CRT_SECTION();

  /* SysTick Config*/
  if(SysTick_Config(SystemCoreClock/10000))
  {
    /* Capture error */
    while (1);
  }

  EXT_CRT_SECTION();

  /* Accelerometer init */
  I2C1_Init();
  if(FALSE == Altimu10_Init())
  {
    /* Capture error */
    while(1);
  }

  InitUSART2(19200);
  USART_puts(USART2, "Init complete! Hello World!\r\n"); // just send a message to indicate that it works
  

  initMotorPWM();
  initReceiverPWM();
  
  /* Init structure with 1000hZ sample rate, 0.5 beta and 0 inclination (3.5 degrees is inclination in Ljubljana, Slovenia) on July, 2016 */
  TM_AHRSIMU_Init(&AHRSIMU, 1000, 0.5, 0);
  
  STM_LEDInit(LED1);
  while(1)
  {
    /* Delay */
    DelayResolution100us(10000/TICK_PER_SEC);

    /* Toggle LED1 */
    STM_LEDToggle(LED1);
       
    /* Read data from accelerometer */
    Accl_Get(&X,&Y,&Z);
    Gyro_Accl_Get (&gX, &gY, &gZ, &aX, &aY, &aZ);
    Magn_Get (&mX, &mY, &mZ);
    Press_Get (&press);

    
    /* Convert data to gees, deg/sec and microTesla respectively */
    gx = ( (float)gX * 2000.0f) / 32768.0f;
    gy = ( (float)gY * 2000.0f) / 32768.0f;
    gz = ( (float)gZ * 2000.0f) / 32768.0f;
    
    ax = ( (float)aX * 2.0f) / 32768.0f;
    ay = ( (float)aY * 2.0f) / 32768.0f;
    az = ( (float)aZ * 2.0f) / 32768.0f;
    
    mx = (float)mX;
    my = (float)mY;
    mz = (float)mZ;
    
    /* Call update function */
    /* This function must be called periodically in inteervals set by sample rate on initialization process */
    TM_AHRSIMU_UpdateAHRS(&AHRSIMU, AHRSIMU_DEG2RAD(gx), AHRSIMU_DEG2RAD(gy), AHRSIMU_DEG2RAD(gz), ax, ay, az, mx, my, mz);
    
    /* Read new roll, pitch and yaw values */
    printf("Roll: %f; Pitch: %f, Yaw: %f\r\n", AHRSIMU.Roll, AHRSIMU.Pitch, AHRSIMU.Yaw);
       
    setFrontLeftPwmValue(Throttle);
    setFrontRightPwmValue(Throttle);
    setRearLeftPwmValue(Throttle);
    setRearRightPwmValue(Throttle);
    
 /*   
    FrontLeft   = throttle + pitch + roll - yaw
    FrontRight  = throttle + pitch - roll + yaw
    RearLeft    = throttle - pitch + roll + yaw
    RearRight   = throttle - pitch - roll - yaw
    
    Pitch Down (forward flight):
      FrontLeft Motor and FrontRight Motor decrease thrust
      RearLeft Motor and RearRight Motor increase thrust
    Pitch Up (back flight):
      FrontLeft Motor and FrontRight Motor increase thrust
      RearLeft Motor and RearRight Motor decrease thrust
    Roll Left (left flight):
      FrontRight Motor and RearLeft Motor increase thrust
      FrontLeft Motor and RearRight Motor decrease thrust
    Roll Right (right flight):
      FrontRight Motor and RearLeft Motor decreases thrust
      FrontLeft Motor and RearRight Motor increases thrust
    Yaw Left (turn left):
      FrontRight Motor and RearRight Motor increase thrust
      FrontLeft Motor and RearLeft Motor decrease thrust
    Yaw Right (turn right):
      FrontRight Motor and RearRight Motor decrease thrust
      FrontLeft Motor and RearLeft Motor increase thrust
    
    accelerationX = (signed int)(((signed int)rawData_X) * 3.9);
    accelerationY = (signed int)(((signed int)rawData_Y) * 3.9);
    accelerationZ = (signed int)(((signed int)rawData_Z) * 3.9);
    pitch = 180 * atan (accelerationX/sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ))/M_PI;
    roll = 180 * atan (accelerationY/sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI;
    yaw = 180 * atan (accelerationZ/sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI;
    
    filteredData = (1-weight)*filteredData + weight*newData
*/

  }
  
}


