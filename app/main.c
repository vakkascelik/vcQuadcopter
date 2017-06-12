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
extern uint32_t Yaw;            // DutyCycle2
extern uint32_t Pitch;          // DutyCycle3
extern uint32_t Roll;           // DutyCycle4

int32_t Yaw_offset;
int32_t Pitch_offset;
int32_t Roll_offset;

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

  char log[1024];
    
  Int16S X, Y, Z;
  Int16S gX, gY, gZ, aX, aY, aZ;
  Int16S mX, mY, mZ;
  Int32S press; 
  
  float gx, gy, gz, ax, ay, az, mx, my, mz;
  
  float KP_yaw, KI_yaw, KD_yaw;
  float KP_pitch, KI_pitch, KD_pitch;
  float KP_roll, KI_roll, KD_roll;
  
  float pError_yaw, iError_yaw, dError_yaw, previousError_yaw;
  float pError_pitch, iError_pitch, dError_pitch, previousError_pitch;
  float pError_roll, iError_roll, dError_roll, previousError_roll;
  
  int16_t yawPID, pitchPID, rollPID;
  
  uint16_t fronleft_Throttle, frontright_Throttle, rearleft_Throttle, rearright_Throttle;
  
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
  DelayResolution100us(100000/TICK_PER_SEC);
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
    
    Yaw_offset=Yaw-4096;
    Pitch_offset=Pitch-4096;
    Roll_offset=Roll-1482;
    
    /* Read new roll, pitch and yaw values */
    sprintf(log,"Throttle:%d, Roll: %d; Pitch: %d, Yaw: %d --- pRoll: %f; pPitch: %f, pYaw: %f\r\n", Throttle, Roll_offset, Pitch_offset, Yaw_offset, AHRSIMU.Roll, AHRSIMU.Pitch, AHRSIMU.Yaw);
    USART_puts(USART2, log);
 /*   
    // PID Control
    pError_yaw = AHRSIMU.Yaw - Output;
    iError_yaw = iError_yaw + pError_yaw*dt;
    dError_yaw = (pError_yaw - previousError_yaw);
    previousError_yaw = pError_yaw;
    
    pError_pitch = AHRSIMU.Pitch - Output;
    iError_pitch = iError_pitch + pError_pitch*dt;
    dError_pitch = (pError_pitch - previousError_pitch);
    previousError_pitch = pError_pitch;
    
    pError_roll = AHRSIMU.Roll - Output;
    iError_roll = iError_roll + pError_roll*dt;
    dError_roll = (pError_roll - previousError_roll);
    previousError_roll = pError_roll;
*/
    yawPID   = 0;//KP_yaw*pError_yaw     + KI_yaw*iError_yaw     + KD_yaw*dError_yaw;
    pitchPID = 0;//KP_pitch*pError_pitch + KI_pitch*iError_pitch + KD_pitch*dError_pitch;
    rollPID  = 0;//KP_roll*pError_roll   + KI_roll*iError_roll   + KD_roll*dError_roll;
    
    fronleft_Throttle   = (Throttle + pitchPID + rollPID - yawPID + Pitch_offset + Roll_offset - Yaw_offset);
    frontright_Throttle = (Throttle + pitchPID - rollPID + yawPID + Pitch_offset - Roll_offset + Yaw_offset);
    rearleft_Throttle   = (Throttle - pitchPID + rollPID + yawPID - Pitch_offset + Roll_offset + Yaw_offset);
    rearright_Throttle  = (Throttle - pitchPID - rollPID - yawPID - Pitch_offset - Roll_offset - Yaw_offset);
      
    setFrontLeftPwmValue((uint16_t)fronleft_Throttle);
    setFrontRightPwmValue((uint16_t)frontright_Throttle);
    setRearLeftPwmValue((uint16_t)rearleft_Throttle);
    setRearRightPwmValue((uint16_t)rearright_Throttle);
    
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
    
*/

  }
  
}


