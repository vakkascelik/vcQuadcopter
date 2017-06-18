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
/* HCSR04 Instance */
TM_HCSR04_t HCSR04;

static volatile Int32U TimingDelay;

extern uint16_t Throttle;       // DutyCycle1
extern uint32_t Yaw;            // DutyCycle2
extern uint32_t Pitch;          // DutyCycle3
extern uint32_t Roll;           // DutyCycle4

int32_t Yaw_offset;
int32_t Pitch_offset;
int32_t Roll_offset;

uint16_t fronleft_Throttle, frontright_Throttle, rearleft_Throttle, rearright_Throttle;

float yawPID, pitchPID, rollPID;
  
float KP_yaw=0.5, KI_yaw=0, KD_yaw=0.1;
float KP_pitch=0.5, KI_pitch=0, KD_pitch=0.1;
float KP_roll=0.5, KI_roll=0, KD_roll=0.1;

float pError_yaw, iError_yaw, dError_yaw, previousError_yaw;
float pError_pitch, iError_pitch, dError_pitch, previousError_pitch;
float pError_roll, iError_roll, dError_roll, previousError_roll;  

extern void initReceiverPWM();

extern void initMotorPWM(uint16_t value);
extern void setFrontLeftPwmValue (uint16_t value);
extern void setFrontRightPwmValue (uint16_t value);
extern void setRearLeftPwmValue (uint16_t value);
extern void setRearRightPwmValue (uint16_t value);
extern void calibrateESCs(void);

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
  
  uint16_t usart_receive;
  
  float yawErr=0, rollErr=0, pitchErr=0;
  float firstYawRead, firstRollRead, firstPitchRead;
  int firstSenseRead=1;
  
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
  
  
  STM_LEDInit(LED1);

  initMotorPWM(1072);
/*  DelayResolution100us(30000);  // 3 seconds delay
  calibrateESCs();
  DelayResolution100us(20000);  // 2 seconds delay
*/  initReceiverPWM();
  
  /* Init structure with 1000hZ sample rate, 0.5 beta and 0 inclination (3.5 degrees is inclination in Ljubljana, Slovenia) on July, 2016 */
  TM_AHRSIMU_Init(&AHRSIMU, 1000, 0.5, 0);
  
  /* Initialize distance sensor1 on pins; ECHO: PD0, TRIGGER: PC1 */
//  if (!TM_HCSR04_Init(&HCSR04, GPIOD, GPIO_PIN_0, GPIOC, GPIO_PIN_1)) {
      /* Sensor is not ready to use */
      /* Maybe wiring is incorrect */
  /*    while (1) {
          STM_LEDToggle(LED1);
          Delayms(100);
      }*/
//  }
    
  DelayResolution100us(100000/TICK_PER_SEC);
  while(1)
  {
    /* Delay */
    DelayResolution100us(10000/TICK_PER_SEC);

    /* Toggle LED1 */
    STM_LEDToggle(LED1);
    
    /* Read distance from sensor 1 */
    /* Distance is returned in cm and also stored in structure */
    /* You can use both ways */
//    TM_HCSR04_Read(&HCSR04);
       
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
    
    Yaw_offset=Yaw-1482;
    Pitch_offset=Pitch-1535;
    Roll_offset=Roll-1485;
     
    if( 1065<Throttle && Throttle< 1900)
    {
      if(firstSenseRead<100)
      {
        firstSenseRead++;
        firstYawRead=AHRSIMU.Yaw;
        firstRollRead=AHRSIMU.Roll;
        firstPitchRead=AHRSIMU.Pitch;  
        if(firstRollRead<0)
          firstRollRead+=360;
      }
      else
      {
        yawErr=AHRSIMU.Yaw-firstYawRead;
        rollErr=AHRSIMU.Roll-firstRollRead;
        pitchErr=AHRSIMU.Pitch-firstPitchRead;
        
        // PID Control
        pError_yaw = firstYawRead - AHRSIMU.Yaw;
        iError_yaw = iError_yaw + pError_yaw;
        dError_yaw = (pError_yaw - previousError_yaw);
        previousError_yaw = pError_yaw;
        if (pError_yaw == 0)
          iError_yaw = 0;
        if ( abs(pError_yaw) > 40)
          iError_yaw = 0;
      
        pError_pitch = firstPitchRead - AHRSIMU.Pitch;
        iError_pitch = iError_pitch + pError_pitch;
        dError_pitch = (pError_pitch - previousError_pitch);
        previousError_pitch = pError_pitch;
        if (pError_pitch == 0)
          iError_pitch = 0;
        if ( abs(pError_pitch) > 40)
          iError_pitch = 0;  
        
        
        if(AHRSIMU.Roll<0)
          AHRSIMU.Roll+=360;
        pError_roll = firstRollRead - AHRSIMU.Roll;
        iError_roll = iError_roll + pError_roll;
        dError_roll = (pError_roll - previousError_roll);
        previousError_roll = pError_roll;
        if (pError_roll == 0)
          iError_roll = 0;
        if ( abs(pError_roll) > 40)
          iError_roll = 0;

        yawPID   = KP_yaw*pError_yaw     + KI_yaw*iError_yaw     + KD_yaw*dError_yaw;
        pitchPID = KP_pitch*pError_pitch + KI_pitch*iError_pitch + KD_pitch*dError_pitch;
        rollPID  = KP_roll*pError_roll   + KI_roll*iError_roll   + KD_roll*dError_roll;

        /* Read new roll, pitch and yaw values */
        sprintf(log,"Throttle:%d, Roll: %d; Pitch: %d, Yaw: %d --- pRoll: %f; pPitch: %f, pYaw: %f\r\n", Throttle, Roll_offset, Pitch_offset, Yaw_offset, AHRSIMU.Roll, AHRSIMU.Pitch, AHRSIMU.Yaw);
     //   sprintf(log,"Throttle:%d, Roll: %d; Pitch: %d, Yaw: %d --- pRoll: %f; pPitch: %f, pYaw: %f\r\n", Throttle, Roll_offset, Pitch_offset, Yaw_offset, pError_roll, pError_pitch, pError_yaw);
        USART_puts(USART2, log);
    
        fronleft_Throttle   = (Throttle - Pitch_offset - Roll_offset + Yaw_offset);
        frontright_Throttle = (Throttle - Pitch_offset + Roll_offset - Yaw_offset);
        rearleft_Throttle   = (Throttle + Pitch_offset + Roll_offset - Yaw_offset);
        rearright_Throttle  = (Throttle + Pitch_offset - Roll_offset + Yaw_offset);
        
 //       sprintf(log,"fronleft_Throttle:%d, frontright_Throttle: %d; rearleft_Throttle: %d, rearright_Throttle: %d \r\n", fronleft_Throttle, frontright_Throttle, rearleft_Throttle, rearright_Throttle);
 //       USART_puts(USART2, log);
                
        setFrontLeftPwmValue((uint16_t)fronleft_Throttle);
        setFrontRightPwmValue((uint16_t)frontright_Throttle);
        setRearLeftPwmValue((uint16_t)rearleft_Throttle);
        setRearRightPwmValue((uint16_t)rearright_Throttle);
      }
    }
    
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


