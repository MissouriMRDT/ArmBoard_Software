#include <RoveI2CEnergia_TivaTM4C1294NCPDT.h>
#include "Debug.h"
#include "Energia.h"
#include "RovePinMapEnergia_TivaTM4C1294NCPDT.h"
#include "ClockingEnergia_TivaTM4C1294NCPDT.h"
#include "driverlib/pin_map.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

static void masterInitExpClk(uint32_t ui32Base, RoveI2C_Speed speed);
static void initVerifyInput(uint8_t i2cIndex, RoveI2C_Speed speed, uint8_t clockPin, uint8_t dataPin);
static RoveI2C_Error transferHandleError(uint32_t i2cBase);
static RoveI2C_Error transferHandleError(uint32_t i2cBase, uint32_t stopControl);

static const uint32_t i2cIndexToI2cBase[] = 
{
  I2C0_BASE, I2C1_BASE, I2C2_BASE, I2C3_BASE, I2C4_BASE, I2C5_BASE, I2C6_BASE, I2C7_BASE, I2C8_BASE, I2C9_BASE
};

static const uint32_t i2cIndexToI2cPeriph[] =
{
  SYSCTL_PERIPH_I2C0, SYSCTL_PERIPH_I2C1, SYSCTL_PERIPH_I2C2, SYSCTL_PERIPH_I2C3, SYSCTL_PERIPH_I2C4, SYSCTL_PERIPH_I2C5, SYSCTL_PERIPH_I2C6,
  SYSCTL_PERIPH_I2C7, SYSCTL_PERIPH_I2C8, SYSCTL_PERIPH_I2C9
};

static const uint32_t pinMapToI2cBase_Scl[] =
{
  0,           // dummy
  0,           // 01 - 3.3v       X8_01
  0,             // 02 - PE_4       X8_03
  0,             // 03 - PC_4       X8_05
  0,             // 04 - PC_5       X8_07
  0,             // 05 - PC_6       X8_09
  0,             // 06 - PE_5       X8_11
  0,             // 07 - PD_3       X8_13
  0,             // 08 - PC_7       X8_15
  I2C0_BASE,     // 09 - PB_2       X8_17
  0,     // 10 - PB_3       X8_19
  0,             // 11 - PP_2       X9_20
  0,             // 12 - PN_3       X9_18
  0,             // 13 - PN_2       X9_16
  I2C7_BASE,             // 14 - PD_0       X9_14
  0,             // 15 - PD_1       X9_12
  0,           // 16 - RST        X9_10
  0,             // 17 - PH_3       X9_08
  0,             // 18 - PH_2       X9_06
  0,             // 19 - PM_3       X9_04
  0,           // 20 - GND        X9_02
  0,           // 21 - 5v         X8_02
  0,           // 22 - GND        X8_04
  0,             // 23 - PE_0       X8_06
  0,             // 24 - PE_1       X8_08
  0,             // 25 - PE_2       X8_10
  0,             // 26 - PE_3       X8_12
  0,             // 27 - PD_7       X8_14
  I2C6_BASE,             // 28 - PA_6       X8_16
  0,             // 29 - PM_4       X8_18
  0,             // 30 - PM_5       X8_20
  0,             // 31 - PL_3       X9_19
  0,             // 32 - PL_2       X9_17
  I2C2_BASE,     // 33 - PL_1       X9_15
  0,             // 34 - PL_0       X9_13
  0,             // 35 - PL_5       X9_11
  0,             // 36 - PL_4       X9_09
  I2C1_BASE,     // 37 - PG_0       X9_07
  0,             // 38 - PF_3       X9_05
  0,             // 39 - PF_2       X9_03
  0,             // 40 - PF_1       X9_01
  0,           // 41 - 3.3v       X6_01
  I2C8_BASE,             // 42 - PD_2       X6_03
  0,             // 43 - PP_0       X6_05
  0,             // 44 - PP_1       X6_07
  0,             // 45 - PD_4       X6_09
  0,             // 46 - PD_5       X6_11
  0,             // 47 - PQ_0       X6_13
  0,             // 48 - PP_4       X6_15
  I2C2_BASE,             // 49 - PN_5       X6_17
  0,             // 50 - PN_4       X6_19
  0,             // 51 - PM_6       X7_20
  0,             // 52 - PQ_1       X7_18
  0,             // 53 - PP_3       X7_16
  0,             // 54 - PQ_3       X7_14
  0,             // 55 - PQ_2       X7_12
  0,           // 56 - RESET      X7_10
  0,             // 57 - PA_7       X7_08
  I2C2_BASE,             // 58 - PP_5       X7_06
  0,             // 59 - PM_7       X7_04
  0,           // 60 - GND        X7_02
  0,           // 61 - 5v         X6_02
  0,             // 62 - GND        X6_04
  I2C5_BASE,             // 63 - PB_4       X6_06
  0,             // 64 - PB_5       X6_08
  0,             // 65 - PK_0       X6_10
  0,             // 66 - PK_1       X6_12
  0,             // 67 - PK_2       X6_14
  0,             // 68 - PK_3       X6_16
  I2C7_BASE,             // 69 - PA_4       X6_18
  0,             // 70 - PA_5       X6_20
  0,             // 71 - PK_7       X7_19
  I2C4_BASE,             // 72 - PK_6       X7_17
  0,             // 73 - PH_1       X7_15
  0,             // 74 - PH_0       X7_13
  0,             // 75 - PM_2       X7_11
  0,             // 76 - PM_1       X7_09
  0,             // 77 - PM_0       X7_07
  0,             // 78 - PK_5       X7_05
  I2C3_BASE,             // 79 - PK_4       X7_03
  0,     // 80 - PG_1       X7_01
  0,             // 81 - PN_1       LED1
  0,             // 82 - PN_0       LED2
  0,             // 83 - PF_4       LED3
  0,             // 84 - PF_0       LED4
  0,             // 85 - PJ_0       USR_SW1
  0,             // 86 - PJ_1       USR_SW2
  0,             // 87 - PD_6       AIN5
  I2C9_BASE,             // 88 - PA_0       JP4
  0,             // 89 - PA_1       JP5
  I2C8_BASE,             // 90 - PA_2       X11_06
  0,             // 91 - PA_3       X11_08
  0,             // 92 - PL_6       unrouted
  0,             // 93 - PL_7       unrouted
  I2C5_BASE,             // 94 - PB_0       X11_58
  0             // 95 - PB_1       unrouted
};

static const uint32_t pinMapToI2cBase_Sda[] =
{
  0,           // dummy
  0,           // 01 - 3.3v       X8_01
  0,             // 02 - PE_4       X8_03
  0,             // 03 - PC_4       X8_05
  0,             // 04 - PC_5       X8_07
  0,             // 05 - PC_6       X8_09
  0,             // 06 - PE_5       X8_11
  I2C8_BASE,             // 07 - PD_3       X8_13
  0,             // 08 - PC_7       X8_15
  0,     // 09 - PB_2       X8_17
  I2C0_BASE,     // 10 - PB_3       X8_19
  0,             // 11 - PP_2       X9_20
  0,             // 12 - PN_3       X9_18
  0,             // 13 - PN_2       X9_16
  0,             // 14 - PD_0       X9_14
  I2C7_BASE,             // 15 - PD_1       X9_12
  0,           // 16 - RST        X9_10
  0,             // 17 - PH_3       X9_08
  0,             // 18 - PH_2       X9_06
  0,             // 19 - PM_3       X9_04
  0,           // 20 - GND        X9_02
  0,           // 21 - 5v         X8_02
  0,           // 22 - GND        X8_04
  0,             // 23 - PE_0       X8_06
  0,             // 24 - PE_1       X8_08
  0,             // 25 - PE_2       X8_10
  0,             // 26 - PE_3       X8_12
  0,             // 27 - PD_7       X8_14
  0,             // 28 - PA_6       X8_16
  0,             // 29 - PM_4       X8_18
  0,             // 30 - PM_5       X8_20
  0,             // 31 - PL_3       X9_19
  0,             // 32 - PL_2       X9_17
  0,     // 33 - PL_1       X9_15
  I2C2_BASE,             // 34 - PL_0       X9_13
  0,             // 35 - PL_5       X9_11
  0,             // 36 - PL_4       X9_09
  0,     // 37 - PG_0       X9_07
  0,             // 38 - PF_3       X9_05
  0,             // 39 - PF_2       X9_03
  0,             // 40 - PF_1       X9_01
  0,           // 41 - 3.3v       X6_01
  0,             // 42 - PD_2       X6_03
  0,             // 43 - PP_0       X6_05
  0,             // 44 - PP_1       X6_07
  0,             // 45 - PD_4       X6_09
  0,             // 46 - PD_5       X6_11
  0,             // 47 - PQ_0       X6_13
  0,             // 48 - PP_4       X6_15
  0,             // 49 - PN_5       X6_17
  I2C2_BASE,             // 50 - PN_4       X6_19
  0,             // 51 - PM_6       X7_20
  0,             // 52 - PQ_1       X7_18
  0,             // 53 - PP_3       X7_16
  0,             // 54 - PQ_3       X7_14
  0,             // 55 - PQ_2       X7_12
  0,           // 56 - RESET      X7_10
  I2C6_BASE,             // 57 - PA_7       X7_08
  0,             // 58 - PP_5       X7_06
  0,             // 59 - PM_7       X7_04
  0,           // 60 - GND        X7_02
  0,           // 61 - 5v         X6_02
  0,             // 62 - GND        X6_04
  0,             // 63 - PB_4       X6_06
  I2C5_BASE,             // 64 - PB_5       X6_08
  0,             // 65 - PK_0       X6_10
  0,             // 66 - PK_1       X6_12
  0,             // 67 - PK_2       X6_14
  0,             // 68 - PK_3       X6_16
  0,             // 69 - PA_4       X6_18
  I2C7_BASE,             // 70 - PA_5       X6_20
  I2C4_BASE,             // 71 - PK_7       X7_19
  0,             // 72 - PK_6       X7_17
  0,             // 73 - PH_1       X7_15
  0,             // 74 - PH_0       X7_13
  0,             // 75 - PM_2       X7_11
  0,             // 76 - PM_1       X7_09
  0,             // 77 - PM_0       X7_07
  I2C3_BASE,             // 78 - PK_5       X7_05
  0,             // 79 - PK_4       X7_03
  I2C1_BASE,     // 80 - PG_1       X7_01
  0,             // 81 - PN_1       LED1
  0,             // 82 - PN_0       LED2
  0,             // 83 - PF_4       LED3
  0,             // 84 - PF_0       LED4
  0,             // 85 - PJ_0       USR_SW1
  0,             // 86 - PJ_1       USR_SW2
  0,             // 87 - PD_6       AIN5
  0,             // 88 - PA_0       JP4
  I2C9_BASE,             // 89 - PA_1       JP5
  0,             // 90 - PA_2       X11_06
  I2C8_BASE,             // 91 - PA_3       X11_08
  0,             // 92 - PL_6       unrouted
  0,             // 93 - PL_7       unrouted
  0,             // 94 - PB_0       X11_58
  I2C5_BASE             // 95 - PB_1       unrouted
};

static const uint32_t pinMapToI2cConfig[] = 
{
  0,           // dummy
  0,           // 01 - 3.3v       X8_01
  0,             // 02 - PE_4       X8_03
  0,             // 03 - PC_4       X8_05
  0,             // 04 - PC_5       X8_07
  0,             // 05 - PC_6       X8_09
  0,             // 06 - PE_5       X8_11
  GPIO_PD3_I2C8SDA,             // 07 - PD_3       X8_13
  0,             // 08 - PC_7       X8_15
  GPIO_PB2_I2C0SCL,     // 09 - PB_2       X8_17
  GPIO_PB3_I2C0SDA,     // 10 - PB_3       X8_19
  0,             // 11 - PP_2       X9_20
  0,             // 12 - PN_3       X9_18
  0,             // 13 - PN_2       X9_16
  GPIO_PD0_I2C7SCL,             // 14 - PD_0       X9_14
  GPIO_PD1_I2C7SDA,             // 15 - PD_1       X9_12
  0,           // 16 - RST        X9_10
  0,             // 17 - PH_3       X9_08
  0,             // 18 - PH_2       X9_06
  0,             // 19 - PM_3       X9_04
  0,           // 20 - GND        X9_02
  0,           // 21 - 5v         X8_02
  0,           // 22 - GND        X8_04
  0,             // 23 - PE_0       X8_06
  0,             // 24 - PE_1       X8_08
  0,             // 25 - PE_2       X8_10
  0,             // 26 - PE_3       X8_12
  0,             // 27 - PD_7       X8_14
  GPIO_PA6_I2C6SCL,             // 28 - PA_6       X8_16
  0,             // 29 - PM_4       X8_18
  0,             // 30 - PM_5       X8_20
  0,             // 31 - PL_3       X9_19
  0,             // 32 - PL_2       X9_17
  GPIO_PL1_I2C2SCL,     // 33 - PL_1       X9_15
  GPIO_PL0_I2C2SDA,             // 34 - PL_0       X9_13
  0,             // 35 - PL_5       X9_11
  0,             // 36 - PL_4       X9_09
  GPIO_PG0_I2C1SCL,     // 37 - PG_0       X9_07
  0,             // 38 - PF_3       X9_05
  0,             // 39 - PF_2       X9_03
  0,             // 40 - PF_1       X9_01
  0,           // 41 - 3.3v       X6_01
  GPIO_PD2_I2C8SCL,             // 42 - PD_2       X6_03
  0,             // 43 - PP_0       X6_05
  0,             // 44 - PP_1       X6_07
  0,             // 45 - PD_4       X6_09
  0,             // 46 - PD_5       X6_11
  0,             // 47 - PQ_0       X6_13
  0,             // 48 - PP_4       X6_15
  GPIO_PN5_I2C2SCL,             // 49 - PN_5       X6_17
  GPIO_PN4_I2C2SDA,             // 50 - PN_4       X6_19
  0,             // 51 - PM_6       X7_20
  0,             // 52 - PQ_1       X7_18
  0,             // 53 - PP_3       X7_16
  0,             // 54 - PQ_3       X7_14
  0,             // 55 - PQ_2       X7_12
  0,           // 56 - RESET      X7_10
  GPIO_PA7_I2C6SDA,             // 57 - PA_7       X7_08
  GPIO_PP5_I2C2SCL,             // 58 - PP_5       X7_06
  0,             // 59 - PM_7       X7_04
  0,           // 60 - GND        X7_02
  0,           // 61 - 5v         X6_02
  0,             // 62 - GND        X6_04
  GPIO_PB4_I2C5SCL,             // 63 - PB_4       X6_06
  GPIO_PB5_I2C5SDA,             // 64 - PB_5       X6_08
  0,             // 65 - PK_0       X6_10
  0,             // 66 - PK_1       X6_12
  0,             // 67 - PK_2       X6_14
  0,             // 68 - PK_3       X6_16
  GPIO_PA4_I2C7SCL,             // 69 - PA_4       X6_18
  GPIO_PA5_I2C7SDA,             // 70 - PA_5       X6_20
  GPIO_PK7_I2C4SDA,             // 71 - PK_7       X7_19
  GPIO_PK6_I2C4SCL,             // 72 - PK_6       X7_17
  0,             // 73 - PH_1       X7_15
  0,             // 74 - PH_0       X7_13
  0,             // 75 - PM_2       X7_11
  0,             // 76 - PM_1       X7_09
  0,             // 77 - PM_0       X7_07
  GPIO_PK5_I2C3SDA,             // 78 - PK_5       X7_05
  GPIO_PK4_I2C3SCL,             // 79 - PK_4       X7_03
  GPIO_PG1_I2C1SDA,     // 80 - PG_1       X7_01
  0,             // 81 - PN_1       LED1
  0,             // 82 - PN_0       LED2
  0,             // 83 - PF_4       LED3
  0,             // 84 - PF_0       LED4
  0,             // 85 - PJ_0       USR_SW1
  0,             // 86 - PJ_1       USR_SW2
  0,             // 87 - PD_6       AIN5
  GPIO_PA0_I2C9SCL,             // 88 - PA_0       JP4
  GPIO_PA1_I2C9SDA,             // 89 - PA_1       JP5
  GPIO_PA2_I2C8SCL,             // 90 - PA_2       X11_06
  GPIO_PA3_I2C8SDA,             // 91 - PA_3       X11_08
  0,             // 92 - PL_6       unrouted
  0,             // 93 - PL_7       unrouted
  GPIO_PB0_I2C5SCL,             // 94 - PB_0       X11_58
  GPIO_PB1_I2C5SDA             // 95 - PB_1       unrouted
};

RoveI2C_Handle i2cInit(uint8_t i2cIndex, RoveI2C_Speed speed, uint8_t clockPin, uint8_t dataPin)
{
  initVerifyInput(i2cIndex, speed, clockPin, dataPin);

  uint32_t i2cBase = i2cIndexToI2cBase[i2cIndex];
  uint8_t dataPinPortRef = pinToPortRef[dataPin];
  uint8_t clockPinPortRef = pinToPortRef[clockPin];

  uint32_t i2cPeriph = i2cIndexToI2cPeriph[i2cIndex];
  uint32_t dataPinPortPeriph = portRefToPeriphBase[dataPinPortRef];
  uint32_t clockPinPortPeriph = portRefToPeriphBase[clockPinPortRef];
  uint32_t dataPinPortBase = portRefToPortBase[dataPinPortRef];
  uint32_t clockPinPortBase = portRefToPortBase[clockPinPortRef];

  //enable the i2c module
  SysCtlPeripheralEnable(i2cPeriph);

  // Wait for the I2C module to be ready.
  while(!SysCtlPeripheralReady(i2cPeriph));

  //reset the i2c module -- in case of re-initialization
  SysCtlPeripheralReset(i2cPeriph);

  // Wait for the I2C module to be ready.
  while(!SysCtlPeripheralReady(i2cPeriph));

  //enable gpio module
  SysCtlPeripheralEnable(dataPinPortPeriph);
  SysCtlPeripheralEnable(clockPinPortPeriph);
  while(!SysCtlPeripheralReady(dataPinPortPeriph) || !SysCtlPeripheralReady(clockPinPortPeriph));

  //configure the gpio pins for using the i2c module as a source
  GPIOPinConfigure(pinMapToI2cConfig[clockPin]);
  GPIOPinConfigure(pinMapToI2cConfig[dataPin]);

  //configure the gpio pins for i2c operation
  GPIOPinTypeI2CSCL(clockPinPortBase, pinToPinMask[clockPin]);
  GPIOPinTypeI2C(dataPinPortBase, pinToPinMask[dataPin]);

  //set default timeout value to as high as possible
  I2CMasterTimeoutSet(i2cBase, 0xFF);

  // Enable and initialize the I2C0 master module. 
  masterInitExpClk(i2cBase, speed);

  //clear I2C FIFOs
  HWREG(i2cBase + I2C_O_FIFOCTL) = 80008000;

  RoveI2C_Handle handle;
  handle.index = i2cIndex;

  return handle;
}

RoveI2C_Error roveI2cSend(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t msg)
{

  uint32_t i2cBase = i2cIndexToI2cBase[handle.index];
  bool receive = false;

  //slave address is bits 6:0 of byte
  SlaveAddr &= 0b01111111;

  // Tell the master module what address it will place on the bus when
  // communicating with the slave.
  I2CMasterSlaveAddrSet(i2cBase, SlaveAddr, receive);

  //if another master is using the bus, return that the line is busy
  if(I2CMasterBusBusy(i2cBase))
  {
    return(I2CERROR_BUSY);
  }

  //put data to be sent into FIFO
  I2CMasterDataPut(i2cBase, msg);

  //Initiate send of data from the MCU
  I2CMasterControl(i2cBase, I2C_MASTER_CMD_SINGLE_SEND);

  //do data transfer with slave
  return transferHandleError(i2cBase);
}

RoveI2C_Error roveI2cSend(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t reg, uint8_t msg)
{
  uint8_t compactedMsg[2];
  compactedMsg[0] = reg;
  compactedMsg[1] = msg;

  return(roveI2cSendBurst(handle, SlaveAddr, compactedMsg, 2));
}

RoveI2C_Error roveI2cSendBurst(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t msg[], size_t msgSize)
{

  uint32_t i2cBase = i2cIndexToI2cBase[handle.index];
  bool receive = false;
  RoveI2C_Error errorGot;

  //slave address is bits 6:0 of byte
  SlaveAddr = SlaveAddr & 0b01111111;

  if(msgSize == 1)
  {
    return roveI2cSend(handle, SlaveAddr, msg[0]);
  }

  // Tell the master module what address it will place on the bus when
  // communicating with the slave.
  I2CMasterSlaveAddrSet(i2cBase, SlaveAddr, receive);

  //if another master is using the bus, return that the line is busy
  if(I2CMasterBusBusy(i2cBase))
  {
    return(I2CERROR_BUSY);
  }

  //send more of the data, up till the last byte, using the
  //BURST_SEND_CONT command of the I2C module
  for(uint32_t i = 0; i < (msgSize) - 1; i++)
  {
    //put next piece of data into I2C FIFO
    I2CMasterDataPut(i2cBase, msg[i]);

    //send next data that was just placed into FIFO
    if(i == 0)
    {
      I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_START);
    }
    else
    {
      I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_CONT);
    }

    //do data transfer with slave
    errorGot = transferHandleError(i2cBase, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
    if(errorGot != I2CERROR_NONE)
    {
      return errorGot;
    }
  }

  //put last piece of data into I2C FIFO
  I2CMasterDataPut(i2cBase, msg[msgSize-1]);

  //send next data that was just placed into FIFO
  I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_FINISH);

  //do data transfer with slave
  return transferHandleError(i2cBase);
}

RoveI2C_Error roveI2cSendBurst(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t reg, uint8_t msg[], size_t msgSize)
{
  //make an array big enough to hold the message plus the register address
  uint8_t compactedMsg[msgSize+1];

  //transfer data array into the compacted message array, with the first element being the reg and all elements after being the msg array concacted on
  compactedMsg[0] = reg;
  for(int i = 0; i < msgSize; i++)
  {
    compactedMsg[i+1] = msg[i];
  }

  //call the multi byte send function. It will send the register address first, followed by the actual message
  return roveI2cSendBurst(handle, SlaveAddr, compactedMsg, msgSize+1);
}

RoveI2C_Error roveI2cReceive(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t* buffer)
{
  uint32_t i2cBase = i2cIndexToI2cBase[handle.index];
  bool receive = true;
  RoveI2C_Error errorGot;

  //slave address is bits 6:0 of byte
  SlaveAddr = SlaveAddr & 0b01111111;

  //specify that we are going to read from slave device with 3rd argument = true
  I2CMasterSlaveAddrSet(i2cBase, SlaveAddr, receive);

  //if another master is using the bus, return that the line is busy
  if(I2CMasterBusBusy(i2cBase))
  {
    return(I2CERROR_BUSY);
  }

  //send control byte and read from the register we specified earlier
  I2CMasterControl(i2cBase, I2C_MASTER_CMD_SINGLE_RECEIVE);

  //do data transfer with slave
  errorGot = transferHandleError(i2cBase);
  if(errorGot == I2CERROR_NONE)
  {
    //return data pulled from the specified register, returns uint32_t even though it only contains a byte
    *buffer = I2CMasterDataGet(i2cBase);
  }
  return errorGot;
}

RoveI2C_Error roveI2cReceive(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t reg, uint8_t* buffer)
{
  uint32_t i2cBase = i2cIndexToI2cBase[handle.index];

  bool receive = false;
  RoveI2C_Error errorGot;

  //specify that we are writing (a register address) to the
  //slave device
  I2CMasterSlaveAddrSet(i2cBase, SlaveAddr, receive);

  //if another master is using the bus, return that the line is busy
  if(I2CMasterBusBusy(i2cBase))
  {
    return(I2CERROR_BUSY);
  }

  //specify register to be read on the slave device
  I2CMasterDataPut(i2cBase, reg);

  //send control byte and register address byte to slave device
  //say we're sending a burst for some intricate i2c protocol-y reason.
  I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_START);

  //do data transfer with slave
  errorGot = transferHandleError(i2cBase, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
  if(errorGot != I2CERROR_NONE)
  {
    return errorGot;
  }

  receive = true;

  //specify that we are going to read from slave device
  I2CMasterSlaveAddrSet(i2cBase, SlaveAddr, receive);

  //send control byte and read from the register we specified earlier
  I2CMasterControl(i2cBase, I2C_MASTER_CMD_SINGLE_RECEIVE);

  //do data transfer with slave
  errorGot = transferHandleError(i2cBase);
  if(errorGot == I2CERROR_NONE)
  {
     //return data pulled from the specified register, returns uint32_t even though it only contains a byte
     *buffer = I2CMasterDataGet(i2cBase);
  }
  return errorGot;
}

RoveI2C_Error roveI2cReceiveBurst(RoveI2C_Handle handle, uint16_t SlaveAddr, uint8_t* buffer, size_t sizeOfReceive)
{
  uint8_t * receivedData = buffer;
  uint32_t i2cBase = i2cIndexToI2cBase[handle.index];
  bool receive = true;
  RoveI2C_Error errorGot;

  //specify that we are going to read from slave device
  I2CMasterSlaveAddrSet(i2cBase, SlaveAddr, receive);

  //if another master is using the bus, return that the line is busy
  if(I2CMasterBusBusy(i2cBase))
  {
    return(I2CERROR_BUSY);
  }

  for(uint32_t i = 0; i < (sizeOfReceive) - 1; i++)
  {
    //receive control byte and read from the register we specified earlier
    if(i == 0)
    {
      I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_START);
    }
    else
    {
      I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    }
    
    //do data transfer with slave
    errorGot = transferHandleError(i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
    if(errorGot != I2CERROR_NONE)
    {
      return errorGot;
    }
    else
    {
      //return data pulled from specified register
      receivedData [i] = I2CMasterDataGet(i2cBase);
    }
  }

  //send next data that was just placed into FIFO
  I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

  //do data transfer with slave
  errorGot = transferHandleError(i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
  if(errorGot == I2CERROR_NONE)
  {
    //return data pulled from specified register
    receivedData [sizeOfReceive-1] = I2CMasterDataGet(i2cBase);
  }
  return errorGot;
}

RoveI2C_Error roveI2cReceiveBurst(RoveI2C_Handle handle, uint16_t SlaveAddr,  uint8_t reg, uint8_t* buffer, size_t sizeOfReceive)
{
  uint8_t* receivedData = buffer;
  uint32_t i2cBase = i2cIndexToI2cBase[handle.index];
  bool receive = false;
  RoveI2C_Error errorGot;

  //specify that we are writing (a register address) to the
  //slave device
  I2CMasterSlaveAddrSet(i2cBase, SlaveAddr, receive);

  //if another master is using the bus, return that the line is busy
  if(I2CMasterBusBusy(i2cBase))
  {
    return(I2CERROR_BUSY);
  }

  //specify register to be read on the slave device
  I2CMasterDataPut(i2cBase, reg);

  //send control byte and register address byte to slave device
  I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_START);

  //do data transfer with slave
  errorGot = transferHandleError(i2cBase, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
  if(errorGot != I2CERROR_NONE)
  {
    return errorGot;
  }

  receive = true;

  //specify that we are going to read from slave device
  I2CMasterSlaveAddrSet(i2cBase, SlaveAddr, receive);

  for(uint32_t i = 0; i < (sizeOfReceive) - 1; i++)
  {
    //receive control byte and read from the register we specified earlier
    if(i == 0)
    {
      I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_START);
    }
    else
    {
      I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    }

    //do data transfer with slave
    errorGot = transferHandleError(i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
    if(errorGot != I2CERROR_NONE)
    {
      return errorGot;
    }
    else
    {
      //return data pulled from specified register
      receivedData [i] = I2CMasterDataGet(i2cBase);
    }
  }

  //finish receive
  I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

  //do data transfer with slave
  errorGot = transferHandleError(i2cBase);
  if(errorGot == I2CERROR_NONE)
  {
    //return data pulled from specified register
    receivedData [sizeOfReceive-1] = I2CMasterDataGet(i2cBase);
  }
  return errorGot;
}

// Enable and initialize the I2C0 master module.  Use the system clock for
// the I2C module, and calculate speed based off system clock 
// and passed desired speed while enabling the I2C module with the passed base.
// Based off of the firmware function I2CMasterInitExpClk
// that unfortunately doesn't cover Fast speed
static void masterInitExpClk(uint32_t ui32Base, RoveI2C_Speed speed)
{
  uint32_t ui32SCLFreq;
  uint32_t ui32TPR;
  uint32_t ui32I2CClk = getCpuClockFreq();

  //
  // Must enable the device before doing anything else.
  //
  I2CMasterEnable(ui32Base);

  //
  // Get the desired SCL speed.
  //
  if(speed == I2CSPEED_STANDARD)
  {
    ui32SCLFreq = 100000;
  }
  else if(speed == I2CSPEED_FULL)
  {
    ui32SCLFreq = 400000;
  }
  else if(speed == I2CSPEED_FAST)
  {
    ui32SCLFreq = 1000000;
  }
  else //default is the standard speed
  {
    ui32SCLFreq = 100000;
  }

  //
  // Compute the clock divider that achieves the fastest speed less than or
  // equal to the desired speed.  The numerator is biased to favor a larger
  // clock divider so that the resulting clock is always less than or equal
  // to the desired clock, never greater.
  //
  ui32TPR = ((ui32I2CClk + (2 * 10 * ui32SCLFreq) - 1) / (2 * 10 * ui32SCLFreq)) - 1;
  HWREG(ui32Base + I2C_O_MTPR) = ui32TPR;
}

static void initVerifyInput(uint8_t i2cIndex, RoveI2C_Speed speed, uint8_t clockPin, uint8_t dataPin)
{
  if(i2cIndex > 9)
  {
    debugFault("I2c: index out of bounds");
  }
  if(clockPin > 95 || dataPin > 95)
  {
    debugFault("I2c: clock or data pin out of bounds");
  }
  if(speed == I2CSPEED_HIGH)
  {
    debugFault("I2c: High speed not supported");
  }

  uint32_t i2cBase = i2cIndexToI2cBase[i2cIndex];

  //verify that both of the pin inputs correspond to the i2c module chosen by the index 
  if(!(i2cBase == pinMapToI2cBase_Scl[clockPin] && i2cBase == pinMapToI2cBase_Sda[dataPin])) 
  {
    debugFault("I2c: pins did not match i2c module");
  }
}

//attempt to have I2C module do a transfer with slave.
//Returns any errors encountered
static RoveI2C_Error transferHandleError(uint32_t i2cBase)
{
  bool timedOut = false;

  //wait for MCU to start transaction
  while(!I2CMasterBusy(i2cBase));

  // Wait until MCU is done transferring or it times out.
  while(I2CMasterBusy(i2cBase) && !timedOut)
  {
    //check to see if the clock out bit in the master control register has been set or not; if it has, then
    //the device has held the clock line low for too long and the module needs reset
    timedOut = (HWREG(i2cBase + I2C_O_MCS) & I2C_MCS_CLKTO) > 0 ? true: false;
  }

  if(timedOut)
  {
    return(I2CERROR_TIMEOUT);
  }
  else if(I2CMasterErr(i2cBase) == I2C_MASTER_ERR_ADDR_ACK || I2CMasterErr(i2cBase) == I2C_MASTER_ERR_DATA_ACK)
  {
    return(I2CERROR_ACK);
  }
  else if(I2CMasterErr(i2cBase) != I2C_MASTER_ERR_NONE)
  {
    return(I2CERROR_OTHER);
  }
  else
  {
    return(I2CERROR_NONE);
  }
}

//attempts to do data transfer with slave.
//returns any errors encountered.
//If an error is encountered besides timeout, then function will load
//the stop control int into the i2C register to try and get the hardware to do its own error handling.
//Module is frozen if timeout is encountered, so doesn't try in that case.
static RoveI2C_Error transferHandleError(uint32_t i2cBase, uint32_t stopControl)
{
  RoveI2C_Error errorGot;

  //do data transfer with slave
  errorGot = transferHandleError(i2cBase);
  if(errorGot != I2CERROR_NONE && errorGot != I2CERROR_TIMEOUT)
  {
    I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
  }

  return errorGot;
}
