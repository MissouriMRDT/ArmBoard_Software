/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => TivaC1294/TivaC129E Pin Module Pulse Width Modulation Generator (PWM Gen)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Todo SYSCTL_PERIPH_PWM0, PWM0_BASE should be in pwmGenHardware not hard coded as 1294XL => periph_0, but 123G => periph_0 AND periph_1
// Todo forward declare / include INVALID from future RoveHal hardware_map.h
// todo PWMDeadBandDisable( PWM0_BASE, PWM_GEN ); ?already disabled by default?

#include "RovePwmGen.h"

#include <stdint.h>

#include "Energia.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "tm4c1294ncpdt.h"

const uint8_t INVALID = 0; 

namespace roveware ////////////////////////////////////////////////////////////////////////////////////////////
{
  //////////////////////////////////
  bool pwm_gen_periph_begun = false;

  ///////////////////////////////////////////////////////////////////////////////////
  void setPwmGen ( uint32_t PWM_PIN_MUX,
                   uint32_t PWM_GEN,
                   uint32_t CLOCK_DIV,
                   uint32_t PORT_BASE_ADDRESS,
                   uint32_t PIN_BIT_MASK )
  {
    if (      pwm_gen_periph_begun == false )
    {         pwm_gen_periph_begun  = true;

              SysCtlPeripheralEnable( SYSCTL_PERIPH_PWM0 );
      while (!SysCtlPeripheralReady(  SYSCTL_PERIPH_PWM0 ))
      {   };
    }

    PWMClockSet(      PWM0_BASE, CLOCK_DIV );
    PWMGenConfigure(  PWM0_BASE, PWM_GEN, PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DOWN );
    PWMGenEnable(     PWM0_BASE, PWM_GEN );

    GPIOPinConfigure( PWM_PIN_MUX );
    GPIOPinTypePWM(   PORT_BASE_ADDRESS, PIN_BIT_MASK );
  }

  ///////////////////////////////////////////////////////////////
  void pwmGenWrite ( uint32_t PWM_GEN,
                     uint32_t PWM_OUT,
                     uint32_t PWM_BIT_MASK,
                     uint16_t WIDTH_TICKS_16,
                     uint16_t PERIOD_TICKS_16 )
  {
    PWMGenPeriodSet(  PWM0_BASE, PWM_GEN,      PERIOD_TICKS_16 );
    PWMPulseWidthSet( PWM0_BASE, PWM_OUT,      WIDTH_TICKS_16 );
    PWMOutputState(   PWM0_BASE, PWM_BIT_MASK, true );
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  struct PwmGenHardware pwmGenHardware ( uint8_t pin )
  {
         if ( pin == PF_0    ) { return { GPIO_PF0_M0PWM0, PWM_GEN_0, PWM_OUT_0, PWM_OUT_0_BIT, GPIO_PORTF_BASE, GPIO_PIN_0 }; }
    else if ( pin == PF_1    ) { return { GPIO_PF1_M0PWM1, PWM_GEN_0, PWM_OUT_1, PWM_OUT_1_BIT, GPIO_PORTF_BASE, GPIO_PIN_1 }; }
    else if ( pin == PF_2    ) { return { GPIO_PF2_M0PWM2, PWM_GEN_1, PWM_OUT_2, PWM_OUT_2_BIT, GPIO_PORTF_BASE, GPIO_PIN_2 }; }
    else if ( pin == PF_3    ) { return { GPIO_PF3_M0PWM3, PWM_GEN_1, PWM_OUT_3, PWM_OUT_3_BIT, GPIO_PORTF_BASE, GPIO_PIN_3 }; }
    else if ( pin == PG_0    ) { return { GPIO_PG0_M0PWM4, PWM_GEN_2, PWM_OUT_4, PWM_OUT_4_BIT, GPIO_PORTG_BASE, GPIO_PIN_0 }; }
    else if ( pin == PG_1    ) { return { GPIO_PG1_M0PWM5, PWM_GEN_2, PWM_OUT_5, PWM_OUT_5_BIT, GPIO_PORTG_BASE, GPIO_PIN_1 }; }
    else if ( pin == PK_4    ) { return { GPIO_PK4_M0PWM6, PWM_GEN_3, PWM_OUT_6, PWM_OUT_6_BIT, GPIO_PORTK_BASE, GPIO_PIN_4 }; }
    else if ( pin == PK_5    ) { return { GPIO_PK5_M0PWM7, PWM_GEN_3, PWM_OUT_7, PWM_OUT_7_BIT, GPIO_PORTK_BASE, GPIO_PIN_5 }; }
    else                       { return { INVALID,         INVALID,   INVALID,   INVALID,       INVALID,         INVALID    }; }
  }

  ///////////////////////////////////////////////////
  bool isPwmGenValid( uint8_t pin )
  {
    if      ( pin == PF_0    ) { return { true  }; }
    else if ( pin == PF_1    ) { return { true  }; }
    else if ( pin == PF_2    ) { return { true  }; }
    else if ( pin == PF_3    ) { return { true  }; }
    else if ( pin == PG_0    ) { return { true  }; }
    else if ( pin == PG_1    ) { return { true  }; }
    else if ( pin == PK_4    ) { return { true  }; }
    else if ( pin == PK_5    ) { return { true  }; }
    else                       { return { false }; }
  }

} // end namespace roveware ///////////////////////////////////////////////////////////////////////////////////