/*
 *  DSHOT:    Generation of up to 6 DSHOT signals using DMA
 *
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */

// Includes
#include <Arduino.h>
#include "DMAChannel.h"
#include "DSHOT.h"

/*
 *  Constants
 */

#define F_TMR F_BUS_ACTUAL

/* Defining DSHOT600 timings expressed in F_TMR periods
 * DSHOT600 has the following timings:
 *
 *          1670ns
 *          --------->
 *          ______
 * 1 bit :  |     |___|
 *          1250ns
 *          ____
 * 0 bit :  |   |_____|
 *          625ns
 *
 * On the teensy 3.5, F_TMR == 60000000 (60MHz)
 * On the teensy 4.0, F_TMR == 600000000 (600Mhz)
 */
const uint16_t DSHOT_short_pulse  = uint64_t(F_TMR) * DSHOT_SP_DURATION / 1000000000;     // DSHOT short pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_long_pulse   = uint64_t(F_TMR) * DSHOT_LP_DURATION / 1000000000;     // DSHOT long pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_bit_length   = uint64_t(F_TMR) * DSHOT_BT_DURATION / 1000000000;     // DSHOT bit duration (nb of F_BUS periods)

/*
 *  Global variables
 */

// Number of initialized DSHOT outputs
uint8_t             DSHOT_n;

// DMA eFlexPWM modules
volatile IMXRT_FLEXPWM_t  *DSHOT_mods[DSHOT_NB_DMA_CHAN]  = { &IMXRT_FLEXPWM2, 
                                                              &IMXRT_FLEXPWM1,  
                                                              &IMXRT_FLEXPWM1,  
                                                              &IMXRT_FLEXPWM4,  
                                                              &IMXRT_FLEXPWM4, 
                                                              &IMXRT_FLEXPWM2
                                                            };

// DMA eFlexPWM submodules
volatile uint8_t          DSHOT_sm[DSHOT_NB_DMA_CHAN]     = { 0, 
                                                              3, 
                                                              2, 
                                                              0, 
                                                              1, 
                                                              2
                                                            };

// DMA eFlexPWM submodule PWM channel selector: A=0, B=1, X=2
volatile uint8_t  	      DSHOT_abx[DSHOT_NB_DMA_CHAN]    = { 0, 
                                                              0, 
                                                              2, 
                                                              0, 
                                                              0, 
                                                              1
                                                            };

// Output pins
volatile uint8_t          DSHOT_pin[DSHOT_NB_DMA_CHAN]    = { 4, 
                                                              8, 
                                                              24, 
                                                              22, 
                                                              23, 
                                                              9
                                                            };

// Output pin ALT mux
volatile uint8_t          DSHOT_pinmux[DSHOT_NB_DMA_CHAN] = { 1, 
                                                              6, 
                                                              4, 
                                                              1, 
                                                              1, 
                                                              2
                                                            };

// DMA source
volatile uint8_t          DSHOT_dmamux[DSHOT_NB_DMA_CHAN] = { DMAMUX_SOURCE_FLEXPWM2_WRITE0,
                                                              DMAMUX_SOURCE_FLEXPWM1_WRITE3,
                                                              DMAMUX_SOURCE_FLEXPWM1_WRITE2,
                                                              DMAMUX_SOURCE_FLEXPWM4_WRITE0,
                                                              DMAMUX_SOURCE_FLEXPWM4_WRITE1,
                                                              DMAMUX_SOURCE_FLEXPWM2_WRITE2
                                                            };


// DMA objects
DMAChannel          dma[DSHOT_MAX_OUTPUTS];

// DMA data
volatile uint16_t   DSHOT_dma_data[DSHOT_MAX_OUTPUTS][DSHOT_DMA_LENGTH];

/* 
 * DMA termination interrupt service routine (ISR) for each DMA channel
 */
#define DSHOT_DMA_interrupt_routine( DSHOT_CHANNEL ) \
void DSHOT_DMA_interrupt_routine_ ## DSHOT_CHANNEL( void ) { \
  dma[DSHOT_CHANNEL].clearInterrupt( ); \
  (*DSHOT_mods[DSHOT_CHANNEL]).MCTRL &= FLEXPWM_MCTRL_RUN( 1 << DSHOT_sm[DSHOT_CHANNEL] );  \
}

DSHOT_DMA_interrupt_routine( 0 );
DSHOT_DMA_interrupt_routine( 1 );
DSHOT_DMA_interrupt_routine( 2 );
DSHOT_DMA_interrupt_routine( 3 );
DSHOT_DMA_interrupt_routine( 4 );
DSHOT_DMA_interrupt_routine( 5 );

void (*DSHOT_DMA_ISR[6])()  = { DSHOT_DMA_interrupt_routine_0,
                                DSHOT_DMA_interrupt_routine_1,
                                DSHOT_DMA_interrupt_routine_2,
                                DSHOT_DMA_interrupt_routine_3,
                                DSHOT_DMA_interrupt_routine_4,
                                DSHOT_DMA_interrupt_routine_5
                              };


/*
 *  Initialize the DMA hardware in order to be able
 *  to generate 6 DSHOT outputs.
 */
void DSHOT_init( int n ) {
  int i, j;

  if ( n <= DSHOT_MAX_OUTPUTS )
    DSHOT_n = n;
  else
    DSHOT_n = DSHOT_MAX_OUTPUTS;

  // Initialize DMA data
  for ( i = 0; i < DSHOT_n; i++ ) {
    for ( j = 0; j < DSHOT_DMA_LENGTH; j++ ) {
      DSHOT_dma_data[i][j] = 0;
    }
  }

  // Configure pins on the board as DSHOT outputs
  // These pins are configured as eFlexPWM (FLEXPWMn) PWM outputs
  for ( i = 0; i < DSHOT_n; i++ ) {
    *(portConfigRegister( DSHOT_pin[i] ))  = DSHOT_pinmux[i];
  }

  // Configure eFlexPWM modules and submodules for PWM generation
  // --- submodule specific registers ---
  // INIT: initial counter value
  // VAL0: PWM_X compare value
  // VAL1: counter max value
  // VAL2: must be 0 for edge-aligned PWM
  // VAL3: PWM_A compare value
  // VAL4: must be 0 for edge-aligned PWM
  // VAL5: PWM_B compare value
  // OCTRL: invert polarity of PWMq FLEXPWM_SMOCTRL_POLq
  // DMAEN: FLEXPWM_SMDMAEN_VALDE to enable DMA
  // --- module specific registers ---
  // OUTEN: output enable for submodule n and PWM q FLEXPWM_OUTEN_PWMq_EN( 1 << n )
  for ( i = 0; i < DSHOT_n; i++ ) {
    (*DSHOT_mods[i]).SM[DSHOT_sm[i]].INIT = 0;
    (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL0 = 0;
    (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL1 = DSHOT_bit_length;
    (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL2 = 0;
    (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL3 = 0;
    (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL4 = 0;
    (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL5 = 0;
    if ( DSHOT_abx[i] == 2 ) {
      (*DSHOT_mods[i]).SM[DSHOT_sm[i]].OCTRL = FLEXPWM_SMOCTRL_POLX;
      (*DSHOT_mods[i]).OUTEN |= FLEXPWM_OUTEN_PWMX_EN(1 << DSHOT_sm[i]);
    } else if ( DSHOT_abx[i] == 1 ) {
      (*DSHOT_mods[i]).OUTEN |= FLEXPWM_OUTEN_PWMB_EN(1 << DSHOT_sm[i]);
    } else {
      (*DSHOT_mods[i]).OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << DSHOT_sm[i]);
    }
    (*DSHOT_mods[i]).SM[DSHOT_sm[i]].DMAEN = FLEXPWM_SMDMAEN_VALDE;
  }

  // Each DMA channel is linked to a unique eFlexPWM submodule
  // DMA channels are triggered by independant hardware events
  for ( i = 0; i < DSHOT_n; i++ ) {
    dma[i].sourceBuffer( DSHOT_dma_data[i], DSHOT_DMA_LENGTH * sizeof( uint16_t ) );
    if ( DSHOT_abx[i] == 2 ) {
      dma[i].destination( (uint16_t&) (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL0 );
    } else if ( DSHOT_abx[i] == 1 ) {
      dma[i].destination( (uint16_t&) (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL5 );
    } else {
      dma[i].destination( (uint16_t&) (*DSHOT_mods[i]).SM[DSHOT_sm[i]].VAL3 );
    }
    dma[i].triggerAtHardwareEvent( DSHOT_dmamux[i] );
    dma[i].interruptAtCompletion( );
    dma[i].attachInterrupt( DSHOT_DMA_ISR[i] );
    dma[i].enable( );
  }


}

//
//  Send the DSHOT signal through all the configured channels
//  "cmd" points to the DSHOT_MAX_OUTPUTS DSHOT commands to send
//
//  Returns an error code in case of failure, 0 otherwise:
//
int DSHOT_send( uint16_t *cmd) {

  int       i, j;
  uint16_t  data;

  // Initialize DMA buffers
  for ( i = 0; i < DSHOT_n; i++ ) {

    // Check cmd value
    if ( cmd[i] > DSHOT_MAX_VALUE ) {
      return DSHOT_ERROR_RANGE;
    }

    // Compute the packet to send
    // 11 first MSB = command
    // 12th MSB = telemetry request
    // 4 LSB = CRC
    data = ( cmd[i] << 5 );
    data |= ( ( data >> 4 ) ^ ( data >> 8 ) ^ ( data >> 12 ) ) & 0x0f;

    // Generate DSHOT timings corresponding to the packet
    for ( j = 0; j < DSHOT_DSHOT_LENGTH; j++ )  {

        if ( data & ( 1 << ( DSHOT_DSHOT_LENGTH - 1 - j ) ) ) {
            DSHOT_dma_data[i][j] = DSHOT_long_pulse;
        } else {
            DSHOT_dma_data[i][j] = DSHOT_short_pulse;
        }

    }
  }

  // Clear error flag on all DMA channels
  for ( i = 0; i < DSHOT_n; i++ ) {
      dma[i].clearError( );
  }


  // Start DMA by activating the clocks
  // Clocks are disabled again by the DMA ISRs
  IMXRT_FLEXPWM2.MCTRL |= FLEXPWM_MCTRL_RUN(15); 
  IMXRT_FLEXPWM1.MCTRL |= FLEXPWM_MCTRL_RUN(15); 
  IMXRT_FLEXPWM4.MCTRL |= FLEXPWM_MCTRL_RUN(15); 

  // Wait the theoretical time needed by DMA + some margin
  delayMicroseconds( (unsigned int)( ( DSHOT_BT_DURATION * ( DSHOT_DMA_LENGTH + DSHOT_DMA_MARGIN ) ) / 1000 ) );

  return 0;
}
