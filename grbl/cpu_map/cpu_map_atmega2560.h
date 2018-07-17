/*
  cpu_map_atmega2560.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* This cpu_map file serves as a central pin mapping settings file for AVR Mega 2560 */


#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#else
#define GRBL_PLATFORM "Atmega2560"
#endif


// Serial port pins
#define SERIAL_RX USART0_RX_vect
#define SERIAL_UDRE USART0_UDRE_vect

// Increase Buffers to make use of extra SRAM
//#define RX_BUFFER_SIZE		256
//#define TX_BUFFER_SIZE		128
//#define BLOCK_BUFFER_SIZE	36
//#define LINE_BUFFER_SIZE	100

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
#define STEP_DDR      DDRK
#define STEP_PORT     PORTK
#define STEP_PIN      PINK
#define X_STEP_BIT    0 // MEGA2560 Digital Pin PK0
#define Y_STEP_BIT    1 // MEGA2560 Digital Pin PK1
#define Z_STEP_BIT    2 // MEGA2560 Digital Pin PK2
#define A_STEP_BIT    5 // MEGA2560 Digital Pin PK5
#define B_STEP_BIT    6 // MEGA2560 Digital Pin PK6
#define C_STEP_BIT    7 // MEGA2560 Digital Pin PK7
#define D_STEP_BIT    4 // MEGA2560 Digital Pin PK4
#define E_STEP_BIT    3 // MEGA2560 Digital Pin PK3
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)|(1<<A_STEP_BIT)|(1<<B_STEP_BIT)|(1<<C_STEP_BIT)|(1<<D_STEP_BIT)|(1<<E_STEP_BIT)) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
//todo: CONVERT YO BUFFER FANOUT
#define DIRECTION_DDRB     DDRB
#define DIRECTION_PORTB    PORTB
#define DIRECTION_PINB     PINB

#define DIRECTION_DDRC     DDRC
#define DIRECTION_PORTC    PORTC
#define DIRECTION_PINC     PINC

#define DIRECTION_DDRG     DDRG
#define DIRECTION_PORTG    PORTG
#define DIRECTION_PING     PING

#define DIRECTION_DDRL     DDRL
#define DIRECTION_PORTL    PORTL
#define DIRECTION_PINL     PINL

#define X_DIRECTION_BIT   0 // MEGA2560 Digital Pin PC0
#define Y_DIRECTION_BIT   2 // MEGA2560 Digital Pin PG2
#define Z_DIRECTION_BIT   0 // MEGA2560 Digital Pin PG0
#define A_DIRECTION_BIT   3 // MEGA2560 Digital Pin PB3
#define B_DIRECTION_BIT   2 // MEGA2560 Digital Pin PB2
#define C_DIRECTION_BIT   0 // MEGA2560 Digital Pin PB0
#define D_DIRECTION_BIT   4 // MEGA2560 Digital Pin PL4
#define E_DIRECTION_BIT   6 // MEGA2560 Digital Pin PL6 
#define DIRECTION_MASKB ((1<<A_DIRECTION_BIT)|(1<<B_DIRECTION_BIT)|(1<<C_DIRECTION_BIT))
#define DIRECTION_MASKC (1<<X_DIRECTION_BIT)
#define DIRECTION_MASKG ((1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT))
#define DIRECTION_MASKL ((1<<D_DIRECTION_BIT)|(1<<E_DIRECTION_BIT)) // All direction bits



// Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_DDR   DDRL
#define STEPPERS_DISABLE_PORT  PORTL
#define STEPPERS_DISABLE_BIT   3 // MEGA2560 Digital Pin PL3
#define STEPPERS_DISABLE_MASK (1<<STEPPERS_DISABLE_BIT)

// Define homing/hard limit switch input pins and limit interrupt vectors.
// NOTE: All limit bit pins must be on the same port
//todo: CONVERT TO ISR FANOUT
#define LIMIT_DDRL       DDRL
#define LIMIT_PORTL      PORTL
#define LIMIT_PINL       PINL

#define LIMIT_DDRG       DDRG
#define LIMIT_PORTG      PORTG
#define LIMIT_PING       PING

#define LIMIT_DDRC       DDRC
#define LIMIT_PORTC      PORTC
#define LIMIT_PINC       PINC

#define LIMIT_DDRB       DDRB
#define LIMIT_PORTB      PORTB
#define LIMIT_PINB       PINB

#define LIMIT_DDRD       DDRD
#define LIMIT_PORTD      PORTD
#define LIMIT_PIND       PIND

#define X_LIMIT_BIT     2 // MEGA2560 Digital Pin PL2
#define Y_LIMIT_BIT     1 // MEGA2560 Digital Pin PG1
#define Z_LIMIT_BIT     1 // MEGA2560 Digital Pin PC1
#define A_LIMIT_BIT     1 // MEGA2560 Digital Pin PB1
#define B_LIMIT_BIT     7 // MEGA2560 Digital Pin PL7
#define C_LIMIT_BIT     7 // MEGA2560 Digital Pin PD7
#define D_LIMIT_BIT     1 // MEGA2560 Digital Pin ?
#define E_LIMIT_BIT     1 // MEGA2560 Digital Pin ?
#define LIMIT_MASKL ((1<<X_LIMIT_BIT)|(1<<B_LIMIT_BIT))
#define LIMIT_MASKG (1<<Y_LIMIT_BIT)
#define LIMIT_MASKC (1<<Z_LIMIT_BIT)
#define LIMIT_MASKB (1<<A_LIMIT_BIT)
#define LIMIT_MASKD (1<<C_LIMIT_BIT)

//Stopped porting firmware here, as the GRBL flow logic will need redone to complete porting to infinity
#define LIMIT_INT_disabled 1
#if defined(LIMIT_INT_disabled)
	#pragma message " atmega2560 only support pin change ISR on port B, C and D. GRBL must be refactored to poll in the step move loop"
#endif
#define LIMIT_INT      PCIE0  // Pin change interrupt enable pin
#define LIMIT_INT_vect  PCINT0_vect
#define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register





// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_DDR      DDRE
#define SPINDLE_ENABLE_PORT     PORTE
#define SPINDLE_ENABLE_BIT      3 // MEGA2560 Digital Pin PE3
#define SPINDLE_DIRECTION_DDR   DDRE
#define SPINDLE_DIRECTION_PORT  PORTE
#define SPINDLE_DIRECTION_BIT   4 // MEGA2560 Digital Pin PE4


// Define flood and mist coolant enable output pins.
// NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
// a later date if flash and memory space allows.
#define COOLANT_FLOOD_DDR     DDRH
#define COOLANT_FLOOD_PORT    PORTH
#define COOLANT_FLOOD_BIT     3 // MEGA2560 Digital Pin PH3
#ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
#define COOLANT_MIST_DDR    DDRE
#define COOLANT_MIST_PORT   PORTE
#define COOLANT_MIST_BIT    5 // MEGA2560 Digital Pin PE5
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
#define CONTROL_DDR       DDRC
#define CONTROL_PIN       PINC
#define CONTROL_PORT      PORTC
#define RESET_BIT         6  // MEGA2560 Analog Pin PC6
#define FEED_HOLD_BIT     1  // MEGA2560 Analog Pin PA7
#define CYCLE_START_BIT   2  // MEGA2560 Analog Pin PC7
#define SAFETY_DOOR_BIT   3  // MEGA2560 Analog Pin PA6
#define CONTROL_INT       PCIE2  // Pin change interrupt enable pin
#define CONTROL_INT_vect  PCINT2_vect
#define CONTROL_PCMSK     PCMSK2 // Pin change interrupt register
//#define CONTROL_MASK ((1<<RESET_BIT)|(1<<FEED_HOLD_BIT)|(1<<CYCLE_START_BIT)|(1<<SAFETY_DOOR_BIT))
//#define CONTROL_INVERT_MASK CONTROL_MASK // May be re-defined to only invert certain control pins.

  #define CONTROL_RESET_BIT         RESET_BIT
  #define CONTROL_FEED_HOLD_BIT     FEED_HOLD_BIT
  #define CONTROL_CYCLE_START_BIT   CYCLE_START_BIT
  #define CONTROL_SAFETY_DOOR_BIT   SAFETY_DOOR_BIT
  #define CONTROL_MASK      ((1<<CONTROL_RESET_BIT)|(1<<CONTROL_FEED_HOLD_BIT)|(1<<CONTROL_CYCLE_START_BIT)|(1<<CONTROL_SAFETY_DOOR_BIT))
  #define CONTROL_INVERT_MASK   CONTROL_MASK // May be re-defined to only invert certain control pins.


// Define probe switch input pin.
#define PROBE_DDR       DDRL
#define PROBE_PIN       PINL
#define PROBE_PORT      PORTL
#define PROBE_BIT       1  // MEGA2560 Analog Pin PL1
#define PROBE_MASK      (1<<PROBE_BIT)

// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE
  // Advanced Configuration Below You should not need to touch these variables
  // Set Timer up to use TIMER4B which is attached to Digital Pin 7
  #define PWM_MAX_VALUE       255.0
  #define TCCRA_REGISTER		TCCR4A
  #define TCCRB_REGISTER		TCCR4B
  #define OCR_REGISTER		OCR4B

  #define COMB_BIT			COM4B1
  #define WAVE0_REGISTER		WGM40
  #define WAVE1_REGISTER		WGM41
  #define WAVE2_REGISTER		WGM42
  #define WAVE3_REGISTER		WGM43

  #define SPINDLE_PWM_DDR		DDRG
  #define SPINDLE_PWM_PORT    PORTG
  #define SPINDLE_PWM_BIT		0 // MEGA2560 Digital Pin 41
  
  //??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
    // Variable spindle configuration below. Do not change unless you know what you are doing.
  // NOTE: Only used when variable spindle is enabled.
  #define SPINDLE_PWM_MAX_VALUE     255 // Don't change. 328p fast PWM mode fixes top value as 255.
  #ifndef SPINDLE_PWM_MIN_VALUE
    #define SPINDLE_PWM_MIN_VALUE   1   // Must be greater than zero.
  #endif
  #define SPINDLE_PWM_OFF_VALUE     0
  #define SPINDLE_PWM_RANGE         (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)
  #define SPINDLE_TCCRA_REGISTER	  TCCR4A
  #define SPINDLE_TCCRB_REGISTER	  TCCR4B
  #define SPINDLE_OCR_REGISTER      OCR4B
  #define SPINDLE_COMB_BIT	        COM4B1

  // Prescaled, 8-bit Fast PWM mode.
  #define SPINDLE_TCCRA_INIT_MASK   ((1<<WGM20) | (1<<WGM21))  // Configures fast PWM mode.
  // #define SPINDLE_TCCRB_INIT_MASK   (1<<CS20)               // Disable prescaler -> 62.5kHz
  // #define SPINDLE_TCCRB_INIT_MASK   (1<<CS21)               // 1/8 prescaler -> 7.8kHz (Used in v0.9)
  // #define SPINDLE_TCCRB_INIT_MASK   ((1<<CS21) | (1<<CS20)) // 1/32 prescaler -> 1.96kHz
  #define SPINDLE_TCCRB_INIT_MASK      (1<<CS22)               // 1/64 prescaler -> 0.98kHz (J-tech laser)

  // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
  //#define SPINDLE_PWM_DDR	  DDRB
  //#define SPINDLE_PWM_PORT  PORTB
  //#define SPINDLE_PWM_BIT	  3    // Uno Digital Pin 11

  
  
#endif // End of VARIABLE_SPINDLE
